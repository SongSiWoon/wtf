#include "mmodelcmdsender.h"
#include "mmodelagent.h"
#include "mmodelcomm.h"
#include "logger.h"
#include "sleeper.h"
#include "agent.h"
#include <QDateTime>
#include <QMutexLocker>
#include <QtMath>


#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

#define MAV_CMD_SET_LPOS_OFFSET		(31010)
#define MAV_CMD_SET_SCENARIO_START_TIME			(31011)
#define MAV_CMD_SET_SCENARIO_EMERGENCY_LAND		(31012)
#define MAV_CMD_SET_SCENARIO_OFFSET				(31013)

#define MAX_BUF_SIZE				(300)

CMModelCmdSender::CMModelCmdSender(QObject* parent)    
    : QObject(parent)
{
}

CMModelCmdSender::CMModelCmdSender(CMModelAgent* aAgent, QObject* parent)
	: QObject(parent)
{
	mTargetX = 0.0f;
	mTargetY = 0.0f;
	mTargetZ = 0.0f;
	mHead = 0.0;

    mTakeOffReady = false;
    mLandReady = false;
	mGyroCalib = false;
    mAccelCalib = false;
	mLevelCalib = false;
	mReboot = false;
    mEmbeddedScenarioMode = true;

    mAgent = aAgent;
    mComm  = mAgent->comm();

	mSysID = 255;
    mTimeOutCount = 0;

	mPrevTime = 0;    

    m1HzTimer = new QTimer(this);
    connect(m1HzTimer, SIGNAL(timeout()), this, SLOT(onTimeout1Hz()));
    m1HzTimer->start(500);
}

CMModelCmdSender::~CMModelCmdSender()
{
    m1HzTimer->stop();

    if ( m1HzTimer != NULL ) delete m1HzTimer;
}

void CMModelCmdSender::arm()
{    
	mAgent->data()->resetAck();
    mTakeOffReady = true;
}

void CMModelCmdSender::disarm()
{
	mAgent->data()->resetAck();
	mLandReady = true;
}

int CMModelCmdSender::reboot()
{
	mAgent->data()->resetAck();
	mReboot = true;

	return 0;
}

int CMModelCmdSender::procArm()
{
	mavlink_command_long_t com = {};

	com.target_system    = mAgent->sysID();
	com.target_component = 0;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = true;
	com.param1           = 0.5;		// ARM

	// encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(mSysID, 50, &message, &com);

	// send data
	int len = transmit(message);

	return len;
}

int CMModelCmdSender::procDisarm()
{
	mavlink_command_long_t com = {};

	com.target_system    = mAgent->sysID();
	com.target_component = 0;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = true;
	com.param1           = -0.5;	// DISARM

	// encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(mSysID, 50, &message, &com);

	// send data
	int len = transmit(message);

	return len;

}

int CMModelCmdSender::procReboot()
{
	mavlink_command_int_t cmd;

	memset(&cmd, 0, sizeof(cmd));

	cmd.target_system    = mAgent->sysID();
	cmd.target_component = MAV_COMP_ID_ALL;
	cmd.command          = MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
	cmd.param1           = 1; //reboot to bootloader (3) or just reboot(1)
	cmd.param2           = 0;
	cmd.param3           = 0;
	cmd.param4           = 0;

	// encode
	mavlink_message_t message;
	mavlink_msg_command_int_encode(mSysID, 50, &message, &cmd);

	// send data
	int len = transmit(message);

	return len;
}

int CMModelCmdSender::automode()
{


    mavlink_message_t message;
    mavlink_set_mode_t mode;


	mode.target_system = mAgent->sysID();
    //mode.base_mode = 189; //MAV_MODE_MANUAL_ARMED;         //209 0xD1
    // 189 0xBD, 1011 1101
    mode.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                     //MAV_MODE_FLAG_TEST_ENABLED |
                     MAV_MODE_FLAG_AUTO_ENABLED |
                     MAV_MODE_FLAG_GUIDED_ENABLED |
                     MAV_MODE_FLAG_STABILIZE_ENABLED |                
                     //MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                     MAV_MODE_FLAG_SAFETY_ARMED ;
    if ( mAgent->info("mode") == QString("hils") ) {
        mode.base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
    }

    mode.custom_mode = 262144;

    // make data
    mavlink_msg_set_mode_encode(mSysID,50, &message, &mode);


    // send data
	int len = transmit(message);

    return len;
}

int CMModelCmdSender::offboard()
{
    mavlink_command_long_t com;

	const int PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;

    int mode             = MAV_MODE_FLAG_SAFETY_ARMED |
                           MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if ( mAgent->info("mode") == QString("hils") ) {
        mode            |= MAV_MODE_FLAG_HIL_ENABLED;
    }

	com.target_system    = mAgent->sysID();
    com.target_component = 0;
	com.command          = MAV_CMD_DO_SET_MODE;
    com.confirmation     = true;
    com.param1           = mode;
	com.param2           = PX4_CUSTOM_MAIN_MODE_OFFBOARD;

    // encode
    mavlink_message_t message;
	mavlink_msg_command_long_encode(mSysID, 50, &message, &com);

    // send data
	int len = transmit(message);

    return len;
}

int CMModelCmdSender::manual()
{
    mavlink_command_long_t com;

    const int PX4_CUSTOM_MAIN_MODE_MANUAL = 1;

    int mode             = MAV_MODE_FLAG_SAFETY_ARMED |
                           MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if ( mAgent->info("mode") == QString("hils") ) {
        mode            |= MAV_MODE_FLAG_HIL_ENABLED;
    }

    com.target_system    = mAgent->sysID();
    com.target_component = 0;
    com.command          = MAV_CMD_DO_SET_MODE;
    com.confirmation     = true;
    com.param1           = mode;
    com.param2           = PX4_CUSTOM_MAIN_MODE_MANUAL;

    // encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(mSysID, 50, &message, &com);

    // send data
    int len = transmit(message);

    return len;
}

int CMModelCmdSender::offboard_waypoint()
{
    mavlink_message_t message;
    mavlink_set_position_target_local_ned_t sp;

    // initialize	
    memset(&message, 0, sizeof(message));
	sp.target_system = mAgent->sysID();
    sp.target_component = 0;
	sp.type_mask  = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.time_boot_ms = QDateTime::currentMSecsSinceEpoch();
	sp.x = target(TARGET_X);
	sp.y = target(TARGET_Y);
	sp.z = target(TARGET_Z);

	sp.yaw = mHead;
    sp.yaw_rate = 0;

    // encode
    mavlink_msg_set_position_target_local_ned_encode(mSysID, 50, &message, &sp);

    // send data
	int len = transmit(message);

	return len;
}

int CMModelCmdSender::target_offboard()
{
	mavlink_message_t message;
    //mavlink_target_offboard_t sp;
    mavlink_set_position_target_local_ned_t sp;

	// initialize
	memset(&message, 0, sizeof(message));
    memset(&sp, 0, sizeof(sp));

	// create message;
    sp.target_system = mAgent->sysID();
    sp.target_component = 0;
    sp.time_boot_ms = QDateTime::currentMSecsSinceEpoch();
    sp.x = target(TARGET_X);
    sp.y = target(TARGET_Y);
    sp.z = target(TARGET_Z);
    sp.yaw  = qDegreesToRadians(mHead);

//	sp.led_type = mLedType;
//	sp.led_r = mLedR;
//	sp.led_g = mLedG;
//	sp.led_b = mLedB;
//	sp.led_bright = mLedBright;
//	sp.led_speed = mLedSpeed;

//	// encode
//	mavlink_msg_target_offboard_encode(mSysID, 50, &message, &sp);

    mavlink_msg_set_position_target_local_ned_encode(mSysID, 50, &message, &sp);

    // send data
    int len = transmit(message);

	return len;
}

int CMModelCmdSender::heartbit()
{
    mavlink_heartbeat_t heart;
    heart.type = 0x06;
    heart.autopilot = 0x08;
    heart.base_mode = 0xc0;
    heart.custom_mode = 0x0000;
	heart.system_status = 0x04;
    heart.mavlink_version = 3;

    mavlink_message_t message;
    memset(&message, 0, sizeof(message));
    mavlink_msg_heartbeat_encode(mSysID, 50, &message, &heart);

    // send data
	int len = transmit(message);

    return len;

}

int CMModelCmdSender::move(float aX, float aY, float aZ, float aHead)
{
	QMutexLocker locker(&mMutex);

	mTargetX = aX;
	mTargetY = aY;
	mTargetZ = -aZ;
	mHead = aHead;


	return 0;
}

int CMModelCmdSender::led(int aType, int aR, int aG, int aB, int aBrightness, int aSpeed)
{
	QMutexLocker locker(&mMutex);

	mLedType = aType;
	mLedR = aR;
	mLedG = aG;
	mLedB = aB;
	mLedBright = aBrightness;
	mLedSpeed = aSpeed;

    return 0;
}

int CMModelCmdSender::ledcmd(int aType, int aR, int aG, int aB, int aBrightness, int aSpeed)
{
    mavlink_led_control_t led;
    led.r = aR;
    led.g = aG;
    led.b = aB;
    led.brightness = aBrightness;
    led.type = aType;

    // encode
    mavlink_message_t message;
    memset(&message, 0, sizeof(message));
    mavlink_msg_led_control_encode(mSysID, 50, &message, &led);

    // send data	
    int len = transmit(message);

    return len;
}

float CMModelCmdSender::target(CMModelCmdSender::Target aType)
{
	QMutexLocker locker(&mMutex);	
	switch (aType ) {
	case TARGET_X:
		return mTargetX;
		break;
	case TARGET_Y:
		return mTargetY;
		break;
	case TARGET_Z:
		return mTargetZ;
		break;
	default:
		qDebug("ERROR: Invalid target %d", aType);
		break;
	}
    return 0;
}

void CMModelCmdSender::setEmbeddedScenario(bool aMode)
{
    mEmbeddedScenarioMode = aMode;
}

int CMModelCmdSender::upload_scneario(const QString file_path)
{
    return 0;
}

int CMModelCmdSender::procCalibGyro()
{
	mavlink_command_long_t cmd;

	memset(&cmd, 0, sizeof(cmd));

	cmd.target_system    = mAgent->sysID();
	cmd.target_component = 0;
	cmd.command          = MAV_CMD_PREFLIGHT_CALIBRATION;
	cmd.confirmation     = true;
	cmd.param1           = 1;			// gyro
	cmd.param2           = 0;
	cmd.param3           = 0;			// Ground pressure:
	cmd.param4           = 0;
	cmd.param5           = 0;			// 1: accelerometer calib, 2: (board offset)level calib
	cmd.param6           = 0;
	cmd.param7           = 0;

	// encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(mSysID, 50, &message, &cmd);

	// send data
	int len = transmit(message);

	return len;
}

int CMModelCmdSender::procCalibLevel()
{
	mavlink_command_long_t cmd;

	memset(&cmd, 0, sizeof(cmd));

	cmd.target_system    = mAgent->sysID();
	cmd.target_component = 0;
	cmd.command          = MAV_CMD_PREFLIGHT_CALIBRATION;
	cmd.confirmation     = true;
	cmd.param1           = 0;			// gyro
	cmd.param2           = 0;
	cmd.param3           = 0;			// Ground pressure:
	cmd.param4           = 0;
	cmd.param5           = 2;			// 1: accelerometer calib, 2: (board offset)level calib
	cmd.param6           = 0;
	cmd.param7           = 0;

	// encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(mSysID, 50, &message, &cmd);

	// send data
	int len = transmit(message);

    return len;
}

int CMModelCmdSender::procCalibAccel()
{
    mavlink_command_long_t cmd;

    memset(&cmd, 0, sizeof(cmd));

    cmd.target_system    = mAgent->sysID();
    cmd.target_component = 0;
    cmd.command          = MAV_CMD_PREFLIGHT_CALIBRATION;
    cmd.confirmation     = true;
    cmd.param1           = 0;			// gyro
    cmd.param2           = 0;
    cmd.param3           = 0;			// Ground pressure:
    cmd.param4           = 0;
    cmd.param5           = 1;			// 1: accelerometer calib, 2: (board offset)level calib
    cmd.param6           = 0;
    cmd.param7           = 0;

    // encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(mSysID, 50, &message, &cmd);

    // send data
    int len = transmit(message);

    return len;

}

int CMModelCmdSender::control(int aRoll, int aPitch, int aYaw, int aThrust)
{
    mavlink_manual_control_t man;

    man.x = aPitch;
    man.y = aRoll;
    man.z = aThrust;
    man.r = aYaw;

    // make data
    mavlink_message_t message;
    memset(&message, 0, sizeof(message));
    mavlink_msg_manual_control_encode(mSysID, 50, &message, &man);      

    // send data
	int len = transmit(message);

	return len;
}

int CMModelCmdSender::requstParam(const QString aName)
{
	mavlink_param_request_read_t param;

	memset(&param, 0, sizeof(param));
	param.target_system = mAgent->sysID();
	param.target_component = 1;
	strncpy(param.param_id, aName.toLatin1().data(), aName.length());
	param.param_index = -1;    

	// make data
	mavlink_message_t message;
	memset(&message, 0, sizeof(message));
	mavlink_msg_param_request_read_encode(mSysID, 50, &message, &param);

	// send data
	int len = transmit(message);

	return len;
}

int CMModelCmdSender::setParam(const QString aName, const QVariant aValue)
{
	mavlink_param_set_t param;

	memset(&param, 0, sizeof(param));
	param.target_system = mAgent->sysID();
	param.target_component = 1;
	strncpy(param.param_id, aName.toLatin1().data(), aName.length());


	switch (aValue.type()) {
	case QVariant::Char:
	{
		param.param_type  = MAV_PARAM_TYPE_UINT8;		
		int value = aValue.toInt();
		memcpy(&param.param_value, &value, sizeof(value));
	}
		break;
	case QVariant::Int:
	{
		param.param_type  = MAV_PARAM_TYPE_INT32;
		int value = aValue.toInt();
		memcpy(&param.param_value, &value, sizeof(value));

	}
		break;
	case QVariant::UInt:
	{
		param.param_type  = MAV_PARAM_TYPE_UINT32;
		unsigned int value = aValue.toUInt();
		memcpy(&param.param_value, &value, sizeof(value));
	}
		break;
	case QVariant::Double:
		param.param_type  = MAV_PARAM_TYPE_REAL64;
		param.param_value = (float)aValue.toDouble();
		break;
    case QMetaType::Float:
        param.param_type  = MAV_PARAM_TYPE_REAL32;
        param.param_value = (float)aValue.toFloat();
        break;
	default:
        qDebug("WARN: unknown type %d", aValue.type());
		param.param_type  = MAV_PARAM_TYPE_REAL64;
		param.param_value = (float)aValue.toDouble();
		break;
	}

	// make data
	mavlink_message_t message;
	memset(&message, 0, sizeof(message));
	mavlink_msg_param_set_encode(mSysID, 50, &message, &param);

	// send data
	int len = transmit(message);

    return len;
}

int CMModelCmdSender::calib_aceel()
{
    mAgent->data()->resetAck();
    mAccelCalib = true;

    return 0;
}

int CMModelCmdSender::calib_gyro()
{
	mAgent->data()->resetAck();
	mGyroCalib = true;

	return 0;
}

int CMModelCmdSender::calib_level()
{
	mAgent->data()->resetAck();
	mLevelCalib = true;

	return 0;
}

int CMModelCmdSender::setOffset(float aX, float aY, float aZ)
{
	mavlink_command_long_t com;

	memset(&com, 0, sizeof(com));
	com.target_system    = mAgent->sysID();
	com.target_component = 0;
	com.command          = MAV_CMD_SET_LPOS_OFFSET;
	com.confirmation     = true;
	com.param1           = aX;
	com.param2           = aY;
	com.param3           = aZ;

	// encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(mSysID, 50, &message, &com);

	// send data
	int len = transmit(message);

	return len;
}

int CMModelCmdSender::reserveScenarioStartTime(float aStartTime)
{
    mavlink_scenario_cmd_t cmd;

    memset(&cmd, 0, sizeof(cmd));
    cmd.cmd             = SCENARIO_CMD_SET_START_TIME;
    cmd.target_system   = mAgent->sysID();
    cmd.param1          = aStartTime;			// unit : second

    // encode
    mavlink_message_t message;
    mavlink_msg_scenario_cmd_encode(mSysID, 50, &message, &cmd);

    // send data
    int len = transmit(message);

    return len;
}

int CMModelCmdSender::stopScenario()
{
    mavlink_scenario_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd             = SCENARIO_CMD_STOP_SCENARIO;
    cmd.target_system   = mAgent->sysID();

    // encode
    mavlink_message_t message;
    mavlink_msg_scenario_cmd_encode(mSysID, 50, &message, &cmd);

    // send data
    int len = transmit(message);

    return len;
}

int CMModelCmdSender::resetScenario()
{
    mavlink_scenario_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd             = SCENARIO_CMD_RESET_CONFIGS;
    cmd.target_system   = mAgent->sysID();

    // encode
    mavlink_message_t message;
    mavlink_msg_scenario_cmd_encode(mSysID, 50, &message, &cmd);

    // send data
    int len = transmit(message);

    return len;
}

int CMModelCmdSender::setScenarioConfs(float aOffsetX, float aOffsetY, float aRot, QString aFilename)
{
    mavlink_scenario_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));

    if ( aFilename.length() >= sizeof(cmd.param5) ) {
        qDebug("WARN: cannot use the filename");
    }

    cmd.cmd             = SCENARIO_CMD_SET_CONFIGS;
    cmd.target_system   = mAgent->sysID();
    cmd.param1          = aOffsetX;     // unit : m
    cmd.param2          = aOffsetY;     // unit : m
	cmd.param3			= qDegreesToRadians(aRot);		// unit : radian (for roation)
    strncpy((char*)cmd.param5, aFilename.toLatin1().data(), aFilename.length());

    // encode
    mavlink_message_t message;
    mavlink_msg_scenario_cmd_encode(mSysID, 50, &message, &cmd);

    // send data
    int len = transmit(message);

    return len;
}

int CMModelCmdSender::emergencyLanding()
{
    mavlink_scenario_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd             = SCENARIO_CMD_EMERGENCY_LAND;
    cmd.target_system   = mAgent->sysID();

    // encode
    mavlink_message_t message;
    mavlink_msg_scenario_cmd_encode(mSysID, 50, &message, &cmd);

    // send data
    int len = transmit(message);

    return len;
}

int CMModelCmdSender::testRTKOff()
{
    mavlink_scenario_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));

    cmd.cmd             = SCENARIO_CMD_ENUM_ENUM_END;
    cmd.target_system   = mAgent->sysID();

    // encode
    mavlink_message_t message;
    mavlink_msg_scenario_cmd_encode(mSysID, 50, &message, &cmd);

    // send data
    int len = transmit(message);

    return len;
}

int CMModelCmdSender::transmit(const mavlink_message_t &aMsg)
{
	uint8_t  buf[MAX_BUF_SIZE];
	uint16_t bufLen = 0;
	qint64   sentLen = 0;

	QMutexLocker locker(&mMutex);

	// convery mavlink message to raw data
	bufLen = mavlink_msg_to_send_buffer(buf, &aMsg);

	// send data
	if ( bufLen > 0 && bufLen < MAX_BUF_SIZE) {
		sentLen = mComm->write(QByteArray((char*)buf, bufLen), mAgent->id());
	}
	else {
		qDebug("ERROR: cannot send the data to drone");
	}

//    qDebug(">>>>> led : %d %d", sentLen, aMsg.msgid);
	return sentLen;
}

void CMModelCmdSender::onTimeout()
{    
	qint64 time = QDateTime::currentMSecsSinceEpoch();
	if ( mPrevTime != 0 ) mTimerPeriod = time - mPrevTime;
    mPrevTime = time;

    if ( mTakeOffReady ) {
        mTimeOutCount++;
        this->procArm();
		if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_COMPONENT_ARM_DISARM) == MAV_RESULT_ACCEPTED  ) {
			mTakeOffReady = false;
			mTimeOutCount = 0;
		}
    }
	else if ( mLandReady ) {
        mTimeOutCount++;
        this->procDisarm();
		if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_COMPONENT_ARM_DISARM) == MAV_RESULT_ACCEPTED ) {
            mLandReady = false;
            mTimeOutCount = 0;
        }
    }
	else if ( mGyroCalib ) {
        if ( mTimeOutCount % 5 == 0 ) {this->procCalibGyro();}
		mTimeOutCount++;

		if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_PREFLIGHT_CALIBRATION) == MAV_RESULT_ACCEPTED ) {
			mGyroCalib = false;
			mTimeOutCount = 0;
		}

	}
	else if ( mLevelCalib ) {
        if ( mTimeOutCount % 5 == 0 ) {this->procCalibLevel();}
		mTimeOutCount++;

		if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_PREFLIGHT_CALIBRATION) == MAV_RESULT_ACCEPTED ) {
			mLevelCalib = false;
			mTimeOutCount = 0;
		}

	}
    else if ( mAccelCalib ) {
        if ( mTimeOutCount % 5 == 0 ) {this->procCalibAccel();}
        mTimeOutCount++;

        if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_PREFLIGHT_CALIBRATION) == MAV_RESULT_ACCEPTED ) {
            mAccelCalib = false;
            mTimeOutCount = 0;
        }
    }
	else if ( mReboot ) {
        if ( mTimeOutCount % 5 == 0 ) {this->procReboot();}
		mTimeOutCount++;

		if ( mTimeOutCount > 20 || mAgent->data()->checkAck(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN) == MAV_RESULT_ACCEPTED ) {
			mReboot = false;
			mTimeOutCount = 0;
		}

	}
	else {
        if (!mEmbeddedScenarioMode) {
            target_offboard();
        }
	}
}

void CMModelCmdSender::onTimeout1Hz()
{
    //heartbit();
}
