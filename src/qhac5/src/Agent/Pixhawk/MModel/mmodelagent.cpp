#include "mmodelagent.h"
#include "mmodelcomm.h"
#include "mavlinkdata.h"
#include "rosdata.h"
#include "mmodelcmdsender.h"

CMModelAgent::CMModelAgent(QObject *parent)
    :IAgent(parent)
{
}

CMModelAgent::CMModelAgent(QMap<QString, QString> aProperty, QObject *parent)
    :IAgent(aProperty, parent)
{
	mSysID = 0;
	mIPAddr = QString("");
	mLedColor = QColor(255,255,255);
    mData = nullptr;    
    mSender = nullptr;

    mTimer = new QTimer(this);
}

CMModelAgent::~CMModelAgent()
{
    mTimer->stop();

    if ( mData != nullptr ) delete mData;    
    if ( mSender != nullptr ) delete mSender;
    if ( mTimer != nullptr) delete mTimer;
    if ( mFTP != nullptr )  delete mFTP;

}

void CMModelAgent::init()
{
	// init default gain
	initDefaultGainValue();

	// init default params
	QString type = this->info("type").toString().toUpper();
	if ( type == QString("XMODEL") ) {
		initXModelDefaultParam();
	}
	else if ( type == QString("MMODEL") ) {
		initMModelDefaultParam();
	}
	else  {
		qDebug("ERROR: There is no init default params");
	}
    // init FTP
    mFTP = new FileManager(this, this);

	// init navigation data (from MAVLink)
    mData = new CMavLinkData(this, this);
    mData->setAgent(this);
    mData->setFileManager(mFTP);

	// init communication (singleton pattern)
	int port = 0;
	if ( this->info("mode") == QString("hils") ) {
		port = 14550;
	}
	else if ( this->info("mode") == QString("sitl") ) {
		port = 14550;
	}
	else if ( this->info("mode") == QString("udp") ) {
		port = 9750;
	}
	else {
		port = 9750;
		qDebug("WARN: default port : 9750");
	}
	if(info("port") == "") addInfo("port", QString("%1").arg(port));

	mIPAddr = info("ip").toString();

	mComm = CMModelComm::instance(port);
	mComm->addAgent(this);

	// init commander
	mSender = new CMModelCmdSender(this);
    connect(mTimer, SIGNAL(timeout()), mSender, SLOT(onTimeout()));
    mTimer->start(1000/10);		// 10 Hz
}

int CMModelAgent::cmd(const char *aCmd, QVariant aArg1, QVariant aArg2, QVariant aArg3, QVariant aArg4, QVariant aArg5, QVariant aArg6)
{
    Q_UNUSED(aArg5);

    QString item = QString(aCmd).toUpper().trimmed();
    if  ( item == "ARM" ) {
    	printf("CMModelAgent::cmd ARM\n");
		double posx = data("POSX").toDouble();
		double posy = data("POSY").toDouble();
		mSender->move(posx, posy, 0, 0);
		mSender->arm();
    }
    else if (item == "DISARM" ) {
        mSender->disarm();
    }
    else if (item == "MOVE" ) {
		mSender->move(aArg1.toFloat(), aArg2.toFloat(), aArg3.toFloat(), aArg4.toFloat());
    }
    else if (item == "TAKEOFF" ) {
		double posx = data("POSX").toDouble();
		double posy = data("POSY").toDouble();
		mSender->offboard();
		mSender->move(posx, posy, aArg1.toDouble(), aArg2.toDouble());
    }
	else if (item == "LANDING" ) {
		double posx = data("POSX").toDouble();
		double posy = data("POSY").toDouble();
		mSender->move(posx, posy, -1.0, aArg1.toDouble());
    }
    else if ( item == "OFFBOARD" ) {
        mSender->offboard();
    }
    else if ( item == "MANUAL" ) {
        mSender->manual();
    }
	else if ( item == "LED" ) {
		mLedColor = QColor(aArg2.toInt(), aArg3.toInt(), aArg4.toInt(), aArg5.toInt());
        mSender->led(aArg1.toInt(), aArg2.toInt(), aArg3.toInt(), aArg4.toInt(), aArg5.toInt(), aArg6.toInt());
	}
    else if ( item == "LEDCMD" ) {
        mLedColor = QColor(aArg2.toInt(), aArg3.toInt(), aArg4.toInt(), aArg5.toInt());
        mSender->ledcmd(aArg1.toInt(), aArg2.toInt(), aArg3.toInt(), aArg4.toInt(), aArg5.toInt(), aArg6.toInt());
    }
	else if ( item == "CALIB_GYRO") {
		mSender->calib_gyro();
	}
	else if ( item == "CALIB_LEVEL") {
		mSender->calib_level();
	}
    else if ( item == "CALIB_ACCEL") {
        mSender->calib_aceel();
    }
	else if ( item == "RESET_PARAM") {
		mData->resetParams();
	}
	else if ( item == "CHECK_PARAM" ) {
		checkParams(aArg1.toString());
	}
	else if ( item == "SET_PARAM") {
		mSender->setParam(aArg1.toString(), aArg2);
	}
	else if ( item == "SET_OFFSET") {
		mSender->setOffset(aArg1.toFloat(), aArg2.toFloat(), aArg3.toFloat());
	}
	else if ( item == "REBOOT") {
		mSender->reboot();
	}
    else if ( item == "STOP_SCENARIO") {
        mSender->stopScenario();
    }
	else if ( item == "RESERVE_SCENARIO_TIME") {
		mSender->reserveScenarioStartTime(aArg1.toFloat());
	}
    else if ( item == "SET_SCENARIO_CONFS" ) {
		mSender->setScenarioConfs(aArg1.toFloat(), aArg2.toFloat(), aArg3.toFloat(), aArg4.toString());
    }
    else if ( item == "RESET_SCENARIO_CONFS" ) {
        mSender->resetScenario();
    }
    else if ( item == "EMERGENCY_LAND") {
        mSender->emergencyLanding();
    }
    else if ( item == "TEST_RTK_OFF") {
        mSender->testRTKOff();
    }
    else if ( item == "OFF_EMBEDDED_SCENARIO") {
        mSender->setEmbeddedScenario(false);
    }
    else if ( item == "ON_EMBEDDED_SCENARIO") {
        mSender->setEmbeddedScenario(true);
    }
    else if ( item == "UPLOAD_SCEN") {
        mFTP->uploadPath(aArg1.toString(), aArg2.toString());
    }
    else if ( item == "REMOVE_SCEN") {
        mFTP->removeFile(aArg1.toString());
    }
    else if ( item == "CREATE_DIR") {
        mFTP->createDirectory(aArg1.toString());
    }
    else if ( item == "REMOVE_DIR") {
        mFTP->removeDirectory(aArg1.toString());
    }
    else if ( item == "LIST_DIR") {
        mFTP->listDirectory(aArg1.toString());
    } else {
        qDebug("ERROR: Not determined command (%s)", aCmd);
        return -1;
    }

    return 0;
}

QVariant CMModelAgent::data(const char *aName)
{
	QMutexLocker locker(&mMutex);

	QString item = QString(aName).toUpper().trimmed();

	if ( item == "SYSID" ) {
		return mSysID;
	}
	else if ( item == "TARGETX" ) {
		return mSender->target(CMModelCmdSender::TARGET_X);
    }
    else if ( item == "TARGETY" ) {
		return mSender->target(CMModelCmdSender::TARGET_Y);
    }
    else if ( item == "TARGETZ" ) {
		return mSender->target(CMModelCmdSender::TARGET_Z);
    }
	else if ( item == "PARAM_STATUS" ) {
		return paramStatus();
	}
	else if ( item == "DEFAULT_PARAMS" ) {
		return mDefaultParams;
	}
	else if ( item == "REAL_PARAMS") {
		return realParams();
	}
	else if ( item == "LED_COLOR") {
		return mLedColor;
	}
	else if ( item == "PERIOD_SENDER" ) {
		return mSender->period();
	}
	else if ( item == "READY_TO_FLY") {
		return readyToFly();
	}
    else if ( item == "READY_TO_FLY_FROM_MONITORING") {
        return readyToFlyMonitoring();
    }
    else if ( item == "WARNING_FLIGHT_FROM_MONITORING" ) {
        return warningFlight();
    }
    else if ( item == "EMERGENCY_FLIGHT_FROM_MONITORING" ) {
        return emergencyFlight();
    }
	else if ( item == "ACK_CALIBCMD") {
		return mData->checkAck(MAV_CMD_PREFLIGHT_CALIBRATION);
	}
	else if ( item == "ACK_REBOOT") {
		return mData->checkAck(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN);
	}
    else if ( item == "EMBEDDED_SC_OFFSET" ) {
        uint32_t status = mData->data("MONITORING_STATUS1").toUInt();
        return checkBit(status, CMavLinkData::INIT_EMBEDDED_SC_OFFSET);
    }
    else if ( item == "EMBEDDED_SC_FILE" ) {
        uint32_t status = mData->data("MONITORING_STATUS1").toUInt();
        return checkBit(status, CMavLinkData::INIT_EMBEDDED_SC_FILE);
    }
    else if ( item == "EMBEDDED_SC_START_TIME" ) {
        uint32_t status = mData->data("MONITORING_STATUS1").toUInt();
        return checkBit(status, CMavLinkData::INIT_EMBEDDED_SC_START_TIME);
    }
    else if ( item == "IS_ARMED" ) {
        uint32_t status = mData->data("MONITORING_STATUS1").toUInt();
        return checkBit(status, CMavLinkData::ARM_STATUS);
    }
    else if ( item == "IS_LANDED" ) {
        uint32_t status = mData->data("STATUS_LANDED").toUInt();
        return checkBit(status, CMavLinkData::STATUS_LANDED);
    }
    else if ( item == "DIR_FILES") {
        return mDirFiles;
    }
    else {
        return mData->data(QString(aName));
    }
}

QVariant CMModelAgent::dataROS(const char *aName)
{
	// printf("CMModelAgent::dataROS!! %s\n",aName);
	QMutexLocker locker(&mMutex);

	QString item = QString(aName).toUpper().trimmed();

	if ( item == "SYSID" ) {
		return mSysID;
	}
	else if ( item == "MODE" ) {
		// return mSender->target(CMModelCmdSender::TARGET_X);
		return "ROS DATA TEST";
    }
    else {
        // return mData->data(QString(aName));
        return mSysID;
    }
}

void CMModelAgent::initMModelDefaultParam()
{
	// TODO : write the default parameter for MModel
	if ( info("sysid") == "" ) {
		qDebug("WARN: please write sysid.");
		mSysID = this->id();
		addInfo("sysid", QString("%1").arg(mSysID));
	}
	else {
		mSysID = info("sysid").toInt();
	}

	mDefaultParams["PWM_MIN"] = QVariant(1000);
	mDefaultParams["BAT_N_CELLS"] = QVariant(4);

	mDefaultParams["MIS_TAKEOFF_ALT"] = QVariant(2.5f);
	mDefaultParams["MIS_YAWMODE"] = QVariant(0);
	mDefaultParams["RTL_RETURN_ALT"] = QVariant(5.0f);
	//mDefaultParams["COM_RC_IN_MODE"] = QVariant(2);
	mDefaultParams["SENS_EN_LL40LS"] = QVariant(0);


	mDefaultParams["MPC_MANTHR_MIN"] = QVariant(0.0f);
	mDefaultParams["MPC_TILTMAX_AIR"] = QVariant(20.0f);
	mDefaultParams["MPC_XY_P"] = QVariant(1.0f);
	mDefaultParams["MPC_XY_VEL_P"] = QVariant(0.25f);
	mDefaultParams["MPC_XY_VEL_I"] = QVariant(0.02f);
	mDefaultParams["MPC_XY_VEL_D"] = QVariant(0.02f);
	mDefaultParams["MPC_XY_VEL_MAX"] = QVariant(6.0f);
	//mDefaultParams["MPC_Z_VEL_MAX_UP"] = QVariant(3.0f);
	mDefaultParams["MPC_Z_VEL_P"] = QVariant(0.2f);
	mDefaultParams["MPC_Z_VEL_I"] = QVariant(0.1f);
	mDefaultParams["MPC_Z_VEL_D"] = QVariant(0.03f);
	mDefaultParams["MPC_THR_MIN"] = QVariant(0.12f);
	mDefaultParams["MPC_THR_MAX"] = QVariant(0.8f);
	mDefaultParams["MPC_THR_HOVER"] =QVariant(0.5f);
	mDefaultParams["MPC_HOLD_MAX_XY"] = QVariant(0.8f);

	mDefaultParams["MC_ROLL_P"] = QVariant(6.5f);
	mDefaultParams["MC_ROLLRATE_P"] = QVariant(0.1f);
	mDefaultParams["MC_ROLLRATE_I"] = QVariant(0.05f);
	mDefaultParams["MC_ROLLRATE_D"] = QVariant(0.003f);
	mDefaultParams["MC_PITCH_P"] = QVariant(6.5f);
	mDefaultParams["MC_PITCHRATE_P"] = QVariant(0.15f);
	mDefaultParams["MC_PITCHRATE_I"] = QVariant(0.05f);
	mDefaultParams["MC_PITCHRATE_D"] = QVariant(0.003f);
	mDefaultParams["MC_YAW_P"] = QVariant(2.8f);
	mDefaultParams["MC_YAWRATE_P"] = QVariant(0.2f);
	mDefaultParams["MC_YAWRATE_I"] = QVariant(0.1f);
	mDefaultParams["MC_YAWRATE_D"] = QVariant(0.0f);
	mDefaultParams["MC_YAWRATE_MAX"] = QVariant(200.0f);

	mDefaultParams["INAV_W_MOC_P"] = QVariant(10.0f);
	mDefaultParams["INAV_W_Z_BARO"] = QVariant(0.5f);

	mDefaultParams["SYS_COMPANION"] = QVariant(0);

	mDefaultParams["ATT_W_MAG"] = QVariant(0.1f);

	mDefaultParams["LPE_RTK_P"] = QVariant(0.2f);
	mDefaultParams["LPE_VIC_P"] = QVariant(0.2f);
	mDefaultParams["LPE_FUSION"] = QVariant(2193);
	mDefaultParams["LPE_PN_P"] = QVariant(0.1f);
	mDefaultParams["LPE_PN_V"] = QVariant(0.1f);
	mDefaultParams["LPE_X_LP"] = QVariant(5.0f);

}

void CMModelAgent::initXModelDefaultParam()
{
	if ( info("sysid") == "" ) {
		qDebug("WARN: please write sysid.");
		mSysID = this->id();
		addInfo("sysid", QString("%1").arg(mSysID));
	}
	else {
		mSysID = info("sysid").toInt();
	}	

	mDefaultParams["PWM_MIN"] = QVariant(1000);
	mDefaultParams["BAT_N_CELLS"] = QVariant(4);

	mDefaultParams["MIS_TAKEOFF_ALT"] = QVariant(2.5f);
	mDefaultParams["MIS_YAWMODE"] = QVariant(0);
    mDefaultParams["RTL_RETURN_ALT"] = QVariant(5.0f);
	//mDefaultParams["COM_RC_IN_MODE"] = QVariant(2);
    //mDefaultParams["SENS_EN_LL40LS"] = QVariant(0);


    mDefaultParams["MPC_MANTHR_MIN"] = QVariant(0.08f);
	mDefaultParams["MPC_TILTMAX_AIR"] = QVariant(10.0f);
    mDefaultParams["MPC_XY_P"] = QVariant(0.95f);
    mDefaultParams["MPC_XY_VEL_P"] = QVariant(0.15f);
	mDefaultParams["MPC_XY_VEL_I"] = QVariant(0.02f);
	mDefaultParams["MPC_XY_VEL_D"] = QVariant(0.02f);
    mDefaultParams["MPC_XY_VEL_MAX"] = QVariant(2.0f);
	mDefaultParams["MPC_Z_P"] = QVariant(1.0f);
    mDefaultParams["MPC_Z_VEL_MAX_UP"] = QVariant(1.5f);
    mDefaultParams["MPC_Z_VEL_MAX_DN"] = QVariant(1.5f);
    mDefaultParams["MPC_Z_VEL_P"] = QVariant(0.2f);
	mDefaultParams["MPC_Z_VEL_I"] = QVariant(0.1f);
    mDefaultParams["MPC_Z_VEL_D"] = QVariant(0.05f);
	mDefaultParams["MPC_THR_MIN"] = QVariant(0.12f);
    mDefaultParams["MPC_THR_MAX"] = QVariant(0.9f);
    mDefaultParams["MPC_THR_HOVER"] =QVariant(0.5f);
	mDefaultParams["MPC_HOLD_MAX_XY"] = QVariant(0.8f);

	mDefaultParams["MC_ROLL_P"] = QVariant(6.5f);
    mDefaultParams["MC_ROLLRATE_P"] = QVariant(0.08f);
	mDefaultParams["MC_ROLLRATE_I"] = QVariant(0.05f);
    mDefaultParams["MC_ROLLRATE_D"] = QVariant(0.001f);
	mDefaultParams["MC_PITCH_P"] = QVariant(6.5f);
	mDefaultParams["MC_PITCHRATE_P"] = QVariant(0.15f);
	mDefaultParams["MC_PITCHRATE_I"] = QVariant(0.05f);
	mDefaultParams["MC_PITCHRATE_D"] = QVariant(0.003f);
    mDefaultParams["MC_YAW_P"] = QVariant(4.0f);
	mDefaultParams["MC_YAWRATE_P"] = QVariant(0.2f);
	mDefaultParams["MC_YAWRATE_I"] = QVariant(0.1f);
	mDefaultParams["MC_YAWRATE_D"] = QVariant(0.0f);
	mDefaultParams["MC_YAWRATE_MAX"] = QVariant(200.0f);

	mDefaultParams["INAV_W_MOC_P"] = QVariant(10.0f);
    mDefaultParams["INAV_W_Z_BARO"] = QVariant(0.5f);
	mDefaultParams["INAV_W_XY_RTK_P"] = QVariant(1.0f);
	mDefaultParams["INAV_W_XY_RTK_V"] = QVariant(2.0f);
	mDefaultParams["INAV_W_Z_RTK_P"] = QVariant(2.0f);
	mDefaultParams["INAV_W_Z_RTK_V"] = QVariant(2.0f);
	mDefaultParams["INAV_W_ACC_BIAS"] = QVariant(0.05f);
	mDefaultParams["INAV_DELAY_GPS"] = QVariant(0.15f);

	mDefaultParams["SYS_COMPANION"] = QVariant(0);
    mDefaultParams["SYS_MC_EST_GROUP"] = QVariant(0);

	mDefaultParams["ATT_W_MAG"] = QVariant(0.1f);

//	mDefaultParams["LPE_RTK_P"] = QVariant(0.2f);
//	mDefaultParams["LPE_VIC_P"] = QVariant(0.2f);
//	mDefaultParams["LPE_FUSION"] = QVariant(2193);
//	mDefaultParams["LPE_PN_P"] = QVariant(0.1f);
//	mDefaultParams["LPE_PN_V"] = QVariant(0.1f);
//	mDefaultParams["LPE_X_LP"] = QVariant(5.0f);
//	mDefaultParams["LPE_GPS_VXY"] = QVariant(0.25f);
//	mDefaultParams["LPE_VXY_PUB"] = QVariant(0.3f);
//	mDefaultParams["LPE_RTK_P"] = QVariant(0.3f);
//	mDefaultParams["LPE_RTK_V"] = QVariant(0.3f);
//	mDefaultParams["LPE_ACC_XY"] = QVariant(0.012f);
//	mDefaultParams["LPE_GPS_DELAY"] = QVariant(0.29f);

    mDefaultParams["COM_ARM_MAG"] = QVariant(0.15f);
	mDefaultParams["COM_ARM_IMU_ACC"] = QVariant(0.7f);
	mDefaultParams["COM_ARM_IMU_GYR"] = QVariant(0.25f);

	mDefaultParams["SDLOG_MODE"] = QVariant(1);

	mDefaultParams["CBRK_VELPOSERR"] = QVariant(201607);

    mDefaultParams["COM_RC_LOSS_T"] = QVariant(2400.0f);
}

void CMModelAgent::initDefaultGainValue()
{
    // TODO : check necessary properties (info)
}

void CMModelAgent::checkParams(QString name)
{
	if ( name == "" || name == "0" ) {
		// request all parameters
		QMapIterator<QString, QVariant> i(mDefaultParams);
		while (i.hasNext()) {
			i.next();
			mSender->requstParam(i.key());
		}
	}
	else {
		mSender->requstParam(name);
	}

}

bool CMModelAgent::checkBit(uint32_t aValue, uint aBit)
{
    return (aValue & (1<<aBit)) > 0;
}

QString CMModelAgent::readyToFly()
{
	if ( mData->data("BATTERY").toInt() < 30 ) {
		return "ERROR: LOW BATTERY";
	}
	else if ( fabsf(mData->data("ROLL").toFloat()) > 5.0 ) {
		return "ERROR: unstable ROLL";
	}
	else if ( fabsf(mData->data("PITCH").toFloat()) > 5.0 ) {
		return "ERROR: unstable PITCH";
	}
	else if ( fabsf(mData->data("VEL_X").toFloat()) > 0.3 ) {
		return "ERROR: high velocity x";
	}
	else if ( fabsf(mData->data("VEL_Y").toFloat()) > 0.3 ) {
		return "ERROR: high velocity y";
	}
	else if ( fabsf(mData->data("VEL_Z").toFloat()) > 0.1 ) {
		return "ERROR: high velocity z";
	}
	else if ( mData->data("RTK_FIXED").toFloat() != 1 ) {
		return "WAIT: not fixed rtk mode";
	}
	else {
		return "OK";
    }
}

QString CMModelAgent::readyToFlyMonitoring()
{
    uint32_t status = mData->data("MONITORING_STATUS1").toUInt();

    if ( checkBit(status, CMavLinkData::SAFETY_LOCK_STATUS) ) {
        return "ERROR: Locking";
    }

    if ( checkBit(status, CMavLinkData::TEMPERATURE_PROBLEM) ) {
        return "ERROR: Temperature Problem";
    }

    if ( mData->data("MSG_INTERVAL_TIME").toLongLong() > 50000 ) {
        return "ERROR: unstable comm(drone->gcs)";
    }

    if ( checkBit(status, CMavLinkData::INIT_PITCH_PROBLEM) ) {
        return "ERROR: unstable PITCH";
    }

    if ( checkBit(status, CMavLinkData::INIT_ROLL_PROBLEM) ) {
        return "ERROR: unstable ROLL";
    }

    if ( checkBit(status, CMavLinkData::INIT_VELX_PROBLEM) ) {
        return "ERROR: unstable Velocity X";
    }

    if ( checkBit(status, CMavLinkData::INIT_VELY_PROBLEM) ) {
        return "ERROR: unstable Velocity Y";
    }

    if ( checkBit(status, CMavLinkData::INIT_VELZ_PROBLEM) ) {
        return "ERROR: unstable Velocity Z";
    }

	if ( checkBit(status, CMavLinkData::AGE_CORR_LV1_PROBLEM) ) {
		return "ERROR: AGE_CORR_LV1_PROBLEM";
	}

	if ( checkBit(status, CMavLinkData::AGE_CORR_LV2_PROBLEM) ) {
		return "ERROR: AGE_CORR_LV2_PROBLEM";
	}

    if ( checkBit(status, CMavLinkData::PERI_5V_POWER_PROBLEM) ) {
        return "ERROR: unstable Periperal Power";
    }

    if ( !checkBit(status, CMavLinkData::RTKGPS_FIXED_MODE) ) {
        return "ERROR: RTKGPS is not ready";
    }

	if ( checkBit(status, CMavLinkData::INAV_BIAS_XY_PROBLEM) ) {
		return "ERROR: INAV_BIAS_XY_PROBLEM";
	}

	if ( checkBit(status, CMavLinkData::INAV_BIAS_Z_PROBLEM) ) {
		return "ERROR: INAV_BIAS_Z_PROBLEM";
	}

	if ( checkBit(status, CMavLinkData::ACC_INCONSISTENT_PROBLEM) ) {
		return "ERROR: ACC_INCONSISTENT_PROBLEM";
	}

	if ( checkBit(status, CMavLinkData::GYR_INCONSISTENT_PROBLEM) ) {
		return "ERROR: GYR_INCONSISTENT_PROBLEM";
	}

	if ( checkBit(status, CMavLinkData::MAG_INCONSISTENT_PROBLEM) ) {
		return "ERROR: MAG_INCONSISTENT_PROBLEM";
	}

//	if ( checkBit(status, CMavLinkData::INIT_EMBEDDED_SC_OFFSET) ) {
//        return "ERROR: not ready to embedded sc offset ";
//    }

//    if ( !checkBit(status, CMavLinkData::INIT_EMBEDDED_SC_FILE) ) {
//        return "ERROR: not ready to embedded sc file";
//    }

    if ( !checkBit(status, CMavLinkData::INIT_EMBEDDED_SC_START_TIME) ) {
        return "OK(ON)";
    }


    return "OK";
}

QString CMModelAgent::warningFlight()
{
    uint32_t status = mData->data("MONITORING_STATUS1").toUInt();

    if ( checkBit(status, CMavLinkData::ARM_STATUS) ) {
        if ( checkBit(status, CMavLinkData::BATTERY_PROBLEM) ) {
            return "YES";
        }
        else if ( checkBit(status, CMavLinkData::COMM_PROBLEM) ) {
            return "YES";
        }
    }
    return "NO";
}

QString CMModelAgent::emergencyFlight()
{
    uint32_t status = mData->data("MONITORING_STATUS1").toUInt();

    if ( checkBit(status, CMavLinkData::ARM_STATUS) ) {
        if ( checkBit(status, CMavLinkData::BATTERY_PROBLEM) ) {
            return "BATTERY_PROBLEM";
        }
        if ( checkBit(status, CMavLinkData::TEMPERATURE_PROBLEM) ) {
            return "TEMPERATURE_PROBLEM";
        }
        if ( checkBit(status, CMavLinkData::COMM_PROBLEM) ) {
            return "COMM_PROBLEM";
        }
		if ( checkBit(status, CMavLinkData::AGE_CORR_LV1_PROBLEM) ) {
			return "ERROR: AGE_CORR_LV1_PROBLEM";
		}
		if ( checkBit(status, CMavLinkData::AGE_CORR_LV2_PROBLEM) ) {
			return "ERROR: AGE_CORR_LV2_PROBLEM";
		}
		if ( !checkBit(status, CMavLinkData::RTKGPS_FIXED_MODE) ) {
            return "RTK_FIXED_MODE_PROBLEM";
        }
		if ( checkBit(status, CMavLinkData::INAV_BIAS_XY_PROBLEM) ) {
			return "INAV_BIAS_XY_PROBLEM";
		}
		if ( checkBit(status, CMavLinkData::INAV_BIAS_Z_PROBLEM) ) {
			return "INAV_BIAS_Z_PROBLEM";
		}
		if ( checkBit(status, CMavLinkData::AGE_CORR_LV1_PROBLEM) ) {
			return "AGE_CORR_LV1_PROBLEM";
		}
		if ( checkBit(status, CMavLinkData::AGE_CORR_LV2_PROBLEM) ) {
			return "AGE_CORR_LV2_PROBLEM";
		}

	}

    return "NO";
}

QString CMModelAgent::paramStatus()
{
	QMapIterator<QString, QVariant> i(mDefaultParams);
	while (i.hasNext()) {
		i.next();
		//qDebug() << i.key() << ": " << i.value();
		if ( mData->param(i.key()).isNull() ) {
			return "--";
		}
		if ( mData->param(i.key()) != i.value() ) {
			return QString("FAIL[%1]:%2(default:%3)")
					.arg(i.key())
                    .arg(mData->param(i.key()).toFloat())
                    .arg(i.value().toFloat());
		}
	}

	return "OK";
}

QMap<QString, QVariant> CMModelAgent::realParams()
{
	QMap<QString, QVariant>  realParams;

	QMapIterator<QString, QVariant> i(mDefaultParams);
	while (i.hasNext()) {
		i.next();

		if ( mData->param(i.key()).isNull() ) {
			realParams[i.key()] = QString("--");
		}
		else {
			realParams[i.key()] = mData->param(i.key());
		}
	}

	return realParams;
}

