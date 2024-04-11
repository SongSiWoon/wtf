#include "cmodelagent.h"
#include "mavlinkdata.h"
#include "rosdata.h"
#include "cmodelcmdsender.h"
#include "paramcheckworker.h"

CCModelAgent::CCModelAgent(QObject *parent)
    :IAgent(parent)
{
}

CCModelAgent::CCModelAgent(QMap<QString, QString> aProperty, QObject *parent)
    :IAgent(aProperty, parent)
{
	mSysID = 0;
	mIPAddr = QString("");
	mLedColor = QColor(255,255,255);
    mData = nullptr;    
    mSender = nullptr;

    mTimer = new QTimer(this);
    mRosTimer = new QTimer(this);
}

CCModelAgent::~CCModelAgent()
{
    mTimer->stop();
    mRosTimer->stop();

    if ( mData != nullptr ) delete mData;    
    if ( mSender != nullptr ) delete mSender;
    if ( mTimer != nullptr) delete mTimer;
    if ( mRosTimer != nullptr) delete mRosTimer;
}

void CCModelAgent::init()
{
	// init default gain
	initDefaultGainValue();

	// init default params
	QString type = this->info("type").toString().toUpper();
    if ( type == QString("CMODEL") ) {
        initCModelDefaultParam();
	}
	else  {
		qDebug("ERROR: There is no init default params");
	}

	// init navigation data (from MAVLink)
    mData = new CROSData(this, this);
    mData->setAgent(this);

	// init commander
    mSender = new CCModelCmdSender(this);
    connect(mTimer, SIGNAL(timeout()), mSender, SLOT(onTimeout()));
    mTimer->start(1000/10);		// 10 Hz

//    connect(mRosTimer, SIGNAL(timeout()), mData, SLOT(onTimeout()));
//    mRosTimer->start(1000/2);		// 2 Hz

    isInitialized = true;
}

int CCModelAgent::cmd(const char *aCmd, QVariant aArg1, QVariant aArg2, QVariant aArg3, QVariant aArg4, QVariant aArg5, QVariant aArg6)
{
//    Q_UNUSED(aArg5);

    QString item = QString(aCmd).toUpper().trimmed();
    if  ( item == "ARM" ) {
        mArmPos.setLatitude(data("GLOBAL_LAT").toDouble());
        mArmPos.setLongitude(data("GLOBAL_LON").toDouble());
        mArmPos.setAltitude(data("GLOBAL_ALT").toDouble());

        mData->setAgentBaseDiffAlt(aArg1.toDouble() - mArmPos.altitude());
        mSender->arm();
    }
    else if (item == "DISARM" ) {
        mSender->disarm();
    }
    else if (item == "LOCK" ) {
        if (!data()->data("ISARMED").toBool())
            mSender->lock();
    }
    else if (item == "UNLOCK" ) {
        mSender->unlock();
    }
    else if (item == "MOVE" ) {
        // Lat, Lon, Alt, Yaw
        mSender->reposition(aArg1.toFloat(), aArg2.toFloat(), aArg3.toFloat(), aArg4.toDouble());
    }
    else if (item == "MOVE_NED" ) {        
        // arg1 : NED position <QVector3D >
        // arg2 : heading <double>

        QGeoCoordinate pos = NED2LLH(aArg1.value<QVector3D>());
        mSender->reposition(pos.latitude(), pos.longitude(), pos.altitude(), aArg2.toDouble());
        qDebug() << "NED : " << pos;
    }
    else if (item == "SETPOINT" ) {
        // arg1 : NEU position <QVector3D>
        // arg2 : heading <double>

        QVector3D sp = aArg1.value<QVector3D>();
        mSender->setpoint(sp.x(), sp.y(), sp.z(), aArg2.toDouble());
    }
    else if (item == "TAKEOFF" ) {
        double lat = data("GLOBAL_LAT").toDouble();
        double lon = data("GLOBAL_LON").toDouble();
        double alt = data("GLOBAL_ALT").toDouble();
        mSender->takeoff(lat, lon, alt + aArg1.toDouble(), aArg2.toDouble());
    }
    else if (item == "LANDING" ) {
        mSender->landing();
    }
    else if ( item == "OFFBOARD" ) {
        mSender->offboard();
    }
    else if ( item == "AUTOMISSION" ) {
        mSender->automission();
    }
    else if ( item == "MANUAL" ) {
        mSender->manual();
    }
    else if ( item == "OCM_POS" ) {
        mSender->offboard_position_mode();
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
	else if ( item == "REBOOT") {
		mSender->reboot();
	}
    else if ( item == "RESET_LPOS") {
        double lat = data("GLOBAL_LAT").toDouble();
        double lon = data("GLOBAL_LON").toDouble();
        double alt = data("GLOBAL_ALT").toDouble();
        mSender->resetLpos(lat, lon, alt);
    }
    else if ( item == "START_GST") {
        if (!data("IS_GST_RUNNING").toBool())
            mSender->startStreamCmd();
    }
    else if ( item == "STOP_GST") {
        if (data("IS_GST_RUNNING").toBool())
            mSender->stopStreamCmd();
    }
    else if ( item == "SET_RESOLUTION_4K") {
        mSender->sendOsmoCmd("video_resolution", "3840x2160@30");
    }
    else if ( item == "POSITION_SELFIE") {
        mSender->sendOsmoCmd("position", "selfie");
    }
    else if ( item == "POSITION_NORMAL") {
        mSender->sendOsmoCmd("position", "normal");
    }
    else if ( item == "RECORD_START") {
        mSender->sendOsmoCmd("record", "start");
    }
    else if ( item == "RECORD_STOP") {
        mSender->sendOsmoCmd("record", "stop");
    }
    else {
        qDebug("ERROR: Not determined command (%s)", aCmd);
        return -1;
    }

    return 0;
}

QVariant CCModelAgent::data(const char *aName)
{
	QMutexLocker locker(&mMutex);
	QString item = QString(aName).toUpper().trimmed();

	if ( item == "SYSID" ) {
		return mSysID;
	}
	else if ( item == "TARGETX" ) {
        return mSender->target(CCModelCmdSender::TARGET_X);
    }
    else if ( item == "TARGETY" ) {
        return mSender->target(CCModelCmdSender::TARGET_Y);
    }
    else if ( item == "TARGETZ" ) {
        return mSender->target(CCModelCmdSender::TARGET_Z);
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
    else if ( item == "ARMPOS_NED") {
        return LLH2NED(mArmPos);
    }
    else {
        return mData->data(QString(aName));
    }
}

QVariant CCModelAgent::dataROS(const char *aName)
{
    // printf("CCModelAgent::dataROS!! %s\n",aName);
	QMutexLocker locker(&mMutex);

	QString item = QString(aName).toUpper().trimmed();

	if ( item == "SYSID" ) {
		return mSysID;
	}
	else if ( item == "MODE" ) {
        // return mSender->target(CCModelCmdSender::TARGET_X);
		// if (mData->status().armed)
		// 	return (mData->status().mode + " ARMED").c_str();
		// else
		// 	return (mData->status().mode + " UNARMED").c_str();
		return "ROS DATA TEST";
    }
    else {
        // return mData->data(QString(aName));
        return mSysID;
    }
}

void CCModelAgent::initCModelDefaultParam()
{
	// TODO : write the default parameter for BModel
	if ( info("sysid") == "" ) {
        qDebug("WARN: please write sysid.");
		mSysID = this->id();
		addInfo("sysid", QString("%1").arg(mSysID));
	}
	else {
		mSysID = info("sysid").toInt();
	}

    mDefaultParams["BAT_N_CELLS"] = QVariant(4);
    mDefaultParams["BAT_V_CHARGED"] = QVariant(4.20f);
    mDefaultParams["BAT_V_EMPTY"] = QVariant(3.60f);
    mDefaultParams["BAT_V_DIV"] = QVariant(12.1f);
    mDefaultParams["CBRK_IO_SAFETY"] = QVariant(22027);
    mDefaultParams["CBRK_USB_CHK"] = QVariant(197848);
    mDefaultParams["COM_ARM_EKF_AB"] = QVariant(0.0022f);
    mDefaultParams["COM_ARM_EKF_GB"] = QVariant(0.0011f);
    mDefaultParams["COM_ARM_IMU_ACC"] = QVariant(1.0f);
    mDefaultParams["COM_FLTMODE1"] = QVariant(8);
    mDefaultParams["COM_FLTMODE4"] = QVariant(1);
    mDefaultParams["COM_FLTMODE6"] = QVariant(2);
    mDefaultParams["COM_OF_LOSS_T"] = QVariant(0.5f);
    mDefaultParams["COM_RC_OVERRIDE"] = QVariant(0);
    mDefaultParams["EKF2_BARO_NOISE"] = QVariant(2.0f);
    mDefaultParams["EKF2_GPS_P_NOISE"] = QVariant(0.1f);
    mDefaultParams["EKF2_GPS_V_NOISE"] = QVariant(0.2f);
    mDefaultParams["EKF2_REQ_EPH"] = QVariant(2.0f);
    mDefaultParams["EKF2_REQ_EPV"] = QVariant(2.0f);
    mDefaultParams["EKF2_HGT_MODE"] = QVariant(1);
    mDefaultParams["GPS_1_CONFIG"] = QVariant(0);
    mDefaultParams["IMU_DGYRO_CUTOFF"] = QVariant(0.0f);
    mDefaultParams["IMU_GYRO_RATEMAX"] = QVariant(400);
    mDefaultParams["MC_PITCHRATE_P"] = QVariant(0.19f);
    mDefaultParams["MC_PITCHRATE_I"] = QVariant(0.1f);
    mDefaultParams["MC_PITCHRATE_D"] = QVariant(0.01f);
    mDefaultParams["MC_PITCHRATE_MAX"] = QVariant(100.0f);
    mDefaultParams["MC_ROLLRATE_P"] = QVariant(0.19f);
    mDefaultParams["MC_ROLLRATE_I"] = QVariant(0.1f);
    mDefaultParams["MC_ROLLRATE_D"] = QVariant(0.01f);
    mDefaultParams["MC_ROLLRATE_MAX"] = QVariant(100.0f);
    mDefaultParams["MC_YAWRATE_P"] = QVariant(0.3f);
    mDefaultParams["MC_YAWRATE_I"] = QVariant(0.1f);
    mDefaultParams["MC_YAWRATE_D"] = QVariant(0.01f);
    mDefaultParams["MC_YAWRATE_MAX"] = QVariant(70.0f);
    mDefaultParams["MC_YAW_P"] = QVariant(3.0f);
    mDefaultParams["MPC_JERK_MAX"] = QVariant(8.0f);
    mDefaultParams["MPC_JERK_AUTO"] = QVariant(8.0f);
    mDefaultParams["MPC_LAND_ALT1"] = QVariant(2.5f);
    mDefaultParams["MPC_LAND_ALT2"] = QVariant(0.5f);
    mDefaultParams["MPC_LAND_VEL_XY"] = QVariant(10.0f);
    mDefaultParams["MPC_LAND_SPEED"] = QVariant(0.4f);
    mDefaultParams["MPC_POS_MODE"] = QVariant(1);
    mDefaultParams["MPC_TILTMAX_AIR"] = QVariant(35.0f);
    mDefaultParams["MPC_XY_MAN_EXPO"] = QVariant(0.0f);
    mDefaultParams["MPC_XY_P"] = QVariant(0.95f);
    mDefaultParams["MPC_XY_VEL_P_ACC"] = QVariant(2.0f);
    mDefaultParams["MPC_XY_VEL_I_ACC"] = QVariant(0.1f);
    mDefaultParams["MPC_XY_VEL_D_ACC"] = QVariant(0.3f);
    mDefaultParams["MPC_Z_P"] = QVariant(1.0f);
    mDefaultParams["MPC_Z_MAN_EXPO"] = QVariant(0.0f);
    mDefaultParams["MPC_Z_VEL_P_ACC"] = QVariant(4.0f);
    mDefaultParams["MPC_Z_VEL_I_ACC"] = QVariant(1.0f);
    mDefaultParams["MPC_Z_VEL_D_ACC"] = QVariant(0.3f);
    mDefaultParams["MPC_Z_VEL_MAX_DN"] = QVariant(2.0f);
    mDefaultParams["MPC_Z_VEL_MAXUP"] = QVariant(3.0f);
    mDefaultParams["MPC_VEL_MANUAL"] = QVariant(5.0f);
    mDefaultParams["MPC_XY_VEL_MAX"] = QVariant(5.0f);
    mDefaultParams["MPC_JERK_MAX"] = QVariant(8.0f);
    mDefaultParams["MPC_ACC_HOR"] = QVariant(3.0f);
    mDefaultParams["MPC_ACC_HOR_MAX"] = QVariant(5.0f);
    mDefaultParams["MPC_Y_MAX"] = QVariant(70.0f);
    mDefaultParams["MPC_YAW_EXPO"] = QVariant(0);
    mDefaultParams["NAV_ACC_RAD"] = QVariant(2.0f);
    mDefaultParams["PWM_MAX"] = QVariant(1950);
    mDefaultParams["PWM_MIN"] = QVariant(1050);
    mDefaultParams["RTL_DESCEND_ALT"] = QVariant(10.0f);
    mDefaultParams["RTL_LAND_DELAY"] = QVariant(0.0f);
    mDefaultParams["RTL_RETURN_ALT"] = QVariant(30.0f);
    mDefaultParams["SDLOG_PROFILE"] = QVariant(1);
    mDefaultParams["SENS_EN_THERMAL"] = QVariant(0);
    mDefaultParams["SER_TEL2_BAUD"] = QVariant(57600);
    mDefaultParams["SYS_MC_EST_GROUP"] = QVariant(2);
    mDefaultParams["MAV_0_MODE"] = QVariant(7);
}


void CCModelAgent::initDefaultGainValue()
{
    // TODO : check necessary properties (info)
}

void CCModelAgent::checkParams(QString name)
{
    if ( name == "" || name == "0" ) {
		// request all parameters
        QThread* thread = new QThread;
        ParamcheckWorker* worker = new ParamcheckWorker();
        worker->moveToThread(thread);
        worker->mSenderCModel = mSender;
        worker->aName = QString("");
        worker->mDefaultParams = mDefaultParams;
        connect(thread, SIGNAL(started()), worker, SLOT (process()));
        thread->start();
	}
	else {
        QThread* thread = new QThread;
        ParamcheckWorker* worker = new ParamcheckWorker();
        worker->moveToThread(thread);
        worker->mSenderCModel = mSender;
        worker->aName = name;
        connect(thread, SIGNAL(started()), worker, SLOT (process()));
        thread->start();
	}

}

bool CCModelAgent::checkBit(uint32_t aValue, uint aBit)
{
    return (aValue & (1<<aBit)) > 0;
}

QString CCModelAgent::readyToFly()
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

QString CCModelAgent::readyToFlyMonitoring()
{
    uint32_t status = mData->data("MONITORING_STATUS1").toUInt();

//    if ( checkBit(status, CMavLinkData::SAFETY_LOCK_STATUS) ) {
//        return "ERROR: Locking";
//    }

//    if ( checkBit(status, CMavLinkData::TEMPERATURE_PROBLEM) ) {
//        return "ERROR: Temperature Problem";
//    }

//    qDebug() << "MSG_INTERVAL : " << mData->data("MSG_INTERVAL_TIME").toLongLong();
    if ( mData->data("MSG_INTERVAL_TIME").toLongLong() > 10000 ) {
        return "ERROR: unstable comm(drone->gcs)";
    }

//    if ( checkBit(status, CMavLinkData::INIT_PITCH_PROBLEM) ) {
//        return "ERROR: unstable PITCH";
//    }

//    if ( checkBit(status, CMavLinkData::INIT_ROLL_PROBLEM) ) {
//        return "ERROR: unstable ROLL";
//    }

//    if ( checkBit(status, CMavLinkData::INIT_VELX_PROBLEM) ) {
//        return "ERROR: unstable Velocity X";
//    }

//    if ( checkBit(status, CMavLinkData::INIT_VELY_PROBLEM) ) {
//        return "ERROR: unstable Velocity Y";
//    }

//    if ( checkBit(status, CMavLinkData::INIT_VELZ_PROBLEM) ) {
//        return "ERROR: unstable Velocity Z";
//    }

//	if ( checkBit(status, CMavLinkData::AGE_CORR_LV1_PROBLEM) ) {
//		return "ERROR: AGE_CORR_LV1_PROBLEM";
//	}

//	if ( checkBit(status, CMavLinkData::AGE_CORR_LV2_PROBLEM) ) {
//		return "ERROR: AGE_CORR_LV2_PROBLEM";
//	}

//    if ( checkBit(status, CMavLinkData::PERI_5V_POWER_PROBLEM) ) {
//        return "ERROR: unstable Periperal Power";
//    }

//    if ( !checkBit(status, CMavLinkData::RTKGPS_FIXED_MODE) ) {
//        return "ERROR: RTKGPS is not ready";
//    }

//	if ( checkBit(status, CMavLinkData::INAV_BIAS_XY_PROBLEM) ) {
//		return "ERROR: INAV_BIAS_XY_PROBLEM";
//	}

//	if ( checkBit(status, CMavLinkData::INAV_BIAS_Z_PROBLEM) ) {
//		return "ERROR: INAV_BIAS_Z_PROBLEM";
//	}

//	if ( checkBit(status, CMavLinkData::ACC_INCONSISTENT_PROBLEM) ) {
//		return "ERROR: ACC_INCONSISTENT_PROBLEM";
//	}

//	if ( checkBit(status, CMavLinkData::GYR_INCONSISTENT_PROBLEM) ) {
//		return "ERROR: GYR_INCONSISTENT_PROBLEM";
//	}

//	if ( checkBit(status, CMavLinkData::MAG_INCONSISTENT_PROBLEM) ) {
//		return "ERROR: MAG_INCONSISTENT_PROBLEM";
//	}

//	if ( checkBit(status, CMavLinkData::INIT_EMBEDDED_SC_OFFSET) ) {
//        return "ERROR: not ready to embedded sc offset ";
//    }

//    if ( !checkBit(status, CMavLinkData::INIT_EMBEDDED_SC_FILE) ) {
//        return "ERROR: not ready to embedded sc file";
//    }

//    if ( !checkBit(status, CMavLinkData::INIT_EMBEDDED_SC_START_TIME) ) {
//        return "OK(ON)";
//    }

    return QString("OK (Last : %1 sec)").arg((double)mData->data("MSG_INTERVAL_TIME").toLongLong() / 1000);
}

QString CCModelAgent::warningFlight()
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

QString CCModelAgent::emergencyFlight()
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

QString CCModelAgent::paramStatus()
{
	QMapIterator<QString, QVariant> i(mDefaultParams);
	while (i.hasNext()) {
		i.next();
//        qDebug() << i.key() << ": " << i.value();
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

QMap<QString, QVariant> CCModelAgent::realParams()
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


QVector3D CCModelAgent::LLH2NED(QGeoCoordinate pos)
{
    // Calc x,y,z of pos with refPos
    double NED_X = REF_POS.distanceTo(QGeoCoordinate(pos.latitude(), REF_POS.longitude(), REF_POS.altitude()));
    if (pos.latitude() < REF_POS.latitude())
        NED_X = -NED_X;
    double NED_Y = REF_POS.distanceTo(QGeoCoordinate(REF_POS.latitude(), pos.longitude(), REF_POS.altitude()));
    if (pos.longitude() < REF_POS.longitude())
        NED_Y = -NED_Y;
    double NED_Z = -(pos.altitude() - REF_POS.altitude());
    return QVector3D(NED_X, NED_Y, NED_Z);
}

QGeoCoordinate CCModelAgent::NED2LLH(QVector3D pos)
{
    // Calc lat, lon, alt of pos with refPos
    QGeoCoordinate LLHPosition = QGeoCoordinate(REF_POS.latitude(), REF_POS.longitude(), REF_POS.altitude());
    LLHPosition = LLHPosition.atDistanceAndAzimuth(pos.x(), 0, -pos.z());
    LLHPosition = LLHPosition.atDistanceAndAzimuth(pos.y(), 90);
    return LLHPosition;
}

