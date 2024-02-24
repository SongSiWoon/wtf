#include "mavlinkdata.h"
#include "agent.h"

#include <QDataStream>
#include <QMutexLocker>
#include <QMetaEnum>
#include <qmath.h>
#include <QDateTime>

union px4_custom_mode {
    struct {
        uint16_t reserved;
        uint8_t main_mode;
        uint8_t sub_mode;
    };
    uint32_t data;
    float data_float;
};

enum PX4_CUSTOM_MAIN_MODE {
    PX4_CUSTOM_MAIN_MODE_MANUAL = 1,
    PX4_CUSTOM_MAIN_MODE_ALTCTL,
    PX4_CUSTOM_MAIN_MODE_POSCTL,
    PX4_CUSTOM_MAIN_MODE_AUTO,
    PX4_CUSTOM_MAIN_MODE_ACRO,
    PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    PX4_CUSTOM_MAIN_MODE_STABILIZED
};

//enum PX4_STATE {
//    MAV_STATE_UNINIT = 0,
//    MAV_STATE_BOOT,
//    MAV_STATE_CALIBRATING,
//    MAV_STATE_STAN DBY,
//    MAV_STATE_ACTIVE,
//    MAV_STATE_CRITICAL,
//    MAV_STATE_EMERGENCY,
//    MAV_STATE_POWEROFF
//};

#define RAD2DEG		(57.0)

CMavLinkData::CMavLinkData(IAgent* agent, QObject* parent)
    : QObject(parent), mAgent(agent)
{
    mSeq = 0;
    mRecvTime = 0;
    mRecvTime_LocalPos = 0;
    mRecvTime_Monitoring = 0;
    mStrModeList << "" << "MANUAL" << "ALTCTL" << "POSCTL" << "AUTO" << "ACRO" << "OFFBOARD" << "STABILIZED";
    mStrStateList << "UNINIT" << "BOOT" << "CALIB" << "STANDBY" << "ARM" << "CRITICAL" << "EMERGENCY" << "POWEROFF";

    memset(&mHeart, 0, sizeof(mHeart));
    memset(&mHud, 0, sizeof(mHud));
    memset(&mStatus, 0, sizeof(mStatus));
    memset(&mLocalPos, 0, sizeof(mLocalPos));
    memset(&mDist, 0, sizeof(mDist));
    memset(&mTargetLocal, 0, sizeof(mTargetLocal));
    memset(&mGPSRaw, 0, sizeof(mGPSRaw));
    memset(&mViconEst, 0, sizeof(mViconEst));
	memset(&mStatusText, 0, sizeof(mStatusText));
	memset(&mAttitude, 0, sizeof(mAttitude));
	memset(&mRTK, 0, sizeof(mRTK));
	memset(&mAck, 0, sizeof(mAck));
    memset(&mMonitoring, 0, sizeof(mMonitoring));
}

CMavLinkData::~CMavLinkData()
{
}


void CMavLinkData::show()
{
    QMutexLocker locker(&mMutex);

}

QString CMavLinkData::log() const
{
    return QString("");
}


QVariant CMavLinkData::data(const QString &aItem)
{
    QString item = aItem.toUpper().trimmed();

    if  ( item == "LOCALPOS" ) {
        if ( mRecvTime_LocalPos > mRecvTime_Monitoring ) {
            return QString("%1,%2,%3")
                    .arg((double)mLocalPos.x,6,'f',2)
                    .arg((double)mLocalPos.y,6,'f',2)
                    .arg((double)mLocalPos.z,6,'f',2);
        }
        else {
            return QString("%1,%2,%3")
                    .arg((double)mMonitoring.pos_x,6,'f',2)
                    .arg((double)mMonitoring.pos_y,6,'f',2)
                    .arg((double)-mMonitoring.pos_z,6,'f',2);
        }
    }
    else if ( item == "LOCALVEL" ) {
        return QString("%1,%2,%3")
                .arg((double)mLocalPos.vx,6,'f',2)
                .arg((double)mLocalPos.vy,6,'f',2)
                .arg((double)mLocalPos.vz,6,'f',2);
    }
	else if ( item == "VEL_X") {
		return mLocalPos.vx;
	}
	else if ( item == "VEL_Y") {
		return mLocalPos.vy;
	}
	else if ( item == "VEL_Z") {
		return mLocalPos.vz;
	}
    else if ( item == "STATE" ) {
        px4_custom_mode mode;
        mode.data = mHeart.custom_mode;
        if ( mode.main_mode < mStrModeList.size() ) {
            return mStrStateList[mHeart.system_status];
        }
        else {
            return QString("");
        }
    }
    else if ( item == "MODE") {
        px4_custom_mode mode;        
        mode.data = mHeart.custom_mode;
        if ( mode.main_mode < mStrModeList.size() ) {
            return mStrStateList[mHeart.system_status] + ":" + mStrModeList[mode.main_mode];
        }
        else {
            return QString("");
        }

    }
    else if ( item == "BATTERY") {
        //return mStatus.battery_remaining;
        return mMonitoring.battery;
    }
    else if ( item == "DISTANCE") {
        return mDist.current_distance;
    }
    else if ( item == "GPS_STATUS") {
        return QString("EPH:%1,EPV:%2")
                .arg(mGPSRaw.eph)
                .arg(mGPSRaw.epv);
    }
    else if ( item == "POSX" ) {
        if ( mRecvTime_LocalPos > mRecvTime_Monitoring ) {
            return mLocalPos.x;
        }
        else {
            return mMonitoring.pos_x;
        }
    }
    else if ( item == "POSY" ) {
        if ( mRecvTime_LocalPos > mRecvTime_Monitoring ) {
            return mLocalPos.y;
        }
        else {
            return mMonitoring.pos_y;
        }
    }
    else if ( item == "POSZ" ) {
        if ( mRecvTime_LocalPos > mRecvTime_Monitoring ) {
            return mLocalPos.z;
        }
        else {
            return mMonitoring.pos_z;
        }
    }
    else if ( item == "POSVX" ) {
        return mLocalPos.vx;
    }
    else if ( item == "POSVY" ) {
        return mLocalPos.vy;
    }
    else if ( item == "POSVZ" ) {
        return mLocalPos.vz;
    }
    else if ( item == "LPSP" ) {
		return QString("X:%1, Y:%2, Z:%3")
				.arg(mTargetLocal.x,6,'f',2)
				.arg(mTargetLocal.y,6,'f',2)
				.arg(mTargetLocal.z,6,'f',2);
	}
	else if ( item == "LPSP_X") {
		return mTargetLocal.x;
	}
	else if ( item == "LPSP_Y") {
		return mTargetLocal.y;
	}
	else if ( item == "LPSP_Z") {
		return mTargetLocal.z;
	}
    else if ( item == "HEADING") {
        return mHud.heading;
    }
	else if ( item == "STATUSTEXT") {
		return mStatusText.text;
	}
	else if ( item == "ATTITUDE") {
		return QString("R:%1,P:%2,Y:%3")
				.arg(mAttitude.roll*RAD2DEG,6,'f',2)
				.arg(mAttitude.pitch*RAD2DEG,6,'f',2)
				.arg(mAttitude.yaw*RAD2DEG,6,'f',2);
	}
	else if (item == "ROLL" ) {
		return mAttitude.roll*RAD2DEG;
	}
	else if (item == "PITCH" ) {
		return mAttitude.pitch*RAD2DEG;
	}
	else if (item == "YAW" ) {
		return mAttitude.yaw*RAD2DEG;
	}
	else if (item == "RTK_STATUS" ) {
		bool rtk_lv1 = monitoringFlag(mMonitoring.status1, CMavLinkData::AGE_CORR_LV1_PROBLEM);
		bool rtk_lv2 = monitoringFlag(mMonitoring.status1, CMavLinkData::AGE_CORR_LV2_PROBLEM);
//		return QString("IAR:%1, nSAT:%2, nOBS:%3 Flag:%4")
//				.arg(mRTK.iar_num_hypotheses)
//				.arg(mRTK.nsats)
//				.arg(mRTK.accuracy)
//				.arg(mRTK.rtk_health);
		return QString("nBase:%1, nRover:%2 Flag:%3(%4:%5)")
                .arg(mMonitoring.rtk_nbase)
                .arg(mMonitoring.rtk_nrover)
				.arg(this->data("RTK_FIXED").toBool())
				.arg(rtk_lv1)
				.arg(rtk_lv2);

	}	
	else if (item == "RTK" ) {
        return QString("X:%1, Y:%2, Z:%3")
				.arg(mRTK.baseline_a_mm*0.001f,6,'f',2)
				.arg(mRTK.baseline_b_mm*0.001f,6,'f',2)
				.arg(mRTK.baseline_c_mm*0.001f,6,'f',2);
	}
	else if (item == "RTK_TOW") {
        //return (float)mRTK.tow*0.001f;		// unit: second
        return mMonitoring.tow*0.001f;
	}
	else if (item == "RTK_FIXED") {
        //return mRTK.rtk_health;
        return monitoringFlag(mMonitoring.status1, RTKGPS_FIXED_MODE);
	}
    else if (item == "RTK_READY") {
        //return mRTK.rtk_health == 1 ? "YES" : "NO";
        return monitoringFlag(mMonitoring.status1, RTKGPS_FIXED_MODE) == 1 ? "YES" : "NO";
    }
	else if (item == "RTK_N") {
		return (float)mRTK.baseline_a_mm * 0.001f;
	}
	else if (item == "RTK_E") {
		return (float)mRTK.baseline_b_mm * 0.001f;
	}
	else if (item == "RTK_D") {
		return (float)mRTK.baseline_c_mm * 0.001f;
	}
	else if (item == "RTK_IAR") {
		return mRTK.iar_num_hypotheses;
	}
	else if (item == "RTK_BASE_PERIOD") {
		return mRTK.time_last_baseline_ms;
	}
	else if ( item == "RTK_MAX_BASE_PERIOD") {
		return mRTK.rtk_rate;
	}
    else if ( item == "MONITORING_STATUS1") {
        return mMonitoring.status1;
    }
    else if ( item == "MONITORING_STATUS1_HEX") {
        return QString("0x%1").arg(mMonitoring.status1, 8, 16, QLatin1Char( '0' ));
    }
    else if ( item == "MONITORING_STR") {
        return strMonitoringStatus();
    }
    else if ( item == "MSG_INTERVAL_TIME") {
        qint64 t = QDateTime::currentMSecsSinceEpoch();
        return t - mRecvTime;
    }
    else {
        return QString("--");
    }

}

bool CMavLinkData::monitoringFlag(uint32_t aValue, uint aBit)
{
    return (aValue & (1<<aBit)) > 0;
}

QString CMavLinkData::strMonitoringEnum(CMavLinkData::MonitoringFlagType aType)
{
    const QMetaObject metaObject = CMavLinkData::staticMetaObject;
    int enumIndex = metaObject.indexOfEnumerator("MonitoringFlagType");
    if(enumIndex == -1) {
        /* The enum does not contain the specified enum */
        return "";
    }
    QMetaEnum en = metaObject.enumerator(enumIndex);
    return QString(en.valueToKey(aType));
}

QString CMavLinkData::strMonitoringStatus()
{
    QString strStatus = "";

    strStatus = QString("===ID:%1===\n").arg(mAgent->id());
    for ( int i = 0 ; i < CMavLinkData::NUM_OF_MONITORING ; i++ ) {
        CMavLinkData::MonitoringFlagType flag = (CMavLinkData::MonitoringFlagType)i;
        QString name = strMonitoringEnum(flag);
        bool value = monitoringFlag(mMonitoring.status1, flag);
        QString item = QString("%1:%2\n").arg(name).arg(value);
        strStatus += item;
    }

    return strStatus;
}

bool CMavLinkData::updateData(const QByteArray &aByteArray)
{
    bool result = false;
    mavlink_message_t message;
    mavlink_status_t status;
    uint8_t msgReceived = false;

    for ( int i = 0 ; i < aByteArray.size() ; i++ ) {
        uint8_t cp = aByteArray.at(i);
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
        // TODO: error check

        if ( msgReceived ) {            
            result = updateMsg(message);
        }
    }

    return result;
}

bool CMavLinkData::updateMsg(const mavlink_message_t& aMsg)
{
    // check recved time
    mRecvTime = QDateTime::currentMSecsSinceEpoch();

    // parse message
    switch (aMsg.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:                
        mavlink_msg_heartbeat_decode(&aMsg, &mHeart);
        mState = mHeart.custom_mode;		
        break;
    case MAVLINK_MSG_ID_VFR_HUD:
        mavlink_msg_vfr_hud_decode(&aMsg, &mHud);
        break;
    case MAVLINK_MSG_ID_HIGHRES_IMU:
        mavlink_highres_imu_t imu;
        mavlink_msg_highres_imu_decode(&aMsg, &imu);
        break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        mRecvTime_LocalPos = mRecvTime;
        mavlink_msg_local_position_ned_decode(&aMsg, &mLocalPos);
        break;
    case MAVLINK_MSG_ID_GPS_RAW_INT:
        mavlink_msg_gps_raw_int_decode(&aMsg, &mGPSRaw);
        break;
    case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
        mavlink_msg_vicon_position_estimate_decode(&aMsg, &mViconEst);
        break;
    case MAVLINK_MSG_ID_SYS_STATUS:  
        mavlink_msg_sys_status_decode(&aMsg, &mStatus);
        break;
    case MAVLINK_MSG_ID_MISSION_REQUEST:
        mavlink_mission_request_t  request;
        mavlink_msg_mission_request_decode(&aMsg, &request);
        break;
    case MAVLINK_MSG_ID_DISTANCE_SENSOR:        
        mavlink_msg_distance_sensor_decode(&aMsg, &mDist);
        break;
    case MAVLINK_MSG_ID_MISSION_ACK:
        break;		
    case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
		mavlink_msg_position_target_local_ned_decode(&aMsg, &mTargetLocal);
        break;
    case MAVLINK_MSG_ID_STATUSTEXT:
		mavlink_msg_statustext_decode(&aMsg, &mStatusText);		
		break;
	case MAVLINK_MSG_ID_PARAM_VALUE:
		update_param_value(&aMsg);
		break;
	case MAVLINK_MSG_ID_ATTITUDE:
		mavlink_msg_attitude_decode(&aMsg, &mAttitude);
		break;
	case MAVLINK_MSG_ID_COMMAND_ACK:
		mavlink_msg_command_ack_decode(&aMsg, &mAck);
		break;
	case MAVLINK_MSG_ID_GPS_RTK:
        mavlink_msg_gps_rtk_decode(&aMsg, &mRTK);
		break;
    case MAVLINK_MSG_ID_MONITORING:
        mRecvTime_Monitoring  = mRecvTime;
        mavlink_msg_monitoring_decode(&aMsg, &mMonitoring);
        break;
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
//        qDebug("RECEIVE FTP");
        mFTP->receiveMessage(aMsg);
        break;
    default:
        //qDebug("Unknown ID (%d)", aMsg.msgid);
        break;

    }

	return true;
}

QVariant CMavLinkData::param(const QString aName)
{
	return mParams[aName];
}

void CMavLinkData::resetParams()
{
	mParams.clear();
}

void CMavLinkData::resetAck()
{
	memset(&mAck, 0, sizeof(mAck));
}

int CMavLinkData::checkAck(const uint16_t aCmd)
{
	if ( mAck.command == aCmd ) {
		return mAck.result;
	}
	else {
		return -1;
	}
}

uint CMavLinkData::toUInt(const QByteArray &aBuffer)
{
    uint result = 0;
    memcpy(&result, aBuffer.data(), 4);
    return result;
}

ushort CMavLinkData::toUShort(const QByteArray &aBuffer)
{
    ushort result = 0;
    memcpy(&result, aBuffer.data(), 2);
	return result;
}

void CMavLinkData::update_param_value(const mavlink_message_t* aMsg)
{
	mavlink_param_value_t paramValue;
	QVariant value;
    qint8   i8_value = 0;
    quint8  u8_value = 0;
    qint16  i16_value = 0;
    quint16 u16_value = 0;
    qint32  i32_value = 0;
    quint32 u32_value = 0;

	mavlink_msg_param_value_decode(aMsg, &paramValue);

	switch (paramValue.param_type ) {
	case MAV_PARAM_TYPE_UINT8:        
        memcpy(&u8_value, &paramValue.param_value, sizeof(u8_value));
        value = QVariant(u8_value);
		break;
	case MAV_PARAM_TYPE_INT8:
        memcpy(&i8_value, &paramValue.param_value, sizeof(i8_value));
        value = QVariant(i8_value);
		break;
    case MAV_PARAM_TYPE_UINT16:
        memcpy(&u16_value, &paramValue.param_value, sizeof(u16_value));
        value = QVariant(u16_value);
		break;
	case MAV_PARAM_TYPE_INT16:
        memcpy(&i16_value, &paramValue.param_value, sizeof(i16_value));
        value = QVariant(i16_value);
		break;
    case MAV_PARAM_TYPE_UINT32:
        memcpy(&u32_value, &paramValue.param_value, sizeof(u32_value));
        value = QVariant(u32_value);
		break;
	case MAV_PARAM_TYPE_INT32:
        memcpy(&i32_value, &paramValue.param_value, sizeof(i32_value));
        value = QVariant(i32_value);
		break;
	case MAV_PARAM_TYPE_INT64:        
        value = QVariant((quint64)paramValue.param_value);
		break;
	case MAV_PARAM_TYPE_REAL32:        
        value = QVariant((float)paramValue.param_value);
		break;
	case MAV_PARAM_TYPE_REAL64:                
        value = QVariant((double)paramValue.param_value);
		break;

	default:
		break;
	}

    char name[17] = {};
    memcpy(name, paramValue.param_id, 16);
    mParams[name] = value;
}


