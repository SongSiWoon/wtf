#include "rosdata.h"
#include "agent.h"

#include <QDataStream>
#include <QMutexLocker>
#include <QMetaEnum>
#include <qmath.h>
#include <QDateTime>
#include <QVector3D>

using std::placeholders::_1;

#define RAD2DEG		(57.0)
#define DEG2RAD     (0.0174533)

CROSData::CROSData(IAgent* agent, QObject* parent)
    : QObject(parent), mAgent(agent)
{
    mSeq = 0;
    mRecvTime = 0;
    mRecvTime_LocalPos = 0;
    mRecvTime_Monitoring = 0;
    mStrNavStateList << "MANUAL(0)" << "ALTCTL(1)" << "POSCTL(2)" << "AUTO_MISSION(3)" << "AUTO_LOITER(4)" << "AUTO_RTL(5)" << "AUTO_RCRECOVER(6)" << "AUTO_RTGS" << "AUTO_LANDENGFAIL" << "AUTO_LANDGPSFAIL"
                     << "ACRO(10)" << "UNUSED" << "DESCEND" << "TERMINATION" << "OFFBOARD(14)" << "STAB(15)" << "RATTITUDE" << "AUTO_TAKEOFF(17)" << "AUTO_LAND" << "AUTO_FOLLOW_TARGET" << "AUTO_PRECLAND"
                     << "ORBIT" << "MAX";
    mStrArmingStateList << "" << "INIT" << "STANDBY" << "ARMED" << "STANDBY_ERROR" << "SHUTDOWN" << "IN_AIR_RESTORE" << "MAX";

	memset(&mRTK, 0, sizeof(mRTK));
	memset(&mAck, 0, sizeof(mAck));
    // memset(&mMonitoring, 0, sizeof(mMonitoring));
    
    mTargetX = 0;
    mTargetY = 0;
    mTargetZ = 0;

    initSubscription();
}

CROSData::~CROSData()
{
}

void CROSData::show()
{
    QMutexLocker locker(&mMutex);

}

QString CROSData::log() const
{
    return QString("");
}


QVariant CROSData::data(const QString &aItem)
{
    QString item = aItem.toUpper().trimmed();

    if  ( item == "LOCALPOS" ) {
        return QString("%1,%2,%3")
             .arg((double)mMonitoringRos.pos_x,6,'f',2)
             .arg((double)mMonitoringRos.pos_y,6,'f',2)
             .arg((double)-mMonitoringRos.pos_z,6,'f',2);
    }
    else if ( item == "LOCALVEL" ) {
        return "";
    }
	else if ( item == "VEL_X") {
        return 0;
	}
	else if ( item == "VEL_Y") {
        return 0;
	}
	else if ( item == "VEL_Z") {
        return 0;
		// return mLocalPosVelRos.twist.linear.z;
	}
    else if ( item == "STATE" ) {
        return "";
    }
    else if ( item == "MODE") {
        QString mode = "UNKNOWN";
        if(monitoringFlag(mMonitoringRos.status1, MANUAL_MODE))
            mode = "MANUAL";
        else if(monitoringFlag(mMonitoringRos.status1, OFFBOARD_MODE))
            mode = "OFFBOARD";
        else if(monitoringFlag(mMonitoringRos.status1, AUTO_MODE))
            mode = "AUTO";

        return QString("%1, ARM:%2, GST:%3").arg(mStrNavStateList[mStatusRos.nav_state]).arg(monitoringFlag(mMonitoringRos.status1, ARM_STATUS) == true).arg(mGstRunning);
    }
    else if ( item == "ISARMED") {
        return monitoringFlag(mMonitoringRos.status1, ARM_STATUS);
    }
    else if ( item == "IS_GST_RUNNING") {
        return mGstRunning;
    }
    else if ( item == "BATTERY") {
        return mMonitoringRos.battery;
    }
    else if ( item == "DISTANCE") {
        return 0;
    }
    else if ( item == "GPS_STATUS") {
        return "";
    }
    else if ( item == "POSX" ) {
        return (double)mMonitoringRos.pos_x;
    }
    else if ( item == "POSY" ) {
        return (double)mMonitoringRos.pos_y;
    }
    else if ( item == "POSZ" ) {
        return (double)mMonitoringRos.pos_z;
    }
    else if ( item == "POS" ) {
        return QVector3D((double)mMonitoringRos.pos_x,
                         (double)mMonitoringRos.pos_y,
                        -(double)mMonitoringRos.pos_z);
    }
    else if ( item == "POSVX" ) {
        return 0;
    }
    else if ( item == "POSVY" ) {
        return 0;
    }
    else if ( item == "POSVZ" ) {
        return 0;
    }
    else if ( item == "LPSP" ) {
		return QString("X:%1, Y:%2, Z:%3")
				.arg(mTargetX,6,'f',2)
				.arg(mTargetY,6,'f',2)
				.arg(mTargetZ,6,'f',2);
	}
	else if ( item == "LPSP_X") {
		return mTargetX;
	}
	else if ( item == "LPSP_Y") {
		return mTargetY;
	}
    else if ( item == "LPSP_Z") {
		return mTargetZ;
	}
    else if ( item == "HEADING") {
        return mMonitoringRos.head*RAD2DEG;
    }
	else if ( item == "STATUSTEXT") {
        if (!mLogMessageQueue.isEmpty()){
            auto tmpLogMessage = mLogMessageQueue.dequeue();
            std::string statustext(tmpLogMessage.text.begin(), tmpLogMessage.text.end());
            return statustext.c_str();
        }else{
            return "";
        }
	}
	else if ( item == "ATTITUDE") {
        return "";
	}
	else if (item == "ROLL" ) {
        return "";
	}
	else if (item == "PITCH" ) {
        return "";
	}
	else if (item == "YAW" ) {
        return "";
	}
	else if (item == "RTK_STATUS" ) {
         bool rtk_lv1 = monitoringFlag(mMonitoringRos.status1, CROSData::AGE_CORR_LV1_PROBLEM);
         bool rtk_lv2 = monitoringFlag(mMonitoringRos.status1, CROSData::AGE_CORR_LV2_PROBLEM);
         return QString("nBase:%1, nRover:%2 Flag:%3(%4:%5)")
                .arg(mMonitoringRos.rtk_nbase)
                .arg(mMonitoringRos.rtk_nrover)
                .arg(this->data("RTK_FIXED").toBool())
                .arg(rtk_lv1)
                .arg(rtk_lv2);

	}	
	else if (item == "RTK" ) {
        return QString("");
	}
	else if (item == "RTK_TOW") {
         return mMonitoringRos.utc_usec*0.001f;
	}
    else if (item == "GLOBAL_LAT") {
        return mMonitoringRos.lat;
    }
    else if (item == "GLOBAL_LON") {
        return mMonitoringRos.lon;
    }
    else if (item == "GLOBAL_ALT") {
        return mMonitoringRos.alt;
    }
    else if (item == "LLH" ) {
        return QVector3D((double)mMonitoringRos.lat, (double)mMonitoringRos.lon, (double)mMonitoringRos.alt);
    }
	else if (item == "RTK_FIXED") {        
         return monitoringFlag(mMonitoringRos.status1, RTKGPS_FIXED_MODE);
	}
    else if (item == "RTK_READY") {       
         return monitoringFlag(mMonitoringRos.status1, RTKGPS_FIXED_MODE) == 1 ? "YES" : "NO";
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
         return mMonitoringRos.status1;
    }
    else if ( item == "MONITORING_STATUS1_HEX") {
         return QString("0x%1").arg(mMonitoringRos.status1, 8, 16, QLatin1Char( '0' ));
    }
    else if ( item == "MONITORING_STR") {
        return strMonitoringStatus();
    }
    else if ( item == "MSG_INTERVAL_TIME") {
        qint64 t = QDateTime::currentMSecsSinceEpoch();
        if ((t - mRecvTime_Monitoring) > 10000 ) {
            mCommFlag = false;
        }
        return t - mRecvTime_Monitoring;
    }
    else if ( item == "AGENT_BASE_ALT_DIFF") {
        return agentBaseAltDiff;
    }
    else if ( item == "PX4_OSMO_BATT") {
        return QString("%1/%2").arg(mMonitoringRos.battery).arg(mOsmo.battery);
    }
    else if ( item == "OSMO_BATT") {
        return mOsmo.battery;
    }
    else {
        return QString("--");
    }

}

bool CROSData::monitoringFlag(uint32_t aValue, uint aBit)
{
    return (aValue & (1<<aBit)) > 0;
}

QString CROSData::strMonitoringEnum(CROSData::MonitoringFlagType aType)
{
    const QMetaObject metaObject = CROSData::staticMetaObject;
    int enumIndex = metaObject.indexOfEnumerator("MonitoringFlagType");
    if(enumIndex == -1) {
        /* The enum does not contain the specified enum */
        return "";
    }
    QMetaEnum en = metaObject.enumerator(enumIndex);
    return QString(en.valueToKey(aType));
}

QString CROSData::strMonitoringStatus()
{
    QString strStatus = "";

//    strStatus = QString("===ID:%1===\n").arg(mAgent->data("SYSID").toInt());
//    for ( int i = 0 ; i < CROSData::NUM_OF_MONITORING ; i++ ) {
//        CROSData::MonitoringFlagType flag = (CROSData::MonitoringFlagType)i;
//        QString name = strMonitoringEnum(flag);
//        bool value = 0;
//        // bool value = monitoringFlag(mMonitoringRos.status1, flag);
//        QString item = QString("%1:%2\n").arg(name).arg(value);
//        strStatus += item;
//    }

    return strStatus;
}

bool CROSData::updateData(const QByteArray &aByteArray)
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

void CROSData::updateTarget(float x, float y, float z, float h)
{
//    printf("CROSData::updateTarget %lf %lf %lf\n",x,y,z);
    mTargetX = x;
    mTargetY = y;
    mTargetZ = z;
    mTargetH = h;
}

void CROSData::updateTargetGlobal(double lat, double lng, double altitude, double yaw)
{
    mTargetLat = lat;
    mTargetLng = lng;
    mTargetAlt = altitude;
    mTargetYaw = yaw;
}

bool CROSData::updateMsg(const mavlink_message_t& aMsg)
{
	return true;
}

void CROSData::saveParam(const QString aName, QVariant value)
{
    mParams[aName] = value;
}

QVariant CROSData::param(const QString aName)
{
    return mParams[aName];
}

void CROSData::resetParams()
{
	mParams.clear();
}

void CROSData::resetAck()
{
	memset(&mAck, 0, sizeof(mAck));
}

void CROSData::setAckForROS(const uint16_t aCmd, uint8_t result)
{
    mAck.command = aCmd;
    mAck.result = result;
}

int CROSData::checkAck(const uint16_t aCmd)
{
	if ( mAck.command == aCmd ) {
		return mAck.result;
	}
	else {
		return -1;
	}
}

uint CROSData::toUInt(const QByteArray &aBuffer)
{
    uint result = 0;
    memcpy(&result, aBuffer.data(), 4);
    return result;
}

ushort CROSData::toUShort(const QByteArray &aBuffer)
{
    ushort result = 0;
    memcpy(&result, aBuffer.data(), 2);
	return result;
}

void CROSData::update_param_value(const mavlink_message_t* aMsg)
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

void CROSData::updateMonitoring(const px4_msgs::msg::Monitoring::SharedPtr msg)
{
    mRecvTime_Monitoring = QDateTime::currentMSecsSinceEpoch();
    mMonitoringRos.timestamp = msg->timestamp;
    mMonitoringRos.battery = msg->battery;
    mMonitoringRos.pos_x = msg->pos_x + init_pos_y;
    mMonitoringRos.pos_y = msg->pos_y + init_pos_x;
    mMonitoringRos.pos_z = msg->pos_z;
    mMonitoringRos.head = msg->head;
    mMonitoringRos.lat = msg->lat;
    mMonitoringRos.lon = msg->lon;
    mMonitoringRos.alt = msg->alt;
    mMonitoringRos.rtk_nbase = msg->rtk_nbase;
    mMonitoringRos.rtk_nrover = msg->rtk_nrover;
    mMonitoringRos.status1 = msg->status1;
    mMonitoringRos.status2 = msg->status2;

    if (mCommFlag == false) {
        qDebug() << "Connection Get for #" << this->mAgent->data("SYSID") << ". Change Setpoint";
        this->updateTarget(mMonitoringRos.pos_x, mMonitoringRos.pos_y, mMonitoringRos.pos_z, mTargetH);
    }
    mCommFlag = true;
}

void CROSData::updateVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    mStatusRos.timestamp = msg->timestamp;
    mStatusRos.arming_state = msg->arming_state;
    mStatusRos.nav_state = msg->nav_state;
}

void CROSData::updateVehicleCommandAck(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{
    mVehicleCommandAck.command = msg->command;
    mVehicleCommandAck.result = msg->result;
    mVehicleCommandAck.result_param1 = msg->result_param1;
    mVehicleCommandAck.result_param2 = msg->result_param2;
}


void CROSData::updateLogMessage(const px4_msgs::msg::LogMessage::SharedPtr msg)
{
    px4_msgs::msg::LogMessage newLogMessage;
    newLogMessage.timestamp = msg->timestamp;
    newLogMessage.severity = msg->severity;
    newLogMessage.text = msg->text;

    mLogMessageQueue.enqueue(newLogMessage);
}


QList<QString> CROSData::getParamRequested() {
    return param_requested;
}

void CROSData::parameterValueCallback(const px4_msgs::msg::UavcanParameterValue::SharedPtr msg) {
    std::string param_id(msg->param_id.begin(), msg->param_id.end());
    param_requested.removeOne(QString::fromUtf8(param_id.c_str()));
    if(msg->param_type == px4_msgs::msg::UavcanParameterRequest::PARAM_TYPE_REAL32) {
        this->saveParam(QString::fromUtf8(param_id.c_str()), msg->real_value);
    } else {
        this->saveParam(QString::fromUtf8(param_id.c_str()), (int) msg->int_value);
    }
}

void CROSData::osmoCallback(const agent_msg::msg::Osmo2Status::SharedPtr msg)
{
    mOsmo.framerate = msg->framerate;
    mOsmo.sensor = msg->sensor;
    mOsmo.battery = msg->battery;
}

void CROSData::agentManagerStatusCallback(const agent_msg::msg::AgentStatus::SharedPtr msg) {
    mGstRunning = msg->iswork_gstlaunch;
}

void CROSData::sendOsmoCmd(const QString cmd, const QString args) {
    auto agent_osmo_request = std::make_shared<agent_msg::srv::Command::Request>();
    agent_osmo_request->cmd = cmd.toStdString();
    agent_osmo_request->args = args.toStdString();
    agent_manager_result = agent_osmo_client->async_send_request(agent_osmo_request);
}

void CROSData::sendStartStreamingCmd() {
    qDebug() << "sendStartStreamingCmd!!!";
    auto agent_manager_request = std::make_shared<agent_msg::srv::Command::Request>();
    agent_manager_request->cmd = "start_streaming";

    // Hanvision
//    agent_manager_request->args = QString("gst-launch-1.0 v4l2src device=\"/dev/video0\" ! \"video/x-raw, width=1920, height=1080, format=(string)UYVY\" ! tee name=t ! queue leaky=1 ! nvvidconv ! \"video/x-raw(memory:NVMM), format=(string)NV12\" ! videorate max-rate=10 ! nvv4l2h264enc maxperf-enable=1 bitrate=3000000 ! h264parse ! mp4mux ! filesink location=/home/karidrone/Videos/AGENT%1-%2.mp4 -e t. ! queue ! nvvidconv ! \"video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12\" ! nvv4l2h264enc maxperf-enable=1 bitrate=1000000 preset-level=4 profile=4 iframeinterval=180 insert-sps-pps=true ! h264parse ! flvmux ! rtmpsink location=rtmp://live.hanvision.xyz/publish/kdrone%3").arg(std::to_string(mAgent->data("SYSID").toInt()).c_str()).arg(QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss")).arg(std::to_string(mAgent->id()).c_str()).toStdString().c_str();

    // sysid
//    agent_manager_request->args = QString("gst-launch-1.0 v4l2src device=\"/dev/video0\" ! \"video/x-raw, width=1920, height=1080, format=(string)UYVY\" ! nvvidconv ! \"video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12\" ! videorate max-rate=10 ! nvv4l2h264enc maxperf-enable=1 bitrate=1000000 ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=143.248.101.117 port=%1").arg(std::to_string(mAgent->data("SYSID").toInt()+5000).c_str()).toStdString().c_str();

    // sysid, eureka with width, height, bitrate, max_rate.
//    int gst_width = 1920;
//    int gst_height = 1080;
//    int gst_bitrate = 100000;
//    int gst_max_rate = 5;
    // tcp
//    agent_manager_request->args = QString("echo 111").toStdString().c_str();
//    QString time_format = "-yyyyMMdd-HHmmss";
//    QDateTime curDateTime = QDateTime::currentDateTime();
//    agent_manager_request->args = QString("gst-launch-1.0 -v fdsrc ! tee name=t ! queue leaky=1 ! filesink location=/home/karidrone/Videos/AGENT%1.mp4 t. ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw(memory:NVMM), width=(int)%2, height=(int)%3, format=(string)NV12 ! queue ! videorate max-rate=%4 ! nvv4l2h264enc maxperf-enable=1 bitrate=%5 preset-level=4 profile=4 iframeinterval=360 insert-sps-pps=true ! h264parse ! flvmux metadatacreator=$(date+%s%N) ! tcpclientsink host=143.248.69.53 port=%6")
//            .arg(mAgent->data("SYSID").toString() + curDateTime.toString(time_format)).arg(gst_width).arg(gst_height).arg(gst_max_rate).arg(gst_bitrate).arg(std::to_string(mAgent->data("SYSID").toInt()+5000).c_str()).toStdString().c_str();
//    qDebug() << agent_manager_request->args.c_str();

    // udp
//    agent_manager_request->args = QString("gst-launch-1.0 v4l2src device=\"/dev/video0\" ! \"video/x-raw, width=1920, height=1080, format=(string)UYVY\" ! nvvidconv ! \"video/x-raw(memory:NVMM), width=(int)%1, height=(int)%2, format=(string)NV12\" ! videorate max-rate=%3 ! nvv4l2h264enc maxperf-enable=1 bitrate=%4 preset-level=4 profile=4 iframeinterval=360 insert-sps-pps=true ! h264parse ! flvmux metadatacreator=$(date +%s%N) ! udpsink host=143.248.99.77 port=%5").arg(gst_width).arg(gst_height).arg(gst_max_rate).arg(gst_bitrate).arg(std::to_string(mAgent->data("SYSID").toInt()+5000).c_str()).toStdString().c_str();
    agent_manager_result = agent_manager_client->async_send_request(agent_manager_request);
}

void CROSData::sendStopStreamingCmd() {
    auto agent_manager_request = std::make_shared<agent_msg::srv::Command::Request>();
    agent_manager_request->cmd = "kill_process";
    agent_manager_request->args = "gst-launch-1.0";
    agent_manager_result = agent_manager_client->async_send_request(agent_manager_request);
}

void CROSData::initSubscription()
{
    mQHAC3Node = rclcpp::Node::make_shared(("agent_" + std::to_string(mAgent->data("SYSID").toInt()) + "_qhac3_node"));

    /** For iris **/
    rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
    qos.reliable();

    // Subscribers++
    std::string topic_prefix = ros2Header + std::to_string(mAgent->data("SYSID").toInt());
    mVehicleStatusSub_ = mQHAC3Node->create_subscription<px4_msgs::msg::VehicleStatus>(topic_prefix + "/vehicle_status", qos, std::bind(&CROSData::updateVehicleStatus, this, _1));
    mMonitoringSub_ = mQHAC3Node->create_subscription<px4_msgs::msg::Monitoring>(topic_prefix + "/monitoring", qos, std::bind(&CROSData::updateMonitoring, this, _1));
    mVehicleCommandAckSub_ = mQHAC3Node->create_subscription<px4_msgs::msg::VehicleCommandAck>(topic_prefix + "/vehicle_command_ack", qos, std::bind(&CROSData::updateVehicleCommandAck, this, _1));
    mLogMessageSub_ = mQHAC3Node->create_subscription<px4_msgs::msg::LogMessage>(topic_prefix + "/log_message", qos, std::bind(&CROSData::updateLogMessage, this, _1));
    mUavcanParameterValueSub_ = mQHAC3Node->create_subscription<px4_msgs::msg::UavcanParameterValue>(topic_prefix + "/uavcan_parameter_value", qos, std::bind(&CROSData::parameterValueCallback, this, _1));
    mOsmoSub_ = mQHAC3Node->create_subscription<agent_msg::msg::Osmo2Status>(topic_prefix + "/osmo2", qos, std::bind(&CROSData::osmoCallback, this, _1));


    // Publishers
    rclcpp::QoS qos_cmd = rclcpp::SystemDefaultsQoS();
    qos_cmd.reliable();
    mCommandQHACPub_ = mQHAC3Node->create_publisher<px4_msgs::msg::VehicleCommand>(topic_prefix + "/vehicle_command", qos_cmd);
    mUavcanParameterRequestQHACPub_ = mQHAC3Node->create_publisher<px4_msgs::msg::UavcanParameterRequest>(topic_prefix + "/uavcan_parameter_request", rclcpp::SystemDefaultsQoS());

    // Agent Manager
    mGstRunning = false;
    mAgentStatusSub_ = mQHAC3Node->create_subscription<agent_msg::msg::AgentStatus>(topic_prefix + "/agent_manager_status", qos, std::bind(&CROSData::agentManagerStatusCallback, this, _1));
    agent_manager_client = mQHAC3Node->create_client<agent_msg::srv::Command>(topic_prefix + "/agent_manager");
    agent_osmo_client = mQHAC3Node->create_client<agent_msg::srv::Command>(topic_prefix + "/control");

    init_pos_x = mAgent->info("init_pos_x").toDouble();
    init_pos_y = mAgent->info("init_pos_y").toDouble();
    mTargetX = init_pos_y;
    mTargetY = init_pos_x;

    QThread* thread = new QThread;
    SpinWorker* worker = new SpinWorker();
    worker->mNodePtr = mQHAC3Node;
    worker->moveToThread(thread);
    connect(thread, SIGNAL(started()), worker, SLOT (process()));
    thread->start();
}

void CROSData::publishCommand(px4_msgs::msg::VehicleCommand command) {
    mCommandQHACPub_->publish(command);
}

void CROSData::publishRequestParam(px4_msgs::msg::UavcanParameterRequest req) {
    mUavcanParameterRequestQHACPub_->publish(req);
    std::string param_id(req.param_id.begin(), req.param_id.end());
    if (!param_requested.contains(QString::fromUtf8(param_id.c_str())))
        param_requested.append(QString::fromUtf8(param_id.c_str()));
}

void CROSData::setAgentBaseDiffAlt(double alt) {
    this->agentBaseAltDiff = alt;
}

void CROSData::onTimeout()
{
}
