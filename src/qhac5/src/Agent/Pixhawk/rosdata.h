#ifndef CROSDATA_H
#define CROSDATA_H

#include <common/mavlink.h>
#include "filemanager.h"


#include <QObject>
#include <QByteArray>
#include <QMutex>
#include <QStringList>
#include <QMap>
#include <QVariant>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/uavcan_parameter_request.hpp>
#include <px4_msgs/msg/uavcan_parameter_value.hpp>
#include <px4_msgs/msg/monitoring.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/log_message.hpp>
//#include <agent_msg/srv/command.hpp>
//#include <agent_msg/msg/agent_status.hpp>
//#include <agent_msg/msg/osmo2_status.hpp>
#include <spinworker.h>

class IAgent;
class FileManager;
class CBModelAgent;

class CROSData : public QObject
{
    Q_OBJECT

public:
    enum MonitoringFlagType {
        SAFETY_LOCK_STATUS = 0,
        ARM_STATUS,
        OFFBOARD_MODE,
        MANUAL_MODE,

        AUTO_MODE,
        FAIL_SAFE_MODE,
        BATTERY_PROBLEM,
        RTKGPS_CONNECTION,

        RTKGPS_BASE_RECV,
        RTKGPS_FIXED_MODE,
        RTKGPS_OFFSET,
        COMM_PROBLEM,

        INIT_PITCH_PROBLEM,
        INIT_ROLL_PROBLEM,
        INIT_VELX_PROBLEM,
        INIT_VELY_PROBLEM,

        INIT_VELZ_PROBLEM,
        INIT_EMBEDDED_SC_OFFSET,
        INIT_EMBEDDED_SC_FILE,
        INIT_EMBEDDED_SC_START_TIME,

        PERI_5V_POWER_PROBLEM,
        TEMPERATURE_PROBLEM,
        INAV_BIAS_XY_PROBLEM,
        INAV_BIAS_Z_PROBLEM,

        MAG_INCONSISTENT_PROBLEM,
        ACC_INCONSISTENT_PROBLEM,
        GYR_INCONSISTENT_PROBLEM,
        AGE_CORR_LV1_PROBLEM,

        AGE_CORR_LV2_PROBLEM,
        STATUS_LANDED,

        NUM_OF_MONITORING
    };
    Q_ENUM(MonitoringFlagType);

public:
    explicit CROSData(IAgent* agent, QObject* parent = 0);
    virtual ~CROSData();

public:
    void initSubscription();

public Q_SLOTS:
            void onTimeout();

public:
    void setAgent(IAgent* aAgent) {mAgent = aAgent;}
    void setFileManager(FileManager* aFTP) {mFTP = aFTP;}
    bool updateData(const QByteArray& aByteArray);
    bool updateMsg(const mavlink_message_t& aMsg);
    void saveParam(const QString aName, QVariant value);
    QVariant param(const QString aName);
    void resetParams();
//    void setBaseCoordinate(QGeoCoordinate qgeo);

    void resetAck();
    void setAckForROS(const uint16_t aCmd, uint8_t result);
    int checkAck(const uint16_t aCmd);
    bool monitoringFlag(uint32_t aValue, uint aBit);
    QString strMonitoringEnum(MonitoringFlagType aType);
    QString strMonitoringStatus();
    void show();
    QString log() const;

    void setAgentBaseDiffAlt(double alt);

    void updateTarget(float x, float y, float z, float H);
    void updateTargetGlobal(double lat, double lng, double altitude, double yaw);
    void publishCommand(px4_msgs::msg::VehicleCommand command);
    void publishRequestParam(px4_msgs::msg::UavcanParameterRequest req);
    void updateMonitoring(const px4_msgs::msg::Monitoring::SharedPtr msg);
    void updateVehicleGPSPosition(const px4_msgs::msg::SensorGps::SharedPtr msg);
    void updateVehicleLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void updateLogMessage(const px4_msgs::msg::LogMessage::SharedPtr msg);
    void updateVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void updateVehicleCommandAck(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);

    void parameterValueCallback(const px4_msgs::msg::UavcanParameterValue::SharedPtr msg);
//    void osmoCallback(const agent_msg::msg::Osmo2Status::SharedPtr msg);
    QList<QString> getParamRequested();

//    void agentManagerStatusCallback(const agent_msg::msg::AgentStatus::SharedPtr msg);
//    void sendStartStreamingCmd();
//    void sendStopStreamingCmd();
    void sendOsmoCmd(const QString cmd, const QString args);

public:
    virtual int                 battery()     {return 0;}
    virtual float               posX()        {return 0;}
    virtual float               posY()        {return 0;}
    virtual float               posZ()        {return 0;}
    virtual float               heading()     {return 0;}

    QVariant data(const QString& aItem);

private:
    uint toUInt(const QByteArray& aBuffer);
    ushort toUShort(const QByteArray& aBuffer);
    void update_param_value(const mavlink_message_t* aMsg);

private:
    IAgent*                     mAgent;
    FileManager*                mFTP;
    uint                        mSeq;
    QMutex                      mMutex;
    QStringList                 mStrNavStateList;
    QStringList                 mStrArmingStateList;
    QStringList                 mStrStateList;
    qint64                      mRecvTime;
    qint64                      mRecvTime_LocalPos = 0;
    qint64                      mRecvTime_Monitoring = 0;
    bool                        mCommFlag = false;

    double                      mRoll = 0, mPitch = 0, mYaw = 0;
    double                      mTargetX, mTargetY, mTargetZ, mTargetH;
    double                      mTargetLat, mTargetLng, mTargetAlt, mTargetYaw;
    double                      agentBaseAltDiff = 0;

    QMap< QString, QVariant >                   mParams;
    px4_msgs::msg::VehicleStatus                mStatusRos;
    px4_msgs::msg::VehicleCommandAck            mVehicleCommandAck;
    px4_msgs::msg::VehicleLocalPosition         mVehicleLocalPosition;
    px4_msgs::msg::SensorGps                    mVehicleGPSPosition;
    px4_msgs::msg::Monitoring                   mMonitoringRos;
//    agent_msg::msg::Osmo2Status                 mOsmo;
    QQueue<px4_msgs::msg::LogMessage>           mLogMessageQueue;
//    QGeoCoordinate                              base_coordinate = QGeoCoordinate(0,0,0);
    bool    mGstRunning;
//    px4_msgs::msg::LogMessage                   mLogMessage;
    // mavlink_local_position_ned_t        mLocalPos;
    // mavlink_distance_sensor_t           mDist;
    // mavlink_gps_raw_int_t               mGPSRaw;
    // mavlink_vicon_position_estimate_t   mViconEst;
    // mavlink_position_target_local_ned_t mTargetLocal;
    // mavlink_statustext_t				mStatusText;
    // mavlink_attitude_t					mAttitude;
    mavlink_gps_rtk_t					mRTK;
    mavlink_command_ack_t				mAck;
    // mavlink_monitoring_t                mMonitoring;

    QList<QString>                      param_requested;

    rclcpp::Node::SharedPtr             mQHAC3Node;
//    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr mSensorCombinedSub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr mVehicleStatusSub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr mVehicleCommandAckSub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr mVehicleLocalPositionSub_;
    rclcpp::Subscription<px4_msgs::msg::Monitoring>::SharedPtr mMonitoringSub_;
    rclcpp::Subscription<px4_msgs::msg::LogMessage>::SharedPtr mLogMessageSub_;
    rclcpp::Subscription<px4_msgs::msg::UavcanParameterValue>::SharedPtr mUavcanParameterValueSub_;
    rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr mVehicleGPSPositionSub_;
//    rclcpp::Subscription<agent_msg::msg::Osmo2Status>::SharedPtr mOsmoSub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr mCommandQHACPub_;
    rclcpp::Publisher<px4_msgs::msg::UavcanParameterRequest>::SharedPtr mUavcanParameterRequestQHACPub_;
//    rclcpp::Publisher<px4_msgs::msg::OffboardSetpoint>::SharedPtr mOffboardSetpointQHACPub_;
//    rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr mVehicleLocalPositionSetpointQHACPub_;
//    rclcpp::Publisher<px4_msgs::msg::TelemetryStatus>::SharedPtr mTelemetryStatusPub_;

//    rclcpp::Subscription<agent_msg::msg::AgentStatus>::SharedPtr mAgentStatusSub_;
//    rclcpp::Client<agent_msg::srv::Command>::SharedPtr agent_manager_client;
//    rclcpp::Client<agent_msg::srv::Command>::SharedPtr agent_osmo_client;
//    rclcpp::Client<agent_msg::srv::Command>::SharedFuture agent_manager_result;

    rclcpp::executors::SingleThreadedExecutor::SharedPtr mROS2Executor;


    std::string                 ros2Header = "/drone";

    double                     init_pos_x = 0, init_pos_y = 0;
};

#endif // CROSDATA_H
