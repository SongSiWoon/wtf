#ifndef PIXHAWKDATA_H
#define PIXHAWKDATA_H

#include <mmodel/mavlink.h>
#include "filemanager.h"


#include <QObject>
#include <QByteArray>
#include <QMutex>
#include <QStringList>
#include <QMap>
#include <QVariant>


class IAgent;
class FileManager;

class CMavLinkData : public QObject
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
    explicit CMavLinkData(IAgent* agent, QObject* parent = 0);
    virtual ~CMavLinkData();

public:
    void setAgent(IAgent* aAgent) {mAgent = aAgent;}
    void setFileManager(FileManager* aFTP) {mFTP = aFTP;}
    bool updateData(const QByteArray& aByteArray);
	bool updateMsg(const mavlink_message_t& aMsg);
	QVariant param(const QString aName);
	void resetParams();

	void resetAck();
	int checkAck(const uint16_t aCmd);
    bool monitoringFlag(uint32_t aValue, uint aBit);
    QString strMonitoringEnum(MonitoringFlagType aType);
    QString strMonitoringStatus();
    void show();
    QString log() const;

public:
    virtual int   battery()     {return mStatus.battery_remaining;}
    virtual int   status()      {return mState;}
    virtual float posX()        {return mLocalPos.x;}
    virtual float posY()        {return mLocalPos.y;}
    virtual float posZ()        {return mLocalPos.z;}
    virtual float heading()     {return 0;}

    QVariant data(const QString& aItem);

private:
    uint toUInt(const QByteArray& aBuffer);
    ushort toUShort(const QByteArray& aBuffer);
	void update_param_value(const mavlink_message_t* aMsg);

private:
    IAgent*                     mAgent;
    FileManager*                mFTP;
    uint                        mSeq;
    uint                        mState;
    QMutex                      mMutex;
    QStringList                 mStrModeList;
    QStringList                 mStrStateList;
    qint64                      mRecvTime;
    qint64                      mRecvTime_LocalPos;
    qint64                      mRecvTime_Monitoring;

	QMap< QString, QVariant >           mParams;
    mavlink_heartbeat_t                 mHeart;
    mavlink_vfr_hud_t                   mHud;
    mavlink_sys_status_t                mStatus;
    mavlink_local_position_ned_t        mLocalPos;
    mavlink_distance_sensor_t           mDist;
    mavlink_gps_raw_int_t               mGPSRaw;
    mavlink_vicon_position_estimate_t   mViconEst;
    mavlink_position_target_local_ned_t mTargetLocal;
	mavlink_statustext_t				mStatusText;
	mavlink_attitude_t					mAttitude;
	mavlink_gps_rtk_t					mRTK;
	mavlink_command_ack_t				mAck;
    mavlink_monitoring_t                mMonitoring;
};

#endif // PIXHAWKDATA_H
