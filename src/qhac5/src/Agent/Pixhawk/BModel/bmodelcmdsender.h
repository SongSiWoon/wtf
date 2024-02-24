#ifndef CBMODELCMDSENDER_H
#define CBMODELCMDSENDER_H

#include "customconfig.h"

#include <QThread>
#include <QUdpSocket>
#include <QMutex>
#include <QTimer>
#include <stdint.h>
#include <sys/types.h>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/uavcan_parameter_request.hpp>

class CBModelAgent;
class CBModelComm;

class CBModelCmdSender : public QObject
{
    Q_OBJECT
public:
    enum Target { TARGET_X, TARGET_Y, TARGET_Z, TARGET_HEAD };
public:
    explicit CBModelCmdSender(QObject* parent = 0);
    explicit CBModelCmdSender(CBModelAgent* aAgent, QObject* parent = 0);
    virtual ~CBModelCmdSender();

public:
    void arm();
    void disarm();
    void lock();
    void unlock();
    void takeoff(double lat, double lng, double yaw, double altitude);
    void landing();
    void reposition(double lat, double lng, double yaw, double altitude);
    int reboot();
    int offboard();
    int automission();
    int manual();
    int move(float aX, float aY, float aZ, float aHead);
    int requestParam(const QString aName);
    int setParam(const QString aName, const QVariant aValue);
    QList<QString> getParamRequested();
    int calib_aceel();
    int calib_gyro();
    int calib_level();
    float target(Target aType);
    qint64 period() { return mTimerPeriod;}
    void startStreamCmd();
    void stopStreamCmd();

private:
    int procCalibGyro();
    int procCalibLevel();
    int procCalibAccel();
    int procArm();
    int procDisarm();
    int procReboot();
    int automode();

public Q_SLOTS:
            void onTimeout();

private:

    QMutex					mMutex;

    CBModelAgent*           mAgent;
    CBModelComm*            mComm;

    uint8_t                 mSysID;

    int                     mTimeOutCount;

    float                   mTargetX;
    float                   mTargetY;
    float                   mTargetZ;
    float                   mHead;

    quint8					mLedType;
    quint8					mLedSpeed;
    quint8					mLedR;
    quint8					mLedG;
    quint8					mLedB;
    quint8					mLedBright;


    bool                    mTakeOffReady;
    bool                    mLandReady;
    bool					mGyroCalib;
    bool					mAccelCalib;
    bool					mLevelCalib;
    bool					mReboot;
    bool                    mEmbeddedScenarioMode;

    qint64                  mPrevTime;
    qint64                  mTimerPeriod;

    // ros::NodeHandle         nh_bmodelcmdsender;
};

#endif // CBMODELCMDSENDER_H
