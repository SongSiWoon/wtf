#ifndef PIXHAWKCMDSENDER_H
#define PIXHAWKCMDSENDER_H

#include "customconfig.h"
#include "mavlinkdata.h"

#include <QThread>
#include <QUdpSocket>
#include <QMutex>
#include <QTimer>
#include <stdint.h>
#include <sys/types.h>

class CMModelAgent;
class CMModelComm;

class CMModelCmdSender : public QObject
{
    Q_OBJECT
public:
	enum Target { TARGET_X, TARGET_Y, TARGET_Z, TARGET_HEAD };
public:
    explicit CMModelCmdSender(QObject* parent = 0);
    explicit CMModelCmdSender(CMModelAgent* aAgent, QObject* parent = 0);
    virtual ~CMModelCmdSender();

public:
    void arm();
    void disarm();
	int reboot();
    int offboard();
    int manual();
    int move(float aX, float aY, float aZ, float aHead);
	int led(int aType, int aR, int aG, int aB, int aBrightness, int aSpeed);
    int ledcmd(int aType, int aR, int aG, int aB, int aBrightness, int aSpeed);
	int requstParam(const QString aName);
	int setParam(const QString aName, const QVariant aValue);
    int calib_aceel();
	int calib_gyro();
	int calib_level();
	int setOffset(float aX, float aY, float aZ);
    int reserveScenarioStartTime(float aStartTime);
    int stopScenario();
    int resetScenario();
	int setScenarioConfs(float aOffsetX, float aOffsetY, float aRot, QString aFilename);
    int emergencyLanding();
    int testRTKOff();
	float target(Target aType);
    void setEmbeddedScenario(bool aMode);
	qint64 period() { return mTimerPeriod;}
    int upload_scneario(const QString file_path);
    int transmit(const mavlink_message_t& aMsg);

private:
	int procCalibGyro();
	int procCalibLevel();
    int procCalibAccel();
    int procArm();
    int procDisarm();
	int procReboot();
    int automode();

	int offboard_waypoint();		// existing offboard waypoint
	int target_offboard();			// optimized target offboard (include led)
    int heartbit();
    int control(int aRoll, int aPitch, int aYaw, int aThrust);


public Q_SLOTS:
    void onTimeout();
    void onTimeout1Hz();

private:

	QMutex					mMutex;

    CMModelAgent*           mAgent;
    CMModelComm*            mComm;

    QTimer*                 m1HzTimer;

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

};

#endif // PIXHAWKCMDSENDER_H
