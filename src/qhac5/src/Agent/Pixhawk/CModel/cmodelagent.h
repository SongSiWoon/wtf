#ifndef CCMODELAGENT_H
#define CCMODELAGENT_H

#include "customconfig.h"
#include "agent.h"
#include "filemanager.h"
#include "rosdata.h"

#include <QTimer>
#include <QObject>
#include <common/mavlink.h>
#include <QColor>
#include <QMutex>
#include <QGeoCoordinate>
#include <QVector3D>

class IAgent;
class CCModelComm;
class CMavLinkData;
class CROSData;
class CCModelCmdSender;

/*
 * The C Model use ROS2 message instead of Mavlink
 */

class CCModelAgent : public IAgent
{

    Q_OBJECT

public:
    explicit CCModelAgent(QObject* parent = 0);
    explicit CCModelAgent(QMap<QString, QString> aProperty, QObject* parent = 0);
    virtual ~CCModelAgent();

public:
	int sysID() {return mSysID;}
	QString ipAddr() {return mIPAddr;}
	virtual int  cmd(const char *aCmd, QVariant aArg1=0 , QVariant aArg2=0, QVariant aArg3=0, QVariant aArg4=0, QVariant aArg5=0, QVariant aArg6=0);
    virtual QVariant data(const char* aName);
    virtual QVariant dataROS(const char *aName);

    CROSData* data() {return mData;}
    CROSData* dataROS() {return mData;}
    CCModelComm*  comm() {return mComm;}
    CCModelCmdSender* sender() {return mSender;}

private:
    void init();
	void initCModelDefaultParam();

    void initDefaultGainValue();
	void checkParams(QString name="");
	bool checkBit(uint32_t aValue, uint aBit);
	QString readyToFly();
    QString readyToFlyMonitoring();
    QString warningFlight();
    QString emergencyFlight();
	QString paramStatus();
	QMap< QString, QVariant >  realParams();

    QVector3D LLH2NED(QGeoCoordinate pos);
    QGeoCoordinate NED2LLH(QVector3D pos);

private:
	QMutex					mMutex;
	int						mSysID;
	QString					mIPAddr;
    QTimer*                 mTimer;
    QTimer*                 mRosTimer;
    CCModelComm*            mComm;
    CROSData*               mData;
    CCModelCmdSender*       mSender;

    QStringList                         mDirFiles;
    QMap< QString, QVariant >           mDefaultParams;
    QColor                              mLedColor;

    QGeoCoordinate                      mArmPos;

    QGeoCoordinate  REF_POS = QGeoCoordinate(REF_LAT, REF_LON, REF_ALT);
};

#endif // CCModelAgent_H
