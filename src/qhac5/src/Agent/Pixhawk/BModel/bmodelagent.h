#ifndef CBMODELAGENT_H
#define CBMODELAGENT_H

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

class IAgent;
class CBModelComm;
class CMavLinkData;
class CROSData;
class CBModelCmdSender;

class CBModelAgent : public IAgent
{

    Q_OBJECT

public:
    explicit CBModelAgent(QObject* parent = 0);
    explicit CBModelAgent(QMap<QString, QString> aProperty, QObject* parent = 0);
    virtual ~CBModelAgent();

public:
	int sysID() {return mSysID;}
	QString ipAddr() {return mIPAddr;}
	virtual int  cmd(const char *aCmd, QVariant aArg1=0 , QVariant aArg2=0, QVariant aArg3=0, QVariant aArg4=0, QVariant aArg5=0, QVariant aArg6=0);
    virtual QVariant data(const char* aName);
    virtual QVariant dataROS(const char *aName);

    CROSData* data() {return mData;}
    CROSData* dataROS() {return mData;}
    CBModelComm*  comm() {return mComm;}
    CBModelCmdSender* sender() {return mSender;}

private:
    void init();
	void initBModelDefaultParam();

    void initDefaultGainValue();
	void checkParams(QString name="");
	bool checkBit(uint32_t aValue, uint aBit);
	QString readyToFly();
    QString readyToFlyMonitoring();
    QString warningFlight();
    QString emergencyFlight();
	QString paramStatus();
	QMap< QString, QVariant >  realParams();

private:
	QMutex					mMutex;
	int						mSysID;
	QString					mIPAddr;
    QTimer*                 mTimer;
    QTimer*                 mRosTimer;
    CBModelComm*            mComm;
    //CMavLinkData*           mData;
    CROSData*               mData;
    CBModelCmdSender*       mSender;

    QStringList             mDirFiles;
	QMap< QString, QVariant >           mDefaultParams;
	QColor					mLedColor;


};

#endif // CBModelAgent_H
