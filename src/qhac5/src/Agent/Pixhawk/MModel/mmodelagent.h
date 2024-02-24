#ifndef CMMODELAGENT_H
#define CMMODELAGENT_H

#include "customconfig.h"
#include "agent.h"
#include "filemanager.h"


#include <QTimer>
#include <QObject>
#include <mmodel/mavlink.h>
#include <QColor>
#include <QMutex>

class IAgent;
class CMModelComm;
class CMavLinkData;
class CMModelCmdSender;
class FileManager;

class CMModelAgent : public IAgent
{

    Q_OBJECT

public:
    explicit CMModelAgent(QObject* parent = 0);
    explicit CMModelAgent(QMap<QString, QString> aProperty, QObject* parent = 0);
    virtual ~CMModelAgent();

public:
	int sysID() {return mSysID;}
	QString ipAddr() {return mIPAddr;}
	virtual int  cmd(const char *aCmd, QVariant aArg1=0 , QVariant aArg2=0, QVariant aArg3=0, QVariant aArg4=0, QVariant aArg5=0, QVariant aArg6=0);
    virtual QVariant data(const char* aName);
    virtual QVariant dataROS(const char *aName);

    CMavLinkData* data() {return mData;}    
    CMModelComm*  comm() {return mComm;}
    CMModelCmdSender* sender() {return mSender;}

private:
    void init();
	void initMModelDefaultParam();
	void initXModelDefaultParam();

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
    CMModelComm*            mComm;
    CMavLinkData*           mData; 
    CMModelCmdSender*       mSender;
    FileManager*            mFTP;

    QStringList             mDirFiles;
	QMap< QString, QVariant >           mDefaultParams;
	QColor					mLedColor;


};

#endif // CMMODELAGENT_H
