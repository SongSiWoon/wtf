#ifndef CMModelComm_H
#define CMModelComm_H

#include <QObject>
#include <QUdpSocket>
#include <QByteArray>
#include <QMap>
#include <QMutex>

class CMModelAgent;

class CMModelComm : public QObject
{
    Q_OBJECT

private:
    CMModelComm(QObject *parent = 0);
    virtual ~CMModelComm();

public:
    static CMModelComm* instance(int aPort);
    static void free();

public:
    void init(int aPort);

public:
	qint64 write(const QByteArray& aData, int aID);
    void addAgent(CMModelAgent* aAgent);

public Q_SLOTS:
    void onRecv();

private:

private:
	QMutex							mMutex;
    static CMModelComm*             mInstance;
    QMap<int, CMModelAgent*>        mAgents;

    QUdpSocket*                     mSocket;
//    QSerialPort*                    mSerial;

    QHostAddress                    mPairAddr;
    quint16                         mPairPort;
	int								mPort;

};

#endif // CMModelComm_H
