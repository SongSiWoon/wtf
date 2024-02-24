#ifndef ICOMM_H
#define ICOMM_H

#include <QObject>
#include <QByteArray>
#include <QMutex>

class IComm : public QObject
{
    Q_OBJECT

public:
    explicit IComm(QObject *parent = 0);
    virtual ~IComm();

public:
//    virtual int init() = 0;
//    virtual int write(const QByteArray& aData) = 0;

    QByteArray popRecvData();
    void pushRecvData(QByteArray aRecvData);

Q_SIGNALS:
    void eventRecv();

public Q_SLOTS:

private:
    QMutex              mMutex;
    QByteArray          mRecvData;
};

#endif // ICOMM_H
