#ifndef CUDPCOMM_H
#define CUDPCOMM_H


#include <QObject>
#include <QUdpSocket>
#include "comm.h"

class IComm;

class CUDPComm : public IComm
{
    Q_OBJECT

public:
    explicit CUDPComm(IComm *parent = 0);
    virtual ~CUDPComm();

public:
    virtual int init();
    virtual int write(const QByteArray& aData);

public Q_SLOTS:
    void recvEventFromUDP();

private:
    QUdpSocket*                     mSocket;

};

#endif // CUDPCOMM_H
