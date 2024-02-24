#include "udpcomm.h"


CUDPComm::CUDPComm(IComm *parent) :
    IComm(parent)
{

}

CUDPComm::~CUDPComm()
{
    mSocket->close();
    delete mSocket;
}

int CUDPComm::init()
{
    mSocket = new QUdpSocket(this);
    //mSocket->bind(QHostAddress::LocalHost, 14550);
    mSocket->bind(QHostAddress::Any, 9750);

    // connect
    connect(mSocket, SIGNAL(readyRead()), this, SLOT(recvEventFromUDP()));

    return 0;
}

int CUDPComm::write(const QByteArray &aData)
{
    qint64 len = 0;
    static QMutex mutex;
    mutex.lock();
    // FIXME :  update the ip address and port number
    len = mSocket->writeDatagram(aData, QHostAddress("127.0.0.1"), 11111);
    mutex.unlock();

    return len;
}

void CUDPComm::recvEventFromUDP()
{
    while( mSocket->hasPendingDatagrams() )   {
        // read raw data
        QHostAddress addr;
        quint16 port = 0;
        QByteArray buffer( mSocket->pendingDatagramSize(), 0 );
        mSocket->readDatagram( buffer.data(), buffer.size(), &addr, (quint16*)&port);

        pushRecvData(buffer);

        Q_EMIT eventRecv();
    }
}

