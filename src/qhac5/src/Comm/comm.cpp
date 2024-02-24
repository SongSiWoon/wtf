#include "comm.h"

IComm::IComm(QObject *parent) : QObject(parent)
{

}

IComm::~IComm()
{

}

QByteArray IComm::popRecvData()
{
    QByteArray data;

    mMutex.lock();
    data = mRecvData;
    mRecvData.clear();
    mMutex.unlock();

    return data;
}

void IComm::pushRecvData(QByteArray aRecvData)
{
    mMutex.lock();
    mRecvData.append(aRecvData);
    mMutex.unlock();
}

