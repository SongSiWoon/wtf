#include "mmodelcomm.h"
#include "mmodelagent.h"
#include "mavlinkdata.h"
#include "sleeper.h"

#include <QMutex>

CMModelComm* CMModelComm::mInstance = NULL;

CMModelComm::CMModelComm(QObject *parent) :
    QObject(parent)
{
	mPort = 0;
}

CMModelComm::~CMModelComm()
{
    mSocket->close();
    delete mSocket;
}

CMModelComm *CMModelComm::instance(int aPort)
{
	static QMutex gMutex;
    if (!mInstance) {
		gMutex.lock();
        mInstance = new CMModelComm();
        mInstance->init(aPort);
		gMutex.unlock();
    }

    return mInstance;
}

qint64 CMModelComm::write(const QByteArray &aData, int aID)
{
    qint64 len = 0;
	mMutex.lock();

	QString ipAddr = mAgents[aID]->ipAddr();

	if ( mPort != 0 && ipAddr != QString("")) {		
        if ( mAgents[aID]->info("mode") == QString("sitl") ) {            
            len = mSocket->writeDatagram(aData, QHostAddress(ipAddr), mAgents[aID]->info("port").toInt());
		}
		else {
			len = mSocket->writeDatagram(aData, QHostAddress(ipAddr), mPort);
		}
	}
	else if ( mPairPort != 0 && ipAddr != QString("") ) {
		len = mSocket->writeDatagram(aData, QHostAddress(ipAddr), mPairPort);
	}
	else {
		qDebug("ERROR: cannot writeDatagram because of uninitial port");
	}

	mMutex.unlock();

    return len;
}

void CMModelComm::addAgent(CMModelAgent *aAgent)
{
    mAgents[aAgent->id()] = aAgent;
}

void CMModelComm::onRecv()
{    
    while( mSocket->hasPendingDatagrams() )   {

        // read raw data
        QHostAddress addr;
        quint16 port = 0;
        QByteArray buffer( mSocket->pendingDatagramSize(), 0 );
        mSocket->readDatagram( buffer.data(), buffer.size(), &addr, (quint16*)&port);

        // check pair port
		//if ( mPairPort == 0 ) {
            mPairAddr = addr;
            mPairPort = port;
		//}

        // retransmit to each agent
        mavlink_message_t message;
        mavlink_status_t status;
        uint8_t msgReceived = false;
        for ( int i = 0 ; i < buffer.size() ; i++ ) {
            uint8_t cp = buffer.at(i);
            msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
            if ( msgReceived ) {
				QMapIterator<int, CMModelAgent*> i(mAgents);
				while (i.hasNext()) {
					i.next();
					if (i.value()->sysID() == message.sysid ) {
						i.value()->data()->updateMsg(message);
						break;
					}
				}
            }
        }

    }
}


void CMModelComm::init(int aPort)
{
    // init
    mSocket   = NULL;
    mPairPort = 0;
	mPort = aPort;

    // create socket
    mSocket = new QUdpSocket(this);

    //mSocket->bind(QHostAddress::LocalHost, 14550);
	mSocket->bind(QHostAddress::Any, mPort);

    // connect
    connect(mSocket, SIGNAL(readyRead()), this, SLOT(onRecv()));


}
