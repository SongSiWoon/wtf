#include "paramcheckworker.h"
#include "sleeper.h"
#include <QDebug>

ParamcheckWorker::ParamcheckWorker()
{

}


ParamcheckWorker::~ParamcheckWorker()
{

}

void ParamcheckWorker::process()        // TODO : integrate mSenderBModel and mSenderCModel
{
    QMap<QString, int> retryCount;

    if(mSenderBModel != nullptr) {
        if(aName == "") {
            QMapIterator<QString, QVariant> i(mDefaultParams);
            while (i.hasNext()) {
                i.next();
                mSenderBModel->requestParam(i.key());
                CSleeper::msleep(150);
            }
        } else  {
            mSenderBModel->requestParam(aName);
            CSleeper::msleep(150);
        }

        QListIterator<QString> retryi(mSenderBModel->getParamRequested());
        while (retryi.hasNext()) {
            retryCount.insert(retryi.next(), 0);
        }

        while (!mSenderBModel->getParamRequested().empty()) {
            QList<QString> retryParams = mSenderBModel->getParamRequested();
            QString retryParam;
            for(int i=0;i<retryParams.size();i++) {
                retryParam = retryParams.at(i);
                int count = retryCount.take(retryParam);
                if (count <= 3) {
                    retryCount[retryParam] = count + 1;
                    mSenderBModel->requestParam(retryParam);
                    CSleeper::msleep(500);
                }
            }
        }
    } else {
        if(aName == "") {
            QMapIterator<QString, QVariant> i(mDefaultParams);
            while (i.hasNext()) {
                i.next();
                mSenderCModel->requestParam(i.key());
                CSleeper::msleep(150);
            }
        } else  {
            mSenderCModel->requestParam(aName);
            CSleeper::msleep(150);
        }

        QListIterator<QString> retryi(mSenderCModel->getParamRequested());
        while (retryi.hasNext()) {
            retryCount.insert(retryi.next(), 0);
        }

        while (!mSenderCModel->getParamRequested().empty()) {
            QList<QString> retryParams = mSenderCModel->getParamRequested();
            QString retryParam;
            for(int i=0;i<retryParams.size();i++) {
                retryParam = retryParams.at(i);
                int count = retryCount.take(retryParam);
                if (count <= 3) {
                    retryCount[retryParam] = count + 1;
                    mSenderCModel->requestParam(retryParam);
                    CSleeper::msleep(500);
                }
            }
        }
    }
}
