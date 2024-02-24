#ifndef PARAMCHECKWORKER_H
#define PARAMCHECKWORKER_H

#include <QObject>
#include <bmodelcmdsender.h>
#include <cmodelcmdsender.h>

class ParamcheckWorker : public QObject
{
    Q_OBJECT
public:
    ParamcheckWorker();
    ~ParamcheckWorker();
    CBModelCmdSender* mSenderBModel = nullptr;        // TODO : Make mSender unified.
    CCModelCmdSender* mSenderCModel = nullptr;
    const QString aName;
    QMap< QString, QVariant >           mDefaultParams;

signals:

public slots:
    void process();
};

#endif // PARAMCHECKWORKER_H
