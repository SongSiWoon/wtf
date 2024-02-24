#include "agent.h"
#include <Eigen/Core>
#include <QDateTime>
#include "logger.h"

IAgent::IAgent(QObject* parent)
    :QObject(parent)
{
}

IAgent::IAgent(QMap<QString, QString> aProperty, QObject *parent)
    :QObject(parent)
{
    mInfo = aProperty;

    mID = mInfo["id"].toInt();
    mScenarioStatus = INIT;


}


IAgent::~IAgent()
{
}

QVariant IAgent::data(const QString aName)
{
    return data(aName.toLatin1().data());
}

QVariant IAgent::dataROS(const QString aName)
{
    return dataROS(aName.toLatin1().data());
}

int IAgent::id()
{
    return mID;
}

void IAgent::setScenarioStatus(const IAgent::ScenarioStatus aStatus)
{
    mScenarioStatus = aStatus;
}

IAgent::ScenarioStatus IAgent::scenarioStatus()
{
    return mScenarioStatus;
}

bool IAgent::addInfo(const QString aName, const QString aValue, bool aOverWrite)
{
	QString name = aName.trimmed().toLower();

	if ( aOverWrite || ! mInfo.contains(name) ) {
		mInfo[name] = aValue;
        return true;
    }
    else {
        return false;
    }
}

QVariant IAgent::info(const char *aName)
{
    if ( !mInfo.contains(QString(aName).trimmed().toLower()) ) { // Jang changed 2016. 5. 9.
        return QVariant("");
    }
    else {
        return QVariant(mInfo[QString(aName).trimmed().toLower()]);
    }
}
