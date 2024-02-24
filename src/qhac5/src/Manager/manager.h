#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <QObject>
#include <QString>
#include <QMap>
#include <QXmlStreamReader>
#include <QTimer>
#include <QThread>
#include <QRect>
#include <QRectF>
#include <QVector3D>
#include <QtMath>
#include <Eigen/Dense>

class IAgent;

class CManager : public QObject
{
    Q_OBJECT

public:
    CManager(QObject* parent = 0);
    virtual ~CManager();

public:
	int  loadAgentFile(const QString& aFilePath);
    void addAgent(const QMap<QString, QString> aAgent);

    int  numOfAgent();
    bool hasAgent(const int aID);


    IAgent* agent(int aID);
    QMap<int, IAgent*> agents() const;

    QString property(const QString& aGroup, const QString& aKey);

    QList<QVector3D> getAgentsPositionsToRect(QRectF qrect);
    void moveAgentsToRect(QRect qrect);

public Q_SLOTS:
    void onWork();
    void onTerminated();

private:
    IAgent* createAgent(const int aID, const char* aIP);
    QMap<QString, QString> parseAgentProperties(QXmlStreamReader& aXml, int* aStatus=NULL);
    QMap<QString, QString> parseMotionCaptureProperties(QXmlStreamReader& aXml, int* aStatus=NULL);
    QMap<QString, QString> parseEmdScenProperties(QXmlStreamReader& aXml, int* aStatus=NULL);
    QMap<QString, QString> parseBaseProperties(QXmlStreamReader& aXml, int* aStatus=NULL);

private:
    QMap< QString, QString >        mEmdscen;
    QMap< QString, QString >        mBase;

    QMap<int, IAgent*>              mAgents;
    QList<Eigen::Vector3f>          mUnknownMark;
};

#endif // DATAMANAGER_H
