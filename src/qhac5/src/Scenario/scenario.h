#ifndef SCENARIO_H
#define SCENARIO_H

#include "customconfig.h"
#include "scenarioevent.h"
#include "hungarian.h"


#include <iostream>
#include <QObject>
#include <QTime>
#include <QList>
#include <QMap>
#include <QXmlStreamReader>
#include <QVector3D>
#include <QGeoCoordinate>

#include <QtMultimedia/QMediaPlayer>

class CController;
class CManager;
class CScenarioEvent;
class CSpline;

class IScenarioEvent;
class IAgent;

class CScenario : public QObject
{
    Q_OBJECT
public:
    enum TRANS_STATE {TERMINATED = 0, SELECTED, ARM, TAKEOFF, UP, MOVE, DOWN, KEEP, UP_BACK, MOVE_BACK, DOWN_BACK, LAND, DISARM};

public:
    explicit CScenario(CManager* aManager, QObject *parent = 0);
    virtual ~CScenario();

public:
    void updateTarget(QList<QGeoCoordinate> aTargetList, QGeoCoordinate aRefPos);

    void start();
	void pause();
    void stop();

    QString agentState(IAgent* agent);
    QGeoCoordinate targetLLH(IAgent* agent);
    QGeoCoordinate initLLH(IAgent* agent);

private:
    QVector3D LLH2NEU(QGeoCoordinate pos);
    QGeoCoordinate NEU2LLH(QVector3D pos);
    QVector3D cpos_from_LLH(IAgent* agent);

    void updateTrajectory();
    bool isCompleteState(IAgent* agent, TRANS_STATE state);
    bool isLowBat(IAgent* agent);
    bool isReadyBat(IAgent* agent);
    IAgent* findReadyAgent();
    QVector3D findTargetPos(IAgent* targetAgent);
    void initAgentState(IAgent* agent);
    bool inConvergeTime(IAgent* agent, bool condition);
    void changeState(IAgent* agent, TRANS_STATE curr_state, TRANS_STATE next_state);    

    /* control traffic and decide whether MOVE or MOVE_BACK is permitted */
    bool isTrafficManaged(IAgent* agent);

    void assignTarget(QList<QVector3D> aTargetList);
    void assignTargetAgain(QList<QVector3D> aTargetList);
    void updateAssignment();
    void showInfo(IAgent* agent);
    bool circleIntesect(QLineF line2 , float r, QVector2D c);
    bool isInGuard(QLineF traj, QLineF guard1, QLineF guard2);

Q_SIGNALS:
    
public Q_SLOTS:
    void timerEvent(QTimerEvent *timerEvent);
    
private:
    CManager*                           mManager;
    QTime                               mTime;
    QList<IScenarioEvent*>              mEventList;
	int                                 mPausedTime;
	bool                                mPause;

    QMap<int, QString>                  mPrevCmd;
    QMap<int, bool>                     mContinuePass;
    QMap<int, CSpline*>                 mCSplineList;

	int                                 mTimerId;

	quint64								mScenarioTotalTime;
    bool                                mIsReady;

    QList<QVector3D>                    mTargetList;

    HungarianAlgorithm                  mHungAlgo;
    float                               HEADING = 270;

    bool                                mFirstAssign;

public:     // For Test
    // TODO: create struct to merge all variables
    QMap<IAgent*, TRANS_STATE>          mNodeState;
    QMap<IAgent*, QVector3D>            mInitPos;           // the position when ARM
    QMap<IAgent*, QVector3D>            mStepPos;           // the target position for each state
    QMap<IAgent*, qint64>               mTransitTime;
    QMap<IAgent*, qint64>               mConvergeTime;
    QMap<IAgent*, IAgent*>              mShiftAgent;
    QMap<IAgent*, QVector3D>            mTargetPos;         // real target position which considers collision avoidance for shift
    QMap<IAgent*, QVector3D>            mOriginTarget;      // original target position which is compared with target position
    QGeoCoordinate                      mRefPos;
};

#endif // SCENARIO_H
