#include "scenario.h"
#include "agent.h"
#include "scenarioevent.h"
#include "manager.h"
#include "cspline.h"
#include "mymath.h"
#include "hungarian.h"

#include <iostream>
#include <QStringList>
#include <QFile>
#include <QtAlgorithms>
#include <limits>
#include <QVector2D>
#include <QLineF>


#define KEEP_CHANGE_INTERVAL        (5*1000)

#undef SIMULATION

const static QStringList strSTATE = {"TERMINATED", "SELECTED", "ARM", "TAKEOFF", "UP", "MOVE", "DOWN", "KEEP", "UP_BACK", "MOVE_BACK", "DOWN_BACK", "LAND", "DISARM"};


CScenario::CScenario(CManager* aManager, QObject *parent) :
    QObject(parent)
{
    mManager = aManager;
	mPause = false;
	mTimerId = -1;
	mScenarioTotalTime = 0;    
    mFirstAssign = false;

    // init node state
    foreach (IAgent* agent, mManager->agents()) {
        mNodeState[agent] = TERMINATED;
        mShiftAgent[agent] == nullptr;
        mConvergeTime[agent] = 0;
    }    
}

CScenario::~CScenario()
{
}

void CScenario::updateTarget(QList<QGeoCoordinate> aTargetList, QGeoCoordinate aRefPos)
{
    mRefPos = aRefPos;

    // init target list
    mTargetList.clear();

    // add target list
    for (int i = 0 ; i < aTargetList.size() ; i++ ) {
        mTargetList.append(LLH2NEU(aTargetList.at(i)));
    }

    if (!mFirstAssign) {        
        // init node state
        foreach (IAgent* agent, mManager->agents()) {
            mNodeState[agent] = TERMINATED;
        }

        // assign target list
        assignTarget(mTargetList);

        mFirstAssign = true;
    }
    else {        
        // assign target list
        assignTargetAgain(mTargetList);
    }

}


void CScenario::assignTarget(QList<QVector3D> aTargetList)
{
    QList<IAgent*> candidate;

    // select agents according to the number of target by shortest path to make square matrix
    foreach (IAgent* agent, mManager->agents()) {
        if ( mNodeState[agent] == TERMINATED && isReadyBat(agent) ) {
            candidate.append(agent);
        }
    }

    // if candidate is none, wait for other nodes (maybe the node will come back soon)
    if (candidate.size() == 0 ) {
        //qDebug("ERROR: no candidates (target:%d)", aTargetList.size());
        return;
    }

    // make cost matrix
    std::vector<std::vector<double>> costMatrix(candidate.size(), std::vector<double>(aTargetList.size()));
    for ( int i = 0 ; i < candidate.size() ; i++ ) {
        IAgent* agent = candidate[i];
        //QVector3D cpos = agent->data("POS").value<QVector3D>();
        QVector3D cpos = cpos_from_LLH(agent);
        for ( int j = 0 ; j < aTargetList.size() ; j++ ) {
            QVector3D tpos = aTargetList[j];
            double dist = tpos.distanceToPoint(cpos);
            costMatrix[i][j] = dist;
        }
    }

    // assign agents : mOriginTarget[agent]
    double cost;
    vector<int> assignment;
    cost = mHungAlgo.Solve(costMatrix, assignment);


    QList<IAgent*> agents;
    for ( int i = 0 ; i < candidate.size() ; i++ ) {
        IAgent* agent = candidate[i];
        if ( assignment[i] >= 0 ) {
            agents.append(agent);

            mNodeState[agent] = SELECTED;
            mTransitTime[agent] = QDateTime::currentMSecsSinceEpoch();
            mConvergeTime[agent] = 0;
            mOriginTarget[agent] = aTargetList[assignment[i]];
            mTargetPos[agent] = findTargetPos(agent);

            foreach (IAgent* shift_agent, mManager->agents()) {
                if ( shift_agent->id() != agent->id()
                     && mOriginTarget[shift_agent] == mOriginTarget[agent]
                     && mNodeState[shift_agent] == KEEP ) {
                    mShiftAgent[shift_agent] = agent;
                }
            }

        }
    }
}

void CScenario::assignTargetAgain(QList<QVector3D> aTargetList)
{
    QList<IAgent*> candidate;

    // select agents according to the number of target by shortest path to make square matrix
    foreach (IAgent* agent, mManager->agents()) {
        if ( mNodeState[agent] == KEEP && mShiftAgent[agent] == nullptr ) {            
            candidate.append(agent);
        }
        else if ( mNodeState[agent] == KEEP && mShiftAgent[agent] != nullptr ) {
            candidate.append(mShiftAgent[agent]);
        }
    }

    if (candidate.size() == 0 ) {
        foreach (IAgent* agent, mManager->agents()) {
            if ( mNodeState[agent] == DOWN && mShiftAgent[agent] == nullptr) {
                candidate.append(agent);
            }
        }

        foreach (IAgent* agent, mManager->agents()) {
            if ( mNodeState[agent] == MOVE && mShiftAgent[agent] == nullptr ) {
                candidate.append(agent);
            }
        }
        foreach (IAgent* agent, mManager->agents()) {
            if ( mNodeState[agent] == UP && mShiftAgent[agent] == nullptr ) {
                candidate.append(agent);
            }
        }
        foreach (IAgent* agent, mManager->agents()) {
            if ( mNodeState[agent] == TAKEOFF ) {
                candidate.append(agent);
            }
        }

        foreach (IAgent* agent, mManager->agents()) {
            if ( mNodeState[agent] == ARM ) {
                candidate.append(agent);
            }
        }

        foreach (IAgent* agent, mManager->agents()) {
            if ( mNodeState[agent] == TERMINATED ) {
                candidate.append(agent);
            }
        }

    }

    // if candidate is none, wait for other nodes (maybe the node will come back soon)
    if (candidate.size() == 0 ) {
        qDebug("ERROR: there is no candidat");
        return;
    }

    while ( candidate.size() > aTargetList.size() ) {
        candidate.removeLast();
    }


    // make cost matrix
    std::vector<std::vector<double>> costMatrix(candidate.size(), std::vector<double>(aTargetList.size()));
    for ( int i = 0 ; i < candidate.size() ; i++ ) {
        IAgent* agent = candidate[i];
            //QVector3D cpos = agent->data("POS").value<QVector3D>();
            QVector3D cpos = cpos_from_LLH(agent);
            for ( int j = 0 ; j < aTargetList.size() ; j++ ) {
                QVector3D tpos = aTargetList[j];
                double dist = tpos.distanceToPoint(cpos);
                costMatrix[i][j] = dist;
            }
        }

        // assign agents : mOriginTarget[agent]
        double cost;
        vector<int> assignment;
        cost = mHungAlgo.Solve(costMatrix, assignment);

        for ( int i = 0 ; i < candidate.size() ; i++ ) {
            IAgent* agent = candidate[i];

            if ( assignment[i] >= 0 ) {
                qDebug(">> %d - %d", assignment[i], agent->id());

                if ( mNodeState[agent] == UP || mNodeState[agent] == MOVE || mNodeState[agent] == DOWN || mNodeState[agent] == KEEP ) {
                    mStepPos[agent] = cpos_from_LLH(agent);
                    mNodeState[agent] = UP;
                }

                QList<IAgent*> prev_host_agents = mShiftAgent.keys(agent);
                int num_host_agent = prev_host_agents.size();
                if ( num_host_agent == 1 ) {
                    IAgent* prevHostAgent = prev_host_agents[0];
                    mNodeState[prevHostAgent] = UP_BACK;
                    mShiftAgent[prevHostAgent] = nullptr;
                }
                else {
                    if ( num_host_agent > 1 ) {
                        qDebug("ERROR: Wrong shifted agent (%d)", num_host_agent);
                }

            }

            QVector3D cpos = cpos_from_LLH(agent);
            mStepPos[agent].setX(cpos.x());
            mStepPos[agent].setY(cpos.y());

            mTransitTime[agent] = QDateTime::currentMSecsSinceEpoch();
            mConvergeTime[agent] = 0;
            mOriginTarget[agent] = aTargetList[assignment[i]];
            mTargetPos[agent] = findTargetPos(agent);
        }
    }
}

void CScenario::updateAssignment()
{
    QList<IAgent*> candidate;

    // select agents according to the number of target by shortest path to make square matrix
    foreach (IAgent* agent, mManager->agents()) {
        if ( mNodeState[agent] == TAKEOFF || mNodeState[agent] == UP ) {
            candidate.append(agent);
        }
    }

    // if candidate is none, wait for other nodes (maybe the node will come back soon)
    if (candidate.size() == 0 ) {
        //qDebug("ERROR: no candidates (target:%d)", aTargetList.size());
        return;
    }

    // get target list from candidates
    QList<QVector3D> aTargetList;
    foreach (IAgent* agent, candidate) {
        aTargetList.append(mOriginTarget[agent]);
    }

    // make cost matrix
    std::vector<std::vector<double>> costMatrix(candidate.size(), std::vector<double>(aTargetList.size()));
    for ( int i = 0 ; i < candidate.size() ; i++ ) {
        IAgent* agent = candidate[i];
        //QVector3D cpos = agent->data("POS").value<QVector3D>();
        QVector3D cpos = cpos_from_LLH(agent);
        for ( int j = 0 ; j < aTargetList.size() ; j++ ) {
            QVector3D tpos = aTargetList[j];
            double dist = tpos.distanceToPoint(cpos);
            costMatrix[i][j] = dist;
        }
    }

    // assign agents : mOriginTarget[agent]
    double cost;
    vector<int> assignment;
    cost = mHungAlgo.Solve(costMatrix, assignment);


    QList<IAgent*> agents;
    for ( int i = 0 ; i < candidate.size() ; i++ ) {
        IAgent* agent = candidate[i];
        if ( assignment[i] >= 0 ) {
            agents.append(agent);

//            mNodeState[agent] = SELECTED;
//            mTransitTime[agent] = QDateTime::currentMSecsSinceEpoch();
//            mConvergeTime[agent] = 0;
            mOriginTarget[agent] = aTargetList[assignment[i]];
            mTargetPos[agent] = findTargetPos(agent);

            foreach (IAgent* shift_agent, mManager->agents()) {
                if ( shift_agent->id() != agent->id()
                     && mOriginTarget[shift_agent] == mOriginTarget[agent]
                     && mNodeState[shift_agent] == KEEP ) {
                    mShiftAgent[shift_agent] = agent;
                }
            }

        }
    }

}

bool CScenario::isTrafficManaged(IAgent *agent)
{
    const qreal MARGIN = 1.0;
    //QVector2D cpos = agent->data("POS").value<QVector3D>().toVector2D();
    QVector2D cpos = cpos_from_LLH(agent).toVector2D();
    QVector2D tpos;
    if ( mNodeState[agent] == KEEP || mNodeState[agent] == UP_BACK || mNodeState[agent] == MOVE_BACK ) {
        tpos = mInitPos[agent].toVector2D();
    }
    else {
        tpos = mTargetPos[agent].toVector2D();
    }


    QPointF p0, p1;
    if ( cpos.x() < tpos.x() ) {
        p0 = cpos.toPointF();
        p1 = tpos.toPointF();
    }
    else {
        p0 = tpos.toPointF();
        p1 = cpos.toPointF();
    }

    qreal a = (p1.y() - p0.y()) / (p1.x() - p0.x());
    qreal b = -a*p0.x() + p0.y();

    qreal dx1 = qSqrt(MARGIN*MARGIN / (1+a*a));
    qreal dy1 = a*dx1;

    qreal dx2 = qSqrt(MARGIN*MARGIN / (1 + (1/(a*a))));
    qreal dy2 = - (1.0/a) * dx2;

    QPointF p0_up = QPointF(p0.x() - dx1 - dx2, p0.y() - dy1 - dy2);
    QPointF p1_up = QPointF(p1.x() + dx1 - dx2, p1.y() + dy1 - dy2);

    QPointF p0_down = QPointF(p0.x() - dx1 + dx2, p0.y() - dy1 + dy2);
    QPointF p1_down = QPointF(p1.x() + dx1 + dx2, p1.y() + dy1 + dy2);


    QLineF targetLine = QLineF(p0.x(), p0.y(), p1.x(), p1.y());
    QLineF guard1 = QLineF(p0_up.x(), p0_up.y(), p1_up.x(), p1_up.y());
    QLineF guard2 = QLineF(p0_down.x(), p0_down.y(), p1_down.x(), p1_down.y());

//    qDebug() << "----------------------";
//    qDebug() << "main :" << targetLine;
//    qDebug() << "up   :" << QLineF(p0_up, p1_up);
//    qDebug() << "down :" << QLineF(p0_down, p1_down);

    foreach (IAgent* other, mManager->agents()) {

        if ( agent == other ) {
            continue;
        }

        //QVector2D g_cpos = other->data("POS").value<QVector3D>().toVector2D();
        QVector2D g_cpos = cpos_from_LLH(other).toVector2D();
        QVector2D g_ipos = mInitPos[other].toVector2D();
        QVector2D g_tpos = mTargetPos[other].toVector2D();

        QLineF traj;
        if ( mNodeState[other] == UP_BACK || mNodeState[other] == MOVE_BACK ) {
            traj = QLineF(g_cpos.x(), g_cpos.y(), g_ipos.x(), g_ipos.y());
        }
        else {
            traj = QLineF(g_cpos.x(), g_cpos.y(), g_tpos.x(), g_tpos.y());
        }

        bool intersect_guard1 = (traj.intersect(guard1, nullptr) == QLineF::BoundedIntersection);
        bool intersect_guard2 = (traj.intersect(guard2, nullptr) == QLineF::BoundedIntersection);
        bool in_guards = isInGuard(traj, guard1, guard2);
        float r = 1.5;

//        if ( mNodeState[agent] == TAKEOFF || mNodeState[agent] == KEEP ) {
//            if ( mNodeState[other] == MOVE || mNodeState[other] == MOVE_BACK ) {
//                bool intersect_circle = circleIntesect(traj , r, cpos);
//                if ( intersect_circle ) {
//                    return false;
//                }
//            }
//        }

        bool intersect_circle = circleIntesect(targetLine , r, g_cpos);

//        if ( mNodeState[agent] == UP || mNodeState[agent] == UP_BACK ) {
//            if ( mNodeState[other] == MOVE || mNodeState[other] == MOVE_BACK ) {
//                if ( intersect_guard1 || intersect_guard2 || in_guards || intersect_circle ) {
//                    //qDebug("intersect1... (%d-%d)", agent->id(), other->id());
//                    return false;
//                }
//            }

//            if ( mNodeState[other] == UP_BACK || mNodeState[other] == UP /*|| mNodeState[other] == DOWN || mNodeState[other] == DOWN_BACK*/ ) {
//                if ( intersect_circle ) {
//                    //qDebug("intersect2... (%d-%d)", agent->id(), other->id());
//                    return false;
//                }
//            }
//        }


        if ( mNodeState[agent] == UP  ) {
            if ( mNodeState[other] == MOVE ) {
                if ( intersect_guard1 || intersect_guard2 || in_guards || intersect_circle ) {
                    //qDebug("intersect1... (%d-%d)", agent->id(), other->id());
                    return false;
                }
            }

            if ( mNodeState[other] == UP || mNodeState[other] == UP_BACK /* || mNodeState[other] == DOWN_BACK*/ ) {
                if ( intersect_circle ) {
                    //qDebug("intersect2... (%d-%d)", agent->id(), other->id());
                    return false;
                }
            }
        }


        if ( mNodeState[agent] == UP_BACK ) {
            if ( mNodeState[other] == MOVE_BACK ) {
                if ( intersect_guard1 || intersect_guard2 || in_guards || intersect_circle ) {
                    //qDebug("intersect1... (%d-%d)", agent->id(), other->id());
                    return false;
                }
            }

            if ( mNodeState[other] == UP_BACK || mNodeState[other] == UP ) {
                if ( intersect_circle ) {
                    //qDebug("intersect2... (%d-%d)", agent->id(), other->id());
                    return false;
                }
            }
        }
    }

    return true;
}


void CScenario::showInfo(IAgent *agent)
{
    qDebug() << "----- Agent ID: " << agent->id() << "-----";
    qDebug() << "State: " << strSTATE[mNodeState[agent]];
    qDebug() << "Curent Pos: " << agent->data("POS").value<QVector3D>()
             << "Target Pos: " << mTargetPos[agent];
    qDebug() << "Init Pos: " << mInitPos[agent]
             << "Step Pos: " << mStepPos[agent]
             << "Origin Pos: " << mOriginTarget[agent];
    if ( mShiftAgent.contains(agent) && mShiftAgent[agent] != nullptr ) {
        qDebug() << "Shift Agent: " << mShiftAgent[agent]->id();
    }
}

bool CScenario::circleIntesect(QLineF line, float r, QVector2D c)
{
    float x1 = line.x1();
    float y1 = line.y1();
    float x2 = line.x2();
    float y2 = line.y2();
    float xc = c.x();
    float yc = c.y();

    float t = ((yc - y1) * (y2-y1) + (xc - x1) * (x2-x1)) / ( (y2-y1) * (y2-y1) + (x2-x1) * (x2-x1));

    float xd = x1 + t * (x2-x1);
    float yd = y1 + t * (y2-y1);

    float d  = qSqrt((xd-xc)*(xd-xc) + (yd-yc)*(yd-yc));
    float d1 = qSqrt((x1-xc)*(x1-xc) + (y1-yc)*(y1-yc));
    float d2 = qSqrt((x2-xc)*(x2-xc) + (y2-yc)*(y2-yc));

    if ( d1 < r || d2 <r || (t > 0 && t < 1  && d < r) ) {
        return true;
    }
    else {
        return false;
    }
}

bool CScenario::isInGuard(QLineF traj, QLineF guard1, QLineF guard2)
{
    qreal a, b;

    // perpendicular guard
//    QLineF guard3 = QLineF(guard1.p1(), guard2.p1());
//    QLineF guard4 = QLineF(guard1.p2(), guard2.p2());

    // for guard 1
    a = guard1.dy()/guard1.dx();
    b = -a*guard1.x1() + guard1.y1();

    qreal g1_diff1 = traj.y1() -  a*traj.x1() + b;
    qreal g1_diff2 = traj.y2() -  a*traj.x2() + b;

    if ( g1_diff1 * g1_diff2  < 0 ) {
        return false;
    }

    // for guard 2
    a = guard2.dy()/guard2.dx();
    b = -a*guard2.x1() + guard2.y1();

    qreal g2_diff1 = traj.y1() -  a*traj.x1() + b;
    qreal g2_diff2 = traj.y2() -  a*traj.x2() + b;

    if ( g2_diff1 * g2_diff2  < 0 ) {
        return false;
    }

    if ( g1_diff1 * g2_diff1 < 0 ) {
        assert(g1_diff2 * g2_diff2 < 0);
        return true;
    }
    else {
        return false;
    }

}

QString CScenario::agentState(IAgent *agent)
{
    QString result;
    int shift_node = 0;
    if ( mShiftAgent[agent] != nullptr ) {
        shift_node = mShiftAgent[agent]->id();
        result = QString("%1/S:%2").arg(strSTATE[mNodeState[agent]]).arg(shift_node);
    }
    else {
        result = QString("%1").arg(strSTATE[mNodeState[agent]]);
    }
    return result;
}

QGeoCoordinate CScenario::targetLLH(IAgent *agent)
{
    return NEU2LLH(mTargetPos[agent]);
}

QGeoCoordinate CScenario::initLLH(IAgent *agent)
{
    return NEU2LLH(mInitPos[agent]);
}

QVector3D CScenario::LLH2NEU(QGeoCoordinate pos)
{
    // Calc x,y,z of pos with mRefPos
    double NED_X = mRefPos.distanceTo(QGeoCoordinate(pos.latitude(), mRefPos.longitude(), mRefPos.altitude()));
    if (pos.latitude() < mRefPos.latitude())
        NED_X = -NED_X;
    double NED_Y = mRefPos.distanceTo(QGeoCoordinate(mRefPos.latitude(), pos.longitude(), mRefPos.altitude()));
    if (pos.longitude() < mRefPos.longitude())
        NED_Y = -NED_Y;
    double NED_Z = (pos.altitude() - mRefPos.altitude());
    return QVector3D(NED_X, NED_Y, NED_Z);
}

QGeoCoordinate CScenario::NEU2LLH(QVector3D pos)
{
    // Calc lat, lon, alt of pos with mRefPos
    QGeoCoordinate LLHPosition = QGeoCoordinate(mRefPos.latitude(), mRefPos.longitude(), mRefPos.altitude());
    LLHPosition = LLHPosition.atDistanceAndAzimuth(pos.x(), 0, pos.z());
    LLHPosition = LLHPosition.atDistanceAndAzimuth(pos.y(), 90);
    return LLHPosition;
}

QVector3D CScenario::cpos_from_LLH(IAgent *agent)
{
    QVector3D p = agent->data("LLH").value<QVector3D>();
    QGeoCoordinate llh = QGeoCoordinate(p.x(), p.y(), p.z());
    return LLH2NEU(llh);
}

void CScenario::start()
{
	// start
	if ( mTimerId < 0) {
		mTime.start();
		mTimerId = this->startTimer(SCENARIO_PERIOD);
	}
	// resume
	else {
		if ( mPause == true) {
			mTime = mTime.addMSecs(mTime.elapsed() - mPausedTime);
			mTimerId = this->startTimer(SCENARIO_PERIOD);
		}
	}

	mPause = false;

}

void CScenario::pause()
{
	mPause = true;
	mPausedTime = mTime.elapsed();

	// stop timer
	if ( mTimerId > 0 ) {
		this->killTimer(mTimerId);
	}
}

void CScenario::stop()
{
	// stop timer
	if ( mTimerId > 0 ) {
		this->killTimer(mTimerId);
		mTimerId = -1;
	}

	// clear event
	mEventList.clear();

	mPause = false;

}

bool CScenario::inConvergeTime(IAgent *agent, bool condition)
{
    const qint64 MAX_CONVERGE_TIME = 2 * 1000;
    qint64 t = QDateTime::currentMSecsSinceEpoch();

    if ( condition ) {
        if ( mConvergeTime[agent] == 0 ) {
            mConvergeTime[agent] = t;
            return false;
        }
        else if ( t - mConvergeTime[agent] > MAX_CONVERGE_TIME ) {
            mConvergeTime[agent] = 0;
            return true;
        }
        else {
            return false;
        }
    }
    return false;
}

bool CScenario::isCompleteState(IAgent *agent, CScenario::TRANS_STATE state)
{
    const float MAX_PERMIT_DIST = 0.5;
    qint64 MIN_TRANSIT_TIME = 1 * 1000;

    qint64 t = QDateTime::currentMSecsSinceEpoch();
    QVector3D cpos = cpos_from_LLH(agent); //agent->data("POS").value<QVector3D>();
    QVector3D tpos = mStepPos[agent];
    float dist = tpos.distanceToPoint(cpos);
    bool condition = dist < MAX_PERMIT_DIST;


    // check state transition interval
    qint64 interval =  QDateTime::currentMSecsSinceEpoch() - mTransitTime[agent];
    if (interval < MIN_TRANSIT_TIME ) {
        return false;
    }

    if ( mNodeState[agent] == SELECTED ) {
#ifdef SIMULATION
        return true;
#else
        return isReadyBat(agent);
#endif
    }
    else if ( mNodeState[agent] == ARM ) {        
        return agent->data("ISARMED").toBool();
    }
    else if ( mNodeState[agent] == TAKEOFF) {        
        return true;
    }
    else if ( mNodeState[agent] == UP && dist < MAX_PERMIT_DIST) {
        return inConvergeTime(agent, condition) && isTrafficManaged(agent) ;
    }
    else if ( mNodeState[agent] == MOVE && dist < MAX_PERMIT_DIST) {
        return inConvergeTime(agent, condition) ;
    }
    else if ( mNodeState[agent] == DOWN && dist < MAX_PERMIT_DIST) {
        return inConvergeTime(agent, condition) ;
    }
    else if ( mNodeState[agent] == KEEP ) {
        if ( isLowBat(agent) && mShiftAgent.contains(agent) ) {
            IAgent* shiftAgent = mShiftAgent[agent];
            if ( shiftAgent != nullptr ) {
                QVector3D s_cpos = cpos_from_LLH(shiftAgent); //shiftAgent->data("POS").value<QVector3D>();
                QVector3D s_tpos = mStepPos[shiftAgent];
                float s_dist = s_tpos.distanceToPoint(s_cpos);
                qint64 s_interval =  t - mTransitTime[shiftAgent];

                if ( mNodeState[shiftAgent] == KEEP && s_dist < MAX_PERMIT_DIST && s_interval > KEEP_CHANGE_INTERVAL ) {
                    mShiftAgent[agent] = nullptr;
                    return true;
                }
            }
        }
        return false;
    }
    else if ( mNodeState[agent] == UP_BACK && dist < MAX_PERMIT_DIST ) {
        return inConvergeTime(agent, condition) && isTrafficManaged(agent);
    }
    else if ( mNodeState[agent] == MOVE_BACK && dist < MAX_PERMIT_DIST) {
        return inConvergeTime(agent, condition);
    }
    else if ( mNodeState[agent] == DOWN_BACK && dist < MAX_PERMIT_DIST) {
        return inConvergeTime(agent, condition);
    }
    else if ( mNodeState[agent] == LAND ) {
        tpos.setZ(0);
        dist = tpos.distanceToPoint(cpos);
        bool is_armed = agent->data("ISARMED").toBool();
        if ( is_armed == false ) {
            return true;
        }
        else if ( dist < MAX_PERMIT_DIST) {
            return true;
        }
        else {
            return false;
        }

    }
    else if ( mNodeState[agent] == DISARM ) {        
        if ( agent->data("ISARMED").toBool() ) {
            return false;
        }
        else {
            return true;
        }
    }
    else if ( mNodeState[agent] == TERMINATED ) {
        return true;
    }
    else {
        //qDebug("ERROR: Wrong node state (%d) dist:%f of Node(%d)", mNodeState[agent], dist, agent->id());
        return false;
    }

}

bool CScenario::isLowBat(IAgent *agent)
{
    if ( mNodeState[agent] == KEEP ) {

#ifdef SIMULATION
        qint64 interval =  QDateTime::currentMSecsSinceEpoch() - mTransitTime[agent];
        if ( interval > 10*1000 ) {
            return true;
        }
#else
        if ( agent->data("BATTERY").toInt() < 50 ) {
            //qDebug("low battery : %d", agent->data("BATTERY").toInt());
            return true;
        }
#endif
    }
    return false;
}

bool CScenario::isReadyBat(IAgent *agent)
{
#ifdef SIMULATION
    return true;
#else
    // TODO: check RTK flags and EKF2 ready
    if ( agent->data("BATTERY").toInt() >= 60 ) {
        return true;
    }
    else {
        return false;
    }
#endif
}

IAgent *CScenario::findReadyAgent()
{
    // TODO: consider distance for target position
    foreach (IAgent* agent, mManager->agents()) {
        if ( mNodeState[agent] == TERMINATED && isReadyBat(agent) ) {
            return agent;
        }
    }
    return NULL;
}

QVector3D CScenario::findTargetPos(IAgent *targetAgent)
{
    int count = 0;
    QVector3D targetPos = mOriginTarget[targetAgent];
    //qDebug("====================== findTargetPos(%d)  ======================", targetAgent->id());
    foreach (IAgent* agent, mManager->agents()) {
        float dist = mTargetPos[agent].distanceToPoint(mOriginTarget[targetAgent]);
        //qDebug("%d-%d : %f %d", targetAgent->id(), agent->id(), dist, mOriginTarget[targetAgent] == mOriginTarget[agent]);
        if ( agent != targetAgent && mNodeState[agent] == KEEP && mOriginTarget[targetAgent] == mOriginTarget[agent] && dist < 0.01 ) {
            count++;
            targetPos.setX(mOriginTarget[targetAgent].x()+1.5);
            targetPos.setY(mOriginTarget[targetAgent].y()+1.5);
            targetPos.setZ(mOriginTarget[targetAgent].z());

            QVector3D cpos = cpos_from_LLH(targetAgent);
            if ( targetPos.distanceToPoint(cpos) < 0.1 ) {
                qDebug("ERROR : same position");
                qDebug() << targetPos;
            }
            //showInfo(agent);
        }
    }
    //qDebug() << targetPos;

    if (count > 1 ) {
        qDebug("ERROR: find duplicated targets");
    }
    return targetPos;
}

void CScenario::initAgentState(IAgent* agent)
{
    // init node state
    if ( agent != nullptr ) {        
        mNodeState[agent] = TERMINATED;
        mShiftAgent[agent] = nullptr;
        mTargetPos[agent] = QVector3D();
        mStepPos[agent] = QVector3D();
        mOriginTarget[agent] = QVector3D();
    }
}

void CScenario::timerEvent(QTimerEvent *timerEvent)
{
    Q_UNUSED(timerEvent);

    updateTrajectory();
}

void CScenario::changeState(IAgent *agent, CScenario::TRANS_STATE curr_state, CScenario::TRANS_STATE next_state)
{
    if ( isCompleteState(agent, curr_state) ) {
        mNodeState[agent] = next_state;
        mTransitTime[agent] = QDateTime::currentMSecsSinceEpoch();
    }
}

void CScenario::updateTrajectory()
{
    // check next state
    QMap<IAgent*, TRANS_STATE>::const_iterator iter;
    for (iter = mNodeState.begin(); iter != mNodeState.end(); ++iter) {
        IAgent* agent = iter.key();
        TRANS_STATE state = iter.value();
        QGeoCoordinate llhPos;

        switch (state) {
        case SELECTED :
            changeState(agent, state, ARM);
            break;
        case ARM :
            mInitPos[agent] = cpos_from_LLH(agent);
            mStepPos[agent].setX(mInitPos[agent].x());
            mStepPos[agent].setY(mInitPos[agent].y());
            agent->cmd("ARM");
            changeState(agent, state, TAKEOFF);
            break;
        case TAKEOFF :
            agent->cmd("TAKEOFF", 1.0, HEADING);
            changeState(agent, state, UP);
            break;
        case UP :
            mStepPos[agent].setZ(mTargetPos[agent].z() + 2.0);
            llhPos = NEU2LLH(mStepPos[agent]);
            agent->cmd("MOVE", llhPos.latitude(), llhPos.longitude(), llhPos.altitude(), HEADING);
            changeState(agent, state, MOVE);
            break;
        case MOVE :
            mStepPos[agent].setX(mTargetPos[agent].x());
            mStepPos[agent].setY(mTargetPos[agent].y());
            llhPos = NEU2LLH(mStepPos[agent]);
            agent->cmd("MOVE", llhPos.latitude(), llhPos.longitude(), llhPos.altitude(), HEADING);
            changeState(agent, state, DOWN);
            break;
        case DOWN :
            mStepPos[agent].setZ(mTargetPos[agent].z());
            llhPos = NEU2LLH(mStepPos[agent]);
            agent->cmd("MOVE", llhPos.latitude(), llhPos.longitude(), llhPos.altitude(), HEADING);
            changeState(agent, state, KEEP);
            break;
        case KEEP :
            agent->cmd("START_GST");
            changeState(agent, state, UP_BACK);
            break;
        case UP_BACK :
            mStepPos[agent].setZ(mTargetPos[agent].z() + 1.0);
            llhPos = NEU2LLH(mStepPos[agent]);
            agent->cmd("STOP_GST");
            agent->cmd("MOVE", llhPos.latitude(), llhPos.longitude(), llhPos.altitude(), HEADING);
            changeState(agent, state, MOVE_BACK);
            break;
        case MOVE_BACK :
            mStepPos[agent].setX(mInitPos[agent].x());
            mStepPos[agent].setY(mInitPos[agent].y());
            llhPos = NEU2LLH(mStepPos[agent]);
            agent->cmd("MOVE", llhPos.latitude(), llhPos.longitude(), llhPos.altitude(), HEADING);
            changeState(agent, state, DOWN_BACK);
            break;
        case DOWN_BACK :
            mStepPos[agent].setZ(mInitPos[agent].z() + 0.5);
            llhPos = NEU2LLH(mStepPos[agent]);
            agent->cmd("MOVE", llhPos.latitude(), llhPos.longitude(), llhPos.altitude(), HEADING);
            changeState(agent, state, LAND);
            break;
        case LAND :
            mStepPos[agent].setZ(-2);
            llhPos = NEU2LLH(mStepPos[agent]);
            agent->cmd("LANDING", HEADING);
            changeState(agent, state, DISARM);
            break;
        case DISARM :
            agent->cmd("DISARM");
            changeState(agent, state, TERMINATED);
            break;
        case TERMINATED :
            mTransitTime[agent] = QDateTime::currentMSecsSinceEpoch();
            initAgentState(agent);
            break;
        default :
            qDebug("ERROR: Wrong STATE (%d)", state);
            break;
        }

        // check battery status and decide to change

        // find low battery agents
        QList<QVector3D> targetList;
        foreach (IAgent* agent, mManager->agents()) {
            if ( mNodeState[agent] == KEEP && isLowBat(agent) && mShiftAgent[agent] == nullptr) {
                targetList.append(mOriginTarget[agent]);
            }
        }

        // assign agents for low battery agents
        if ( targetList.size() > 0 ) {
            assignTarget(targetList);
        }

        // update assignment target
        updateAssignment();

    }
}
