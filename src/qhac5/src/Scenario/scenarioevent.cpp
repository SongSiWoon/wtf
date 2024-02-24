#include "scenarioevent.h"
#include <QDebug>

CScenarioEvent::CScenarioEvent()
{
}

CScenarioEvent::CScenarioEvent(int aTargetID, int aTime, CScenarioEvent::Type aType)
{
    mTargetID = aTargetID;
    mTime = aTime;
    mType = aType;
}

CScenarioEvent::CScenarioEvent(int aTargetID, int aTime, CScenarioEvent::Type aType, double aX, double aY, double aZ, double aHeading)
{
    mTargetID = aTargetID;
    mTime = aTime;
    mType = aType;
    mPosX = aX;
    mPosY = aY;
    mPosZ = aZ;
    mHeading = aHeading;
}

CScenarioEvent::CScenarioEvent(int aTargetID, int aTime, CScenarioEvent::Type aType, float aPitch, float aYaw, float aRoll)
{
    mTargetID = aTargetID;
    mTime = aTime;
    mType = aType;
    mCalibPitch = aPitch;
    mCalibYaw = aYaw;
	mCalibRoll = aRoll;
}

CScenarioEvent::CScenarioEvent(int aTargetID, int aTime, CScenarioEvent::Type aType, int aShape, int aR, int aG, int aB)
{
	mTargetID = aTargetID;
	mTime = aTime;
	mType = aType;
	mShape = aShape;
	mR = aR;
	mG = aG;
	mB = aB;
}

int CScenarioEvent::targetID() const
{
    return mTargetID;
}

int CScenarioEvent::time() const
{
    return mTime;
}

CScenarioEvent::Type CScenarioEvent::type() const
{
    return mType;
}

double CScenarioEvent::posX() const
{
    return mPosX;
}

double CScenarioEvent::posY() const
{
    return mPosY;
}

double CScenarioEvent::posZ() const
{
    return mPosZ;
}

double CScenarioEvent::heading() const
{
    return mHeading;
}

float CScenarioEvent::pitch() const
{
    return mCalibPitch;
}

float CScenarioEvent::roll() const
{
    return mCalibRoll;
}

float CScenarioEvent::yaw() const
{
    return mCalibYaw;
}

int CScenarioEvent::aniDuration() const
{
    return mAniDuration;
}

void CScenarioEvent::show() const
{
    qDebug("=== SCENARIO EVENT === ");
    qDebug("ID : %d", mTargetID);
    qDebug("Time : %d ", mTime);
    qDebug("Type : %d ", mType);
    qDebug("X,Y,Z : %f %f %f ", mPosX, mPosY, mPosZ );
    qDebug("Heading : %f " , mHeading);
}

void CScenarioEvent::showShort() const
{
    qDebug("%d,%d,%d", mTime, mTargetID, mType);
}

bool CScenarioEvent::lessthan(const CScenarioEvent &aC1, const CScenarioEvent &aC2)
{
    return  aC1.time() < aC2.time();
}


/////////////////////////////////////////////////////////////////////////////////////


IScenarioEvent::IScenarioEvent(int aTargetID, int aTime, IScenarioEvent::EventType aEventType)
{
	mTargetID = aTargetID;
	mTime = aTime;
	mEventType = aEventType;
}

void IScenarioEvent::show() const
{
	qDebug("=== SCENARIO EVENT === ");
	qDebug("ID : %d", mTargetID);
	qDebug("Time : %d ", mTime);
	qDebug("Type : %d ", mEventType);
}

bool IScenarioEvent::lessthan(const IScenarioEvent* aC1, const IScenarioEvent* aC2)
{
	return  aC1->time() < aC2->time();
}

CMoveEvent::CMoveEvent(int aTargetID, int aTime, EventType aEventType, double aX, double aY, double aZ, double aHeading)
	: IScenarioEvent(aTargetID, aTime, aEventType)
{
	mPosX = aX;
	mPosY = aY;
	mPosZ = aZ;
	mHeading = aHeading;
}

CLEDEvent::CLEDEvent(int aTargetID, int aTime, EventType aEventType, quint8 aType, quint8 aR, quint8 aG, quint8 aB, quint8 aBright, quint8 aSpeed)
	: IScenarioEvent(aTargetID, aTime, aEventType)
{
	mType = aType;
	mR = aR;
	mG = aG;
	mB = aB;
	mBright = aBright;
	mSpeed = aSpeed;
}

CTakeoffEvent::CTakeoffEvent(int aTargetID, int aTime, IScenarioEvent::EventType aEventType, double aHeight)
	: IScenarioEvent(aTargetID, aTime, aEventType)
{
	mHeight = aHeight;
}

CCmdEvent::CCmdEvent(int aTargetID, int aTime, IScenarioEvent::EventType aEventType)
    : IScenarioEvent(aTargetID, aTime, aEventType)
{
}
