#ifndef SCENARIOEVENT_H
#define SCENARIOEVENT_H

#include <QtGlobal>

class IScenarioEvent
{
public:
	enum EventType {
        ARM,
        DISARM,
        OFFBOARD,
        MANUAL,
		TAKEOFF,
		LANDING,
		MOVE,
		ROTATE,
		ANIMATION,
		START_MUSIC,
		STOP_MUSIC,
		CALIB,
		FOLLOW,
		LANDING_ON,
		LED
	};

public:
	IScenarioEvent(int aTargetID, int aTime, EventType aEventType);

public:
	EventType type() const { return mEventType; }
	int time() const { return mTime; }
	int targetID() const { return mTargetID; }

	void show() const;
	static bool lessthan (const IScenarioEvent* aC1, const IScenarioEvent* aC2);

private:
	EventType					mEventType;
	int                         mTime;          // launch time (msec)
	int                         mTargetID;
};

class CCmdEvent : public IScenarioEvent
{
public:
    CCmdEvent(int aTargetID, int aTime, EventType aEventType);
};

class CTakeoffEvent : public IScenarioEvent
{
public:
	CTakeoffEvent(int aTargetID, int aTime, EventType aEventType, double aHeight);

public:
	double height() { return mHeight; }

private:
	double                      mHeight;
};

class CMoveEvent : public IScenarioEvent
{
public:
	CMoveEvent(int aTargetID, int aTime, EventType aEventType, double aX, double aY, double aZ, double aHeading);

public:
	double x() { return mPosX; }
	double y() { return mPosY; }
	double z() { return mPosZ; }
	double heading() { return mHeading; }

private:
	double                      mPosX;
	double                      mPosY;
	double                      mPosZ;
	double                      mHeading;
};

class CLEDEvent : public IScenarioEvent
{
public:
	CLEDEvent(int aTargetID, int aTime, EventType aEventType, quint8 aType, quint8 aR, quint8 aG, quint8 aB, quint8 aBright, quint8 aSpeed=0);

public:
	quint8 type() { return mType; }
	quint8 r() { return mR; }
	quint8 g() { return mG; }
	quint8 b() { return mB; }
	quint8 bright() { return mBright; }
	quint8 speed() {return mSpeed; }

private:
	quint8				mType;
	quint8				mR;
	quint8				mG;
	quint8				mB;
	quint8				mBright;
	quint8				mSpeed;
};



/////////////////////////////////////////////////////////////////////////////////////////////
/// DEPRECATED
/////////////////////////////////////////////////////////////////////////////////////////////

class CScenarioEvent
{
public:
    enum Type {
        TAKEOFF,
        LANDING,
        MOVE,
        ROTATE,
        ANIMATION,
        START_MUSIC,
        STOP_MUSIC,
        CALIB,
        FOLLOW,
		LANDING_ON,
		LED
    };

public:
    CScenarioEvent();
    CScenarioEvent(int aTargetID, int aTime, CScenarioEvent::Type aType);
    CScenarioEvent(int aTargetID, int aTime, CScenarioEvent::Type aType, double aX, double aY, double aZ, double aHeading);
    CScenarioEvent(int aTargetID, int aTime, CScenarioEvent::Type aType, float aPitch, float aYaw, float aRoll);
	CScenarioEvent(int aTargetID, int aTime, CScenarioEvent::Type aType, int aShape, int aR, int aG, int aB);

public:

public:
    int targetID() const;
    int time() const;
    CScenarioEvent::Type type() const;
    double posX() const;
    double posY() const;
    double posZ() const;
    double heading() const;


    float pitch() const;
    float roll() const;
    float yaw() const;

    int aniDuration() const;
    void show() const;
    void showShort() const;

private:
    int                         mTargetID;
    int                         mTime;          // launch time (msec)
    CScenarioEvent::Type        mType;

    double                      mPosX;          // for MOVE
    double                      mPosY;          // for MOVE
    double                      mPosZ;          // for MOVE
    double                      mHeading;

    float                       mCalibPitch;
    float                       mCalibYaw;
    float                       mCalibRoll;

    int                         mAniDuration;

	int							mShape;
	int							mR;
	int							mG;
	int							mB;


public:
    static bool lessthan (const CScenarioEvent& aC1, const CScenarioEvent& aC2);

};

#endif // SCENARIOEVENT_H
