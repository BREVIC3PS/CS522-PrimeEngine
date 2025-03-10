#ifndef __PYENGINE_STANDARD_GAME_EVENTS_H__
#define __PYENGINE_STANDARD_GAME_EVENTS_H__

// API Abstraction
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

// Outer-Engine includes

// Inter-Engine includes
#include "StandardEvents.h"


// Sibling/Children includes

namespace PE {

namespace Events {

struct Event_FLY_CAMERA : public Event {
	PE_DECLARE_CLASS(Event_FLY_CAMERA);

	Event_FLY_CAMERA(){}
	virtual ~Event_FLY_CAMERA(){}

	Vector3 m_relativeMove;
};

struct Event_ROTATE_CAMERA : public Event {
	PE_DECLARE_CLASS(Event_ROTATE_CAMERA);

	Event_ROTATE_CAMERA() {}
	virtual ~Event_ROTATE_CAMERA(){}

	Vector3 m_relativeRotate; //2D screenspace rotate
};

struct Event_MOVE_UP : public Event {
	PE_DECLARE_CLASS(Event_MOVE_UP);

	Event_MOVE_UP() {}
	virtual ~Event_MOVE_UP() {}

	Vector3 m_relativeMove;
};

struct Event_MOVE_DOWN : public Event {
	PE_DECLARE_CLASS(Event_MOVE_DOWN);

	Event_MOVE_DOWN() {}
	virtual ~Event_MOVE_DOWN() {}

	Vector3 m_relativeMove;
};

struct Event_MOVE_LEFT : public Event {
	PE_DECLARE_CLASS(Event_MOVE_LEFT);

	Event_MOVE_LEFT() {}
	virtual ~Event_MOVE_LEFT() {}

	Vector3 m_relativeMove;
};

struct Event_MOVE_RIGHT : public Event {
	PE_DECLARE_CLASS(Event_MOVE_RIGHT);

	Event_MOVE_RIGHT() {}
	virtual ~Event_MOVE_RIGHT() {}

	Vector3 m_relativeMove;
};

struct Event_START_SIMULATION : public Event {
	PE_DECLARE_CLASS(Event_START_SIMULATION);

	Event_START_SIMULATION() {}
	virtual ~Event_START_SIMULATION() {}
};

}; // namespace Events
}; // namespace PE

#endif
