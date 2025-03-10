
#include "StandardGameEvents.h"

#include "../Lua/LuaEnvironment.h"

namespace PE {
namespace Events {

	PE_IMPLEMENT_CLASS1(Event_FLY_CAMERA, Event);
	PE_IMPLEMENT_CLASS1(Event_ROTATE_CAMERA, Event);
	PE_IMPLEMENT_CLASS1(Event_START_SIMULATION, Event);
	PE_IMPLEMENT_CLASS1(Event_MOVE_UP, Event);
	PE_IMPLEMENT_CLASS1(Event_MOVE_DOWN, Event);
	PE_IMPLEMENT_CLASS1(Event_MOVE_LEFT, Event);
	PE_IMPLEMENT_CLASS1(Event_MOVE_RIGHT, Event);

};
};
