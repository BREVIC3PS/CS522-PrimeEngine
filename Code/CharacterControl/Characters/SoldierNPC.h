#ifndef _CHARACTER_CONTROL_SOLDIER_NPC_
#define _CHARACTER_CONTROL_SOLDIER_NPC_

#include "PrimeEngine/Events/Component.h"
#include "PrimeEngine/Scene/MeshInstance.h"

#include "../Events/Events.h"
using namespace PE;
using namespace PE::Components;
struct PE::Components::MeshInstance;
namespace CharacterControl{

namespace Components {

struct SoldierNPC : public PE::Components::Component
{
	PE_DECLARE_CLASS(SoldierNPC);

	SoldierNPC(PE::GameContext &context, PE::MemoryArena arena, PE::Handle hMyself, Events::Event_CreateSoldierNPC *pEvt);

	virtual void addDefaultComponents();

	struct SoldierNPCMovementSM* MovementSM;
	MeshInstance* pMeshIns;

	
};
}; // namespace Components
}; // namespace CharacterControl
#endif

