#pragma once
#ifndef _CHARACTER_CONTROL_TARGET_
#define _CHARACTER_CONTROL_TARGET_
#include "PrimeEngine/Events/Component.h"
#include "PrimeEngine/Scene/SceneNode.h"
#include "PrimeEngine/Math/Matrix4x4.h"
#include "CharacterControl/Characters/SoldierNPC.h"

namespace CharacterControl {
	namespace Events {
		/*struct Event_CREATE_TARGET : public PE::Events::Event
		{
			PE_DECLARE_CLASS(Event_CREATE_TARGET);

			Matrix4x4 m_base;
		};*/
	}
	namespace Components {

		struct Target : public PE::Components::Component
		{
			PE_DECLARE_CLASS(Target);

			Target(PE::GameContext& context, PE::MemoryArena arena, PE::Handle hMyself, Vector3 &pos, Events::Event_CreateSoldierNPC* pEvt);

			virtual void addDefaultComponents();

			Matrix4x4 m_base;

			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_UPDATE)
				virtual void do_UPDATE(PE::Events::Event* pEvt);

			PE::Components::SceneNode* getParentsSceneNode();

			float GameTime = 0;

			Vector3 BeginPosition;
		};
	}; // namespace Components
}; // namespace CharacterControl
#endif