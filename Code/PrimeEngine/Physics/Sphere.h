#pragma once
#ifndef SPHERE
#define SPHERE

#include "PhysicsShape.h"

namespace PE {
	namespace Components {
		struct Sphere : public PhysicsShape
		{
			PE_DECLARE_CLASS(Sphere);

			void addDefaultComponents();

			Sphere(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself);

			virtual void DebugRender() override;

			virtual AABB calculateAABB() override;

			virtual void UpdatePosition(float deltaTime) override;

			virtual void UpdateRotation(float deltaTime) override;

			virtual Vector3 ComputeCollisionNormal(const Vector3& collisionPoint) override;

			virtual void UpdateInverseInertiaTensorWorld() override;

			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_CALCULATE_TRANSFORMATIONS);
			void do_CALCULATE_TRANSFORMATIONS(Events::Event* pEvt);

			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_PHYSICS_START);
			virtual void do_PHYSICS_START(Events::Event* pEvt) override;

			virtual ~Sphere() {}

			float radius = 2;
			Vector3 center;
			Vector3 TransformedCenter;

		};
	}
}

#endif