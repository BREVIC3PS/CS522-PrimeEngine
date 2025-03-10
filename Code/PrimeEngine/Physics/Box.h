#pragma once
#ifndef BOX
#define BOX

#include "PhysicsShape.h"

namespace PE {
	namespace Components {
		struct Box : public PhysicsShape
		{
			PE_DECLARE_CLASS(Box);

			void addDefaultComponents();

			virtual void DebugRender() override;

			virtual AABB calculateAABB() override;

			virtual void UpdatePosition(float deltaTime) override;

			virtual void UpdateRotation(float deltaTime) override;

			virtual Vector3 ComputeCollisionNormal(const Vector3& collisionPoint) override;

			virtual Vector3 GetSupport(Vector3& dir) override;

			virtual void UpdateInverseInertiaTensorWorld() override;

			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_CALCULATE_TRANSFORMATIONS);
			void do_CALCULATE_TRANSFORMATIONS(Events::Event* pEvt);

			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_PHYSICS_START);
			virtual void do_PHYSICS_START(Events::Event* pEvt);

			Box(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself);
			Box(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself, Vector3 _Max, Vector3 _Min, Vector3 _Corners[8]);
			virtual ~Box() {}

			Vector3 Corners[8];
			Vector3 Min; // Minimum corner (Min_X, Min_Y, Min_Z)
			Vector3 Max; // Maximum corner (Max_X, Max_Y, Max_Z)

			Vector3 TransformedCorners[8];
			Vector3 TransformedMin; // Minimum corner (Min_X, Min_Y, Min_Z)
			Vector3 TransformedMax; // Maximum corner (Max_X, Max_Y, Max_Z)

			int subdivisions = 0;

			int edges[12][2] = {
				{ 0, 1 },{ 1, 2 },{ 2, 3 },{ 3, 0 }, // bottom
				{ 4, 5 },{ 5, 6 },{ 6, 7 },{ 7, 4 }, // top
				{ 0, 4 },{ 1, 5 },{ 2, 6 },{ 3, 7 }  // connection
			};
		};
	}
}

#endif // !BOX
