#ifndef __PYENGINE_2_0_PHYSICSMANAGER_H__
#define __PYENGINE_2_0_PHYSICSMANAGER_H__

#define NOMINMAX
// API Abstraction
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

// Outer-Engine includes
#include <assert.h>
// Inter-Engine includes
#include "PrimeEngine/MemoryManagement/Handle.h"
#include "PrimeEngine/PrimitiveTypes/PrimitiveTypes.h"
#include "../Events/Component.h"
#include "../Utils/Array/Array.h"
#include "../Scene/RootSceneNode.h"
#include "../Scene/CameraManager.h"

#include "PrimeEngine/Events/Component.h"
#include "PrimeEngine/Scene/Mesh.h"


// definitions of standard game events. the events that any game could potentially use
#include "PrimeEngine/Events/StandardGameEvents.h"
#include <vector>

// Sibling/Children includes
namespace PE {
	namespace Components {

		struct Sphere
		{
			Vector3 Center;
			float Radius;
		};

		struct Ray
		{
			Vector3 origin;
			Vector3 direction;
		};

		struct PhysicsManager : public Component
		{
			PE_DECLARE_CLASS(PhysicsManager);


			// Singleton ------------------------------------------------------------------

			// Constructor -------------------------------------------------------------
			PhysicsManager(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself);
			// Methods      ------------------------------------------------------------
			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_PHYSICS_START);
			void do_PHYSICS_START(Events::Event* pEvt);

			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_PRE_RENDER_needsRC);
			void do_PRE_RENDER_needsRC(PE::Events::Event* pEvt);

			// Component ------------------------------------------------------------

			virtual void addDefaultComponents();
			void RenderBoundingBox(PE::Components::Mesh* myMesh, Vector3& Pos, Matrix4x4& transform);
			// Individual events -------------------------------------------------------

		private:

			float SphereRadius = .65f;
			bool boxCollected = false;
			std::vector<BoundingBox> groundBoxes;

			void RenderSphere(const Sphere& sphere, const Matrix4x4& transform);
			Vector3 ClosestPointOnBoundingBox(const Vector3& point, const BoundingBox& box);
			float DistanceBetweenSphereAndBoundingBox(const Sphere& sphere, const BoundingBox& box);
			bool RayIntersectsBoundingBox(const Ray& ray, const BoundingBox& box, float& hitDistance);
			bool RayIntersectsOBB(const Ray& ray, const BoundingBox& obb, const Matrix4x4& obbTransform, float& hitDistance);
			bool IsSoldierOnGround(const Vector3& soldierPos, float& groundHeight);
			void CollectGroundBoundingBoxes();
		};




	}; // namespace Components
}; // namespace PE
#endif
