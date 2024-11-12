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
//#include "PrimeEngine/Scene/Mesh.h"
#include "Sphere.h"
#include "Box.h"


// definitions of standard game events. the events that any game could potentially use
#include "PrimeEngine/Events/StandardGameEvents.h"
#include <vector>

// Sibling/Children includes
namespace PE {
	namespace Components {

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

			void SolveContacts(float deltaTime);
			
			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_START_SIMULATION);
			void do_START_SIMULATION(PE::Events::Event* pEvt);

			/*PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_CALCULATE_TRANSFORMATIONS);
			virtual void do_CALCULATE_TRANSFORMATIONS(Events::Event* pEvt);*/

			// Component ------------------------------------------------------------

			virtual void addDefaultComponents();
			// Individual events -------------------------------------------------------

		private:

			float SphereRadius = .65f;
			bool boxCollected = false;
			
			// Define gravity and threshold constants
			const float gravity = -9.81f; // Gravity acceleration
			const float groundThreshold = 0.1f; // Threshold distance to consider the soldier is on the ground
			std::vector<ContactManifold> contactManifolds;

			bool CheckSphereCollision(Sphere* sphere1, Sphere* sphere2, Vector3& collisionPoint, float& PenetrationDepth);
			bool CheckBoxCollision(Box* box1, Box* box2, Vector3& collisionPoint, float& PenetrationDepth);
			bool CheckSphereBoxCollision(Sphere* sphere, Box* box, Vector3& collisionPoint, float& PenetrationDepth);
			void updateCollisions(const float& deltaTime);
			void ResolveCollision(PhysicsShape* shapeA, PhysicsShape* shapeB, const Vector3& collisionPoint, float deltaTime);
			void ResolveCollisionAngular(PhysicsShape* shapeA, PhysicsShape* shapeB, const Vector3& collisionPoint,float PenetrationDepth, float deltaTime);
			void CollectContact(PhysicsShape* shapeA, PhysicsShape* shapeB, const Vector3& collisionPoint, float PenetrationDepth, float deltaTime);
			ContactManifold* FindOrCreateContactManifold(PhysicsShape* shapeA, PhysicsShape* shapeB);
			void InitializeContactPoints(ContactManifold& manifold);
			void UpdateContactManifolds();
			void SolveContact(PhysicsShape* shapeA, PhysicsShape* shapeB, ContactPoint& contact, float deltaTime);
			bool AreShapesInContact(PhysicsShape* shapeA, PhysicsShape* shapeB);
			//Vector3 ClosestPointOnBoundingBox(const Vector3& point, const BoundingBox& box);
			//float DistanceBetweenSphereAndBoundingBox(const Sphere& sphere, const BoundingBox& box);
			//bool RayIntersectsBoundingBox(const Ray& ray, const BoundingBox& box, float& hitDistance);
			//bool RayIntersectsOBB(const Ray& ray, const BoundingBox& obb, const Matrix4x4& obbTransform, float& hitDistance);
			/*bool IsSoldierOnGround(const Vector3& soldierPos, float& groundHeight);
			void CollectGroundBoundingBoxes();*/
		};




	}; // namespace Components
}; // namespace PE
#endif
