#pragma once
#ifndef PHYSICS_SHAPE
#define PHYSICS_SHAPE

#include <assert.h>
#include "../Events/Component.h"
#include <PrimeEngine/Math/Vector3.h>
#include <PrimeEngine/Math/Matrix4x4.h>
#include <vector>
//#include <PrimeEngine/Scene/MeshInstance.h>

#define EPSILON 1e-6

namespace PE {
	namespace Components {

		enum ShapeType
		{
			ST_Shpere,
			ST_Box
		};

		struct AABB
		{
		public:
			Vector3 min; // AABB的最小点
			Vector3 max; // AABB的最大点

			AABB() {}

			AABB(const Vector3& minPoint, const Vector3& maxPoint)
				: min(minPoint), max(maxPoint) {}

			// 检查两个AABB是否有重叠
			inline bool Intersects(const AABB& other) const
			{
				return (min.m_x <= other.max.m_x && max.m_x >= other.min.m_x) &&
					(min.m_y <= other.max.m_y && max.m_y >= other.min.m_y) &&
					(min.m_z <= other.max.m_z && max.m_z >= other.min.m_z);
			}
		};

		struct ContactPoint {
			Vector3 position;
			Vector3 normal;
			float penetrationDepth;
			float accumulatedNormalImpulse;
			float accumulatedTangentImpulse;
			Vector3 rA; // 从 shapeA 质心到接触点的向量
			Vector3 rB; // 从 shapeB 质心到接触点的向量
		};



		struct PhysicsShape : public Component
		{
			PE_DECLARE_CLASS(PhysicsShape);


			// Constructor -------------------------------------------------------------
			// when created in a scene node
			PhysicsShape(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself);
			virtual ~PhysicsShape() {}

			virtual void OnOverlap(PhysicsShape* OtherShape, Vector3 CollidePoint,const float& deltaTime);

			virtual void OnOverlap(PhysicsShape* OtherShape);

			virtual void DebugRender();

			virtual AABB getAABB();

			virtual void addDefaultComponents();

			virtual void SetPosition(Vector3& newPosition);
			Vector3 GetPosition() { return m_base.getPos(); }

			virtual void UpdatePosition(float deltatime) = 0;

			virtual void UpdateRotation(float deltatime) = 0;

			virtual void UpdateInverseInertiaTensorWorld() = 0;

			virtual Vector3 ComputeCollisionNormal(const Vector3& collisionPoint) = 0;

			//virtual void ResolveCollision(PhysicsShape* shapeA, PhysicsShape* shapeB, const Vector3& collisionPoint, const Vector3& collisionNormal, const float& deltaTime);

			void ApplyForce(const Vector3& newForce);

			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_MOVE);
			virtual void do_MOVE(Events::Event* pEvt);

			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_CALCULATE_TRANSFORMATIONS);
			virtual void do_CALCULATE_TRANSFORMATIONS(Events::Event* pEvt);

			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_PHYSICS_START);
			virtual void do_PHYSICS_START(Events::Event* pEvt);

			// Component ------------------------------------------------------------
			//virtual void handleEvent(Events::Event *pEvt) = 0;

			// Individual events -------------------------------------------------------
			Matrix4x4 m_base; // local transform
			Matrix4x4 m_worldTransform;
			bool m_inheritPositionOnly = true;

			struct MeshInstance* m_meshIns;

			virtual AABB calculateAABB() = 0;

			bool m_isTransformDirty = true; // 变换脏标记
			AABB m_cachedAABB;              // 缓存的 AABB

			Vector3 DebugRenderColor;

			ShapeType PhysicsShapeType;

			bool ReadyToCollide = false;

			bool EnableGravity = true;
			bool EnablePhysics = true;
			bool EnableCollision = true;
			bool IsDynamic = true;

			//Physics porperty
			float mass;              
			Vector3 velocity;   
			Vector3 acceleration;    
			Vector3 force;           

			float restitution;       
			float friction;         
			Vector3 accumulatedNormalImpulse = Vector3(0,0,0); // 累积的法向冲量
			bool isOnGround = false;                  // 是否与地面接触

			Vector3 angularVelocity;
			Matrix3x3 inverseInertiaTensorWorld;
			Matrix3x3 inertiaTensorLocal;
			Matrix3x3 inverseInertiaTensorLocal;

			float width = 0;
			float height = 0;
			float depth = 0;

			float Ixx = 0;
			float Iyy = 0;
			float Izz = 0;

		};

		struct ContactManifold {
			PhysicsShape* shapeA;
			PhysicsShape* shapeB;
			std::vector<ContactPoint> contacts;
		};
	}; // namespace Components
}; // namespace PE

#endif // !PHYSICS_SHAPE
