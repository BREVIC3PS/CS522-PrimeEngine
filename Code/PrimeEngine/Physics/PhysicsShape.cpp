#include "PhysicsShape.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"
#include "PrimeEngine/Events/StandardEvents.h"
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"
#include "PrimeEngine/Scene/DebugRenderer.h"

using namespace PE::Events;

namespace PE
{
	namespace Components
	{
		PE_IMPLEMENT_CLASS1(PhysicsShape, Component);
		PhysicsShape::PhysicsShape(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself)
			:Component(context, arena, hMyself),
			mass(1.0f),
			velocity(Vector3(0, 0, 0)),
			angularVelocity(Vector3(0,0,0)),
			acceleration(Vector3(0, 0, 0)),
			force(Vector3(0, 0, 0)),
			restitution(0.5f), 
			friction(0.5f)     
		{
		}

		void PhysicsShape::OnOverlap(PhysicsShape* OtherShape, Vector3 CollidePoint, const float& deltaTime)
		{
			if (!EnablePhysics)return;
			
				
			DebugRenderColor = Vector3((rand() % 255) / 255.0f, (rand() % 255) / 255.0f, (rand() % 255) / 255.0f);
				
			
		}

		void PhysicsShape::OnOverlap(PhysicsShape* OtherShape)
		{
			if (OtherShape->PhysicsShapeType != this->PhysicsShapeType)
			{
				DebugRenderColor = Vector3(1.f, 0.f, 0.f);
			}
		}

		void PhysicsShape::DebugRender()
		{
			AABB myAABB = getAABB();
			Vector3 AABBLineMin = myAABB.min;
			Vector3 AABBLineMax = myAABB.max;
			Vector3 AABBLine[4] = { AABBLineMin ,Vector3(1,1,1),AABBLineMax ,Vector3(1,1,1) };
			DebugRenderer::Instance()->createLineMesh(
				false,
				m_worldTransform,
				&AABBLine[0].m_x,
				2,
				0.f);
		}

		AABB PhysicsShape::getAABB()
		{
			m_cachedAABB = calculateAABB();
			if (m_isTransformDirty)
			{
			}
			return m_cachedAABB;
		}

		void PhysicsShape::addDefaultComponents()
		{
			PE_REGISTER_EVENT_HANDLER(Events::Event_MOVE, PhysicsShape::do_MOVE);
			PE_REGISTER_EVENT_HANDLER(Events::Event_PHYSICS_START, PhysicsShape::do_PHYSICS_START);
			PE_REGISTER_EVENT_HANDLER(Events::Event_CALCULATE_TRANSFORMATIONS, PhysicsShape::do_CALCULATE_TRANSFORMATIONS);
		}

		void PhysicsShape::SetPosition(Vector3& newPosition)
		{
			Matrix4x4& m = m_base;
			m.setPos(newPosition);
			m_isTransformDirty = true;
		}

		void ResolveCollision(PhysicsShape* shapeA, PhysicsShape* shapeB, const Vector3& collisionPoint, const Vector3& collisionNormal, const float& deltaTime)
		{
			
		}



		void PhysicsShape::ApplyForce(const Vector3& newForce)
		{
			force += newForce;
		}

		void PhysicsShape::do_MOVE(Events::Event* pEvt)
		{
			Events::Event_MOVE* pRealEvent = (Events::Event_MOVE*)(pEvt);
			Matrix4x4& m = m_base;
			m.setPos(m.getPos() + pRealEvent->m_dir);
			m_isTransformDirty = true;
		}

		void PhysicsShape::do_CALCULATE_TRANSFORMATIONS(Events::Event* pEvt)
		{
			Handle hParentPS = Component::getFirstParentByType<PhysicsShape>();
			if (hParentPS.isValid())
			{
				Matrix4x4 tmp = hParentPS.getObject<PhysicsShape>()->m_worldTransform;
				if (m_inheritPositionOnly)
				{
					Vector3 pos = tmp.getPos();
					tmp.loadIdentity();
					tmp.setPos(pos);
				}
				m_worldTransform = tmp * m_base;
			}
			else
			{
				m_worldTransform = m_base;
			}
			UpdateInverseInertiaTensorWorld();
		}

		void PhysicsShape::do_PHYSICS_START(Events::Event* pEvt)
		{
			if (!EnablePhysics || !IsDynamic)return;

			Event_PHYSICS_START* pRealEvent = (Event_PHYSICS_START*)(pEvt);
			float deltaTime = pRealEvent->m_frameTime;

			if (mass > 0 && EnableGravity && !isOnGround)
			{
				Vector3 gravityForce = Vector3(0, -9.81f * mass, 0); // 重力加速度为 9.81 m/s^2

				Vector3 torque = Vector3(0,0,0).crossProduct(gravityForce);

				angularVelocity += inverseInertiaTensorWorld * torque * deltaTime;

				force += gravityForce;
			}

			ApplyForce(force);


			// 计算加速度：a = F / m
			acceleration = force / mass;

			// 更新速度：v = v0 + a * dt
			velocity += acceleration * deltaTime;


			// 更新位置：x = x0 + v * dt
			// 对于 Sphere 和 Box，需要更新它们的中心或位置
			UpdatePosition(deltaTime);

			// 清除作用力，准备下一帧
			force = Vector3(0, 0, 0);

			UpdateRotation(deltaTime);

			// 重置接触状态
			isOnGround = false;
		}

	}
}

