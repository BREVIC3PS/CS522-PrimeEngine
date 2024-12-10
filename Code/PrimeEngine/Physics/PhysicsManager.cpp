#include "PhysicsManager.h"
#include "PrimeEngine/Events/StandardEvents.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"
//#include <PrimeEngine/Scene/MeshInstance.h>
#include "PrimeEngine/Scene/DebugRenderer.h"

//#include <PrimeEngine/Scene/SkeletonInstance.h>
//#include <CharacterControl/Characters/SoldierNPC.h>
//#include <CharacterControl/Characters/SoldierNPCMovementSM.h>


float Clamp(float value, float minVal, float maxVal)
{
	return std::max(minVal, std::min(value, maxVal));
}

namespace PE {

	namespace Components {

		PE_IMPLEMENT_CLASS1(PhysicsManager, Component);
		using namespace PE::Events;
		//using namespace CharacterControl::Components;


		PhysicsManager::PhysicsManager(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself)
			:Component(context, arena, hMyself)
		{
		}

		void PhysicsManager::do_PHYSICS_START(Events::Event* pEvt)
		{
			Event_PHYSICS_START* pRealEvent = (Event_PHYSICS_START*)(pEvt);

			int iterations = 5;
			float deltaTime = pRealEvent->m_frameTime / iterations;
			for (int j = 0; j < iterations; j++)
			{
				for (int i = 1; i < m_components.m_size; i++)//since index0 is Log component
				{
					PhysicsShape* shape1 = m_components[i].getObject<PhysicsShape>();
					shape1->do_CALCULATE_TRANSFORMATIONS(nullptr);
				}


				updateCollisions(deltaTime, manifolds);

				Resolve(manifolds, deltaTime);


				UpdateShapes(deltaTime, pEvt);

				manifolds.clear();
			}

			//UpdateContactManifolds();

			/*for (auto& manifold : contactManifolds)
			{
				InitializeContactPoints(manifold);
			}*/

			//SolveContacts(pRealEvent->m_frameTime);
			
		}

		void PhysicsManager::do_PRE_RENDER_needsRC(PE::Events::Event* pEvt)
		{
			for (int i = 0; i < m_components.m_size; i++)
			{
				Handle& h = m_components[i];

				PhysicsShape* pShape = h.getObject<PhysicsShape>();
				if (pShape->isInstanceOf<PhysicsShape>())pShape->DebugRender();
			}
		}

		void PhysicsManager::do_START_SIMULATION(PE::Events::Event* pEvt)
		{
			for (int i = 0; i < m_components.m_size; i++)
			{
				Handle& h = m_components[i];

				PhysicsShape* pShape = h.getObject<PhysicsShape>();
				if (pShape->isInstanceOf<Sphere>())
				{
					pShape->EnableGravity = true;
					pShape->EnablePhysics = true;
				}
				if (pShape->isInstanceOf<Box>())
				{
					pShape->EnableGravity = true;
					pShape->EnablePhysics = true;
				}
			}

			
		}

		void PhysicsManager::addDefaultComponents()
		{
			Component::addDefaultComponents();

			PE_REGISTER_EVENT_HANDLER(Event_PHYSICS_START, PhysicsManager::do_PHYSICS_START);
			PE_REGISTER_EVENT_HANDLER(Event_PRE_RENDER_needsRC, PhysicsManager::do_PRE_RENDER_needsRC);
			PE_REGISTER_EVENT_HANDLER(Event_START_SIMULATION, PhysicsManager::do_START_SIMULATION);
			//PE_REGISTER_EVENT_HANDLER(Events::Event_CALCULATE_TRANSFORMATIONS, PhysicsManager::do_CALCULATE_TRANSFORMATIONS);

		}

		

		void PhysicsManager::UpdateShapes(float deltaTime, Events::Event* pEvt)
		{
			for (int i = 1; i < m_components.m_size; i++)//since index0 is Log component
			{
				PhysicsShape* shape1 = m_components[i].getObject<PhysicsShape>();
				if (shape1)
				{
					shape1->Integrate(deltaTime);
				}
			}
		}

		bool PhysicsManager::CheckSphereCollision(Sphere* sphere1, Sphere* sphere2, Vector3& collisionPoint, float& PenetrationDepth)
		{
			// 计算两个球心的方向向量
			Vector3 direction = sphere2->TransformedCenter - sphere1->TransformedCenter;

			// 计算中心点之间的距离
			float distance = direction.length();

			// 检查是否发生碰撞
			float radiusSum = sphere1->radius + sphere2->radius;
			if (distance <= radiusSum)
			{
				// 归一化方向向量
				Vector3 collisionNormal = direction / distance;

				// 计算碰撞点（位于两个球体表面之间）
				collisionPoint = sphere1->TransformedCenter + collisionNormal * sphere1->radius;

				PenetrationDepth = (sphere1->radius + sphere2->radius) - (sphere1->GetPosition() - sphere2->GetPosition()).length();

				return true;
			}
			else
			{
				return false;
			}
		}

		bool PhysicsManager::CheckBoxCollision(Box* box1, Box* box2, Vector3& collisionPoint, float &PenetrationDepth)
		{
			// 检查是否发生碰撞
			bool xOverlap = (box1->TransformedMin.m_x <= box2->TransformedMax.m_x) && (box1->TransformedMax.m_x >= box2->TransformedMin.m_x);
			bool yOverlap = (box1->TransformedMin.m_y <= box2->TransformedMax.m_y) && (box1->TransformedMax.m_y >= box2->TransformedMin.m_y);
			bool zOverlap = (box1->TransformedMin.m_z <= box2->TransformedMax.m_z) && (box1->TransformedMax.m_z >= box2->TransformedMin.m_z);

			if (xOverlap && yOverlap && zOverlap)
			{
				// 计算重叠区域的最小和最大点
				float overlapMinX = std::max(box1->TransformedMin.m_x, box2->TransformedMin.m_x);
				float overlapMaxX = std::min(box1->TransformedMax.m_x, box2->TransformedMax.m_x);

				float overlapMinY = std::max(box1->TransformedMin.m_y, box2->TransformedMin.m_y);
				float overlapMaxY = std::min(box1->TransformedMax.m_y, box2->TransformedMax.m_y);

				float overlapMinZ = std::max(box1->TransformedMin.m_z, box2->TransformedMin.m_z);
				float overlapMaxZ = std::min(box1->TransformedMax.m_z, box2->TransformedMax.m_z);

				// 计算在每个轴上的重叠量
				float overlapX = overlapMaxX - overlapMinX;
				float overlapY = overlapMaxY - overlapMinY;
				float overlapZ = overlapMaxZ - overlapMinZ;

				// 计算穿透深度，取最小的重叠量
				PenetrationDepth = std::min({ overlapX, overlapY, overlapZ });

				// 确定碰撞法线方向，根据最小重叠轴
				Vector3 collisionNormal(0.0f, 0.0f, 0.0f);

				if (PenetrationDepth == overlapX)
				{
					// X轴方向的最小重叠
					if (box1->GetPosition().m_x < box2->GetPosition().m_x)
						collisionNormal = Vector3(-1.0f, 0.0f, 0.0f); // box1 在 box2 的左边
					else
						collisionNormal = Vector3(1.0f, 0.0f, 0.0f);  // box1 在 box2 的右边
				}
				else if (PenetrationDepth == overlapY)
				{
					// Y轴方向的最小重叠
					if (box1->GetPosition().m_y < box2->GetPosition().m_y)
						collisionNormal = Vector3(0.0f, -1.0f, 0.0f); // box1 在 box2 的下面
					else
						collisionNormal = Vector3(0.0f, 1.0f, 0.0f);  // box1 在 box2 的上面
				}
				else // penetrationDepth == overlapZ
				{
					// Z轴方向的最小重叠
					if (box1->GetPosition().m_z < box2->GetPosition().m_z)
						collisionNormal = Vector3(0.0f, 0.0f, -1.0f); // box1 在 box2 的前面
					else
						collisionNormal = Vector3(0.0f, 0.0f, 1.0f);  // box1 在 box2 的后面
				}

				// 计算重叠区域的中心点，作为碰撞点
				collisionPoint.m_x = (overlapMinX + overlapMaxX) * 0.5f;
				collisionPoint.m_y = (overlapMinY + overlapMaxY) * 0.5f;
				collisionPoint.m_z = (overlapMinZ + overlapMaxZ) * 0.5f;

				// 如果需要，将碰撞法线传递出去
				// collisionNormal 参数可以添加到函数参数列表中

				return true;
			}
			else
			{
				return false;
			}
		}

		bool PhysicsManager::CheckSphereBoxCollision(Sphere* sphere, Box* box, Vector3& collisionPoint, float& PenetrationDepth)
		{
			// 获取球心坐标
			Vector3 sphereCenter = sphere->TransformedCenter;

			// 初始化最近点为球心
			Vector3 closestPoint = sphereCenter;

			// 对每个轴，找到球心在盒子范围内的最近点
			if (sphereCenter.m_x < box->TransformedMin.m_x) closestPoint.m_x = box->TransformedMin.m_x;
			else if (sphereCenter.m_x > box->TransformedMax.m_x) closestPoint.m_x = box->TransformedMax.m_x;

			if (sphereCenter.m_y < box->TransformedMin.m_y) closestPoint.m_y = box->TransformedMin.m_y;
			else if (sphereCenter.m_y > box->TransformedMax.m_y) closestPoint.m_y = box->TransformedMax.m_y;

			if (sphereCenter.m_z < box->TransformedMin.m_z) closestPoint.m_z = box->TransformedMin.m_z;
			else if (sphereCenter.m_z > box->TransformedMax.m_z) closestPoint.m_z = box->TransformedMax.m_z;

			// 计算球心与最近点之间的距离平方
			Vector3 diff = sphereCenter - closestPoint;
			float distanceSquared = diff.dotProduct(diff);

			// 检查是否发生碰撞
			if (distanceSquared <= (sphere->radius * sphere->radius))
			{
				float distance = sqrt(distanceSquared);
				// 计算穿透深度
				PenetrationDepth = sphere->radius - distance;

				// 碰撞点即为最近点
				collisionPoint = closestPoint;

				// 如果需要，计算碰撞法线
				// Vector3 collisionNormal = (distance > 0.0f) ? diff / distance : Vector3(1.0f, 0.0f, 0.0f);

				return true;
			}
			else
			{
				return false;
			}
		}

		void PhysicsManager::updateCollisions(const float& deltaTime, std::vector<std::shared_ptr<ContactManifold>>& collisions)
		{
			for (int i = 1; i < m_components.m_size; i++)//since index0 is Log component
			{
				PhysicsShape* shape1 = m_components[i].getObject<PhysicsShape>();

				for (int j = i + 1; j < m_components.m_size; j++)
				{
					PhysicsShape* shape2 = m_components[j].getObject<PhysicsShape>();

					if (!shape1->ReadyToCollide || !shape2->ReadyToCollide)continue;

					if (!shape1->EnableCollision || !shape2->EnableCollision)continue;

					bool collision = false;

					AABB* AABB_shape1 = shape1->getAABB();
					AABB* AABB_shape2 = shape2->getAABB();

					if (!AABB_shape1->Intersects(*AABB_shape2))continue;

					Vector3 CollidePoint;
					float PenetrationDepth;

					//if (shape1->isInstanceOf<Sphere>() && shape2->isInstanceOf<Sphere>())
					//{
					//	collision = CheckSphereCollision(static_cast<Sphere*>(shape1), static_cast<Sphere*>(shape2), CollidePoint, PenetrationDepth);
					//}
					//else if (shape1->isInstanceOf<Box>() && shape2->isInstanceOf<Box>())
					//{
					//	collision = CheckBoxCollision(static_cast<Box*>(shape1), static_cast<Box*>(shape2), CollidePoint, PenetrationDepth);
					//}
					//else if (shape1->isInstanceOf<Sphere>() && shape2->isInstanceOf<Box>())
					//{
					//	collision = CheckSphereBoxCollision(static_cast<Sphere*>(shape1), static_cast<Box*>(shape2), CollidePoint, PenetrationDepth);
					//}
					//else if (shape1->isInstanceOf<Box>() && shape2->isInstanceOf<Sphere>())
					//{
					//	collision = CheckSphereBoxCollision(static_cast<Sphere*>(shape2), static_cast<Box*>(shape1), CollidePoint, PenetrationDepth);
					//}

					//if (collision)
					//{
					//	// 处理碰撞响应
						/*shape1->OnOverlap(shape2, CollidePoint,deltaTime);
						shape2->OnOverlap(shape1, CollidePoint,deltaTime);*/

					//	// 解析碰撞
					//	ResolveCollisionAngular(shape1, shape2, CollidePoint, PenetrationDepth, deltaTime);
					//	//CollectContact(shape1, shape2, CollidePoint, PenetrationDepth, deltaTime);
					//}

					if (shape1->isInstanceOf<Box>() && shape2->isInstanceOf<Box>())
					{
						CollisionDetector CD;
						CD.CollideDetection(shape1, shape2, collisions);

						/*if (collisions.size() > 0)
						{
							shape1->OnOverlap(shape2, collisions[collisions.size() - 1]->contactPoints[0].globalPositionA, deltaTime);
							shape2->OnOverlap(shape1, collisions[collisions.size() - 1]->contactPoints[0].globalPositionB, deltaTime);
						}*/
					}
				}
			}
		}

		void PhysicsManager::ResolveCollisionAngular(PhysicsShape* shapeA, PhysicsShape* shapeB, const Vector3& collisionPoint, float PenetrationDepth, float deltaTime)
		{
			// 计算碰撞法线
			Vector3 normalA = shapeA->ComputeCollisionNormal(collisionPoint);
			Vector3 normalB = shapeB->ComputeCollisionNormal(collisionPoint);
			// 为了统一，选择一个方向作为碰撞法线，这里可以使用 normalA 的反方向
			Vector3 collisionNormal = (normalA + normalB).normalized();

			// 检查两个物体是否启用物理
			bool A_isDynamic = shapeA->EnablePhysics && shapeA->mass > 0 && shapeA->IsDynamic;
			bool B_isDynamic = shapeB->EnablePhysics && shapeB->mass > 0 && shapeB->IsDynamic;

			// 如果两个物体都是静态的，不需要处理
			if (!A_isDynamic && !B_isDynamic)
				return;

			// Vectors from centers of mass to collision point
			Vector3 rA = collisionPoint - shapeA->GetPosition();
			Vector3 rB = collisionPoint - shapeB->GetPosition();

			// Velocities at the point of contact
			Vector3 vA = shapeA->velocity + shapeA->angularVelocity.crossProduct(rA);
			Vector3 vB = shapeB->velocity + shapeB->angularVelocity.crossProduct(rB);

			Vector3 relativeVelocity = vA - vB;
			float velocityAlongNormal = relativeVelocity.dotProduct(collisionNormal);

			// If velocities are separating, no collision response needed
			if (velocityAlongNormal > 0)
				return;

			// 计算恢复系数（取两个物体的最小值）
			float e = std::min(shapeA->restitution, shapeB->restitution);

			// Compute the denominator
			float inverseMassA = A_isDynamic ? (1.0f / shapeA->mass) : 0.0f;
			float inverseMassB = B_isDynamic ? (1.0f / shapeB->mass) : 0.0f;
			
			// 计算 angularTermA
			float angularTermA = 0.0f;
			if (A_isDynamic)
			{
				Vector3 crossA = rA.crossProduct(collisionNormal);
				Vector3 angularInertiaA = shapeA->inverseInertiaTensorWorld * crossA;
				angularTermA = collisionNormal.dotProduct(angularInertiaA.crossProduct(rA));
			}

			// 计算 angularTermB
			float angularTermB = 0.0f;
			if (B_isDynamic)
			{
				Vector3 crossB = rB.crossProduct(collisionNormal);
				Vector3 angularInertiaB = shapeB->inverseInertiaTensorWorld * crossB;
				angularTermB = collisionNormal.dotProduct(angularInertiaB.crossProduct(rB));
			}

			float denominator = inverseMassA + inverseMassB + angularTermA + angularTermB;
			if (denominator == 0.0f)
				return;


			// 计算冲量标量
			float j = -(1 + e) * velocityAlongNormal / denominator;

			// 处理接触状态，防止物体下陷
			const float contactThreshold = 0.01f;
			bool isContact = std::abs(velocityAlongNormal * deltaTime) < contactThreshold;

			if (isContact)
			{
				// 在接触状态下，将恢复系数设为 0，防止反弹
				e = 0.0f;
				// 重新计算冲量标量
				j = -velocityAlongNormal / denominator;
			}

			// 计算冲量向量
			Vector3 impulse = j * collisionNormal;

			// 更新物体速度
			if (A_isDynamic)
			{
				shapeA->velocity += impulse * inverseMassA;
			}

			if (B_isDynamic)
			{
				shapeB->velocity -= impulse * inverseMassB;
			}

			// 应用冲量到角速度
			if (A_isDynamic)
			{
				shapeA->angularVelocity += shapeA->inverseInertiaTensorWorld * (rA.crossProduct(impulse));
			}
			if (B_isDynamic)
			{
				shapeB->angularVelocity -= shapeB->inverseInertiaTensorWorld * (rB.crossProduct(impulse));
			}

			// 处理摩擦力
			// 计算切向速度
			Vector3 tangent = relativeVelocity - (velocityAlongNormal * collisionNormal);
			if (!tangent.isZero())
			{
				tangent.normalize();

				// 计算相对切向速度大小
				float velocityAlongTangent = relativeVelocity.dotProduct(tangent);

				// 计算摩擦冲量分母
				Vector3 tangentCrossA = rA.crossProduct(tangent);
				Vector3 angularInertiaTangentA = shapeA->inverseInertiaTensorWorld * tangentCrossA;
				float angularTermTangentA = tangent.dotProduct(angularInertiaTangentA.crossProduct(rA));

				Vector3 tangentCrossB = rB.crossProduct(tangent);
				Vector3 angularInertiaTangentB = shapeB->inverseInertiaTensorWorld * tangentCrossB;
				float angularTermTangentB = tangent.dotProduct(angularInertiaTangentB.crossProduct(rB));

				float frictionDenominator = inverseMassA + inverseMassB + angularTermTangentA + angularTermTangentB;

				// 计算摩擦冲量
				float mu = sqrt(shapeA->friction * shapeB->friction);
				float jt = -velocityAlongTangent / frictionDenominator;

				// 限制摩擦冲量，满足库仑摩擦模型
				float maxFriction = j * mu;
				if (std::abs(jt) > std::abs(maxFriction))
				{
					jt = (jt > 0 ? 1 : -1) * maxFriction;
				}

				// 计算摩擦冲量向量
				Vector3 frictionImpulse = jt * tangent;

				// 更新物体速度（考虑摩擦力）
				if (A_isDynamic)
				{
					shapeA->velocity += frictionImpulse * inverseMassA;
					shapeA->angularVelocity += shapeA->inverseInertiaTensorWorld * (rA.crossProduct(frictionImpulse));
				}

				if (B_isDynamic)
				{
					shapeB->velocity -= frictionImpulse * inverseMassB;
					shapeB->angularVelocity -= shapeB->inverseInertiaTensorWorld * (rB.crossProduct(frictionImpulse));
				}
			}

			// 更新接触状态
			if (isContact)
			{
				shapeA->isOnGround = true;
				shapeB->isOnGround = true;
			}

			const float percent = 0.8f; // 修正百分比
			const float slop = 0.01f;   // 容忍的穿透量
			Vector3 correction = (std::max(PenetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent * collisionNormal;

			if (A_isDynamic)
			{
				shapeA->SetPosition(shapeA->GetPosition() + correction * inverseMassA);
			}

			if (B_isDynamic)
			{
				shapeB->SetPosition(shapeB->GetPosition() - correction * inverseMassB);
			}
		}
		

		ContactManifold* PhysicsManager::FindOrCreateContactManifold(PhysicsShape* shapeA, PhysicsShape* shapeB)
		{
			// 搜索现有的 ContactManifold
			for (auto& manifold : contactManifolds)
			{
				if ((manifold.colliderA == shapeA && manifold.colliderB == shapeB) ||
					(manifold.colliderA == shapeB && manifold.colliderB == shapeA))
				{
					return &manifold;
				}
			}

			// 未找到，创建新的 ContactManifold
			ContactManifold newManifold;
			newManifold.colliderA = shapeA;
			newManifold.colliderB = shapeB;
			// 可以在这里初始化其他成员变量

			// 将新的 ContactManifold 添加到列表，并返回指针
			contactManifolds.push_back(newManifold);
			return &contactManifolds.back();
		}


		/*
		void PhysicsManager::SolveContact(PhysicsShape* shapeA, PhysicsShape* shapeB, ContactPoint& contact, float deltaTime)
		{
			// 获取物体的属性
			float invMassA = shapeA->IsDynamic ? (1.0f / shapeA->mass) : 0.0f;
			float invMassB = shapeB->IsDynamic ? (1.0f / shapeB->mass) : 0.0f;
			Matrix3x3 invInertiaA = shapeA->inverseInertiaTensorWorld;
			Matrix3x3 invInertiaB = shapeB->inverseInertiaTensorWorld;

			// 相对速度
			Vector3 vA = shapeA->velocity + shapeA->angularVelocity.crossProduct(contact.rA);
			Vector3 vB = shapeB->velocity + shapeB->angularVelocity.crossProduct(contact.rB);
			Vector3 relativeVelocity = vA - vB;

			//contact.normal = Vector3(0, 1, 0);

			// 计算法向速度分量
			float vn = relativeVelocity.dotProduct(contact.normal);

			// 计算 Baumgarte 稳定化项
			const float baumgarte = 0.2f; // 稳定化系数
			float bias = -baumgarte * (1.0f / deltaTime) * std::min(0.0f, contact.penetrationDepth + 0.01f);

			// 计算有效质量
			Vector3 raCrossN = contact.rA.crossProduct(contact.normal);
			Vector3 rbCrossN = contact.rB.crossProduct(contact.normal);
			float angularA = (invInertiaA * (raCrossN)).dotProduct(raCrossN);
			float angularB = (invInertiaB * (rbCrossN)).dotProduct(rbCrossN);
			float effectiveMass = invMassA + invMassB + angularA + angularB;

			// 计算拉格朗日乘子（冲量）
			float lambda = -(vn + bias) / effectiveMass;

			// 累积冲量
			float oldImpulse = contact.accumulatedNormalImpulse;
			contact.accumulatedNormalImpulse = std::max(oldImpulse + lambda, 0.0f);
			float deltaImpulse = contact.accumulatedNormalImpulse - oldImpulse;

			// 应用冲量
			Vector3 impulse = deltaImpulse * contact.normal;

			if (shapeA->IsDynamic)
			{
				shapeA->velocity += impulse * invMassA;
				shapeA->angularVelocity += invInertiaA * (contact.rA.crossProduct(impulse));
			}

			if (shapeB->IsDynamic)
			{
				shapeB->velocity -= impulse * invMassB;
				shapeB->angularVelocity -= invInertiaB * (contact.rB.crossProduct(impulse));
			}

			// 处理摩擦力
			// 计算切向速度
			vA = shapeA->velocity + shapeA->angularVelocity.crossProduct(contact.rA);
			vB = shapeB->velocity + shapeB->angularVelocity.crossProduct(contact.rB);
			relativeVelocity = vA - vB;

			Vector3 tangent = (relativeVelocity - relativeVelocity.dotProduct(contact.normal) * contact.normal).normalized();
			float vt = relativeVelocity.dotProduct(tangent);

			// 计算摩擦力的有效质量
			Vector3 raCrossT = contact.rA.crossProduct(tangent);
			Vector3 rbCrossT = contact.rB.crossProduct(tangent);
			angularA = (invInertiaA * (raCrossT)).dotProduct(raCrossT);
			angularB = (invInertiaB * (rbCrossT)).dotProduct(rbCrossT);
			effectiveMass = invMassA + invMassB + angularA + angularB;

			// 计算摩擦力的冲量
			float maxFriction = shapeA->friction * shapeB->friction * contact.accumulatedNormalImpulse;
			float lambdaFriction = -vt / effectiveMass;

			// 限制摩擦冲量
			float oldFrictionImpulse = contact.accumulatedTangentImpulse;
			contact.accumulatedTangentImpulse = Clamp(oldFrictionImpulse + lambdaFriction, -maxFriction, maxFriction);
			float deltaFrictionImpulse = contact.accumulatedTangentImpulse - oldFrictionImpulse;

			// 应用摩擦冲量
			Vector3 frictionImpulse = deltaFrictionImpulse * tangent;

			if (shapeA->IsDynamic)
			{
				shapeA->velocity += frictionImpulse * invMassA;
				shapeA->angularVelocity += invInertiaA * (contact.rA.crossProduct(frictionImpulse));
			}

			if (shapeB->IsDynamic)
			{
				shapeB->velocity -= frictionImpulse * invMassB;
				shapeB->angularVelocity -= invInertiaB * (contact.rB.crossProduct(frictionImpulse));
			}
		}
		*/
		bool PhysicsManager::AreShapesInContact(PhysicsShape* shapeA, PhysicsShape* shapeB)
		{
			

			AABB* aabbA = shapeA->getAABB();
			AABB* aabbB = shapeB->getAABB();

			return aabbA->Intersects(*aabbB);
		}

		void PhysicsManager::Resolve(std::vector<std::shared_ptr<ContactManifold>>& manifolds, float deltaTime)
		{
			
			for each (std::shared_ptr<ContactManifold> manifold in manifolds)
			{
				for (int i = 0; i < manifold->contactPointCount; i++)
				{
					InitContactConstranst(manifold, i, deltaTime);
				}
			}

			
			for each (std::shared_ptr<ContactManifold> manifold in manifolds)
			{
				for (int i = 0; i < manifold->contactPointCount; i++)
				{
					SolveContactConstranst(manifold, i, deltaTime);
				}
			}
		}

		void PhysicsManager::InitContactConstranst(std::shared_ptr<ContactManifold> manifold, int idx, float deltaTime)
		{
			manifold->contactPoints[idx].m_jN.Init(manifold, idx, JacobianType::Normal, manifold->contactPoints[idx].normal, deltaTime);
			manifold->contactPoints[idx].m_jT.Init(manifold, idx, JacobianType::Tangent, manifold->contactPoints[idx].tangent1, deltaTime);
			manifold->contactPoints[idx].m_jB.Init(manifold, idx, JacobianType::Tangent, manifold->contactPoints[idx].tangent2, deltaTime);
		}

		void PhysicsManager::SolveContactConstranst(std::shared_ptr<ContactManifold> manifold, int idx, float deltaTime)
		{
			manifold->contactPoints[idx].m_jN.Solve(manifold, idx, manifold->contactPoints[idx].normal, deltaTime);
			manifold->contactPoints[idx].m_jT.Solve(manifold, idx, manifold->contactPoints[idx].tangent1, deltaTime);
			manifold->contactPoints[idx].m_jB.Solve(manifold, idx, manifold->contactPoints[idx].tangent2, deltaTime);
		}

		
	};
};