#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"
#include "Sphere.h"
#include "../Events/Component.h"
#include "PrimeEngine/Events/StandardEvents.h"
#include "PrimeEngine/Scene/DebugRenderer.h"
namespace PE
{
	namespace Components
	{
		PE_IMPLEMENT_CLASS1(Sphere,PhysicsShape);

		void Sphere::addDefaultComponents()
		{
			Component::addDefaultComponents();
			//PE_REGISTER_EVENT_HANDLER(Events::Event_CALCULATE_TRANSFORMATIONS, Sphere::do_CALCULATE_TRANSFORMATIONS);
			//PE_REGISTER_EVENT_HANDLER(Events::Event_PHYSICS_START, Sphere::do_PHYSICS_START);
		}

		Sphere::Sphere(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself)
			:PhysicsShape(context, arena, hMyself)
		{
			DebugRenderColor = Vector3(0.0f, 1.0f, 0.0f);
			PhysicsShapeType = ShapeType::ST_Shpere;
		}

		void Sphere::DebugRender()
		{
			const int numSegments = 12; // 渲染精度

			const static int numPts = numSegments * 3 * 2; // 每个纬线圈和经线圈各有 numSegments 条线段，乘以 3（XY、XZ、YZ 三个平面）
			Vector3 linepts[numPts * 2];

			int iPt = 0;
			//  (XY, XZ, YZ)
			for (int j = 0; j < 3; ++j)
			{
				for (int i = 0; i < numSegments; ++i)
				{
					float theta1 = (float(i) / numSegments) * 2.0f * 3.14159265f;
					float theta2 = (float(i + 1) / numSegments) * 2.0f * 3.14159265f;

					Vector3 start, end;
					switch (j)
					{
					case 0: // XY plane
						start = Vector3(TransformedCenter.m_x + radius * cos(theta1), TransformedCenter.m_y + radius * sin(theta1), TransformedCenter.m_z);
						end = Vector3(TransformedCenter.m_x + radius * cos(theta2), TransformedCenter.m_y + radius * sin(theta2), TransformedCenter.m_z);
						break;
					case 1: // XZ plane
						start = Vector3(TransformedCenter.m_x + radius * cos(theta1), TransformedCenter.m_y, TransformedCenter.m_z + radius * sin(theta1));
						end = Vector3(TransformedCenter.m_x + radius * cos(theta2), TransformedCenter.m_y, TransformedCenter.m_z + radius * sin(theta2));
						break;
					case 2: // YZ plane
						start = Vector3(TransformedCenter.m_x, TransformedCenter.m_y + radius * cos(theta1), TransformedCenter.m_z + radius * sin(theta1));
						end = Vector3(TransformedCenter.m_x, TransformedCenter.m_y + radius * cos(theta2), TransformedCenter.m_z + radius * sin(theta2));
						break;
					}

					linepts[iPt++] = start;
					linepts[iPt++] = DebugRenderColor;

					linepts[iPt++] = end;
					linepts[iPt++] = DebugRenderColor;
				}
			}

			bool hasTransform = true;
			DebugRenderer::Instance()->createLineMesh(
				hasTransform,
				m_worldTransform,
				&linepts[0].m_x,
				numPts,
				0.f);

			PhysicsShape::DebugRender();
		}

		AABB Sphere::calculateAABB()
		{
			Vector3 min = TransformedCenter - Vector3(radius, radius, radius); // 最小点
			Vector3 max = TransformedCenter + Vector3(radius, radius, radius); // 最大点
			return AABB(min, max);
		}

		void Sphere::UpdatePosition(float deltaTime)
		{
			if (!EnablePhysics)return;
			// 更新球心位置
			TransformedCenter += velocity * deltaTime;

			// 更新世界变换矩阵（如果有）
			// 根据新的中心位置更新 m_worldTransform
			SetPosition(TransformedCenter);

			
		}

		void Sphere::UpdateRotation(float deltaTime)
		{
			// 更新旋转
			float angularSpeed = angularVelocity.length();

			// 更新旋转
			if (angularSpeed > EPSILON)
			{
				// 计算旋转轴和角度
				Vector3 axis = angularVelocity.normalized();
				float angle = angularSpeed * deltaTime;

				// 围绕轴旋转角度
				m_base.turnAboutAxis(angle, axis);

				// 规范化基向量，保持正交性和单位长度
				m_base.normalizeUVN();
			}
		}

		Vector3 Sphere::ComputeCollisionNormal(const Vector3& collisionPoint)
		{
			return(collisionPoint - GetPosition()).normalized();
		}

		Vector3 Sphere::GetSupport(Vector3& dir)
		{
			// 归一化方向向量
			Vector3 normalizedDirection = dir.normalized();
			// 计算支持点
			return TransformedCenter + normalizedDirection * radius;
		}

		void Sphere::UpdateInverseInertiaTensorWorld()
		{
			// 假设球体的质量和半径已知
			float mass = this->mass;
			float radius = this->radius;

			// 计算惯性张量的标量部分
			float inertiaScalar = (2.0f / 5.0f) * mass * radius * radius;

			// 计算惯性张量的逆标量
			float inverseInertiaScalar = (inertiaScalar != 0.0f) ? (1.0f / inertiaScalar) : 0.0f;

			// 构建逆惯性张量矩阵（单位矩阵乘以逆标量）
			Matrix3x3 inverseInertiaTensor;
			inverseInertiaTensor.setIdentity();
			inverseInertiaTensor = inverseInertiaTensor * inverseInertiaScalar;

			// 对于球体，局部和世界惯性张量是相同的
			this->inverseInertiaTensorWorld = inverseInertiaTensor;
		}
		

		void Sphere::do_CALCULATE_TRANSFORMATIONS(Events::Event* pEvt)
		{
			//if (!ReadyToCollide)SetPosition(m_base.getPos() + Vector3(0, 0, radius));
			PhysicsShape::do_CALCULATE_TRANSFORMATIONS(pEvt);
				TransformedCenter = m_worldTransform * center;
				ReadyToCollide = true;
			if (m_isTransformDirty)
			{
				//m_isTransformDirty = false;
			}
		}

		void Sphere::do_PHYSICS_START(Events::Event* pEvt)
		{
			PhysicsShape::do_PHYSICS_START(pEvt);
			if (!EnablePhysics)return;
		}

	}
}
