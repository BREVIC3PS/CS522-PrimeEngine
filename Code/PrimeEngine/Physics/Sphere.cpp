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
			const int numSegments = 12; // ��Ⱦ����

			const static int numPts = numSegments * 3 * 2; // ÿ��γ��Ȧ�;���Ȧ���� numSegments ���߶Σ����� 3��XY��XZ��YZ ����ƽ�棩
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
			Vector3 min = TransformedCenter - Vector3(radius, radius, radius); // ��С��
			Vector3 max = TransformedCenter + Vector3(radius, radius, radius); // ����
			return AABB(min, max);
		}

		void Sphere::UpdatePosition(float deltaTime)
		{
			if (!EnablePhysics)return;
			// ��������λ��
			TransformedCenter += velocity * deltaTime;

			// ��������任��������У�
			// �����µ�����λ�ø��� m_worldTransform
			SetPosition(TransformedCenter);

			
		}

		void Sphere::UpdateRotation(float deltaTime)
		{
			// ������ת
			float angularSpeed = angularVelocity.length();

			// ������ת
			if (angularSpeed > EPSILON)
			{
				// ������ת��ͽǶ�
				Vector3 axis = angularVelocity.normalized();
				float angle = angularSpeed * deltaTime;

				// Χ������ת�Ƕ�
				m_base.turnAboutAxis(angle, axis);

				// �淶�������������������Ժ͵�λ����
				m_base.normalizeUVN();
			}
		}

		Vector3 Sphere::ComputeCollisionNormal(const Vector3& collisionPoint)
		{
			return(collisionPoint - GetPosition()).normalized();
		}

		Vector3 Sphere::GetSupport(Vector3& dir)
		{
			// ��һ����������
			Vector3 normalizedDirection = dir.normalized();
			// ����֧�ֵ�
			return TransformedCenter + normalizedDirection * radius;
		}

		void Sphere::UpdateInverseInertiaTensorWorld()
		{
			// ��������������Ͱ뾶��֪
			float mass = this->mass;
			float radius = this->radius;

			// ������������ı�������
			float inertiaScalar = (2.0f / 5.0f) * mass * radius * radius;

			// ������������������
			float inverseInertiaScalar = (inertiaScalar != 0.0f) ? (1.0f / inertiaScalar) : 0.0f;

			// ����������������󣨵�λ��������������
			Matrix3x3 inverseInertiaTensor;
			inverseInertiaTensor.setIdentity();
			inverseInertiaTensor = inverseInertiaTensor * inverseInertiaScalar;

			// �������壬�ֲ������������������ͬ��
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
