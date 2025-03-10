#include "Box.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"
#include "PrimeEngine/Events/StandardEvents.h"
#include "PrimeEngine/Scene/DebugRenderer.h"
namespace PE
{
	namespace Components
	{
		PE_IMPLEMENT_CLASS1(Box, PhysicsShape);
		void Box::addDefaultComponents()
		{
			Component::addDefaultComponents();
			//PE_REGISTER_EVENT_HANDLER(Events::Event_CALCULATE_TRANSFORMATIONS, Box::do_CALCULATE_TRANSFORMATIONS);
			//PE_REGISTER_EVENT_HANDLER(Events::Event_PHYSICS_START, Box::do_PHYSICS_START);
		}

		void Box::DebugRender()
		{
			// ����������6���棬ÿ������4�����index
	// ���� TransformedCorners �ĵ�ֲ����£�
	//  0: (-x,-y,-z)
	//  1: ( x,-y,-z)
	//  2: ( x,-y, z)
	//  3: (-x,-y, z)
	//  4: (-x, y,-z)
	//  5: ( x, y,-z)
	//  6: ( x, y, z)
	//  7: (-x, y, z)
	//
	// ��Ķ��壨�ɸ�����ʵ�ʵĵ�˳����е�������
	// ǰ��(Z��):   3,2,6,7
	// ����(Z��):   0,1,5,4
	// ����(X��):   0,3,7,4
	// ����(X��):   1,2,6,5
	// ����(Y��):   4,5,6,7
	// ����(Y��):   0,1,2,3

			static const int faces[6][4] = {
				{3, 2, 6, 7}, // Front
				{0, 1, 5, 4}, // Back
				{0, 3, 7, 4}, // Left
				{1, 2, 6, 5}, // Right
				{4, 5, 6, 7}, // Top
				{0, 1, 2, 3}  // Bottom
			};

			// �沿ϸ�̶ֳȣ������ÿ������� 5x5 �������ߣ����������
			

			// ÿ��ϸ���߻������㣨������ɫ��Ϣ��4��Vector3����
			// ÿ������� (subdivisions - 1) ���ڲ��ߣ����򣩺� (subdivisions - 1) ���ڲ��ߣ�����
			// ÿ����2���㣬ÿ���������ɫ����2*2=4��Vector3
			// ÿ������������ = 2*(subdivisions - 1) * subdivisions ������ݼ�������
			// �������˻���Ҫ���Ϻ���ԭ�е�12���ߣ�����㻹Ҫ�������ǵĻ�����
			// Ϊ������������ȵ�����������������������

			// һ������ (subdivisions - 1) ��ƽ����U������ڲ��ߣ�ÿ���߿�Խ����V����
			// ͬ���� (subdivisions - 1) ��ƽ����V������ڲ��ߣ�ÿ���߿�Խ����U����
			// �ܼ����ڲ�����: (subdivisions - 1)*subdivisions * 2 (��ΪU����line��V����line������ͬ��ʽ)
			// 6���������6��

			int internalLinesPerFace = 2 * (subdivisions - 1) * subdivisions;
			int totalInternalLines = internalLinesPerFace * 6;
			int numPtsInternal = totalInternalLines * 2; // ÿ����2����

			// �ټ����������(12����), ���豣�����ں������
			const int numEdges = 12;
			const int numPtsEdges = numEdges * 2;
			// �����û�����ֻ�뿴��������������ɸ�����Ҫ�����򲻱���

			// ����洢�ռ�: ÿ������һ��colorһ�� (��+color)
			std::vector<Vector3> linepts;
			linepts.reserve((numPtsInternal + numPtsEdges) * 2);

			// ���ȼ����ⲿ���ߣ���ѡ��
			for (int i = 0; i < numEdges; ++i)
			{
				Vector3 start = TransformedCorners[edges[i][0]];
				Vector3 end = TransformedCorners[edges[i][1]];

				linepts.push_back(start);
				linepts.push_back(DebugRenderColor);

				linepts.push_back(end);
				linepts.push_back(DebugRenderColor);
			}

			// �����ڲ���������
			for (int f = 0; f < 6; ++f)
			{
				// ȡ�����ĸ��ǵ�
				Vector3 p0 = TransformedCorners[faces[f][0]];
				Vector3 p1 = TransformedCorners[faces[f][1]];
				Vector3 p2 = TransformedCorners[faces[f][2]];
				Vector3 p3 = TransformedCorners[faces[f][3]];

				// ����p0->p1����ΪU����p0->p3����ΪV����
				Vector3 uDir = p1 - p0;
				Vector3 vDir = p3 - p0;

				// Ϊ�˸��ÿ��ƣ���Ҫȷ��˳��Ϊ�����£����£����ϣ����ϣ���
				// �����ȷ���ĵ�����������Ҫ�Ե���������ʹ���ܹ���һ��������Ρ�
				// ���¼��� faces[f] �Ķ����������˳�򡣷����������

				// ����ƽ��U������ߣ���V����
				// subdivisions�λ����subdivisions+1���ָ�㣨�����߽磩
				// �ڲ������Ǻ���0��subdivisions�������߽�ģ��� i = 1 �� i = subdivisions - 1��
				for (int i = 1; i < subdivisions; ++i)
				{
					float t = (float)i / (float)subdivisions;
					Vector3 start = p0 + vDir * t; // �ӵױ��������ƶ� t ����
					Vector3 end = start + uDir;   // ˮƽ�ƶ�����U����
					linepts.push_back(start);
					linepts.push_back(DebugRenderColor);
					linepts.push_back(end);
					linepts.push_back(DebugRenderColor);
				}

				// ����ƽ��V�������
				for (int j = 1; j < subdivisions; ++j)
				{
					float s = (float)j / (float)subdivisions;
					Vector3 start = p0 + uDir * s; // ������������ƶ� s ����
					Vector3 end = start + vDir;   // ��ֱ�ƶ�����V����
					linepts.push_back(start);
					linepts.push_back(DebugRenderColor);
					linepts.push_back(end);
					linepts.push_back(DebugRenderColor);
				}
			}

			bool hasTransform = false;
			DebugRenderer::Instance()->createLineMesh(
				hasTransform,
				m_worldTransform,
				&linepts[0].m_x,
				(int)(linepts.size() / 2), // ÿ2��Vector3Ϊһ����(color��position�ֿ���)
				0.f
			);

		}

		AABB Box::calculateAABB()
		{

			// ��ʼ�� AABB ����С������
			Vector3 minPoint = TransformedCorners[0];
			Vector3 maxPoint = TransformedCorners[0];

			// �������б任��Ķ��㣬������С���������ֵ
			for (int i = 1; i < 8; ++i)
			{
				const Vector3& point = TransformedCorners[i];

				// ������С��
				if (point.m_x < minPoint.m_x) minPoint.m_x = point.m_x;
				if (point.m_y < minPoint.m_y) minPoint.m_y = point.m_y;
				if (point.m_z < minPoint.m_z) minPoint.m_z = point.m_z;

				// ��������
				if (point.m_x > maxPoint.m_x) maxPoint.m_x = point.m_x;
				if (point.m_y > maxPoint.m_y) maxPoint.m_y = point.m_y;
				if (point.m_z > maxPoint.m_z) maxPoint.m_z = point.m_z;
			}

			// ���ؼ���õ��� AABB
			return AABB(minPoint, maxPoint);
		}

		void Box::UpdatePosition(float deltaTime)
		{
			//if (!EnablePhysics || !IsDynamic)return;
			// ���º��ӵ�λ��
			// ��������һ����ʾλ�õĳ�Ա���������� position
			Vector3 newPos = m_worldTransform.getPos() + velocity * deltaTime;

			// ��������任����
			SetPosition(newPos);

		}

		void Box::UpdateRotation(float deltaTime)
		{
			if (!EnablePhysics || !IsDynamic)return;
			//// ������ת
			//if (angularVelocity.length() > EPSILON)
			//{
			//	Vector3 axis = angularVelocity.normalized();
			//	float angle = angularVelocity.length() * deltaTime;

			//	// Ӧ����ת
			//	m_base.turnAboutAxis(angle, axis);

			//	// ��������ת����
			//	m_base.orthonormalizeRotation();

			//	
			//}

			Vector3 omega = GetAngularVelocity();
			if (omega.lengthSqr() > EPSILON)
			{
				//Quaternion Rotation = m_base.createQuat();
				//Quaternion delta(omega.m_x * deltaTime, omega.m_y * deltaTime, omega.m_z * deltaTime, 1.0f);
				//Quaternion target = delta * Rotation;
				//m_base.setFromQuatAndPos(target, m_base.getPos());

				Quaternion Rotation = m_base.createQuat();

				// ʹ����ǹ�ʽ����������Ԫ��
				float angle = omega.length() * deltaTime;
				Vector3 axis = omega.normalized();
				Quaternion delta = Quaternion(axis, angle);

				// ������ת
				Quaternion target = delta * Rotation;
				m_base.setFromQuatAndPos(target, m_base.getPos());

				// ��������ת���󣬷�ֹ����ۻ�
				m_base.orthonormalizeRotation();
			}

		}

		Vector3 Box::ComputeCollisionNormal(const Vector3& collisionPoint)
		{
			// ����ײ��ת�������ӵľֲ��ռ�
			Matrix4x4 invTransform = m_worldTransform.inverse();
			Vector3 localPoint = invTransform * collisionPoint;

			// ��ȡ���ӵİ�ߴ�
			Vector3 halfExtents = (Max - Min) * 0.5f;

			// ������ӵľֲ�����
			Vector3 localCenter = (Min + Max) * 0.5f;

			// ��������ĵ��ֲ���ײ���ƫ��
			Vector3 d = localPoint - localCenter;

			// ���㵽ÿ����ľ���
			float dx = halfExtents.m_x - fabsf(d.m_x);
			float dy = halfExtents.m_y - fabsf(d.m_y);
			float dz = halfExtents.m_z - fabsf(d.m_z);

			// ȷ���ĸ������
			Vector3 localNormal;
			if (dx <= dy && dx <= dz)
			{
				localNormal = Vector3((d.m_x > 0) ? 1 : -1, 0, 0);
			}
			else if (dy <= dx && dy <= dz)
			{
				localNormal = Vector3(0, (d.m_y > 0) ? 1 : -1, 0);
			}
			else
			{
				localNormal = Vector3(0, 0, (d.m_z > 0) ? 1 : -1);
			}

			// ���ֲ�����ת��������ռ�
			Vector3 worldNormal = m_worldTransform.transformDirection(localNormal);
			return worldNormal.normalized();
		}

		Vector3 Box::GetSupport(Vector3& dir)
		{
			float maxDot = -std::numeric_limits<float>::infinity();
			
			Vector3 supportPoint;

			
			for (int i = 0; i < 8; ++i) {
				
				float dotProduct = TransformedCorners[i].dotProduct(dir);
				
				if (dotProduct > maxDot) {
					maxDot = dotProduct;
					supportPoint = TransformedCorners[i];
				}
			}

			return supportPoint; 
		}

		void Box::UpdateInverseInertiaTensorWorld()
		{
			// ��ȡ�������ת����
			Matrix3x3 rotationMatrix = m_worldTransform.GetRotationMatrix();

			// ������������ϵ�µ����������
			inverseInertiaTensorWorld = rotationMatrix * inverseInertiaTensorLocal * rotationMatrix.transpose();
		}

		void Box::do_CALCULATE_TRANSFORMATIONS(Events::Event* pEvt)
		{
			PhysicsShape::do_CALCULATE_TRANSFORMATIONS(pEvt);

			for (int i = 0; i < 8; ++i)
			{
				TransformedCorners[i] = m_worldTransform * Corners[i];
			}
			// ��ʼ�� TransformedMin �� TransformedMax
			TransformedMin = TransformedCorners[0];
			TransformedMax = TransformedCorners[0];

			// �������е� TransformedCorners������ TransformedMin �� TransformedMax
			for (int i = 1; i < 8; ++i)
			{
				Vector3& corner = TransformedCorners[i];

				if (corner.m_x < TransformedMin.m_x) TransformedMin.m_x = corner.m_x;
				if (corner.m_y < TransformedMin.m_y) TransformedMin.m_y = corner.m_y;
				if (corner.m_z < TransformedMin.m_z) TransformedMin.m_z = corner.m_z;
				if (corner.m_x > TransformedMax.m_x) TransformedMax.m_x = corner.m_x;
				if (corner.m_y > TransformedMax.m_y) TransformedMax.m_y = corner.m_y;
				if (corner.m_z > TransformedMax.m_z) TransformedMax.m_z = corner.m_z;
			}
			ReadyToCollide = true;
			if (m_isTransformDirty)
			{

				//m_isTransformDirty = false;
			}


		}
		void Box::do_PHYSICS_START(Events::Event* pEvt)
		{
			PhysicsShape::do_PHYSICS_START(pEvt);
			if (!EnablePhysics)return;
		}

		PE::Components::Box::Box(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself)
			:PhysicsShape(context, arena, hMyself)
		{
			DebugRenderColor = Vector3(1.f, 1.f, 0.f);
			PhysicsShapeType = ShapeType::ST_Box;
		}

		PE::Components::Box::Box(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself, Vector3 _Max, Vector3 _Min, Vector3 _Corners[8])
			:PhysicsShape(context, arena, hMyself), Max(_Max), Min(_Min)
		{
			DebugRenderColor = Vector3(1.f, 1.f, 0.f);
			PhysicsShapeType = ShapeType::ST_Box;

			for (int i = 0; i < 8; ++i) {
				Corners[i] = _Corners[i];
			}

			width = Max.m_x - Min.m_x;
			height = Max.m_y - Min.m_y;
			depth = Max.m_z - Min.m_z;

			// ����ֲ����������ĶԽ�Ԫ��
			Ixx = (1.0f / 12.0f) * mass * (height * height + depth * depth);
			Iyy = (1.0f / 12.0f) * mass * (width * width + depth * depth);
			Izz = (1.0f / 12.0f) * mass * (width * width + height * height);

			// �����ֲ�������������
			inertiaTensorLocal.clear();
			inertiaTensorLocal.m[0][0] = Ixx;
			inertiaTensorLocal.m[1][1] = Iyy;
			inertiaTensorLocal.m[2][2] = Izz;

			// �����ֲ��������������
			inverseInertiaTensorLocal.clear();
			inverseInertiaTensorLocal.m[0][0] = (Ixx != 0.0f) ? 1.0f / Ixx : 0.0f;
			inverseInertiaTensorLocal.m[1][1] = (Iyy != 0.0f) ? 1.0f / Iyy : 0.0f;
			inverseInertiaTensorLocal.m[2][2] = (Izz != 0.0f) ? 1.0f / Izz : 0.0f;
		}

	}
}
