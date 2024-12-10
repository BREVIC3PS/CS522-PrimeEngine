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
			const static int numEdges = 12;
			const static int numPts = numEdges * 2;
			Vector3 linepts[numPts * 2];

			int iPt = 0;
			for (int i = 0; i < numEdges; ++i)
			{
				Vector3 start =  TransformedCorners[edges[i][0]];
				Vector3 end =  TransformedCorners[edges[i][1]];


				linepts[iPt++] = start;
				linepts[iPt++] = DebugRenderColor;


				linepts[iPt++] = end;
				linepts[iPt++] = DebugRenderColor;
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

		AABB Box::calculateAABB()
		{

			// 初始化 AABB 的最小和最大点
			Vector3 minPoint = TransformedCorners[0];
			Vector3 maxPoint = TransformedCorners[0];

			// 遍历所有变换后的顶点，计算最小和最大坐标值
			for (int i = 1; i < 8; ++i)
			{
				const Vector3& point = TransformedCorners[i];

				// 更新最小点
				if (point.m_x < minPoint.m_x) minPoint.m_x = point.m_x;
				if (point.m_y < minPoint.m_y) minPoint.m_y = point.m_y;
				if (point.m_z < minPoint.m_z) minPoint.m_z = point.m_z;

				// 更新最大点
				if (point.m_x > maxPoint.m_x) maxPoint.m_x = point.m_x;
				if (point.m_y > maxPoint.m_y) maxPoint.m_y = point.m_y;
				if (point.m_z > maxPoint.m_z) maxPoint.m_z = point.m_z;
			}

			// 返回计算得到的 AABB
			return AABB(minPoint, maxPoint);
		}

		void Box::UpdatePosition(float deltaTime)
		{
			if (!EnablePhysics || !IsDynamic)return;
			// 更新盒子的位置
			// 假设您有一个表示位置的成员变量，例如 position
			Vector3 newPos = m_worldTransform.getPos() + velocity * deltaTime;

			// 更新世界变换矩阵
			SetPosition(newPos);

		}

		void Box::UpdateRotation(float deltaTime)
		{
			if (!EnablePhysics || !IsDynamic)return;
			//// 更新旋转
			//if (angularVelocity.length() > EPSILON)
			//{
			//	Vector3 axis = angularVelocity.normalized();
			//	float angle = angularVelocity.length() * deltaTime;

			//	// 应用旋转
			//	m_base.turnAboutAxis(angle, axis);

			//	// 正交化旋转矩阵
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

				// 使用轴角公式构造增量四元数
				float angle = omega.length() * deltaTime;
				Vector3 axis = omega.normalized();
				Quaternion delta = Quaternion(axis, angle);

				// 更新旋转
				Quaternion target = delta * Rotation;
				m_base.setFromQuatAndPos(target, m_base.getPos());

				// 正交化旋转矩阵，防止误差累积
				m_base.orthonormalizeRotation();
			}

		}

		Vector3 Box::ComputeCollisionNormal(const Vector3& collisionPoint)
		{
			// 将碰撞点转换到盒子的局部空间
			Matrix4x4 invTransform = m_worldTransform.inverse();
			Vector3 localPoint = invTransform * collisionPoint;

			// 获取盒子的半尺寸
			Vector3 halfExtents = (Max - Min) * 0.5f;

			// 计算盒子的局部中心
			Vector3 localCenter = (Min + Max) * 0.5f;

			// 计算从中心到局部碰撞点的偏移
			Vector3 d = localPoint - localCenter;

			// 计算到每个面的距离
			float dx = halfExtents.m_x - fabsf(d.m_x);
			float dy = halfExtents.m_y - fabsf(d.m_y);
			float dz = halfExtents.m_z - fabsf(d.m_z);

			// 确定哪个面最近
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

			// 将局部法线转换回世界空间
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
			// 提取物体的旋转矩阵
			Matrix3x3 rotationMatrix = m_worldTransform.GetRotationMatrix();

			// 计算世界坐标系下的逆惯性张量
			inverseInertiaTensorWorld = rotationMatrix * inverseInertiaTensorLocal * rotationMatrix.transpose();
		}

		void Box::do_CALCULATE_TRANSFORMATIONS(Events::Event* pEvt)
		{
			PhysicsShape::do_CALCULATE_TRANSFORMATIONS(pEvt);

			for (int i = 0; i < 8; ++i)
			{
				TransformedCorners[i] = m_worldTransform * Corners[i];
			}
			// 初始化 TransformedMin 和 TransformedMax
			TransformedMin = TransformedCorners[0];
			TransformedMax = TransformedCorners[0];

			// 遍历所有的 TransformedCorners，计算 TransformedMin 和 TransformedMax
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

			// 计算局部惯性张量的对角元素
			Ixx = (1.0f / 12.0f) * mass * (height * height + depth * depth);
			Iyy = (1.0f / 12.0f) * mass * (width * width + depth * depth);
			Izz = (1.0f / 12.0f) * mass * (width * width + height * height);

			// 构建局部惯性张量矩阵
			inertiaTensorLocal.clear();
			inertiaTensorLocal.m[0][0] = Ixx;
			inertiaTensorLocal.m[1][1] = Iyy;
			inertiaTensorLocal.m[2][2] = Izz;

			// 构建局部逆惯性张量矩阵
			inverseInertiaTensorLocal.clear();
			inverseInertiaTensorLocal.m[0][0] = (Ixx != 0.0f) ? 1.0f / Ixx : 0.0f;
			inverseInertiaTensorLocal.m[1][1] = (Iyy != 0.0f) ? 1.0f / Iyy : 0.0f;
			inverseInertiaTensorLocal.m[2][2] = (Izz != 0.0f) ? 1.0f / Izz : 0.0f;
		}

	}
}
