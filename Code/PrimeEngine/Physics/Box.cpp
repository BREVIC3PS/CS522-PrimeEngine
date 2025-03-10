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
			// 定义立方体6个面，每个面有4个点的index
	// 假设 TransformedCorners 的点分布如下：
	//  0: (-x,-y,-z)
	//  1: ( x,-y,-z)
	//  2: ( x,-y, z)
	//  3: (-x,-y, z)
	//  4: (-x, y,-z)
	//  5: ( x, y,-z)
	//  6: ( x, y, z)
	//  7: (-x, y, z)
	//
	// 面的定义（可根据你实际的点顺序进行调整）：
	// 前面(Z正):   3,2,6,7
	// 后面(Z负):   0,1,5,4
	// 左面(X负):   0,3,7,4
	// 右面(X正):   1,2,6,5
	// 上面(Y正):   4,5,6,7
	// 下面(Y负):   0,1,2,3

			static const int faces[6][4] = {
				{3, 2, 6, 7}, // Front
				{0, 1, 5, 4}, // Back
				{0, 3, 7, 4}, // Left
				{1, 2, 6, 5}, // Right
				{4, 5, 6, 7}, // Top
				{0, 1, 2, 3}  // Bottom
			};

			// 面部细分程度，例如给每个面添加 5x5 的网格线（不包括外框）
			

			// 每条细分线会有两点（加上颜色信息即4个Vector3），
			// 每个面会有 (subdivisions - 1) 条内部线（横向）和 (subdivisions - 1) 条内部线（纵向）
			// 每条线2个点，每个点跟个颜色，共2*2=4个Vector3
			// 每个面总线条数 = 2*(subdivisions - 1) * subdivisions （横和纵加起来）
			// 但别忘了还需要加上盒子原有的12条边（如果你还要保留它们的话）。
			// 为简单起见，我们先单独考虑面内线条的数量。

			// 一个面有 (subdivisions - 1) 条平行于U方向的内部线，每条线跨越整个V方向。
			// 同样有 (subdivisions - 1) 条平行于V方向的内部线，每条线跨越整个U方向。
			// 总计面内部线条: (subdivisions - 1)*subdivisions * 2 (因为U方向line和V方向line数量相同形式)
			// 6个面则乘以6。

			int internalLinesPerFace = 2 * (subdivisions - 1) * subdivisions;
			int totalInternalLines = internalLinesPerFace * 6;
			int numPtsInternal = totalInternalLines * 2; // 每条线2个点

			// 再加上外框线条(12条边), 如需保留，在后面加入
			const int numEdges = 12;
			const int numPtsEdges = numEdges * 2;
			// 不过用户可能只想看面内线条，这里可根据需要保留或不保留

			// 分配存储空间: 每个点与一个color一对 (点+color)
			std::vector<Vector3> linepts;
			linepts.reserve((numPtsInternal + numPtsEdges) * 2);

			// 首先加入外部边线（可选）
			for (int i = 0; i < numEdges; ++i)
			{
				Vector3 start = TransformedCorners[edges[i][0]];
				Vector3 end = TransformedCorners[edges[i][1]];

				linepts.push_back(start);
				linepts.push_back(DebugRenderColor);

				linepts.push_back(end);
				linepts.push_back(DebugRenderColor);
			}

			// 绘制内部面网格线
			for (int f = 0; f < 6; ++f)
			{
				// 取出面四个角点
				Vector3 p0 = TransformedCorners[faces[f][0]];
				Vector3 p1 = TransformedCorners[faces[f][1]];
				Vector3 p2 = TransformedCorners[faces[f][2]];
				Vector3 p3 = TransformedCorners[faces[f][3]];

				// 假设p0->p1方向为U方向，p0->p3方向为V方向
				Vector3 uDir = p1 - p0;
				Vector3 vDir = p3 - p0;

				// 为了更好控制，需要确保顺序为（左下，右下，右上，左上），
				// 如果不确定四点次序，你可能需要对点重新排序，使其能构成一个有序矩形。
				// 以下假设 faces[f] 的定义满足这个顺序。否则需调整。

				// 绘制平行U方向的线（从V分向）
				// subdivisions段会产生subdivisions+1个分割点（包含边界）
				// 内部线条是忽略0和subdivisions这两个边界的，即 i = 1 到 i = subdivisions - 1。
				for (int i = 1; i < subdivisions; ++i)
				{
					float t = (float)i / (float)subdivisions;
					Vector3 start = p0 + vDir * t; // 从底边线往上移动 t 比例
					Vector3 end = start + uDir;   // 水平移动整条U长度
					linepts.push_back(start);
					linepts.push_back(DebugRenderColor);
					linepts.push_back(end);
					linepts.push_back(DebugRenderColor);
				}

				// 绘制平行V方向的线
				for (int j = 1; j < subdivisions; ++j)
				{
					float s = (float)j / (float)subdivisions;
					Vector3 start = p0 + uDir * s; // 从左边线往右移动 s 比例
					Vector3 end = start + vDir;   // 垂直移动整个V长度
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
				(int)(linepts.size() / 2), // 每2个Vector3为一个点(color和position分开算)
				0.f
			);

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
			//if (!EnablePhysics || !IsDynamic)return;
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
