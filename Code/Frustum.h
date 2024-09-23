#pragma once
#include "PrimeEngine/Math/Vector3.h"
#include "PrimeEngine/Math/Matrix4x4.h"

		struct Plane {
			Vector3 normal;
			float distance;
		};

		struct Frustum {
			Plane planes[6];
		};

		enum class PlaneDirection {
			Left,   // 左平面
			Right,  // 右平面
			Top,    // 上平面
			Bottom, // 下平面
			Near,   // 近平面
			Far     // 远平面
		};


		static Plane ExtractPlane(const Matrix4x4& matrix, PlaneDirection direction) {
			Plane plane;
			switch (direction) {
			case PlaneDirection::Left:
				// 提取左平面：m3 + m0
				plane.normal.m_x = matrix.m[0][3] + matrix.m[0][0];
				plane.normal.m_y = matrix.m[1][3] + matrix.m[1][0];
				plane.normal.m_z = matrix.m[2][3] + matrix.m[2][0];
				plane.distance = matrix.m[3][3] + matrix.m[3][0];
				break;
			case PlaneDirection::Right:
				// 提取右平面：m3 - m0
				plane.normal.m_x = matrix.m[0][3] - matrix.m[0][0];
				plane.normal.m_y = matrix.m[1][3] - matrix.m[1][0];
				plane.normal.m_z = matrix.m[2][3] - matrix.m[2][0];
				plane.distance = matrix.m[3][3] - matrix.m[3][0];
				break;
			case PlaneDirection::Top:
				// 提取上平面：m3 - m1
				plane.normal.m_x = matrix.m[0][3] - matrix.m[0][1];
				plane.normal.m_y = matrix.m[1][3] - matrix.m[1][1];
				plane.normal.m_z = matrix.m[2][3] - matrix.m[2][1];
				plane.distance = matrix.m[3][3] - matrix.m[3][1];
				break;
			case PlaneDirection::Bottom:
				// 提取下平面：m3 + m1
				plane.normal.m_x = matrix.m[0][3] + matrix.m[0][1];
				plane.normal.m_y = matrix.m[1][3] + matrix.m[1][1];
				plane.normal.m_z = matrix.m[2][3] + matrix.m[2][1];
				plane.distance = matrix.m[3][3] + matrix.m[3][1];
				break;
			case PlaneDirection::Near:
				// 提取近平面：m3 + m2
				plane.normal.m_x = matrix.m[0][3] + matrix.m[0][2];
				plane.normal.m_y = matrix.m[1][3] + matrix.m[1][2];
				plane.normal.m_z = matrix.m[2][3] + matrix.m[2][2];
				plane.distance = matrix.m[3][3] + matrix.m[3][2];
				break;
			case PlaneDirection::Far:
				// 提取远平面：m3 - m2
				plane.normal.m_x = matrix.m[0][3] - matrix.m[0][2];
				plane.normal.m_y = matrix.m[1][3] - matrix.m[1][2];
				plane.normal.m_z = matrix.m[2][3] - matrix.m[2][2];
				plane.distance = matrix.m[3][3] - matrix.m[3][2];
				break;
			}

			// 归一化平面法向量
			float length = sqrt(plane.normal.m_x * plane.normal.m_x +
				plane.normal.m_y * plane.normal.m_y +
				plane.normal.m_z * plane.normal.m_z);
			plane.normal.m_x /= length;
			plane.normal.m_y /= length;
			plane.normal.m_z /= length;
			plane.distance /= length;

			return plane;
		}

		static Frustum CreateSmallerFrustum(const Matrix4x4& viewProjectionMatrix, float scaleFactor) {
			Frustum frustum;

			// 提取平面
			frustum.planes[0] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Left);
			frustum.planes[1] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Right);
			frustum.planes[2] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Top);
			frustum.planes[3] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Bottom);
			frustum.planes[4] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Near);
			frustum.planes[5] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Far);

			// 缩小视锥体的左右、上下平面
			for (int i = 0; i < 4; ++i) {
				frustum.planes[i].distance *= scaleFactor;
			}

			return frustum;
		}
