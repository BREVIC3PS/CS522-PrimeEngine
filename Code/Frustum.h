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
			Left,   // ��ƽ��
			Right,  // ��ƽ��
			Top,    // ��ƽ��
			Bottom, // ��ƽ��
			Near,   // ��ƽ��
			Far     // Զƽ��
		};


		static Plane ExtractPlane(const Matrix4x4& matrix, PlaneDirection direction) {
			Plane plane;
			switch (direction) {
			case PlaneDirection::Left:
				// ��ȡ��ƽ�棺m3 + m0
				plane.normal.m_x = matrix.m[0][3] + matrix.m[0][0];
				plane.normal.m_y = matrix.m[1][3] + matrix.m[1][0];
				plane.normal.m_z = matrix.m[2][3] + matrix.m[2][0];
				plane.distance = matrix.m[3][3] + matrix.m[3][0];
				break;
			case PlaneDirection::Right:
				// ��ȡ��ƽ�棺m3 - m0
				plane.normal.m_x = matrix.m[0][3] - matrix.m[0][0];
				plane.normal.m_y = matrix.m[1][3] - matrix.m[1][0];
				plane.normal.m_z = matrix.m[2][3] - matrix.m[2][0];
				plane.distance = matrix.m[3][3] - matrix.m[3][0];
				break;
			case PlaneDirection::Top:
				// ��ȡ��ƽ�棺m3 - m1
				plane.normal.m_x = matrix.m[0][3] - matrix.m[0][1];
				plane.normal.m_y = matrix.m[1][3] - matrix.m[1][1];
				plane.normal.m_z = matrix.m[2][3] - matrix.m[2][1];
				plane.distance = matrix.m[3][3] - matrix.m[3][1];
				break;
			case PlaneDirection::Bottom:
				// ��ȡ��ƽ�棺m3 + m1
				plane.normal.m_x = matrix.m[0][3] + matrix.m[0][1];
				plane.normal.m_y = matrix.m[1][3] + matrix.m[1][1];
				plane.normal.m_z = matrix.m[2][3] + matrix.m[2][1];
				plane.distance = matrix.m[3][3] + matrix.m[3][1];
				break;
			case PlaneDirection::Near:
				// ��ȡ��ƽ�棺m3 + m2
				plane.normal.m_x = matrix.m[0][3] + matrix.m[0][2];
				plane.normal.m_y = matrix.m[1][3] + matrix.m[1][2];
				plane.normal.m_z = matrix.m[2][3] + matrix.m[2][2];
				plane.distance = matrix.m[3][3] + matrix.m[3][2];
				break;
			case PlaneDirection::Far:
				// ��ȡԶƽ�棺m3 - m2
				plane.normal.m_x = matrix.m[0][3] - matrix.m[0][2];
				plane.normal.m_y = matrix.m[1][3] - matrix.m[1][2];
				plane.normal.m_z = matrix.m[2][3] - matrix.m[2][2];
				plane.distance = matrix.m[3][3] - matrix.m[3][2];
				break;
			}

			// ��һ��ƽ�淨����
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

			// ��ȡƽ��
			frustum.planes[0] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Left);
			frustum.planes[1] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Right);
			frustum.planes[2] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Top);
			frustum.planes[3] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Bottom);
			frustum.planes[4] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Near);
			frustum.planes[5] = ExtractPlane(viewProjectionMatrix, PlaneDirection::Far);

			// ��С��׶������ҡ�����ƽ��
			for (int i = 0; i < 4; ++i) {
				frustum.planes[i].distance *= scaleFactor;
			}

			return frustum;
		}
