#pragma once
#include "Sphere.h"
#include "Box.h"
#include <memory>

#define CONTACT_POINT_COUNT 5
#define CONTACT_DRIFTING_THRESHOLD  0.004f

namespace PE
{
	namespace Components
	{

		struct ContactPoint;
		struct sResults;
		struct CollisionDetector;
		struct MinkowskiDiff;
		struct Jacobian;
		struct ContactManifold;

		struct sResults
		{
			enum eStatus
			{
				Separated,   /* Shapes doesnt penetrate												*/
				Penetrating, /* Shapes are penetrating												*/
				GJK_Failed,  /* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
				EPA_Failed   /* EPA phase fail, bigger problem, need to save parameters, and debug	*/
			} status;

			Vector3 witnessInGlobal[2];
			Vector3 witnessesInFirstLocal[2];
			Vector3 normal;
			float distance;
		};

		struct MinkowskiDiff
		{
			PhysicsShape* box1;
			PhysicsShape* box2;

			inline Vector3 Support1(Vector3& dir);
			inline Vector3 Support2(Vector3& dir);
			Vector3 Support(Vector3& dir);
			Vector3 Support(Vector3& dir, int idx);
		};
		

		enum JacobianType
		{
			Normal,
			Tangent
		};

		struct Jacobian
		{
			Jacobian(JacobianType jt)
			{
				m_jva = Vector3(0, 0, 0);
				m_jwa = Vector3(0, 0, 0);
				m_jvb = Vector3(0, 0, 0);
				m_jwa = Vector3(0, 0, 0);

			}

			void Init(std::shared_ptr<ContactManifold> manifold, int idx, JacobianType jt, Vector3 dir, float dt);
			void Solve(std::shared_ptr<ContactManifold> manifold, int idx, Vector3 dir, float dt);


			JacobianType jacobinType;
			Vector3 m_jva;
			Vector3 m_jwa;
			Vector3 m_jvb;
			Vector3 m_jwb;
			float m_bias;
			float m_effectiveMass;
			float m_totalLambda;

			float Clamp(float value, float minVal, float maxVal)
			{
				return std::max(minVal, std::min(value, maxVal));
			}

		};

		struct ContactPoint {

			ContactPoint(void)
				: normalImpulseSum(0.0f)
				, tangentImpulseSum1(0.0f)
				, tangentImpulseSum2(0.0f)
				, m_jN(JacobianType::Normal)
				, m_jT(JacobianType::Tangent)
				, m_jB(JacobianType::Tangent)
			{
			}

			// contact point data
			Vector3 globalPositionA;	// Penetration point of object A in global coordinate
			Vector3 globalPositionB;	// Penetration point of object B in global coordinate
			Vector3 localPositionA;	// Penetration point of object A in self coordinate
			Vector3 localPositionB;	// Penetration point of object B in self coordinate

			// these 3 vectors form an orthonormal basis
			Vector3 normal;	// Penetration normal vector
			Vector3 tangent1, tangent2;	// two different tangent vectors
			Vector3 rA;	// Penetration vector of A
			Vector3 rB;	// Penetration vector of B

			// penetration depth
			float penetrationDistance;	// Penetration depth

			// for clamping (more on this later)
			float normalImpulseSum;
			float tangentImpulseSum1;
			float tangentImpulseSum2;

			Jacobian m_jN;
			Jacobian m_jT;
			Jacobian m_jB;
		};

		struct ContactManifold {
			PhysicsShape* colliderA;
			PhysicsShape* colliderB;
			int contactPointCount = 0;
			ContactPoint contactPoints[CONTACT_POINT_COUNT];

			void AddContact(ContactPoint point);
			void UpdateContacts();

		};

		struct CollisionDetector
		{
			virtual void CollideDetection(PhysicsShape* Shape1, PhysicsShape* Shape2, std::vector<std::shared_ptr<ContactManifold>>& collisions);

			void InitializeMinkowskiDiff(PhysicsShape* Shape1, PhysicsShape* Shape2, sResults& result, MinkowskiDiff& diff);
			bool Penetration(PhysicsShape* Shape1, PhysicsShape* Shape2, Vector3& guess, sResults& result);
			void GenerateTangents(ContactPoint& contactPoint);
		};


	}



}
