#include "PhysicsManager.h"
#include "PrimeEngine/Events/StandardEvents.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"
#include <PrimeEngine/Scene/MeshInstance.h>
#include "PrimeEngine/Scene/DebugRenderer.h"
#include <PrimeEngine/Scene/SkeletonInstance.h>
#include <CharacterControl/Characters/SoldierNPC.h>
#include <CharacterControl/Characters/SoldierNPCMovementSM.h>


// Define gravity and threshold constants
const float gravity = -9.81f; // Gravity acceleration
const float groundThreshold = 0.1f; // Threshold distance to consider the soldier is on the ground



namespace PE {

	namespace Components {

		using namespace PE::Events;
		using namespace CharacterControl::Components;

		PE_IMPLEMENT_CLASS1(PhysicsManager, Component);

		PhysicsManager::PhysicsManager(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself)
			:Component(context, arena, hMyself)
		{
		}

		void PhysicsManager::do_PHYSICS_START(Events::Event* pEvt)
		{

			Event_PHYSICS_START* pRealEvent = (Event_PHYSICS_START*)(pEvt);
			for (int j = 0; j < m_components.m_size; j++)
			{
				Handle& hSoldier = m_components[j];
				MeshInstance* SoldierMIns = hSoldier.getObject<MeshInstance>();
				if (SoldierMIns && SoldierMIns->isInstanceOf<MeshInstance>())
				{
					// Collect all ground bounding boxes at the beginning of the frame
					if(!boxCollected)
					{
						CollectGroundBoundingBoxes();
						boxCollected = true;
					}
					Mesh* SoldierMesh = SoldierMIns->m_hAsset.getObject<Mesh>();
					if (SoldierMesh && SoldierMesh->isInstanceOf<Mesh>() && SoldierMesh->isSoldier)
					{
						SceneNode* pSoldierSN = SoldierMIns->getFirstParentByTypePtr<SkeletonInstance>()->
							getFirstParentByTypePtr<SceneNode>()->getFirstParentByTypePtr<SceneNode>()
							->getFirstParentByTypePtr<SceneNode>();
						SoldierNPC* Soldier = pSoldierSN->getFirstParentByTypePtr<SoldierNPC>();

						if (pSoldierSN && Soldier)
						{
							SoldierNPCMovementSM* MovementSM = Soldier->MovementSM;
							Vector3 TargetPos = MovementSM->m_targetPostion;
							float speed = MovementSM->speed;

							// Get the soldier's previous position
							Vector3 PrevSoldierPos = pSoldierSN->m_base.getPos();

							// Compute the movement direction
							Vector3 dir = (TargetPos - PrevSoldierPos).normalized();

							// Update the soldier's horizontal position based on movement
							Vector3 SoldierPos = PrevSoldierPos + dir * speed * pRealEvent->m_frameTime;

							// **Vertical Movement and Gravity Application**

							float groundHeight;
							bool isOnGround = IsSoldierOnGround(SoldierPos, groundHeight);

							if (isOnGround)
							{
								// Reset vertical velocity
								MovementSM->verticalVelocity = 0.0f;

								// Set the soldier's vertical position to the ground height
								SoldierPos.m_y = groundHeight;
							}
							else
							{
								// Apply gravity to vertical velocity
								MovementSM->verticalVelocity += gravity * pRealEvent->m_frameTime;

								// Update the soldier's vertical position
								SoldierPos.m_y += MovementSM->verticalVelocity * pRealEvent->m_frameTime;
							}

							// Update the soldier's sphere position
							Sphere SoldierSphere;
							SoldierSphere.Center = SoldierPos;
							SoldierSphere.Center.m_y += SphereRadius;
							SoldierSphere.Radius = SphereRadius;


							// Loop through other components to detect collisions
							for (int i = 0; i < m_components.m_size; i++)
							{
								if (i == j) continue; // Skip self

								Handle& hObject = m_components[i];
								MeshInstance* ObjectMIns = hObject.getObject<MeshInstance>();
								if (ObjectMIns && ObjectMIns->isInstanceOf<MeshInstance>())
								{
									Mesh* ObjectMesh = ObjectMIns->m_hAsset.getObject<Mesh>();
									if (ObjectMesh && ObjectMesh->m_performBoundingVolumeCulling && ObjectMesh->isInstanceOf<Mesh>())
									{
										// Get the object's scene node
										SceneNode* pObjectSN = ObjectMIns->getFirstParentByTypePtr<SceneNode>();

										// Skip ground objects (already handled)
										if (ObjectMesh->isGround)
										{
											continue;
										}

										// Perform collision detection as before
										// **OBB (Object's bounding box) Collision Detection**

										// Get the object's world transform matrix
										Matrix4x4 M = pObjectSN->m_worldTransform;

										// Compute the local bounding box center and half-sizes
										Vector3 Min(ObjectMesh->Min_X, ObjectMesh->Min_Y, ObjectMesh->Min_Z);
										Vector3 Max(ObjectMesh->Max_X, ObjectMesh->Max_Y, ObjectMesh->Max_Z);
										Vector3 localCenter = (Min + Max) * 0.5f;
										Vector3 halfSizes = (Max - Min) * 0.5f;

										// Extract OBB center in world space
										Vector3 OBB_Center = M * localCenter;

										// Extract OBB axes
										Vector3 A0 = M.getU(); // U axis
										Vector3 A1 = M.getV(); // V axis
										Vector3 A2 = M.getN(); // N axis

										// Normalize axes to ensure they are unit vectors
										A0.normalize();
										A1.normalize();
										A2.normalize();

										// Half-sizes along each axis
										float e0 = halfSizes.m_x;
										float e1 = halfSizes.m_y;
										float e2 = halfSizes.m_z;

										// Compute vector from OBB center to sphere center
										Vector3 d = SoldierSphere.Center - OBB_Center;

										// Project d onto each OBB axis to get the distance along that axis
										float dist0 = d.dotProduct(A0);
										float dist1 = d.dotProduct(A1);
										float dist2 = d.dotProduct(A2);

										// Compute the overlap (penetration) along each axis, considering the sphere's radius
										float overlap0 = (e0 + SoldierSphere.Radius) - fabs(dist0);
										float overlap1 = (e1 + SoldierSphere.Radius) - fabs(dist1);
										float overlap2 = (e2 + SoldierSphere.Radius) - fabs(dist2);

										// Check for collision
										if (overlap0 < 0 || overlap1 < 0 || overlap2 < 0)
										{
											// No collision
											continue;
										}
										else
										{
											// Collision detected
											// Identify the axis with the minimum penetration
											float minOverlap = overlap0;
											Vector3 collisionNormal = (dist0 > 0) ? A0 : -A0;

											if (overlap1 < minOverlap)
											{
												minOverlap = overlap1;
												collisionNormal = (dist1 > 0) ? A1 : -A1;
											}
											if (overlap2 < minOverlap)
											{
												minOverlap = overlap2;
												collisionNormal = (dist2 > 0) ? A2 : -A2;
											}

											// Move the soldier back to the previous position
											SoldierPos = PrevSoldierPos;

											// Adjust the soldier's position to resolve interpenetration
											SoldierPos += collisionNormal * minOverlap;

											// Compute the soldier's velocity vector
											Vector3 velocity = dir * speed;
											velocity.m_y = MovementSM->verticalVelocity; // Include vertical velocity

											// Compute the component of velocity normal to the collision
											float vn = velocity.dotProduct(collisionNormal);
											Vector3 v_normal = collisionNormal * vn;

											// Compute the sliding (tangential) component
											Vector3 v_tangent = velocity - v_normal;

											// Update the soldier's position along the sliding direction
											SoldierPos += v_tangent * pRealEvent->m_frameTime;

											// Update the soldier's vertical velocity
											MovementSM->verticalVelocity = v_tangent.m_y;

											// Break out of the collision loop since we handled a collision
											break;
										}
									}
								}
							}
							// After handling collisions, update the soldier's position
							pSoldierSN->m_base.setPos(SoldierPos);
						}
					}
				}
			}
			
		}

		void PhysicsManager::do_PRE_RENDER_needsRC(PE::Events::Event* pEvt)
		{
			for (int i = 0; i < m_components.m_size; i++)
			{
				Handle& h = m_components[i];
				MeshInstance* MIns = h.getObject<MeshInstance>();
				if (MIns && MIns->isInstanceOf<MeshInstance>())
				{
					Mesh* myMesh = MIns->m_hAsset.getObject<Mesh>();
					if (myMesh && myMesh->m_performBoundingVolumeCulling && myMesh->isInstanceOf<Mesh>())
					{
						SceneNode* pSN = MIns->getFirstParentByTypePtr<SceneNode>();
						if (pSN)
						{
							Vector3& Pos = pSN->m_base.getPos();
							RenderBoundingBox(myMesh, Pos, pSN->m_base);
						}
					}
					else if (myMesh && myMesh->isSoldier && myMesh->isInstanceOf<Mesh>())
					{
						// MainSN->SceneNde->RotateSN->SkeletonInstance->MeshInstance
						SceneNode* pSN = MIns->getFirstParentByTypePtr<SkeletonInstance>()->
							getFirstParentByTypePtr<SceneNode>()->getFirstParentByTypePtr<SceneNode>()
							->getFirstParentByTypePtr<SceneNode>();

						if (pSN)
						{
							Vector3& Pos = pSN->m_base.getPos();

							Sphere SoldierSphere;
							SoldierSphere.Center = pSN->m_base.getPos();
							SoldierSphere.Center.m_y += SphereRadius;
							SoldierSphere.Radius = SphereRadius;
							RenderSphere(SoldierSphere, pSN->m_worldTransform);
						}
					}
				}
			}
		}

		void PhysicsManager::addDefaultComponents()
		{
			Component::addDefaultComponents();

			PE_REGISTER_EVENT_HANDLER(Event_PHYSICS_START, PhysicsManager::do_PHYSICS_START);
			PE_REGISTER_EVENT_HANDLER(Event_PRE_RENDER_needsRC, PhysicsManager::do_PRE_RENDER_needsRC);

		}

		void PhysicsManager::RenderBoundingBox(PE::Components::Mesh* myMesh, Vector3& Pos, Matrix4x4& transform)
		{
			int edges[12][2] = {
				{ 0, 1 },{ 1, 2 },{ 2, 3 },{ 3, 0 }, // bottom
				{ 4, 5 },{ 5, 6 },{ 6, 7 },{ 7, 4 }, // top
				{ 0, 4 },{ 1, 5 },{ 2, 6 },{ 3, 7 }  // connection
			};

			Vector3 color(1.0f, 0.0f, 0.0f);
			const int numEdges = 12;
			const int numPts = numEdges * 2;
			Vector3 linepts[numPts * 2];

			int iPt = 0;
			for (int i = 0; i < numEdges; ++i)
			{
				Vector3 start = transform * myMesh->m_BoundingBox.Corners[edges[i][0]];
				Vector3 end = transform * myMesh->m_BoundingBox.Corners[edges[i][1]];


				linepts[iPt++] = start;
				linepts[iPt++] = color;


				linepts[iPt++] = end;
				linepts[iPt++] = color;
			}

			bool hasTransform = true;
			DebugRenderer::Instance()->createLineMesh(
				hasTransform,
				transform,
				&linepts[0].m_x,
				numPts,
				10.f);

		}


		void PhysicsManager::RenderSphere(const Sphere& sphere, const Matrix4x4& transform)
		{
			
			const int numSegments = 12; // 渲染精度
			const float radius = sphere.Radius;
			const Vector3& center = sphere.Center;

			Vector3 color(0.0f, 1.0f, 0.0f); // 球体的颜色 (绿色)color
			const int numPts = numSegments * 3 * 2; // 每个纬线圈和经线圈各有 numSegments 条线段，乘以 3（XY、XZ、YZ 三个平面）
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
						start = Vector3(center.m_x + radius * cos(theta1), center.m_y + radius * sin(theta1), center.m_z);
						end = Vector3(center.m_x + radius * cos(theta2), center.m_y + radius * sin(theta2), center.m_z);
						break;
					case 1: // XZ plane
						start = Vector3(center.m_x + radius * cos(theta1), center.m_y, center.m_z + radius * sin(theta1));
						end = Vector3(center.m_x + radius * cos(theta2), center.m_y, center.m_z + radius * sin(theta2));
						break;
					case 2: // YZ plane
						start = Vector3(center.m_x, center.m_y + radius * cos(theta1), center.m_z + radius * sin(theta1));
						end = Vector3(center.m_x, center.m_y + radius * cos(theta2), center.m_z + radius * sin(theta2));
						break;
					}

					linepts[iPt++] = start;
					linepts[iPt++] = color;

					linepts[iPt++] = end;
					linepts[iPt++] = color;
				}
			}

			bool hasTransform = true;
			DebugRenderer::Instance()->createLineMesh(
				hasTransform,
				transform,
				&linepts[0].m_x,
				numPts,
				0.f);
		}


		Vector3 PhysicsManager::ClosestPointOnBoundingBox(const Vector3& point, const BoundingBox& box)
		{
			Vector3 closestPoint = point;

			// Clamp the point to the bounds of the bounding box
			closestPoint.m_x = std::max(box.Min.m_x, std::min(point.m_x, box.Max.m_x));
			closestPoint.m_y = std::max(box.Min.m_y, std::min(point.m_y, box.Max.m_y));
			closestPoint.m_z = std::max(box.Min.m_z, std::min(point.m_z, box.Max.m_z));

			return closestPoint;
		}

		float PhysicsManager::DistanceBetweenSphereAndBoundingBox(const Sphere& sphere, const BoundingBox& box)
		{
			// Calculate the closest point on the bounding box to the sphere center
			Vector3 closestPoint = ClosestPointOnBoundingBox(sphere.Center, box);

			// Calculate the distance between the sphere center and the closest point
			return (sphere.Center - closestPoint).length();
		}

		bool PhysicsManager::RayIntersectsBoundingBox(const Ray& ray, const BoundingBox& box, float& hitDistance)
		{
			float tmin = (box.Min.m_x - ray.origin.m_x) / ray.direction.m_x;
			float tmax = (box.Max.m_x - ray.origin.m_x) / ray.direction.m_x;

			if (tmin > tmax) std::swap(tmin, tmax);

			float tymin = (box.Min.m_y - ray.origin.m_y) / ray.direction.m_y;
			float tymax = (box.Max.m_y - ray.origin.m_y) / ray.direction.m_y;

			if (tymin > tymax) std::swap(tymin, tymax);

			if ((tmin > tymax) || (tymin > tmax))
				return false;

			if (tymin > tmin)
				tmin = tymin;

			if (tymax < tmax)
				tmax = tymax;

			float tzmin = (box.Min.m_z - ray.origin.m_z) / ray.direction.m_z;
			float tzmax = (box.Max.m_z - ray.origin.m_z) / ray.direction.m_z;

			if (tzmin > tzmax) std::swap(tzmin, tzmax);

			if ((tmin > tzmax) || (tzmin > tmax))
				return false;

			if (tzmin > tmin)
				tmin = tzmin;

			if (tzmax < tmax)
				tmax = tzmax;

			hitDistance = tmin;

			return true;
		}

		bool PhysicsManager::RayIntersectsOBB(const Ray& ray, const BoundingBox& obb, const Matrix4x4& obbTransform, float& hitDistance)
		{
			// Transform the ray into the OBB's local space
			Matrix4x4 invTransform = obbTransform.inverse();
			Vector3 localOrigin = invTransform * ray.origin;
			Vector3 localDirection = invTransform * (ray.direction);

			// Define the local bounding box min and max
			Vector3 localMin = obb.Min;
			Vector3 localMax = obb.Max;

			// Perform ray-AABB intersection in local space
			Ray localRay;
			localRay.origin = localOrigin;
			localRay.direction = localDirection;

			return RayIntersectsBoundingBox(localRay, obb, hitDistance);
		}

		// Function to check if the soldier is on the ground
		bool PhysicsManager::IsSoldierOnGround(const Vector3& soldierPos, float& groundHeight)
		{
			for (const auto& groundBox : groundBoxes)
			{
				// Check if soldier's (x, z) is within ground box's (x, z) range
				if (soldierPos.m_x >= groundBox.Min.m_x && soldierPos.m_x <= groundBox.Max.m_x &&
					soldierPos.m_z >= groundBox.Min.m_z && soldierPos.m_z <= groundBox.Max.m_z)
				{
					// Optionally check if soldier is within a reasonable vertical range
					if (fabs(soldierPos.m_y - groundBox.Max.m_y) <= groundThreshold + SphereRadius)
					{
						groundHeight = groundBox.Max.m_y;
						return true; // Soldier is on this ground box
					}
				}
			}
			return false; // Soldier is not on any ground box
		}

		// Function to collect all ground bounding boxes in the scene
		void PhysicsManager::CollectGroundBoundingBoxes()
		{
			// Iterate over all components to find ground objects
			for (int i = 0; i < m_components.m_size; i++)
			{
				Handle& hGround = m_components[i];
				MeshInstance* GroundMIns = hGround.getObject<MeshInstance>();
				if (GroundMIns && GroundMIns->isInstanceOf<MeshInstance>())
				{
					Mesh* GroundMesh = GroundMIns->m_hAsset.getObject<Mesh>();
					if (GroundMesh && GroundMesh->isInstanceOf<Mesh>() && GroundMesh->isGround) // Assuming 'isGround' flag
					{
						// Get the ground's world transform matrix
						SceneNode* pGroundSN = GroundMIns->getFirstParentByTypePtr<SceneNode>();
						Matrix4x4 groundTransform = pGroundSN->m_worldTransform;

						// Compute the ground's bounding box in world space
						Vector3 Min(GroundMesh->Min_X, GroundMesh->Min_Y, GroundMesh->Min_Z);
						Vector3 Max(GroundMesh->Max_X, GroundMesh->Max_Y, GroundMesh->Max_Z);

						// Transform the bounding box corners to world space
						BoundingBox groundBox;
						groundBox.Corners[0] = groundTransform * GroundMesh->m_BoundingBox.Corners[0];
						groundBox.Corners[1] = groundTransform * GroundMesh->m_BoundingBox.Corners[1];
						groundBox.Corners[2] = groundTransform * GroundMesh->m_BoundingBox.Corners[2];
						groundBox.Corners[3] = groundTransform * GroundMesh->m_BoundingBox.Corners[3];
						groundBox.Corners[4] = groundTransform * GroundMesh->m_BoundingBox.Corners[4];
						groundBox.Corners[5] = groundTransform * GroundMesh->m_BoundingBox.Corners[5];
						groundBox.Corners[6] = groundTransform * GroundMesh->m_BoundingBox.Corners[6];
						groundBox.Corners[7] = groundTransform * GroundMesh->m_BoundingBox.Corners[7];

						// Compute Min and Max in world space for AABB
						groundBox.Min = groundBox.Corners[0];
						groundBox.Max = groundBox.Corners[0];
						for (int k = 1; k < 8; k++)
						{
							groundBox.Min.m_x = std::min(groundBox.Min.m_x, groundBox.Corners[k].m_x);
							groundBox.Min.m_y = std::min(groundBox.Min.m_y, groundBox.Corners[k].m_y);
							groundBox.Min.m_z = std::min(groundBox.Min.m_z, groundBox.Corners[k].m_z);

							groundBox.Max.m_x = std::max(groundBox.Max.m_x, groundBox.Corners[k].m_x);
							groundBox.Max.m_y = std::max(groundBox.Max.m_y, groundBox.Corners[k].m_y);
							groundBox.Max.m_z = std::max(groundBox.Max.m_z, groundBox.Corners[k].m_z);
						}

						groundBoxes.push_back(groundBox);
					}
				}
			}

		}
		
	};
};