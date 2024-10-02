#include "PhysicsManager.h"
#include "PrimeEngine/Events/StandardEvents.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"
#include <PrimeEngine/Scene/MeshInstance.h>
#include "PrimeEngine/Scene/DebugRenderer.h"


namespace PE {

	namespace Components {

		using namespace PE::Events;

		PE_IMPLEMENT_CLASS1(PhysicsManager, Component);

		PhysicsManager::PhysicsManager(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself)
			:Component(context, arena, hMyself)
		{
		}

		void PhysicsManager::do_PHYSICS_START(Events::Event* pEvt)
		{
			OutputDebugStringA("PE::Components::PhysicsManager::do_PHYSICS_START\n");
			
			//m_pContext->getGPUScreen()->ReleaseRenderContextOwnership(m_pContext->m_gameThreadThreadOwnershipMask);
		}

		void PhysicsManager::do_PRE_RENDER_needsRC(PE::Events::Event* pEvt)
		{
			//m_pContext->getGPUScreen()->AcquireRenderContextOwnership(m_pContext->m_gameThreadThreadOwnershipMask);
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
							//if(Pos.getX())
							RenderBoundingBox(myMesh, Pos, pSN->m_base);
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
				Vector3 start = myMesh->m_BoundingBox.Corners[edges[i][0]] + Pos;
				Vector3 end = myMesh->m_BoundingBox.Corners[edges[i][1]] + Pos;


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
				100.f);

		}
	};
};