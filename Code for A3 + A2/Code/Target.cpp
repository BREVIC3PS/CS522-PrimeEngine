#include "Target.h"
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

#include "PrimeEngine/Lua/LuaEnvironment.h"
#include "PrimeEngine/Scene/SkeletonInstance.h"
#include "PrimeEngine/Scene/MeshInstance.h"
#include "PrimeEngine/Scene/RootSceneNode.h"
#include "CharacterControl//Events/Events.h"

using namespace PE::Components;
using namespace PE::Events;
using namespace CharacterControl::Events;

namespace CharacterControl {
	namespace Components {

		PE_IMPLEMENT_CLASS1(Target, Component);

		Target::Target(PE::GameContext& context, PE::MemoryArena arena, PE::Handle hMyself, Vector3& pos, CharacterControl::Events::Event_CreateSoldierNPC* pEvt) :
			Component(context, arena, hMyself)
		{
			m_pContext->getGPUScreen()->AcquireRenderContextOwnership(pEvt->m_threadOwnershipMask);

			PE::Handle hSN("TARGET_SCENE_NODE", sizeof(SceneNode));
			SceneNode* pMainSN = new(hSN) SceneNode(*m_pContext, m_arena, hSN);
			pMainSN->addDefaultComponents();

			pMainSN->m_base.setPos(pos);
			m_base.setPos(pos);
			BeginPosition = pos;

			PE::Handle hImrodMeshInst = PE::Handle("MeshInstance", sizeof(MeshInstance));
			MeshInstance* pImrodMeshInst = new(hImrodMeshInst) MeshInstance(*m_pContext, m_arena, hImrodMeshInst);

			pImrodMeshInst->addDefaultComponents();
			pImrodMeshInst->initFromFile("imrod.x_imrodmesh_mesh.mesha", "Default", pEvt->m_threadOwnershipMask);

			pMainSN->addComponent(hImrodMeshInst);

			RootSceneNode::Instance()->addComponent(hSN);

			// add the scene node as component of soldier without any handlers. this is just data driven way to locate scnenode for soldier's components
			{
				static int allowedEvts[] = { 0 };
				addComponent(hSN, &allowedEvts[0]);
			}

			m_pContext->getGPUScreen()->ReleaseRenderContextOwnership(pEvt->m_threadOwnershipMask);
		}


		void Target::addDefaultComponents()
		{
			Component::addDefaultComponents();

			PE_REGISTER_EVENT_HANDLER(Event_UPDATE, Target::do_UPDATE);
		}

		SceneNode* Target::getParentsSceneNode()
		{
			PE::Handle hParent = getFirstParentByType<Component>();
			if (hParent.isValid())
			{
				// see if parent has scene node component
				return hParent.getObject<Component>()->getFirstComponent<SceneNode>();

			}
			return NULL;
		}


		void Target::do_UPDATE(PE::Events::Event* pEvt)
		{
			// see if parent has scene node component
			//SceneNode* pSN = getParentsSceneNode();
			//if (pSN)
			//{


			//	bool reached = true;

			//	Vector3 curPos = pSN->m_base.getPos();
			//	// not at the spot yet
			//	Event_UPDATE* pRealEvt = (Event_UPDATE*)(pEvt);
			//	Vector3 TargetPos = Vector3(curPos.getX() + 3 * sin(pRealEvt->m_frameTime), curPos.getY(), curPos.getZ() + 3 * cos(pRealEvt->m_frameTime));
			//	float speed = 3.0f;
			//	float allowedDisp = speed * pRealEvt->m_frameTime;

			//	Vector3 dir = (TargetPos - curPos);
			//	dir.normalize();
			//	// instantaneous turn
			//	pSN->m_base.turnInDirection(dir, 3.1415f);
			//	pSN->m_base.setPos(curPos + dir * allowedDisp);

			//}
			
			SceneNode* pSN = getHandle().getObject<Component>()->getFirstComponent<SceneNode>();
			//OutputDebugStringA("PE: PROGRESS: SoldierNPCMovementSM::do_SoldierNPCMovementSM_Event_MOVE_TO(): received event: ");
			Vector3 curPos = pSN->m_base.getPos();
			Event_UPDATE* pRealEvt = (Event_UPDATE*)(pEvt);
			GameTime += pRealEvt->m_frameTime;
			//Vector3 TargetPos = Vector3(curPos.getX() + 3 * sin(pRealEvt->m_frameTime), curPos.getY(), curPos.getZ() + 3 * cos(pRealEvt->m_frameTime));
			//Vector3 TargetPos = Vector3(curPos.getX() + 3 * sin(pRealEvt->m_frameTime), curPos.getY(), curPos.getZ() + 3 * cos(pRealEvt->m_frameTime));
			Vector3 TargetPos = Vector3(BeginPosition.getX() + 8 +  5* sin(GameTime), BeginPosition.getY(), BeginPosition.getZ() - 6 + 5 * cos(GameTime));
			float speed = 5.0f;
			float allowedDisp = speed * pRealEvt->m_frameTime;

			Vector3 dir = (TargetPos - curPos);
			dir.normalize();
			// instantaneous turn
			pSN->m_base.setPos(TargetPos);
			m_base.setPos(TargetPos);
		}
	}; // namespace Components
}; // namespace CharacterControl
