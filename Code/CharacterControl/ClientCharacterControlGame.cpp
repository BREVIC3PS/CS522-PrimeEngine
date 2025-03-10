
#include "ClientCharacterControlGame.h"

#include "PrimeEngine/Scene/SkeletonInstance.h"

#include "ClientGameObjectManagerAddon.h"
#include "Tank/ClientTank.h"
#include "Client/ClientSpaceShipControls.h"
#include "Characters/SoldierNPCAnimationSM.h"
#include "CharacterControl/Characters/SoldierNPCAnimationSM.h"
#include "CharacterControlContext.h"
#include "Target.h"
#include "PrimeEngine/Physics/PhysicsManager.h"
#if PE_PLAT_IS_WIN32
#include "test.h"
#endif

using namespace PE;
using namespace PE::Components;



namespace CharacterControl {
	namespace Components {

		Box* createStaticBox(GameContext* context, MemoryArena& arena, const Vector3& pos, const Vector3& cornerMin, const Vector3& cornerMax, const char* handleName = "PHYSICS_Box", bool IsStatic = true) 
		{
			// 定义八个角
			Vector3 corners[8];
			corners[0] = Vector3(cornerMin.m_x, cornerMin.m_y, cornerMin.m_z);
			corners[1] = Vector3(cornerMin.m_x, cornerMin.m_y, cornerMax.m_z);
			corners[2] = Vector3(cornerMax.m_x, cornerMin.m_y, cornerMax.m_z);
			corners[3] = Vector3(cornerMax.m_x, cornerMin.m_y, cornerMin.m_z);
			corners[4] = Vector3(cornerMin.m_x, cornerMax.m_y, cornerMin.m_z);
			corners[5] = Vector3(cornerMin.m_x, cornerMax.m_y, cornerMax.m_z);
			corners[6] = Vector3(cornerMax.m_x, cornerMax.m_y, cornerMax.m_z);
			corners[7] = Vector3(cornerMax.m_x, cornerMax.m_y, cornerMin.m_z);

			// 创建 Handle
			Handle handle(handleName, sizeof(Box));
			// 在指定位置分配内存创建Box
			Box* box = new(handle) Box(*context, arena, handle, cornerMax, cornerMin, corners);

			// 设置 Box 属性
			box->addDefaultComponents();
			box->EnableGravity = false;
			box->EnablePhysics = false;
			box->IsDynamic = !IsStatic;
			box->m_base.setPos(pos);
			box->DebugRenderColor = Vector3((rand() % 255) / 255.0f, (rand() % 255) / 255.0f, (rand() % 255) / 255.0f);
			// 将组件添加到 PhysicsManager
			context->getPhysicsManager()->addComponent(handle);

			return box;
		}

		void createWallsAroundBox(GameContext* context, MemoryArena& arena, const Vector3& centerPos, const Vector3& cornerMin, const Vector3& cornerMax, float wallThickness = 1.0f) {
			// 基于中心Box的cornerMin和cornerMax计算中心Box的宽度和深度
			float width = cornerMax.m_x - cornerMin.m_x;
			float height = cornerMax.m_y - cornerMin.m_y;
			float depth = cornerMax.m_z - cornerMin.m_z;

			// 左墙壁
			createStaticBox(
				context, arena,
				Vector3(centerPos.m_x - width / 2 - wallThickness / 2, centerPos.m_y, centerPos.m_z),
				Vector3(-wallThickness / 2, -height / 2, -depth / 2),
				Vector3(wallThickness / 2, height / 2, depth / 2),
				"PHYSICS_LeftWall"
			);

			// 右墙壁
			createStaticBox(
				context, arena,
				Vector3(centerPos.m_x + width / 2 + wallThickness / 2, centerPos.m_y, centerPos.m_z),
				Vector3(-wallThickness / 2, -height / 2, -depth / 2),
				Vector3(wallThickness / 2, height / 2, depth / 2),
				"PHYSICS_RightWall"
			);

			// 前墙壁
			createStaticBox(
				context, arena,
				Vector3(centerPos.m_x, centerPos.m_y, centerPos.m_z + depth / 2 + wallThickness / 2),
				Vector3(-width / 2, -height / 2, -wallThickness / 2),
				Vector3(width / 2, height / 2, wallThickness / 2),
				"PHYSICS_FrontWall"
			);

			// 后墙壁
			createStaticBox(
				context, arena,
				Vector3(centerPos.m_x, centerPos.m_y, centerPos.m_z - depth / 2 - wallThickness / 2),
				Vector3(-width / 2, -height / 2, -wallThickness / 2),
				Vector3(width / 2, height / 2, wallThickness / 2),
				"PHYSICS_BackWall"
			);
		}

		// is run after initializing the engine
		// basic demo just adds a light source and a default control scheme
		// so that uses can add simple objects like meshes, skins, light sources, levels, etc. from asset manager
		int ClientCharacterControlGame::initGame()
		{
			// super implementation
			ClientGame::initGame();

			//add game specific context
			CharacterControlContext* pGameCtx = new (m_arena) CharacterControlContext;

			m_pContext->m_pGameSpecificContext = pGameCtx;

			PE::Components::LuaEnvironment* pLuaEnv = m_pContext->getLuaEnvironment();

			// init events, components, and other classes of the project
			CharacterControl::Register(pLuaEnv, PE::GlobalRegistry::Instance());

			// grey-ish background
			m_pContext->getGPUScreen()->setClearColor(Vector4(0.1f, 0.1f, 0.1f, 0.0f));


			// game controls read input queue and post events onto general queue
			// the events from general queue are then passed on to game components
			PE::Handle hDefaultGameControls("GAME_CONTROLS", sizeof(DefaultGameControls));
			m_pContext->m_pDefaultGameControls = new(hDefaultGameControls) DefaultGameControls(*m_pContext, m_arena, hDefaultGameControls);
			m_pContext->m_pDefaultGameControls->addDefaultComponents();

			m_pContext->getGameObjectManager()->addComponent(hDefaultGameControls);

			// initialize game

			// create the GameObjectmanager addon that is in charge of game objects in this demo
			{
				// create the GameObjectmanager addon that is in charge of game objects in this demo
				PE::Handle hGOMAddon = PE::Handle("ClientGameObjectManagerAddon", sizeof(ClientGameObjectManagerAddon));
				pGameCtx->m_pGameObjectManagerAddon = new(hGOMAddon) ClientGameObjectManagerAddon(*m_pContext, m_arena, hGOMAddon);
				pGameCtx->getGameObjectManagerAddon()->addDefaultComponents();

				// add it to game object manager
				// now all game events will be passed through to our GameObjectManagerAddon
				m_pContext->getGameObjectManager()->addComponent(hGOMAddon);
			}

			bool spawnALotOfSoldiersForGpuAnim = false;

			//create tank controls that will be enabled if tank is activated
			{
				// create the GameObjectmanager addon that is in charge of game objects in this demo
				PE::Handle h("TankGameControls", sizeof(TankGameControls));
				pGameCtx->m_pTankGameControls = new(h) TankGameControls(*m_pContext, m_arena, h);
				pGameCtx->getTankGameControls()->addDefaultComponents();

				// add it to game object manager addon
				pGameCtx->getGameObjectManagerAddon()->addComponent(h);

				// start deactivated. needs to be deactivated AFTER adding it to parent components
				pGameCtx->getTankGameControls()->setEnabled(false);


#if !PE_API_IS_D3D11
				if (!spawnALotOfSoldiersForGpuAnim)
				{
					/*for (int i = 0; i < 6; ++i)
						((ClientGameObjectManagerAddon*)(pGameCtx->getGameObjectManagerAddon()))->createTank(
							i, m_pContext->m_gameThreadThreadOwnershipMask);*/
				}
#endif
			}

			{
				PE::Handle h("ClientSpaceShipControls", sizeof(SpaceShipGameControls));
				pGameCtx->m_pSpaceShipGameControls = new(h) SpaceShipGameControls(*m_pContext, m_arena, h);
				pGameCtx->getSpaceShipGameControls()->addDefaultComponents();

				// add it to game object manager addon
				pGameCtx->getGameObjectManagerAddon()->addComponent(h);

				// start deactivated. needs to be deactivated AFTER adding it to parent components
				pGameCtx->getSpaceShipGameControls()->setEnabled(false);

				if (false)
				{
					((ClientGameObjectManagerAddon*)(pGameCtx->getGameObjectManagerAddon()))->createSpaceShip(
						m_pContext->m_gameThreadThreadOwnershipMask);
				}
			}


			if (false)
			{
				PE::Handle hSN("SCENE_NODE", sizeof(SceneNode));
				SceneNode* pSN = new(hSN) SceneNode(*m_pContext, m_arena, hSN);
				pSN->addDefaultComponents();

				pSN->m_base.setPos(Vector3(0, 0, 0));

				{
					PE::Handle hSoldierAnimSM("SoldierNPCAnimationSM", sizeof(SoldierNPCAnimationSM));
					SoldierNPCAnimationSM* pSoldierAnimSM = new(hSoldierAnimSM) SoldierNPCAnimationSM(*m_pContext, m_arena, hSoldierAnimSM);
					pSoldierAnimSM->addDefaultComponents();

					pSoldierAnimSM->m_debugAnimIdOffset = 0;// rand() % 3;


					PE::Handle hSkeletonInstance("SkeletonInstance", sizeof(SkeletonInstance));
					SkeletonInstance* pSkelInst = new(hSkeletonInstance) SkeletonInstance(*m_pContext, m_arena, hSkeletonInstance,
						hSoldierAnimSM);
					pSkelInst->addDefaultComponents();

					pSkelInst->initFromFiles("soldier_Soldier_Skeleton.skela", "Default", m_pContext->m_gameThreadThreadOwnershipMask);
					pSkelInst->setAnimSet("soldier_Soldier_Skeleton.animseta", "Default");


					{
						PE::Handle hMeshInstance("MeshInstance", sizeof(MeshInstance));
						MeshInstance* pMeshInstance = new(hMeshInstance) MeshInstance(*m_pContext, m_arena, hMeshInstance);
						pMeshInstance->addDefaultComponents();

						pMeshInstance->initFromFile("SoldierTransform.mesha", "Default", m_pContext->m_gameThreadThreadOwnershipMask);

						pSkelInst->addComponent(hMeshInstance);
					}


					// 			{
					// 				// create a scene node for gun attached to a joint
					// 				PE::Handle hMyGunSN = PE::Handle("SCENE_NODE", sizeof(JointSceneNode));
					// 				JointSceneNode *pGunSN = new(hMyGunSN) JointSceneNode(*m_pContext, m_arena, hMyGunSN, 38);
					// 				pGunSN->addDefaultComponents();
					// 				{
					// 					PE::Handle hMyGunMesh = PE::Handle("MeshInstance", sizeof(MeshInstance));
					// 					MeshInstance *pGunMeshInstance = new(hMyGunMesh) MeshInstance(*m_pContext, m_arena, hMyGunMesh);
					// 
					// 					pGunMeshInstance->addDefaultComponents();
					// 					pGunMeshInstance->initFromFile(pEvt->m_gunMeshName, pEvt->m_gunMeshPackage, pEvt->m_threadOwnershipMask);
					// 
					// 					// add gun to joint
					// 					pGunSN->addComponent(hMyGunMesh);
					// 				}
					// 				// add gun scene node to the skin
					// 				pSkelInst->addComponent(hMyGunSN);
					// 			}


					Events::SoldierNPCAnimSM_Event_WALK evt;
					pSkelInst->handleEvent(&evt);

					// add skeleton to scene node
					pSN->addComponent(hSkeletonInstance);
				}

				RootSceneNode::Instance()->addComponent(hSN);
			}

			//the soldier creation code expects not having the redner context so release it here, and reacquire afterwards
			m_pContext->getGPUScreen()->ReleaseRenderContextOwnership(m_pContext->m_gameThreadThreadOwnershipMask);

#if PE_API_IS_D3D11
			if (spawnALotOfSoldiersForGpuAnim)
			{
				int smallx = 4;

				for (int y = 0; y < 16; ++y)
					for (int x = 0; x < smallx; ++x)
						((ClientGameObjectManagerAddon*)(m_pContext->get<CharacterControlContext>()->getGameObjectManagerAddon()))->createSoldierNPC(
							Vector3(x * 2.0f, 0.0f, 2.0f * y), m_pContext->m_gameThreadThreadOwnershipMask);
			}
#endif

			m_pContext->getGPUScreen()->AcquireRenderContextOwnership(m_pContext->m_gameThreadThreadOwnershipMask);

			bool spawnALotOfMeshes = false;

			int maxX =1; // maybe need more to get framerate lower

			if (spawnALotOfMeshes)
			{
				for (int ix = 0; ix < maxX; ++ix)
				{
					for (int iy = 0; iy < 1; ++iy)
					{
						PE::Handle hSN("SCENE_NODE", sizeof(SceneNode));
						SceneNode* pMainSN = new(hSN) SceneNode(*m_pContext, m_arena, hSN);
						pMainSN->addDefaultComponents();

						pMainSN->m_base.setPos(Vector3(ix * 2.f, 0, -10.0f - iy * 2.f));
						PE::Handle hImrodMeshInst = PE::Handle("MeshInstance", sizeof(MeshInstance));
						MeshInstance* pImrodMeshInst = new(hImrodMeshInst) MeshInstance(*m_pContext, m_arena, hImrodMeshInst);
						pImrodMeshInst->addDefaultComponents();
						pImrodMeshInst->initFromFile("imrod.x_imrodmesh_mesh.mesha", "Default", m_pContext->m_gameThreadThreadOwnershipMask);
						pMainSN->addComponent(hImrodMeshInst);
						RootSceneNode::Instance()->addComponent(hSN);


						Mesh* myMesh = pImrodMeshInst->m_hAsset.getObject<Mesh>();

					}
				}
			}

			bool spawnALotOfSphere = false;

			//int maxX = 2; // maybe need more to get framerate lower

			if (spawnALotOfSphere)
			{
				for (int ix = 0; ix < maxX; ++ix)
				{
					for (int iy = 0; iy < maxX; ++iy)
					{

						Handle hPS1,hPS2;
						Sphere* pSphere = nullptr;
						Box* pBox = nullptr;

						//PE::Handle hSN("SCENE_NODE", sizeof(SceneNode));
						//SceneNode* pMainSN = new(hSN) SceneNode(*m_pContext, m_arena, hSN);
						//pMainSN->addDefaultComponents();

						//pMainSN->m_base.setPos(Vector3(ix * 2.0f, 0, -10.0f - iy * 2.0f));
						//PE::Handle hImrodMeshInst = PE::Handle("MeshInstance", sizeof(MeshInstance));
						//MeshInstance* pImrodMeshInst = new(hImrodMeshInst) MeshInstance(*m_pContext, m_arena, hImrodMeshInst);
						//pImrodMeshInst->addDefaultComponents();
						//pImrodMeshInst->initFromFile("imrod.x_imrodmesh_mesh.mesha", "Default", m_pContext->m_gameThreadThreadOwnershipMask);
						//pMainSN->addComponent(hImrodMeshInst);
						//RootSceneNode::Instance()->addComponent(hSN);


						//Mesh* myMesh = pImrodMeshInst->m_hAsset.getObject<Mesh>();

						/*hPS1 = Handle("PHYSICS_SPHERE", sizeof(Sphere));
						pSphere = new(hPS1) Sphere(*m_pContext, m_arena, hPS1);
						pSphere->addDefaultComponents();
						pSphere->radius = 0.5;
						pSphere->m_base.setPos(Vector3((ix + 0.3) * 3.0f, 10, (iy+0.3) * 3.0f));
						m_pContext->getPhysicsManager()->addComponent(hPS1);
						pSphere->EnableGravity = false;
						pSphere->EnablePhysics = false;*/

						//createStaticBox(m_pContext, m_arena, Vector3((ix + 0.3) * 3.0f, 10, (iy + 0.3) * 3.0f), Vector3(-1, -1, -1), Vector3(1, 1, 1), "DynamicBox", false);
						//createStaticBox(m_pContext, m_arena, Vector3((ix ) * 3.0f, 0, (iy ) * 3.0f), Vector3(-1, -1, -1), Vector3(1, 1, 1), "DynamicBox", false);

						//Vector3 Corners[8];
						//Corners[0] = Vector3(-1, -1, -1);
						//Corners[1] = Vector3(-1, -1, 1);
						//Corners[2] = Vector3(1, -1, 1);
						//Corners[3] = Vector3(1, -1, -1);
						//Corners[4] = Vector3(-1, 1, -1);
						//Corners[5] = Vector3(-1, 1, 1);
						//Corners[6] = Vector3(1, 1, 1);
						//Corners[7] = Vector3(1, 1, -1);
						//hPS2 = Handle("PHYSICS_Box", sizeof(Box));
						//pBox = new(hPS2) Box(*m_pContext, m_arena, hPS2, Vector3(1, 1, 1), Vector3(-1, -1, -1), Corners);
						//pBox->addDefaultComponents();
						//pBox->EnableGravity = false;
						//pBox->EnablePhysics = false;
						//pBox->m_base.setPos(Vector3(ix * 3.0f, 12, iy * 3.0f));
						//pBox->m_base.setU(Vector3(1,0,0));
						//pBox->m_base.setV(Vector3(0, 1, 0));
						//pBox->m_base.setN(Vector3(0, 0, 1));
						//pBox->m_base.rollRight(45);
						////pBox->m_base.turnDown(45);
						//m_pContext->getPhysicsManager()->addComponent(hPS2);

						// 定义地面的范围和位置
						Vector3 groundCornerMin(-0.5, -0.3, -0.5);
						Vector3 groundCornerMax(0.5, 0.3, 0.5);
						Vector3 groundPosition(ix * 1.0f, 1.1 * iy + 3, 0);

						// 创建地面
						Box* groundBox = createStaticBox(m_pContext, m_arena, groundPosition, groundCornerMin, groundCornerMax, "StaticBox");
						groundBox->IsDynamic = true;


					}
				}
			}

			// 定义地面的范围和位置
			Vector3 groundCornerMin(-10, -1, -10);
			Vector3 groundCornerMax(10, 1, 20);
			Vector3 groundPosition(0, -6, 0);

			// 创建地面
			Box* groundBox = createStaticBox(m_pContext, m_arena, groundPosition, groundCornerMin, groundCornerMax,"StaticBox");
			groundBox->name = "GroundBox";

			bool createWall = true;
			if (createWall)
			{
				// 定义墙壁的厚度和高度
				float wallThickness = 1.f;
				float wallHeight = 10.0f;

				// 墙壁的位置在地面的基础上调整
				float wallYPosition = groundPosition.m_y + (groundCornerMax.m_y - groundCornerMin.m_y) / 2 + wallHeight / 2;

				// 创建前墙（位于地面正前方）
				/*Vector3 frontWallPos(groundPosition.m_x, wallYPosition, groundCornerMax.m_z + wallThickness );
				Vector3 frontWallMin(groundCornerMin.m_x, -wallHeight / 2, -wallThickness / 2);
				Vector3 frontWallMax(groundCornerMax.m_x, wallHeight / 2, wallThickness / 2);*/
				/*Box* frontWall = createStaticBox(m_pContext, m_arena, frontWallPos , frontWallMin, frontWallMax);
				frontWall->name = "FrontWall";*/

				// 创建后墙（位于地面正后方）
				/*Vector3 backWallPos(groundPosition.m_x, wallYPosition, groundCornerMin.m_z - wallThickness );
				Box* backWall = createStaticBox(m_pContext, m_arena, backWallPos , frontWallMin, frontWallMax);
				backWall->name = "BackWall";*/

				// 创建左墙（位于地面左侧）
				Vector3 leftWallPos(groundCornerMin.m_x - wallThickness , wallYPosition, groundPosition.m_z);
				Vector3 sideWallMin(-wallThickness / 2, -wallHeight / 2, groundCornerMin.m_z);
				Vector3 sideWallMax(wallThickness / 2, wallHeight / 2, groundCornerMax.m_z);
				Box* leftWall = createStaticBox(m_pContext, m_arena, leftWallPos /1.5, sideWallMin, sideWallMax);
				//leftWall->m_base.turnLeft(-3.14f);
				//leftWall->m_base.turnAboutAxis(-0.5f, Vector3(0, 0, 1));
				leftWall->name = "LeftWall";
				leftWall->subdivisions = 3;
				leftWall->DebugRenderColor = Vector3(0, 1.0f, 0);

				// 创建右墙（位于地面右侧）
				Vector3 rightWallPos(groundCornerMax.m_x + wallThickness , wallYPosition, groundPosition.m_z + 5);
				Box* rightWall = createStaticBox(m_pContext, m_arena, rightWallPos /1.5, sideWallMin, sideWallMax);
				rightWall->name = "RightWall";
				rightWall->m_base.turnLeft(3.1415f);
				rightWall->subdivisions = 3;
				rightWall->DebugRenderColor = Vector3(0, 1.0f, 0);

				// 创建slop
				Vector3 SlopPos(groundPosition + Vector3(0,5,-8));
				Box* Slop = createStaticBox(m_pContext, m_arena, SlopPos, groundCornerMin, groundCornerMax);
				Slop->m_base.turnDown(0.5);
				Slop->name = "Slop";
				Slop->subdivisions = 3;
				Slop->DebugRenderColor = Vector3(1.0, 1.0f, 0);

				// 创建PushBar
				Vector3 PushBarPos(groundPosition + Vector3(0, 2, -5));
				Box* PushBar = createStaticBox(m_pContext, m_arena, PushBarPos, groundCornerMin, groundCornerMax);
				PushBar->name = "PushBar";
				PushBar->subdivisions = 3;
				PushBar->IsDynamic = false;
				
			}







#if PE_PLAT_IS_WIN32

				int testVar = number_cpp_extern + number_c_extern;// + TestExternIntVar;

				__asm {
					mov eax, [number_cpp_extern]
					add eax, [number_c_extern]
					mov[testVar], eax
					mov eax, [number_c_extern]
				}

				testfunc_c_cdecl();

				testfunc_c_stdcall();

				testfunc_c_fastcall();
#endif

				//here's how you would run this level trough code

				//whenever Lua Code is executed it assumes that the thread DOES NOT HAVE render context
				//so in this case we need to release render context (this function has render context)
				//and then reacquire once lua is done

				m_pContext->getGPUScreen()->ReleaseRenderContextOwnership(m_pContext->m_gameThreadThreadOwnershipMask);

#if PE_PLAT_IS_PSVITA // do it for ps3 becasue right now communication between pyClient and ps3 is not working
				//m_pContext->getLuaEnvironment()->runString("LevelLoader.loadLevel('ccontrollvl0.x_level.levela', 'CharacterControl')");
#endif
	//m_pContext->getLuaEnvironment()->runString("LevelLoader.loadLevel('char_highlight.x_level.levela', 'Basic')");

				m_pContext->getGPUScreen()->AcquireRenderContextOwnership(m_pContext->m_gameThreadThreadOwnershipMask);

				return 1; // 1 (true) = success. no errors. TODO: add error checking


		}

		void ClientCharacterControlGame::RenderBoundingBox(PE::Components::Mesh* myMesh, PE::Components::SceneNode* pMainSN)
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
				Vector3 start = myMesh->m_BoundingBox.Corners[edges[i][0]] + pMainSN->m_base.getPos();
				Vector3 end = myMesh->m_BoundingBox.Corners[edges[i][1]] + pMainSN->m_base.getPos();

				
				linepts[iPt++] = start; 
				linepts[iPt++] = color; 

				
				linepts[iPt++] = end;   
				linepts[iPt++] = color; 
			}

			Matrix4x4 transform;
			transform = (pMainSN->m_worldTransform);

			bool hasTransform = false;
			DebugRenderer::Instance()->createLineMesh(
				hasTransform,
				transform,
				&linepts[0].m_x,
				numPts,
				100000.f);

		}


	}
}
