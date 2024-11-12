#define NOMINMAX
// API Abstraction
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

// Outer-Engine includes

// Inter-Engine includes
#include "PrimeEngine/FileSystem/FileReader.h"
#include "PrimeEngine/APIAbstraction/GPUMaterial/GPUMaterialSet.h"
#include "PrimeEngine/PrimitiveTypes/PrimitiveTypes.h"
#include "PrimeEngine/APIAbstraction/Texture/Texture.h"
#include "PrimeEngine/APIAbstraction/Effect/EffectManager.h"
#include "PrimeEngine/APIAbstraction/GPUBuffers/VertexBufferGPUManager.h"
#include "PrimeEngine/Scene/Skeleton.h"
#include "DefaultAnimationSM.h"
#include "Light.h"

#include "PrimeEngine/GameObjectModel/Camera.h"
#include "PrimeEngine/Physics/PhysicsManager.h"

// Sibling/Children includes
#include "MeshInstance.h"
#include "MeshManager.h"
#include "SceneNode.h"
#include "CameraManager.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"
#include "PrimeEngine/Events/StandardEvents.h"
namespace PE {
namespace Components{

PE_IMPLEMENT_CLASS1(MeshInstance, Component);

MeshInstance::MeshInstance(PE::GameContext &context, PE::MemoryArena arena, Handle hMyself)
: Component(context, arena, hMyself)
, m_culledOut(false)
{
	
}

void MeshInstance::addDefaultComponents()
{
	Component::addDefaultComponents();
}

void MeshInstance::initFromFile(const char *assetName, const char *assetPackage,
		int &threadOwnershipMask)
{
	Handle h = m_pContext->getMeshManager()->getAsset(assetName, assetPackage, threadOwnershipMask);

	initFromRegisteredAsset(h);
	Mesh* pMesh = h.getObject<Mesh>();
	if (pMesh && (pMesh->m_phyiscsEnabled || pMesh->isSoldier))
	{
        Handle hPS;
        Sphere* pSphere = nullptr;  
        Box* pBox = nullptr;        

        switch (pMesh->PhysicsType)
        {
        case(ShapeType::ST_Shpere):
            hPS = Handle("PHYSICS_SPHERE", sizeof(Sphere));
            pSphere = new(hPS) Sphere(*m_pContext, m_arena, hPS);
            pSphere->addDefaultComponents();
            pSphere->m_meshIns = this;
            pSphere->radius = 0.5;
            m_PhysicsRigidHandle = pSphere;
            break;

        case(ShapeType::ST_Box):
            hPS = Handle("PHYSICS_BOX", sizeof(Box));
            pBox = new(hPS) Box(*m_pContext, m_arena, hPS);
            memcpy(pBox->Corners, pMesh->m_BoundingBox.Corners, 8 * sizeof(Vector3));
            pBox->Max = pMesh->m_BoundingBox.Max;
            pBox->Min = pMesh->m_BoundingBox.Min;
            pBox->addDefaultComponents();
            pBox->m_meshIns = this;
            pBox->EnablePhysics = false;
            pBox->EnableGravity = false;
            m_PhysicsRigidHandle = pBox;
            break;

        default:
            break;
        }
        //addComponent(hPS);
        m_pContext->getPhysicsManager()->addComponent(hPS);
	}
}

bool MeshInstance::hasSkinWeights()
{
	Mesh *pMesh = m_hAsset.getObject<Mesh>();
	return pMesh->m_hSkinWeightsCPU.isValid();
}

void MeshInstance::initFromRegisteredAsset(const PE::Handle &h)
{
	m_hAsset = h;
	//add this instance as child to Mesh so that skin knows what to draw
	static int allowedEvts[] = {0};
	m_hAsset.getObject<Component>()->addComponent(m_hMyself, &allowedEvts[0]);
}


}; // namespace Components
}; // namespace PE
