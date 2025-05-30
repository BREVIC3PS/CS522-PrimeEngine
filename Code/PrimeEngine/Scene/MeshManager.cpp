// API Abstraction
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

#include "MeshManager.h"
// Outer-Engine includes

// Inter-Engine includes
#include "PrimeEngine/FileSystem/FileReader.h"
#include "PrimeEngine/APIAbstraction/GPUMaterial/GPUMaterialSet.h"
#include "PrimeEngine/PrimitiveTypes/PrimitiveTypes.h"
#include "PrimeEngine/APIAbstraction/Texture/Texture.h"
#include "PrimeEngine/APIAbstraction/Effect/EffectManager.h"
#include "PrimeEngine/APIAbstraction/GPUBuffers/VertexBufferGPUManager.h"
#include "PrimeEngine/../../GlobalConfig/GlobalConfig.h"
#include "PrimeEngine/Physics/PhysicsManager.h"

#include "PrimeEngine/Geometry/SkeletonCPU/SkeletonCPU.h"

#include "PrimeEngine/Scene/RootSceneNode.h"

#include "Light.h"

// Sibling/Children includes

#include "MeshInstance.h"
#include "Skeleton.h"
#include "SceneNode.h"
#include "DrawList.h"
#include "SH_DRAW.h"
#include "PrimeEngine/Lua/LuaEnvironment.h"

namespace PE {
namespace Components{

PE_IMPLEMENT_CLASS1(MeshManager, Component);
MeshManager::MeshManager(PE::GameContext &context, PE::MemoryArena arena, Handle hMyself)
	: Component(context, arena, hMyself)
	, m_assets(context, arena, 256)
{
}

PE::Handle MeshManager::getAsset(const char *asset, const char *package, int &threadOwnershipMask)
{
	char key[StrTPair<Handle>::StrSize];
	sprintf(key, "%s/%s", package, asset);
	
	int index = m_assets.findIndex(key);
	if (index != -1)
	{
		return m_assets.m_pairs[index].m_value;
	}
	Handle h;

	if (StringOps::endswith(asset, "skela"))
	{
		PE::Handle hSkeleton("Skeleton", sizeof(Skeleton));
		Skeleton *pSkeleton = new(hSkeleton) Skeleton(*m_pContext, m_arena, hSkeleton);
		pSkeleton->addDefaultComponents();
		pSkeleton->initFromFiles(asset, package, threadOwnershipMask);
		h = hSkeleton;
	}
	else if (StringOps::endswith(asset, "mesha"))
	{
		MeshCPU mcpu(*m_pContext, m_arena);
		mcpu.ReadMesh(asset, package, "");
		
		PE::Handle hMesh("Mesh", sizeof(Mesh));
		Mesh *pMesh = new(hMesh) Mesh(*m_pContext, m_arena, hMesh);
		pMesh->addDefaultComponents();

		pMesh->loadFromMeshCPU_needsRC(mcpu, threadOwnershipMask);

#if PE_API_IS_D3D11
		// todo: work out how lods will work
		//scpu.buildLod();
#endif
        // generate collision volume here. or you could generate it in MeshCPU::ReadMesh()
        pMesh->m_phyiscsEnabled = false; // will now perform tests for this mesh

		if (StringOps::startsswith(asset, "nazicar") || StringOps::startsswith(asset, "cobbleplane"))
		{
			pMesh->m_phyiscsEnabled = true;
			pMesh->PhysicsType = ShapeType::ST_Box;

			PositionBufferCPU* pvbcpu = pMesh->m_hPositionBufferCPU.getObject<PositionBufferCPU>();
			float Min_X = pvbcpu->m_values[0]; float Max_X = pvbcpu->m_values[0];
			float Min_Y = pvbcpu->m_values[1]; float Max_Y = pvbcpu->m_values[1];
			float Min_Z = pvbcpu->m_values[2]; float Max_Z = pvbcpu->m_values[2];
			for (int i = 0; i < pvbcpu->m_values.m_size; i+=3)
			{
				if (pvbcpu->m_values[i] < Min_X)Min_X = pvbcpu->m_values[i];
				if (pvbcpu->m_values[i] > Max_X)Max_X = pvbcpu->m_values[i];
				if (pvbcpu->m_values[i+1] < Min_Y)Min_Y = pvbcpu->m_values[i+1];
				if (pvbcpu->m_values[i+1] > Max_Y)Max_Y = pvbcpu->m_values[i+1];
				if (pvbcpu->m_values[i+2] < Min_Z)Min_Z = pvbcpu->m_values[i+2];
				if (pvbcpu->m_values[i+2] > Max_Z)Max_Z = pvbcpu->m_values[i+2];

			}
			pMesh->Min_X = Min_X;
			pMesh->Min_Y = Min_Y;
			pMesh->Min_Z = Min_Z;
			pMesh->Max_X = Max_X;
			pMesh->Max_Y = Max_Y;
			pMesh->Max_Z = Max_Z;

			pMesh->m_BoundingBox.Corners[0] = Vector3(Min_X, Min_Y, Min_Z);
			pMesh->m_BoundingBox.Corners[1] = Vector3(Max_X, Min_Y, Min_Z);
			pMesh->m_BoundingBox.Corners[2] = Vector3(Max_X, Max_Y, Min_Z);
			pMesh->m_BoundingBox.Corners[3] = Vector3(Min_X, Max_Y, Min_Z);
			pMesh->m_BoundingBox.Corners[4] = Vector3(Min_X, Min_Y, Max_Z);
			pMesh->m_BoundingBox.Corners[5] = Vector3(Max_X, Min_Y, Max_Z);
			pMesh->m_BoundingBox.Corners[6] = Vector3(Max_X, Max_Y, Max_Z);
			pMesh->m_BoundingBox.Corners[7] = Vector3(Min_X, Max_Y, Max_Z);

			pMesh->m_BoundingBox.Min = Vector3(Min_X, Min_Y, Min_Z);
			pMesh->m_BoundingBox.Max = Vector3(Max_X, Max_Y, Max_Z);

			if (StringOps::startsswith(asset, "cobbleplane"))
			{
				pMesh->isGround = true;
			}
		}

		if (StringOps::startsswith(asset, "Soldier"))
		{
			pMesh->isSoldier = true;
			pMesh->PhysicsType = ShapeType::ST_Shpere;
		}
			h = hMesh;

	}


	PEASSERT(h.isValid(), "Something must need to be loaded here");

	RootSceneNode::Instance()->addComponent(h);
	m_assets.add(key, h);
	return h;
}

void MeshManager::registerAsset(const PE::Handle &h)
{
	static int uniqueId = 0;
	++uniqueId;
	char key[StrTPair<Handle>::StrSize];
	sprintf(key, "__generated_%d", uniqueId);
	
	int index = m_assets.findIndex(key);
	PEASSERT(index == -1, "Generated meshes have to be unique");
	
	RootSceneNode::Instance()->addComponent(h);
	m_assets.add(key, h);
}

}; // namespace Components
}; // namespace PE
