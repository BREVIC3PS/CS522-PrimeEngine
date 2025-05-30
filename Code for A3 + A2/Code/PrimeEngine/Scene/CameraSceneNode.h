#ifndef __PYENGINE_2_0_CAMERA_SCENE_NODE_H__
#define __PYENGINE_2_0_CAMERA_SCENE_NODE_H__

// API Abstraction
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

// Outer-Engine includes
#include <assert.h>

// Inter-Engine includes
#include "PrimeEngine/Render/IRenderer.h"
#include "PrimeEngine/MemoryManagement/Handle.h"
#include "PrimeEngine/PrimitiveTypes/PrimitiveTypes.h"
#include "../Events/Component.h"
#include "../Utils/Array/Array.h"
#include "PrimeEngine/Math/CameraOps.h"
#include "Frustum.h"
#include "SceneNode.h"


// Sibling/Children includes

namespace PE {
namespace Components {

struct CameraSceneNode : public SceneNode
{

	PE_DECLARE_CLASS(CameraSceneNode);

	// Constructor -------------------------------------------------------------
	CameraSceneNode(PE::GameContext &context, PE::MemoryArena arena, Handle hMyself);

	virtual ~CameraSceneNode(){}

	// Component ------------------------------------------------------------
	virtual void addDefaultComponents();

	PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_CALCULATE_TRANSFORMATIONS);
	virtual void do_CALCULATE_TRANSFORMATIONS(Events::Event *pEvt);

	PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_PRE_RENDER_needsRC);
	virtual void do_PRE_RENDER_needsRC(PE::Events::Event* pEvt);

	void ComputeFrustumCorners(float fovY, float aspectRatio, float nearClip, float farClip);

	void RenderFrustum(const Vector3 frustumCorners[8]);

	// Individual events -------------------------------------------------------
	
	Matrix4x4 m_worldToViewTransform; // objects in world space are multiplied by this to get them into camera's coordinate system (view space)
	Matrix4x4 m_worldToViewTransform2;
	Matrix4x4 m_worldTransform2;
	Matrix4x4 m_viewToProjectedTransform; // objects in local (view) space are multiplied by this to get them to screen space
	Matrix4x4 m_smallerViewToProjectedTransform;
	float m_near, m_far;

	Frustum m_smallerFrustum;
	Vector3 m_frustumCorners[8];

	bool renderFrustum = true;


};
}; // namespace Components
}; // namespace PE
#endif
