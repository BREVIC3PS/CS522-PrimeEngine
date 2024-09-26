#include "CameraSceneNode.h"
#include "../Lua/LuaEnvironment.h"
#include "PrimeEngine/Events/StandardEvents.h"
#include "DebugRenderer.h"
#define Z_ONLY_CAM_BIAS 0.0f


namespace PE {
namespace Components {

void CameraSceneNode::RenderFrustum(const Vector3 frustumCorners[8]) {
	// Define the indices for the lines connecting the frustum corners
	const int indices[] = {
		// Near plane
		0, 1, 1, 2, 2, 3, 3, 0,
		// Far plane
		4, 5, 5, 6, 6, 7, 7, 4,
		// Connect near and far planes
		0, 4, 1, 5, 2, 6, 3, 7
	};

	Vector3 color(0.0f, 1.0f, 0.0f);
	const int numEdges = 12;
	const int numPts = numEdges * 2;
	Vector3 linepts[numPts * 2]; 

	// Loop over the indices and render lines
	int iPt = 0;
	for (int i = 0; i < numPts; i+=2) {

		Vector3 start = frustumCorners[indices[i]];
		Vector3 end = frustumCorners[indices[i + 1]];

		
		linepts[iPt++] = start; 
		linepts[iPt++] = color; 

		
		linepts[iPt++] = end;   
		linepts[iPt++] = color; 

	}
	PE::Components::DebugRenderer::Instance()->createLineMesh(
		false,
		m_worldTransform,
		&linepts[0].m_x,
		numPts,
		0.f);
}

PE_IMPLEMENT_CLASS1(CameraSceneNode, SceneNode);

CameraSceneNode::CameraSceneNode(PE::GameContext &context, PE::MemoryArena arena, Handle hMyself) : SceneNode(context, arena, hMyself)
{
	m_near = 0.05f;
	m_far = 2000.0f;
}
void CameraSceneNode::addDefaultComponents()
{
	Component::addDefaultComponents();
	PE_REGISTER_EVENT_HANDLER(Events::Event_CALCULATE_TRANSFORMATIONS, CameraSceneNode::do_CALCULATE_TRANSFORMATIONS);
	PE_REGISTER_EVENT_HANDLER(Events::Event_PRE_RENDER_needsRC, CameraSceneNode::do_PRE_RENDER_needsRC);
}

void CameraSceneNode::do_CALCULATE_TRANSFORMATIONS(Events::Event *pEvt)
{
	Handle hParentSN = getFirstParentByType<SceneNode>();
	if (hParentSN.isValid())
	{
		Matrix4x4 parentTransform = hParentSN.getObject<PE::Components::SceneNode>()->m_worldTransform;
		m_worldTransform = parentTransform * m_base;
	}
	
	Matrix4x4 &mref_worldTransform = m_worldTransform;

	Vector3 pos = Vector3(mref_worldTransform.m[0][3], mref_worldTransform.m[1][3], mref_worldTransform.m[2][3]);
	Vector3 n = Vector3(mref_worldTransform.m[0][2], mref_worldTransform.m[1][2], mref_worldTransform.m[2][2]);
	Vector3 target = pos + n;
	Vector3 up = Vector3(mref_worldTransform.m[0][1], mref_worldTransform.m[1][1], mref_worldTransform.m[2][1]);

	m_worldToViewTransform = CameraOps::CreateViewMatrix(pos, target, up);

	m_worldTransform2 = mref_worldTransform;

	m_worldTransform2.moveForward(Z_ONLY_CAM_BIAS);

	Vector3 pos2 = Vector3(m_worldTransform2.m[0][3], m_worldTransform2.m[1][3], m_worldTransform2.m[2][3]);
	Vector3 n2 = Vector3(m_worldTransform2.m[0][2], m_worldTransform2.m[1][2], m_worldTransform2.m[2][2]);
	Vector3 target2 = pos2 + n2;
	Vector3 up2 = Vector3(m_worldTransform2.m[0][1], m_worldTransform2.m[1][1], m_worldTransform2.m[2][1]);

	m_worldToViewTransform2 = CameraOps::CreateViewMatrix(pos2, target2, up2);
    
    PrimitiveTypes::Float32 aspect = (PrimitiveTypes::Float32)(m_pContext->getGPUScreen()->getWidth()) / (PrimitiveTypes::Float32)(m_pContext->getGPUScreen()->getHeight());
    
    PrimitiveTypes::Float32 verticalFov = 0.33f * PrimitiveTypes::Constants::c_Pi_F32;

	PrimitiveTypes::Float32 originalVerticalFov = 0.33f * PrimitiveTypes::Constants::c_Pi_F32;
	PrimitiveTypes::Float32 smallerVerticalFov = originalVerticalFov * 0.9f;  // Àı–°10%

    if (aspect < 1.0f)
    {
        //ios portrait view
        static PrimitiveTypes::Float32 factor = 0.5f;
        verticalFov *= factor;
    }


	m_viewToProjectedTransform = CameraOps::CreateProjectionMatrix(verticalFov,
		aspect,
		m_near, m_far);

	float originalFOV = verticalFov;
	float aspectRatio = aspect;
	float nearClip = m_near;
	float farClip = m_far;
	float scaleFactor = 0.5f;

	// Scale the original FOV
	float newFOV = originalFOV * scaleFactor;
	Matrix4x4 scaledProjectionMatrix = CameraOps::CreateProjectionMatrix(newFOV, aspectRatio, nearClip, farClip);

	// Now create the smaller frustum using the adjusted projection matrix
	m_smallerFrustum = CreateSmallerFrustum(m_worldToViewTransform, scaledProjectionMatrix, originalFOV, aspectRatio, nearClip, farClip, scaleFactor);

	ComputeFrustumCorners(newFOV, aspectRatio, nearClip, farClip);

	// Step 2: Transform frustum corners to world space
	TransformFrustumCornersToWorld(m_worldToViewTransform.inverse(), m_frustumCorners);
	
	SceneNode::do_CALCULATE_TRANSFORMATIONS(pEvt);

}

void CameraSceneNode::do_PRE_RENDER_needsRC(PE::Events::Event* pEvt)
{
	Events::Event_PRE_RENDER_needsRC* pRealEvent = (Events::Event_PRE_RENDER_needsRC*)(pEvt);
	

	// Step 3: Render the frustum
	if(renderFrustum)RenderFrustum(m_frustumCorners);
}

void CameraSceneNode::ComputeFrustumCorners(float fovY, float aspectRatio, float nearClip, float farClip) {
	// Compute the height and width of the near and far planes
	float tanFovOver2 = tanf(fovY / 2.0f);

	float nearHeight = 2.0f * tanFovOver2 * nearClip;
	float nearWidth = nearHeight * aspectRatio;

	float farHeight = 2.0f * tanFovOver2 * farClip;
	float farWidth = farHeight * aspectRatio;

	// Near plane corners (in view space)
	m_frustumCorners[0] = Vector3(-nearWidth / 2, nearHeight / 2, nearClip); // Top Left
	m_frustumCorners[1] = Vector3(nearWidth / 2, nearHeight / 2, nearClip); // Top Right
	m_frustumCorners[2] = Vector3(nearWidth / 2, -nearHeight / 2, nearClip); // Bottom Right
	m_frustumCorners[3] = Vector3(-nearWidth / 2, -nearHeight / 2, nearClip); // Bottom Left

	// Far plane corners (in view space)
	m_frustumCorners[4] = Vector3(-farWidth / 2, farHeight / 2, farClip); // Top Left
	m_frustumCorners[5] = Vector3(farWidth / 2, farHeight / 2, farClip); // Top Right
	m_frustumCorners[6] = Vector3(farWidth / 2, -farHeight / 2, farClip); // Bottom Right
	m_frustumCorners[7] = Vector3(-farWidth / 2, -farHeight / 2, farClip); // Bottom Left
}



}; // namespace Components
}; // namespace PE
