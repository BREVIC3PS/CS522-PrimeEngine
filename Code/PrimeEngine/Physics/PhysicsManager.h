#ifndef __PYENGINE_2_0_PHYSICSMANAGER_H__
#define __PYENGINE_2_0_PHYSICSMANAGER_H__

#define NOMINMAX
// API Abstraction
#include "PrimeEngine/APIAbstraction/APIAbstractionDefines.h"

// Outer-Engine includes
#include <assert.h>
// Inter-Engine includes
#include "PrimeEngine/MemoryManagement/Handle.h"
#include "PrimeEngine/PrimitiveTypes/PrimitiveTypes.h"
#include "../Events/Component.h"
#include "../Utils/Array/Array.h"
#include "../Scene/RootSceneNode.h"
#include "../Scene/CameraManager.h"

#include "PrimeEngine/Events/Component.h"
#include "PrimeEngine/Scene/Mesh.h"


// definitions of standard game events. the events that any game could potentially use
#include "PrimeEngine/Events/StandardGameEvents.h"


// Sibling/Children includes
namespace PE {
	namespace Components {

		struct PhysicsManager : public Component
		{
			PE_DECLARE_CLASS(PhysicsManager);


			// Singleton ------------------------------------------------------------------

			// Constructor -------------------------------------------------------------
			PhysicsManager(PE::GameContext& context, PE::MemoryArena arena, Handle hMyself);
			virtual ~PhysicsManager() {}
			// Methods      ------------------------------------------------------------
			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_PHYSICS_START);
			void do_PHYSICS_START(Events::Event* pEvt);

			PE_DECLARE_IMPLEMENT_EVENT_HANDLER_WRAPPER(do_PRE_RENDER_needsRC);
			void do_PRE_RENDER_needsRC(PE::Events::Event* pEvt);

			// Component ------------------------------------------------------------

			virtual void addDefaultComponents();
			void RenderBoundingBox(PE::Components::Mesh* myMesh, Vector3& Pos, Matrix4x4& transform);
			// Individual events -------------------------------------------------------

		private:

		};

	}; // namespace Components
}; // namespace PE
#endif
