/*
* Copyright (c) 2016-2019 Irlan Robson 
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "view_model.h"
#include "model.h"

#include "GLFW/glfw3.h"

#include "tests/softbody.h"
#include "tests/pinned_cloth.h"
#include "tests/particle_types.h"
#include "tests/stretch_mapping.h"
#include "tests/cloth_tearing.h"
#include "tests/cloth_element.h"
#include "tests/sheet.h"
#include "tests/node_types.h"
#include "tests/plastic_softbody.h"
#include "tests/table_cloth.h"

ViewModel::ViewModel(GLFWwindow* window, Model* model)
{
	m_window = window;
	m_model = model;
	m_model->m_properties = &m_properties;
	m_model->m_testProperties = &m_testProperties;
	
	m_properties.RegisterTest( "Pinned Cloth", &PinnedCloth::Create );
	m_properties.RegisterTest( "Particle Types", &ParticleTypes::Create );
	m_properties.RegisterTest( "Stretch Mapping", &StretchMapping::Create );
	m_properties.RegisterTest( "Cloth Tearing", &ClothTearing::Create );
	m_properties.RegisterTest( "Cloth Element", &ClothElement::Create );
	m_properties.RegisterTest( "Sheet", &Sheet::Create );
	m_properties.RegisterTest( "Node Types", &NodeTypes::Create );
	m_properties.RegisterTest( "Plastic Soft Body", &PlasticSoftBody::Create );
	m_properties.RegisterTest("Table Cloth", &TableCloth::Create);
}

ViewModel::~ViewModel()
{
}

b3Vec2 ViewModel::GetCursorPosition() const
{
	double x, y;
	glfwGetCursorPos(m_window, &x, &y);
	return b3Vec2(scalar(x), scalar(y));
}

void ViewModel::Action_SetTest()
{
	m_model->EnableSetTest(true);
}

void ViewModel::Action_PreviousTest()
{
	m_properties.testID = b3Clamp(m_properties.testID - 1, 0, int(m_properties.testCount) - 1);
	m_model->EnableSetTest(true);
}

void ViewModel::Action_NextTest()
{
	m_properties.testID = b3Clamp(m_properties.testID + 1, 0, int(m_properties.testCount) - 1);
	m_model->EnableSetTest(true);
}

void ViewModel::Action_PlayPause()
{
	m_model->EnablePause(!m_model->IsPaused());
}

void ViewModel::Action_SinglePlay()
{
	m_model->SinglePlay();
}

void ViewModel::Action_ResetCamera()
{
	m_model->ResetCamera();
}

void ViewModel::Event_SetWindowSize(int w, int h)
{
	Command_ResizeCamera(scalar(w), scalar(h));
}

void ViewModel::Event_Press_Key(int button)
{
	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (shiftDown)
	{
		if (button == GLFW_KEY_DOWN)
		{
			Command_ZoomCamera(1.0f);
		}

		if (button == GLFW_KEY_UP)
		{
			Command_ZoomCamera(-1.0f);
		}
	}
	else
	{
		Command_Press_Key(button);
	}
}

void ViewModel::Event_Release_Key(int button)
{
	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (!shiftDown)
	{
		Command_Release_Key(button);
	}
}

void ViewModel::Event_Press_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
		if (!shiftDown)
		{
			Command_Press_Mouse_Left(GetCursorPosition());
		}
	}
}

void ViewModel::Event_Release_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
		if (!shiftDown)
		{
			Command_Release_Mouse_Left(GetCursorPosition());
		}
	}
}

void ViewModel::Event_Move_Cursor(float x, float y)
{
	b3Vec2 ps;
	ps.Set(x, y);

	b3Vec2 dp = ps - m_ps0;
	m_ps0 = ps;

	b3Vec2 n = b3Normalize(dp);

	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	bool leftDown = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
	bool rightDown = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

	if (shiftDown)
	{
		if (leftDown)
		{
			scalar ax = -0.005f * B3_PI * n.x;
			scalar ay = -0.005f * B3_PI * n.y;

			Command_RotateCameraY(ax);
			Command_RotateCameraX(ay);
		}

		if (rightDown)
		{
			scalar tx = 0.2f * n.x;
			scalar ty = -0.2f * n.y;

			Command_TranslateCameraX(tx);
			Command_TranslateCameraY(ty);
		}
	}
	else
	{
		Command_Move_Cursor(ps);
	}
}

void ViewModel::Event_Scroll(float dx, float dy)
{
	b3Vec2 n(dx, dy);
	n.Normalize();

	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (shiftDown)
	{
		Command_ZoomCamera(1.0f * n.y);
	}
}


void ViewModel::Command_Press_Key(int button)
{
	m_model->m_test->KeyDown(button);
}

void ViewModel::Command_Release_Key(int button)
{
	m_model->m_test->KeyUp(button);
}

static inline b3Ray3 ConvertScreenToWorldRay(const b3Camera& camera, const b3Vec2& ps)
{
	b3Vec3 pw = camera.ConvertScreenToWorld(b3Vec2(ps.x, ps.y));
	b3Vec3 cp = camera.BuildPosition();
	
	b3Ray3 rw;
	rw.origin = cp;
	rw.direction = pw;
	rw.fraction = camera.GetZFar();
	return rw;	
}

void ViewModel::Command_Press_Mouse_Left(const b3Vec2& ps)
{
	b3Ray3 rw = ConvertScreenToWorldRay(m_model->m_camera, ps);

	m_model->m_test->MouseLeftDown(rw);
}

void ViewModel::Command_Release_Mouse_Left(const b3Vec2& ps)
{
	b3Ray3 rw = ConvertScreenToWorldRay(m_model->m_camera, ps);
	
	m_model->m_test->MouseLeftUp(rw);
}

void ViewModel::Command_Move_Cursor(const b3Vec2& ps)
{
	b3Ray3 rw = ConvertScreenToWorldRay(m_model->m_camera, ps);
	
	m_model->m_test->MouseMove(rw);
}

inline void ViewModel::Command_ResizeCamera(scalar w, scalar h)
{
	m_model->m_camera.SetWidth(w);
	m_model->m_camera.SetHeight(h);
}

inline void ViewModel::Command_RotateCameraX(scalar angle)
{
	m_model->m_camera.AddPolarAngle(angle);
}

inline void ViewModel::Command_RotateCameraY(scalar angle)
{
	m_model->m_camera.AddAzimuthalAngle(angle);
}

inline void ViewModel::Command_TranslateCameraX(scalar d)
{
	m_model->m_camera.TranslateXAxis(d);
}

inline void ViewModel::Command_TranslateCameraY(scalar d)
{
	m_model->m_camera.TranslateYAxis(d);
}

inline void ViewModel::Command_ZoomCamera(scalar d)
{
	m_model->m_camera.AddRadius(d);
}
