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

#include "view.h"
#include "view_model.h"
#include "test.h"

#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#include "GLFW/glfw3.h"

static inline bool GetTestName(void* userData, int idx, const char** name)
{
	Properties* p = (Properties*)userData;
	assert(u32(idx) < p->testCount);
	*name = p->tests[idx].name;
	return true;
}

void DrawString(const b3Color& color, const char* string, ...)
{
	va_list args;
	va_start(args, string);

	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::SetNextWindowPos(ImVec2(0.0f, 40.0f));
	ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::TextColoredV(ImVec4(color.r, color.g, color.b, color.a), string, args);
	ImGui::End();
	ImGui::PopStyleVar();

	va_end(args);	
}

View::View(GLFWwindow* window, ViewModel* viewModel)
{
	m_viewModel = viewModel;
	m_window = window;

	// Create UI
	ImGui::CreateContext();

	ImGuiIO& io = ImGui::GetIO();

	io.IniFilename = NULL;

	ImGui_ImplGlfw_InitForOpenGL(m_window, false);
	ImGui_ImplOpenGL2_Init();

	ImGui::StyleColorsDark();
}

View::~View()
{
	// Destroy UI
	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplGlfw_Shutdown();

	ImGui::DestroyContext();
}

void View::Event_SetWindowSize(int w, int h)
{
	m_viewModel->Event_SetWindowSize(w, h);
}

void View::Event_Press_Key(int button)
{
	m_viewModel->Event_Press_Key(button);
}

void View::Event_Release_Key(int button)
{
	m_viewModel->Event_Release_Key(button);
}

void View::Event_Press_Mouse(int button)
{
	m_viewModel->Event_Press_Mouse(button);
}

void View::Event_Release_Mouse(int button)
{
	m_viewModel->Event_Release_Mouse(button);
}

void View::Event_Move_Cursor(float x, float y)
{
	m_viewModel->Event_Move_Cursor(x, y);
}

void View::Event_Scroll(float dx, float dy)
{
	m_viewModel->Event_Scroll(dx, dy);
}

void View::BeginInterface()
{
	ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
}

void View::Interface()
{
	Properties& properties = m_viewModel->m_properties;
	TestProperties& testProperties = m_viewModel->m_testProperties;

	bool openControls = false;
	bool openAbout = false;
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("Exit", "Alt+F4"))
			{
				glfwSetWindowShouldClose(m_window, true);
			}

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("View"))
		{
			ImGui::MenuItem("Statistics", "", &properties.drawStats);

			ImGui::Separator();

			ImGui::MenuItem("Points", "", &properties.drawPoints);
			ImGui::MenuItem("Lines", "", &properties.drawLines);
			ImGui::MenuItem("Triangles", "", &properties.drawTriangles);

			ImGui::Separator();
			
			ImGui::MenuItem("Reference Grid", "", &properties.drawGrid);

			ImGui::Separator();

			ImGui::MenuItem("Bounding Boxes", "", &testProperties.drawBounds);
			ImGui::MenuItem("Shapes", "", &testProperties.drawShapes);
			ImGui::MenuItem("Contact Points", "", &testProperties.drawContactPoints);
			ImGui::MenuItem("Contact Normals", "", &testProperties.drawContactNormals);
			ImGui::MenuItem("Contact Tangents", "", &testProperties.drawContactTangents);

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Tools"))
		{
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Help"))
		{
			if (ImGui::MenuItem("Controls"))
			{
				openControls = true;
			}
			
			if (ImGui::MenuItem("About"))
			{
				openAbout = true;
			}

			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}
	
	if (openControls)
	{
		ImGui::OpenPopup("Controls");
	}

	if (openAbout)
	{
		ImGui::OpenPopup("About Bounce Testbed");
	}

	ImVec2 buttonSize(-1.0f, 0.0f);
	
	if (ImGui::BeginPopupModal("Controls", NULL, ImGuiWindowFlags_Popup | ImGuiWindowFlags_NoResize))
	{
		ImGui::Text("Rotate the scene using LSHIFT + LMB");
		ImGui::Text("Translate the scene using LSHIFT + RMB");
		ImGui::Text("Zoom in / out the scene using LSHIFT + Mouse Wheel");

		if (ImGui::Button("OK", buttonSize))
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

	if (ImGui::BeginPopupModal("About Bounce Testbed", NULL, ImGuiWindowFlags_Popup | ImGuiWindowFlags_NoResize))
	{
		extern b3Version b3_version;

		ImGui::Text("Bounce Testbed");
		ImGui::Text("Version %d.%d.%d", b3_version.major, b3_version.minor, b3_version.revision);
		ImGui::Text("Copyright (c) Irlan Robson");
		
		if (ImGui::Button("OK", buttonSize))
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

	int width, height;
	glfwGetWindowSize(m_window, &width, &height);
	
	ImGui::SetNextWindowPos(ImVec2(0.0f, 20.0f));
	ImGui::SetNextWindowSize(ImVec2(width, 20.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(0.0f, 0.0f));

	ImGui::Begin("##ToolBar", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar);

	if (ImGui::BeginMenuBar())
	{
		ImGui::PushItemWidth(250.0f);

		ImGui::Separator();
		
		if (ImGui::Combo("##Test", &properties.testID, GetTestName, &properties, properties.testCount, properties.testCount))
		{
			m_viewModel->Action_SetTest();
		}

		ImGui::PopItemWidth();

		ImVec2 menuButtonSize(100.0f, 0.0f);

		ImGui::Separator();

		if (ImGui::Button("Previous", menuButtonSize))
		{
			m_viewModel->Action_PreviousTest();
		}

		if (ImGui::Button("Next", menuButtonSize))
		{
			m_viewModel->Action_NextTest();
		}

		ImGui::Separator();

		if (ImGui::Button("Play/Pause", menuButtonSize))
		{
			m_viewModel->Action_PlayPause();
		}

		if (ImGui::Button("Single Play", menuButtonSize))
		{
			m_viewModel->Action_SinglePlay();
		}

		ImGui::Separator();

		if (ImGui::Button("Restart", menuButtonSize))
		{
			m_viewModel->Action_SetTest();
		}

		ImGui::Separator();

		if (ImGui::Button("Reset Camera", menuButtonSize))
		{
			m_viewModel->Action_ResetCamera();
		}

		ImGui::EndMenuBar();
	}

	ImGui::End();
	
	ImGui::PopStyleVar();

	ImGui::SetNextWindowPos(ImVec2(width - 250.0f, 40.0f));
	ImGui::SetNextWindowSize(ImVec2(250.0f, height - 40.0f));
	ImGui::Begin("Test Settings", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

	ImGui::PushItemWidth(-1.0f);

	ImGui::Text("Hertz");
	ImGui::SliderFloat("##Hertz", &testProperties.hertz, 0.0f, 240.0f, "%.1f");

	ImGui::Text("Force Iterations");
	ImGui::SliderInt("##Force Iterations", &testProperties.forceIterations, 0, 50);

	ImGui::Text("Force Sub-iterations");
	ImGui::SliderInt("##Force Sub-iterations", &testProperties.forceSubIterations, 0, 50);

	ImGui::Text("Velocity Iterations");
	ImGui::SliderInt("##Velocity Iterations", &testProperties.velocityIterations, 0, 50);

	ImGui::Text("Position Iterations");
	ImGui::SliderInt("##Position Iterations", &testProperties.positionIterations, 0, 50);

	ImGui::Checkbox("Warm Start", &testProperties.warmStart);

	ImGui::PopItemWidth();

	ImGui::End();
}

void View::EndInterface()
{
	ImGui::PopStyleVar();

	ImGui::Render();

	ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}
