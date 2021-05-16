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

#ifndef VIEW_MODEL_H
#define VIEW_MODEL_H

#include <bounce/common/math/vec2.h>

struct GLFWwindow;

class Model;

struct TestDef;
class Test;
typedef Test* (*TestCreate)(const TestDef& def);

struct TestEntry
{
	const char* name;
	TestCreate create;
};

#define MAX_TESTS 256

struct Properties
{
	Properties()
	{
		testID = 0;
		testCount = 0;
		drawPoints = true;
		drawLines = true;
		drawTriangles = true;
		drawGrid = true;
		drawStats = false;
	}

	void RegisterTest(const char* name, TestCreate create)
	{
		TestEntry* test = tests + testCount++;
		test->name = name;
		test->create = create;
	}
	
	int testID; 
	TestEntry tests[MAX_TESTS];
	int testCount;

	bool drawPoints;
	bool drawLines;
	bool drawTriangles;
	bool drawGrid;
	bool drawStats;
};

struct TestProperties
{
	TestProperties()
	{
		hertz = 60.0f;
		inv_hertz = 1.0f / hertz;
		velocityIterations = 8;
		positionIterations = 2;
		forceIterations = 1;
		forceSubIterations = 30;
		warmStart = true;
		drawShapes = true;
		drawBounds = false;
		drawContactPoints = true;
		drawContactNormals = false;
		drawContactTangents = false;
	}

	float hertz, inv_hertz;
	int velocityIterations;
	int positionIterations;
	int forceIterations;
	int forceSubIterations;
	bool warmStart;

	bool drawBounds;
	bool drawShapes;
	bool drawContactPoints;
	bool drawContactNormals;
	bool drawContactTangents;
	bool drawContactPolygons;
};

class ViewModel
{
public:
	ViewModel(GLFWwindow* window, Model* model);
	~ViewModel();
	
	void Action_SetTest();
	void Action_PreviousTest();
	void Action_NextTest();
	void Action_PlayPause();
	void Action_SinglePlay();
	void Action_ResetCamera();

	void Event_SetWindowSize(int w, int h);
	void Event_Press_Key(int button);
	void Event_Release_Key(int button);
	void Event_Press_Mouse(int button);
	void Event_Release_Mouse(int button);
	void Event_Move_Cursor(float x, float y);
	void Event_Scroll(float dx, float dy);

	void Command_Press_Key(int button);
	void Command_Release_Key(int button);
	void Command_Press_Mouse_Left(const b3Vec2& ps);
	void Command_Release_Mouse_Left(const b3Vec2& ps);
	void Command_Move_Cursor(const b3Vec2& ps);
	void Command_ResizeCamera(scalar w, scalar h);
	void Command_RotateCameraX(scalar angle);
	void Command_RotateCameraY(scalar angle);
	void Command_TranslateCameraX(scalar d);
	void Command_TranslateCameraY(scalar d);
	void Command_ZoomCamera(scalar d);
private:
	friend class View;
	
	b3Vec2 GetCursorPosition() const;
	
	GLFWwindow* m_window;
	Properties m_properties;
	TestProperties m_testProperties;
	Model* m_model;
	b3Vec2 m_ps0;
};

#endif
