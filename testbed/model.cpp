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

#include "model.h"
#include "view_model.h"
#include "test.h"

Model::Model() :
	m_debugDraw(512, 512, 512, &m_glDebugDraw),
	m_glDebugDraw(512, 512, 512)
{
	m_test = nullptr;
	m_properties = nullptr;
	m_testProperties = nullptr;
	
	m_setTest = true;
	m_pause = true;
	m_singlePlay = false;
	
	m_glDebugDraw.SetCamera(&m_camera);
	m_glDebugDraw.SetClearColor(0.2f, 0.2f, 0.2f, 1.0f);
	
	ResetCamera();
}

Model::~Model()
{
	delete m_test;
}

void Model::Update()
{
	m_debugDraw.EnableDrawPoints(m_properties->drawPoints);
	m_debugDraw.EnableDrawLines(m_properties->drawLines);
	m_debugDraw.EnableDrawTriangles(m_properties->drawTriangles);
	
	m_glDebugDraw.Begin();
	m_debugDraw.Begin();
	
	if (m_setTest)
	{
		ResetCamera();
		
		delete m_test;
		
		TestDef def;
		def.camera = &m_camera;
		def.debugDraw = &m_debugDraw;
		def.properties = m_properties;
		def.testProperties = m_testProperties;
		
		m_test = m_properties->tests[m_properties->testID].create(def);
		
		m_setTest = false;
		m_pause = true;
	}
	
	if (m_pause)
	{
		if (m_singlePlay)
		{
			m_testProperties->inv_hertz = m_testProperties->hertz > 0.0f ? 1.0f / m_testProperties->hertz : 0.0f;
			m_singlePlay = false;
		}
		else
		{
			m_testProperties->inv_hertz = 0.0f;
		}
	}
	else
	{
		m_testProperties->inv_hertz = m_testProperties->hertz > 0.0f ? 1.0f / m_testProperties->hertz : 0.0f;
	}

	if (m_properties->drawGrid)
	{
		b3DrawGrid(&m_debugDraw, b3Vec3_y, b3Vec3_zero, 20, 20, b3Color(0.4f, 0.4f, 0.4f, 1.0f));
	}
	
	m_test->Step();
	
	m_debugDraw.End();	
	m_glDebugDraw.End();
}
