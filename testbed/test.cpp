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

#include "test.h"
#include <bounce/common/graphics/camera.h>

extern u32 b3_allocCalls, b3_maxAllocCalls;

float RandomFloat(float a, float b)
{
	float r = float(rand()) / float(RAND_MAX);
	float d = b - a;
	return a + r * d;
}

Test::Test(const TestDef& def)
{
	m_camera = def.camera;
	m_debugDraw = def.debugDraw;
	m_properties = def.properties;
	m_testProperties = def.testProperties;
	m_draw.m_debugDraw = def.debugDraw;

	b3Draw_draw = &m_draw;

	m_ray.origin.SetZero();
	m_ray.direction.Set(0.0f, 0.0f, -1.0f);
	m_ray.fraction = m_camera->GetZFar();
}

Test::~Test()
{
	b3Draw_draw = nullptr;
}

void Test::Step()
{
	// Draw
	if (m_properties->drawStats)
	{
		DrawString(b3Color_white, "Frame Allocations %d (%d)", b3_allocCalls, b3_maxAllocCalls);
	}
}
