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

#ifndef SOFTBODY_H
#define SOFTBODY_H

#include "../softbody.h"
#include "softbody_dragger.h"
#include "test.h"

class SoftBody : public Test
{
public:
	SoftBody(const TestDef& def) : Test(def)
	{
		m_body = nullptr;
		m_bodyDragger = nullptr;
	}

	~SoftBody()
	{
		delete m_bodyDragger;
		delete m_body;
	}

	void Step()
	{
		Test::Step();

		m_body->Step(m_testProperties->inv_hertz,
			m_testProperties->velocityIterations,
			m_testProperties->positionIterations,
			m_testProperties->forceIterations,
			m_testProperties->forceSubIterations);
		
		Draw();
		
		if (m_bodyDragger->IsDragging())
		{
			b3Vec3 pA = m_bodyDragger->GetPointA();
			b3Vec3 pB = m_bodyDragger->GetPointB();

			b3DrawPoint(m_debugDraw, pA, 4.0f, b3Color_green);

			b3DrawPoint(m_debugDraw, pB, 4.0f, b3Color_green);

			b3DrawSegment(m_debugDraw, pA, pB, b3Color_white);
		}

		extern u32 b3_softBodyForceSolverIterations;
		extern u32 b3_softBodyForceSolverMinSubIterations;
		extern u32 b3_softBodyForceSolverMaxSubIterations;

		DrawString(b3Color_white, "Iterations = %d", b3_softBodyForceSolverIterations);
		DrawString(b3Color_white, "Sub-iterations [min] [max] = [%d] [%d]", b3_softBodyForceSolverMinSubIterations, b3_softBodyForceSolverMaxSubIterations);

		scalar E = m_body->GetEnergy();
		DrawString(b3Color_white, "E = %f", E);
	}
	
	virtual void Draw()
	{
		m_body->Draw();
	}
	
	void MouseMove(const b3Ray3& pw)
	{
		Test::MouseMove(pw);

		if (m_bodyDragger->IsDragging())
		{
			m_bodyDragger->Drag();
		}
	}

	void MouseLeftDown(const b3Ray3& pw)
	{
		Test::MouseLeftDown(pw);

		if (m_bodyDragger->IsDragging() == false)
		{
			m_bodyDragger->StartDragging();
		}
	}

	void MouseLeftUp(const b3Ray3& pw)
	{
		Test::MouseLeftUp(pw);

		if (m_bodyDragger->IsDragging() == true)
		{
			m_bodyDragger->StopDragging();
		}
	}

	UniformSoftBody* m_body;
	b3SoftBodyDragger* m_bodyDragger;
};

#endif
