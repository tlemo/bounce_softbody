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

#include "softbody_dragger.h"
#include <bounce/collision/geometry/geometry.h>
#include <bounce/collision/geometry/ray.h>
#include <bounce/dynamics/softbody.h>
#include <bounce/dynamics/shapes/softbody_triangle_shape.h>
#include <bounce/dynamics/forces/softbody_mouse_force.h>

SoftBodyDragger::SoftBodyDragger(b3Ray* ray, b3SoftBody* body)
{
	m_staticDrag = true;
	m_ray = ray;
	m_body = body;
	m_isDragging = false;
	m_km = 100000.0f;
	m_kd = 1000.0f;
}

bool SoftBodyDragger::StartDragging()
{
	B3_ASSERT(IsDragging() == false);

	b3SoftBodyRayCastSingleOutput rayOut;
	if (m_body->RayCastSingle(&rayOut, m_ray->A(), m_ray->B()) == false)
	{
		return false;
	}

	m_isDragging = true;
	m_x = rayOut.fraction;

	m_p1 = rayOut.triangle->GetParticle1();
	m_p2 = rayOut.triangle->GetParticle2();
	m_p3 = rayOut.triangle->GetParticle3();

	b3Vec3 v1 = m_p1->GetPosition();
	b3Vec3 v2 = m_p2->GetPosition();
	b3Vec3 v3 = m_p3->GetPosition();

	b3Vec3 B = GetPointB();

	scalar wABC[4];
	b3BarycentricCoordinates(wABC, v1, v2, v3, B);

	if (wABC[3] > B3_EPSILON)
	{
		m_u = wABC[0] / wABC[3];
		m_v = wABC[1] / wABC[3];
	}
	else
	{
		m_u = m_v = 0.0f;
	}

	if (m_staticDrag)
	{
		m_t1 = m_p1->GetType();
		m_p1->SetType(e_staticSoftBodyParticle);

		m_t2 = m_p2->GetType();
		m_p2->SetType(e_staticSoftBodyParticle);

		m_t3 = m_p3->GetType();
		m_p3->SetType(e_staticSoftBodyParticle);
	}
	else
	{
		b3SoftBodyParticleDef pd;
		pd.type = e_staticSoftBodyParticle;
		pd.position = GetPointA();

		m_particle = m_body->CreateParticle(pd);

		b3SoftBodyMouseForceDef def;
		def.p1 = m_particle;
		def.p2 = m_p1;
		def.p3 = m_p2;
		def.p4 = m_p3;
		def.w2 = m_u;
		def.w3 = m_v;
		def.w4 = 1.0f - m_u - m_v;
		def.stiffness = m_km;
		def.dampingStiffness = m_kd;
		def.restLength = 0.0f;

		m_mf = (b3SoftBodyMouseForce*)m_body->CreateForce(def);
	}

	return true;
}

void SoftBodyDragger::Drag()
{
	B3_ASSERT(IsDragging() == true);

	b3Vec3 B = GetPointB();

	if (m_staticDrag)
	{
		b3Vec3 A = GetPointA();

		b3Vec3 dx = B - A;

		m_p1->ApplyTranslation(dx);
		m_p2->ApplyTranslation(dx);
		m_p3->ApplyTranslation(dx);
	}
	else
	{
		m_particle->SetPosition(B);
	}
}

void SoftBodyDragger::SetStaticDrag(bool bit)
{
	if (bit == m_staticDrag)
	{
		return;
	}

	if (IsDragging())
	{
		StopDragging();
	}

	m_staticDrag = bit;
}

void SoftBodyDragger::StopDragging()
{
	B3_ASSERT(IsDragging() == true);

	if (m_staticDrag)
	{
		m_p1->SetType(m_t1);
		m_p2->SetType(m_t2);
		m_p3->SetType(m_t3);
	}
	else
	{
		m_body->DestroyForce(m_mf);
		m_body->DestroyParticle(m_particle);
	}

	m_isDragging = false;
}

b3Vec3 SoftBodyDragger::GetPointA() const
{
	B3_ASSERT(IsDragging() == true);

	b3Vec3 v1 = m_p1->GetPosition() + m_p1->GetTranslation();
	b3Vec3 v2 = m_p2->GetPosition() + m_p2->GetTranslation();
	b3Vec3 v3 = m_p3->GetPosition() + m_p3->GetTranslation();

	return m_u * v1 + m_v * v2 + (1.0f - m_u - m_v) * v3;
}

b3Vec3 SoftBodyDragger::GetPointB() const
{
	B3_ASSERT(IsDragging() == true);
	return (1.0f - m_x) * m_ray->A() + m_x * m_ray->B();
}
