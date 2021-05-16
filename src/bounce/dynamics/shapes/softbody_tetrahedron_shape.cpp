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

#include <bounce/dynamics/shapes/softbody_tetrahedron_shape.h>
#include <bounce/dynamics/softbody_particle.h>
#include <bounce/dynamics/softbody.h>

b3SoftBodyTetrahedronShape::b3SoftBodyTetrahedronShape(const b3SoftBodyTetrahedronShapeDef& def, b3SoftBody* body)
{
	m_shapeType = e_softBodyTetrahedronShape;
	m_body = body;
	m_p1 = def.p1;
	m_p2 = def.p2;
	m_p3 = def.p3;
	m_p4 = def.p4;

	b3Vec3 A = def.v1;
	b3Vec3 B = def.v2;
	b3Vec3 C = def.v3;
	b3Vec3 D = def.v4;

	b3Vec3 E1 = B - A;
	b3Vec3 E2 = C - A;
	b3Vec3 E3 = D - A;

	const scalar inv6 = scalar(1) / scalar(6);

	scalar det = b3Det(E1, E2, E3);
	scalar sign = b3Sign(det);
	m_volume = inv6 * sign * det;
}

b3SoftBodyTetrahedronShape::~b3SoftBodyTetrahedronShape()
{

}

b3AABB b3SoftBodyTetrahedronShape::ComputeAABB() const
{
	b3AABB aabb;
	aabb.lowerBound = b3Min(m_p1->m_position, b3Min(m_p2->m_position, b3Min(m_p3->m_position, m_p4->m_position)));
	aabb.upperBound = b3Max(m_p1->m_position, b3Max(m_p2->m_position, b3Max(m_p3->m_position, m_p4->m_position)));
	aabb.Extend(m_radius);
	return aabb;
}

void b3SoftBodyTetrahedronShape::DestroyContacts()
{
}

void b3SoftBodyTetrahedronShape::Synchronize(const b3Vec3& displacement)
{
	b3AABB aabb = ComputeAABB();
	m_body->m_contactManager.m_broadPhase.MoveProxy(m_proxy.proxyId, aabb, displacement);
}

void b3SoftBodyTetrahedronShape::TouchProxy()
{
	m_body->m_contactManager.m_broadPhase.TouchProxy(m_proxy.proxyId);
}