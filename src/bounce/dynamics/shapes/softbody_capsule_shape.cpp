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

#include <bounce/dynamics/shapes/softbody_capsule_shape.h>
#include <bounce/dynamics/softbody_particle.h>
#include <bounce/dynamics/softbody.h>

b3SoftBodyCapsuleShape::b3SoftBodyCapsuleShape(const b3SoftBodyCapsuleShapeDef& def, b3SoftBody* body)
{
	m_shapeType = e_softBodyCapsuleShape;
	m_body = body;
	m_p1 = def.p1;
	m_p2 = def.p2;
}

b3SoftBodyCapsuleShape::~b3SoftBodyCapsuleShape()
{

}

void b3SoftBodyCapsuleShape::DestroyContacts()
{
	b3SoftBodyCapsuleAndCapsuleContact* c = m_body->m_contactManager.m_capsuleAndCapsuleContactList.m_head;
	while (c)
	{
		if (c->m_s1 == this || c->m_s2 == this)
		{
			b3SoftBodyCapsuleAndCapsuleContact* quack = c;
			c = c->m_next;
			m_body->m_contactManager.Destroy(quack);
			continue;
		}

		c = c->m_next;
	}
}

b3AABB b3SoftBodyCapsuleShape::ComputeAABB() const
{
	b3AABB aabb;
	aabb.lowerBound = b3Min(m_p1->m_position, m_p2->m_position);
	aabb.upperBound = b3Max(m_p1->m_position, m_p2->m_position);
	aabb.Extend(m_radius);
	return aabb;
}

void b3SoftBodyCapsuleShape::Synchronize(const b3Vec3& displacement)
{
	b3AABB aabb = ComputeAABB();
	m_body->m_contactManager.m_broadPhase.MoveProxy(m_proxy.proxyId, aabb, displacement);
}

void b3SoftBodyCapsuleShape::TouchProxy()
{
	m_body->m_contactManager.m_broadPhase.TouchProxy(m_proxy.proxyId);
}