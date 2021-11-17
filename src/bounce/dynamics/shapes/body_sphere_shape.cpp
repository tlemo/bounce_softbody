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

#include <bounce/dynamics/shapes/body_sphere_shape.h>
#include <bounce/dynamics/particle.h>
#include <bounce/dynamics/body.h>

b3BodySphereShape::b3BodySphereShape(const b3BodySphereShapeDef& def, b3Body* body) : b3BodyShape(def, body)
{
	m_type = e_bodySphereShape;
	m_p = def.p;
}

b3AABB b3BodySphereShape::ComputeAABB() const
{
	b3AABB aabb;
	aabb.Set(m_p->m_position, m_radius);
	return aabb;
}

void b3BodySphereShape::DestroyContacts()
{
	// Destroy shape contacts
	b3SphereAndShapeContact* c = m_body->m_contactManager.m_shapeContactList.m_head;
	while (c)
	{
		if (c->m_s1 == this)
		{
			b3SphereAndShapeContact* quack = c;
			c = c->m_next;
			m_body->m_contactManager.Destroy(quack);
			continue;
		}

		c = c->m_next;
	}
}