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

#include <bounce_softbody/dynamics/fixtures/world_fixture.h>
#include <bounce_softbody/dynamics/body.h>

b3WorldFixture::b3WorldFixture()
{
}

void b3WorldFixture::Create(b3BlockAllocator* allocator, b3Body* body, const b3WorldFixtureDef& def)
{
	m_shape = def.shape->Clone(allocator);
	m_body = body;
	m_friction = def.friction;
}

void b3WorldFixture::Destroy(b3BlockAllocator* allocator)
{
	b3Shape::Destroy(m_shape, allocator);
}

void b3WorldFixture::DestroyContacts()
{
	b3SphereAndShapeContact* c = m_body->m_contactManager.m_shapeContactList.m_head;
	while (c)
	{
		b3SphereAndShapeContact* c0 = c;
		c = c->m_next;

		if (c0->m_f2 == this)
		{
			m_body->m_contactManager.Destroy(c0);
		}
	}
}