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

#include <bounce/dynamics/contact_manager.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/particle.h>
#include <bounce/dynamics/shapes/body_sphere_shape.h>
#include <bounce/dynamics/shapes/body_world_shape.h>
#include <bounce/common/memory/block_allocator.h>

void b3ContactManager::AddPair(b3BodySphereShape* s1, b3BodyWorldShape* s2)
{
	// Check if there is a contact between the two entities.
	for (b3SphereAndShapeContact* c = m_shapeContactList.m_head; c; c = c->m_next)
	{
		if (c->m_s1 == s1 && c->m_s2 == s2)
		{
			// A contact already exists.
			return;
		}
	}

	// Should the entities collide with each other?
	if (s1->m_p->m_type != e_dynamicParticle)
	{
		return;
	}

	// Call the factory.
	b3SphereAndShapeContact* c = b3SphereAndShapeContact::Create(s1, s2, m_allocator);

	// Push the contact to the contact list.
	m_shapeContactList.PushFront(c);
}

void b3ContactManager::FindNewContacts()
{
	// Run a simple broadphase loop.
	for (b3BodySphereShape* s1 = m_body->m_sphereShapeList.m_head; s1 != nullptr; s1 = s1->m_next)
	{
		b3AABB aabb1 = s1->ComputeAABB();

		for (b3BodyWorldShape* s2 = m_body->m_worldShapeList.m_head; s2 != nullptr; s2 = s2->m_next)
		{
			b3AABB aabb2 = s2->ComputeAABB();

			if (b3TestOverlap(aabb1, aabb2))
			{
				AddPair(s1, s2);
			}
		}
	}
}

void b3ContactManager::Destroy(b3SphereAndShapeContact* contact)
{
	// Remove from the body.
	m_shapeContactList.Remove(contact);
	
	// Call the factory.
	b3SphereAndShapeContact::Destroy(contact, m_allocator);
}

void b3ContactManager::UpdateContacts()
{
	// Update the state of sphere and shape contacts.
	b3SphereAndShapeContact* c = m_shapeContactList.m_head;
	while (c)
	{
		b3BodySphereShape* s1 = c->m_s1;
		b3Particle* p1 = s1->m_p;

		b3BodyWorldShape* s2 = c->m_s2;

		// Cease the contact if entities must not collide with each other.
		if (p1->m_type != e_dynamicParticle)
		{
			b3SphereAndShapeContact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		b3AABB aabb1 = s1->ComputeAABB();
		b3AABB aabb2 = s2->ComputeAABB();

		// Destroy the contact if AABBs are not overlapping.
		bool overlap = b3TestOverlap(aabb1, aabb2);
		if (overlap == false)
		{
			b3SphereAndShapeContact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		// The contact persists.
		c->Update();

		c = c->m_next;
	}
}