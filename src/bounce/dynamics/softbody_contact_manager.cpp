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

#include <bounce/dynamics/softbody_contact_manager.h>
#include <bounce/dynamics/softbody.h>
#include <bounce/dynamics/shapes/softbody_sphere_shape.h>
#include <bounce/dynamics/shapes/softbody_capsule_shape.h>
#include <bounce/dynamics/shapes/softbody_triangle_shape.h>
#include <bounce/dynamics/shapes/softbody_world_shape.h>
#include <bounce/dynamics/softbody_particle.h>

b3SoftBodyContactManager::b3SoftBodyContactManager() : 
	m_sphereAndTriangleContactBlocks(sizeof(b3SoftBodySphereAndTriangleContact)),
	m_sphereAndShapeContactBlocks(sizeof(b3SoftBodySphereAndShapeContact)),
	m_capsuleAndCapsuleContactBlocks(sizeof(b3SoftBodyCapsuleAndCapsuleContact))
{

}

void b3SoftBodyContactManager::FindNewContacts()
{
	B3_PROFILE("Soft Body Find New Contacts");

	m_broadPhase.FindPairs(this);
}

void b3SoftBodyContactManager::AddPair(void* data1, void* data2)
{
	b3SoftBodyShapeBaseProxy* proxy1 = (b3SoftBodyShapeBaseProxy*)data1;
	b3SoftBodyShapeBase* base1 = proxy1->shape;
	b3SoftBodyShapeBaseType baseType1 = base1->m_baseType;
	
	b3SoftBodyShapeBaseProxy* proxy2 = (b3SoftBodyShapeBaseProxy*)data2;
	b3SoftBodyShapeBase* base2 = proxy2->shape;
	b3SoftBodyShapeBaseType baseType2 = base2->m_baseType;

	if (baseType1 == e_softBodyWorldShapeBase && baseType2 == e_softBodyWorldShapeBase)
	{
		return;
	}

	if (baseType1 == e_softBodyShapeBase && baseType2 == e_softBodyShapeBase)
	{
		if (m_body->m_enableSelfCollision == false)
		{
			// Self collision is disabled.
			return;
		}

		b3SoftBodyShape* shape1 = (b3SoftBodyShape*)base1;
		b3SoftBodyShapeType shapeType1 = shape1->GetShapeType();
		
		b3SoftBodyShape* shape2 = (b3SoftBodyShape*)base2;
		b3SoftBodyShapeType shapeType2 = shape2->GetShapeType();

		if (shapeType1 > shapeType2)
		{
			// Ensure type1 < type2.
			b3Swap(proxy1, proxy2);
			b3Swap(base1, base2);
			b3Swap(baseType1, baseType2);
			b3Swap(shape1, shape2);
			b3Swap(shapeType1, shapeType2);
		}
		
		if (shapeType1 == e_softBodySphereShape && shapeType2 == e_softBodySphereShape)
		{
			return;
		}

		if (shapeType1 == e_softBodySphereShape && shapeType2 == e_softBodyCapsuleShape)
		{
			return;
		}

		if (shapeType1 == e_softBodyCapsuleShape && shapeType2 == e_softBodyTriangleShape)
		{
			return;
		}

		if (shapeType1 == e_softBodyTriangleShape && shapeType2 == e_softBodyTriangleShape)
		{
			return;
		}
		
		if (shapeType1 == e_softBodyCapsuleShape && shapeType2 == e_softBodyCapsuleShape)
		{
			b3SoftBodyCapsuleShape* s1 = (b3SoftBodyCapsuleShape*)shape1;
			b3SoftBodyCapsuleShape* s2 = (b3SoftBodyCapsuleShape*)shape2;

			b3SoftBodyParticle* p1 = s1->m_p1;
			b3SoftBodyParticle* p2 = s1->m_p2;

			b3SoftBodyParticle* p3 = s2->m_p1;
			b3SoftBodyParticle* p4 = s2->m_p2;

			bool isntDynamic1 = p1->m_type != e_dynamicSoftBodyParticle && p2->m_type != e_dynamicSoftBodyParticle;
			bool isntDynamic2 = p3->m_type != e_dynamicSoftBodyParticle && p4->m_type != e_dynamicSoftBodyParticle;

			if (isntDynamic1 && isntDynamic2)
			{
				// The entities must not collide with each other.
				return;
			}

			// Do the edges share a vertex?
			if (p1 == p3 || p1 == p4)
			{
				return;
			}

			if (p2 == p3 || p2 == p4)
			{
				return;
			}

			// Check if there is a contact between the two capsules.
			for (b3SoftBodyCapsuleAndCapsuleContact* c = m_capsuleAndCapsuleContactList.m_head; c; c = c->m_next)
			{
				if (c->m_s1 == s1 && c->m_s2 == s2)
				{
					// A contact already exists.
					return;
				}

				if (c->m_s1 == s2 && c->m_s2 == s1)
				{
					// A contact already exists.
					return;
				}
			}

			// Create a new contact.
			b3SoftBodyCapsuleAndCapsuleContact* c = CreateCapsuleAndCapsuleContact();

			c->m_s1 = s1;
			c->m_s2 = s2;
			c->m_normalImpulse = scalar(0);
			c->m_tangentImpulse.SetZero();
			c->m_active = false;

			// Add the contact to the body contact list.
			m_capsuleAndCapsuleContactList.PushFront(c);

			// Success.
			return;
		}

		if (shapeType1 == e_softBodySphereShape && shapeType2 == e_softBodyTriangleShape)
		{
			b3SoftBodySphereShape* s1 = (b3SoftBodySphereShape*)shape1;
			b3SoftBodyTriangleShape* s2 = (b3SoftBodyTriangleShape*)shape2;

			b3SoftBodyParticle* p1 = s1->m_p;

			b3SoftBodyParticle* p2 = s2->m_p1;
			b3SoftBodyParticle* p3 = s2->m_p2;
			b3SoftBodyParticle* p4 = s2->m_p3;

			bool isntDynamic1 = p1->m_type != e_dynamicSoftBodyParticle;
			bool isntDynamic2 = p2->m_type != e_dynamicSoftBodyParticle && p3->m_type != e_dynamicSoftBodyParticle && p4->m_type != e_dynamicSoftBodyParticle;

			if (isntDynamic1 && isntDynamic2)
			{
				// The entities must not collide with each other.
				return;
			}

			if (p1 == p2 || p1 == p3 || p1 == p4)
			{
				// The entities must not collide with each other.
				return;
			}

			// Check if there is a contact between the two entities.
			for (b3SoftBodySphereAndTriangleContact* c = m_sphereAndTriangleContactList.m_head; c; c = c->m_next)
			{
				if (c->m_s1 == s1 && c->m_s2 == s2)
				{
					// A contact already exists.
					return;
				}
			}

			// Create a new contact.
			b3SoftBodySphereAndTriangleContact* c = CreateSphereAndTriangleContact();

			c->m_s1 = s1;
			c->m_s2 = s2;
			c->m_normalImpulse = scalar(0);
			c->m_tangentImpulse.SetZero();
			c->m_active = false;

			// Add the contact to the body contact list.
			m_sphereAndTriangleContactList.PushFront(c);

			return;
		}

		// Succeded.
		return;
	}
	
	if (baseType1 > baseType2)
	{
		// Ensure type1 < type2.
		b3Swap(proxy1, proxy2);
		b3Swap(base1, base2);
		b3Swap(baseType1, baseType2);
	}
	
	B3_ASSERT(baseType1 == e_softBodyShapeBase);
	B3_ASSERT(baseType2 == e_softBodyWorldShapeBase);

	b3SoftBodyShape* shape1 = (b3SoftBodyShape*)base1;
	b3SoftBodyShapeType shapeType1 = shape1->GetShapeType();
	
	b3SoftBodyWorldShape* shape2 = (b3SoftBodyWorldShape*)base2;

	if (shapeType1 == e_softBodyCapsuleShape)
	{
		return;
	}
	
	if (shapeType1 == e_softBodyTriangleShape)
	{
		return;
	}

	if (shapeType1 == e_softBodySphereShape)
	{
		b3SoftBodySphereShape* s1 = (b3SoftBodySphereShape*)shape1;
		b3SoftBodyParticle* p1 = s1->m_p;

		b3SoftBodyWorldShape* s2 = (b3SoftBodyWorldShape*)shape2;

		if (p1->GetType() != e_dynamicSoftBodyParticle)
		{
			// The shapes must not collide with each other.
			return;
		}

		// Check if there is a contact between the two entities.
		for (b3SoftBodySphereAndShapeContact* c = m_sphereAndShapeContactList.m_head; c; c = c->m_next)
		{
			if (c->m_s1 == s1 && c->m_s2 == s2)
			{
				// A contact already exists.
				return;
			}
		}

		// Create a new contact.
		b3SoftBodySphereAndShapeContact* c = CreateSphereAndShapeContact();

		c->m_s1 = s1;
		c->m_s2 = s2;
		c->m_active = false;
		c->m_normalImpulse = scalar(0);
		c->m_tangentImpulse.SetZero();

		// Push the contact to the contact list.
		m_sphereAndShapeContactList.PushFront(c);

		// Success.
		return;
	}
}

b3SoftBodySphereAndTriangleContact* b3SoftBodyContactManager::CreateSphereAndTriangleContact()
{
	void* block = m_sphereAndTriangleContactBlocks.Allocate();
	return new(block) b3SoftBodySphereAndTriangleContact();
}

void b3SoftBodyContactManager::Destroy(b3SoftBodySphereAndTriangleContact* c)
{
	m_sphereAndTriangleContactList.Remove(c);
	c->~b3SoftBodySphereAndTriangleContact();
	m_sphereAndTriangleContactBlocks.Free(c);
}

b3SoftBodySphereAndShapeContact* b3SoftBodyContactManager::CreateSphereAndShapeContact()
{
	void* block = m_sphereAndShapeContactBlocks.Allocate();
	return new(block) b3SoftBodySphereAndShapeContact();
}

void b3SoftBodyContactManager::Destroy(b3SoftBodySphereAndShapeContact* c)
{
	m_sphereAndShapeContactList.Remove(c);
	c->~b3SoftBodySphereAndShapeContact();
	m_sphereAndShapeContactBlocks.Free(c);
}

b3SoftBodyCapsuleAndCapsuleContact* b3SoftBodyContactManager::CreateCapsuleAndCapsuleContact()
{
	void* block = m_capsuleAndCapsuleContactBlocks.Allocate();
	return new(block) b3SoftBodyCapsuleAndCapsuleContact();
}

void b3SoftBodyContactManager::Destroy(b3SoftBodyCapsuleAndCapsuleContact* c)
{
	m_capsuleAndCapsuleContactList.Remove(c);
	c->~b3SoftBodyCapsuleAndCapsuleContact();
	m_capsuleAndCapsuleContactBlocks.Free(c);
}

void b3SoftBodyContactManager::UpdateContacts()
{
	B3_PROFILE("Soft Body Update Contacts");
	
	{
		// Update the state of sphere and shape contacts.
		b3SoftBodySphereAndShapeContact* c = m_sphereAndShapeContactList.m_head;
		while (c)
		{
			b3SoftBodySphereShape* s1 = c->m_s1;
			b3SoftBodyParticle* p1 = s1->m_p;

			// Cease the contact if entities must not collide with each other.
			if (p1->m_type != e_dynamicSoftBodyParticle)
			{
				b3SoftBodySphereAndShapeContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			u32 proxy1 = c->m_s1->m_proxy.proxyId;
			u32 proxy2 = c->m_s2->m_proxy.proxyId;

			// Destroy the contact if AABBs are not overlapping.
			bool overlap = m_broadPhase.TestOverlap(proxy1, proxy2);
			if (overlap == false)
			{
				b3SoftBodySphereAndShapeContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			// The contact persists.
			c->Update();

			c = c->m_next;
		}
	}

	{
		// Update the state of sphere and triangle contacts.
		b3SoftBodySphereAndTriangleContact* c = m_sphereAndTriangleContactList.m_head;
		while (c)
		{
			b3SoftBodySphereShape* s1 = c->m_s1;
			b3SoftBodyParticle* p1 = s1->m_p;

			b3SoftBodyTriangleShape* s2 = c->m_s2;
			b3SoftBodyParticle* p2 = s2->m_p1;
			b3SoftBodyParticle* p3 = s2->m_p2;
			b3SoftBodyParticle* p4 = s2->m_p3;

			bool isntDynamic1 = p1->m_type != e_dynamicSoftBodyParticle;
			
			bool isntDynamic2 = 
				p2->m_type != e_dynamicSoftBodyParticle && 
				p3->m_type != e_dynamicSoftBodyParticle && 
				p4->m_type != e_dynamicSoftBodyParticle;

			// Destroy the contact if shapes must not collide with each other.
			if (isntDynamic1 && isntDynamic2)
			{
				b3SoftBodySphereAndTriangleContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			u32 proxy1 = c->m_s1->m_proxy.proxyId;
			u32 proxy2 = c->m_s2->m_proxy.proxyId;

			// Destroy the contact if primitive AABBs are not overlapping.
			bool overlap = m_broadPhase.TestOverlap(proxy1, proxy2);
			if (overlap == false)
			{
				b3SoftBodySphereAndTriangleContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			// The contact persists.
			c->Update();

			c = c->m_next;
		}
	}

	{
		// Update the state of capsule contacts.
		b3SoftBodyCapsuleAndCapsuleContact* c = m_capsuleAndCapsuleContactList.m_head;
		while (c)
		{
			b3SoftBodyCapsuleShape* s1 = c->m_s1;
			b3SoftBodyParticle* p1 = s1->m_p1;
			b3SoftBodyParticle* p2 = s1->m_p2;

			b3SoftBodyCapsuleShape* s2 = c->m_s2;
			b3SoftBodyParticle* p3 = s2->m_p1;
			b3SoftBodyParticle* p4 = s2->m_p2;
			
			bool isntDynamic1 = p1->m_type != e_dynamicSoftBodyParticle && p2->m_type != e_dynamicSoftBodyParticle;
			bool isntDynamic2 = p3->m_type != e_dynamicSoftBodyParticle && p4->m_type != e_dynamicSoftBodyParticle;

			// Destroy the contact if primitives must not collide with each other.
			if (isntDynamic1 && isntDynamic2)
			{
				b3SoftBodyCapsuleAndCapsuleContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			u32 proxy1 = c->m_s1->m_proxy.proxyId;
			u32 proxy2 = c->m_s2->m_proxy.proxyId;

			// Destroy the contact if AABBs are not overlapping.
			bool overlap = m_broadPhase.TestOverlap(proxy1, proxy2);
			if (overlap == false)
			{
				b3SoftBodyCapsuleAndCapsuleContact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}

			// The contact persists.
			c->Update();

			c = c->m_next;
		}
	}
}