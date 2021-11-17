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

#ifndef B3_CONTACT_MANAGER_H
#define B3_CONTACT_MANAGER_H

#include <bounce/dynamics/contacts/sphere_shape_contact.h>
#include <bounce/common/template/list.h>

class b3Body;
class b3BlockAllocator;

// Contact delegator for b3Body.
class b3ContactManager
{
public:
	void AddPair(b3BodySphereShape* shape1, b3BodyWorldShape* shape2);
	void FindNewContacts();
	void UpdateContacts();

	void Destroy(b3SphereAndShapeContact* contact);

	b3Body* m_body;
	b3BlockAllocator* m_allocator;
	b3List<b3SphereAndShapeContact> m_shapeContactList;
};

#endif