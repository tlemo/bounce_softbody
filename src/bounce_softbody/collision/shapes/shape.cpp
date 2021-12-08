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

#include <bounce_softbody/collision/shapes/shape.h>
#include <bounce_softbody/collision/shapes/sphere_shape.h>
#include <bounce_softbody/collision/shapes/capsule_shape.h>
#include <bounce_softbody/collision/shapes/box_shape.h>
#include <bounce_softbody/common/memory/block_allocator.h>

void b3Shape::Destroy(b3Shape* shape, b3BlockAllocator* allocator)
{
	switch (shape->m_type)
	{
	case e_sphere:
	{
		b3SphereShape* sphere = (b3SphereShape*)shape;
		sphere->~b3SphereShape();
		allocator->Free(sphere, sizeof(b3SphereShape));
		break;
	}
	case e_capsule:
	{
		b3CapsuleShape* capsule = (b3CapsuleShape*)shape;
		capsule->~b3CapsuleShape();
		allocator->Free(capsule, sizeof(b3CapsuleShape));
		break;
	}
	case e_box:
	{
		b3BoxShape* box = (b3BoxShape*)shape;
		box->~b3BoxShape();
		allocator->Free(box, sizeof(b3BoxShape));
		break;
	}
	default:
	{
		B3_ASSERT(false);
	}
	}
}