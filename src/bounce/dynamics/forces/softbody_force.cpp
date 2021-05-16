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

#include <bounce/dynamics/forces/softbody_force.h>
#include <bounce/dynamics/forces/softbody_stretch_force.h>
#include <bounce/dynamics/forces/softbody_shear_force.h>
#include <bounce/dynamics/forces/softbody_spring_force.h>
#include <bounce/dynamics/forces/softbody_mouse_force.h>
#include <bounce/dynamics/forces/softbody_triangle_element_force.h>
#include <bounce/dynamics/forces/softbody_tetrahedron_element_force.h>

b3SoftBodyForce* b3SoftBodyForce::Create(const b3SoftBodyForceDef* def)
{
	b3SoftBodyForce* force = nullptr;
	switch (def->type)
	{
	case e_softBodyStretchForce:
	{
		void* block = b3Alloc(sizeof(b3SoftBodyStretchForce));
		force = new (block) b3SoftBodyStretchForce((b3SoftBodyStretchForceDef*)def);
		break;
	}
	case e_softBodyShearForce:
	{
		void* block = b3Alloc(sizeof(b3SoftBodyShearForce));
		force = new (block) b3SoftBodyShearForce((b3SoftBodyShearForceDef*)def);
		break;
	}
	case e_softBodySpringForce:
	{
		void* block = b3Alloc(sizeof(b3SoftBodySpringForce));
		force = new (block) b3SoftBodySpringForce((b3SoftBodySpringForceDef*)def);
		break;
	}
	case e_softBodyMouseForce:
	{
		void* block = b3Alloc(sizeof(b3SoftBodyMouseForce));
		force = new (block) b3SoftBodyMouseForce((b3SoftBodyMouseForceDef*)def);
		break;
	}
	case e_softBodyTriangleElementForce:
	{
		void* block = b3Alloc(sizeof(b3SoftBodyTriangleElementForce));
		force = new (block) b3SoftBodyTriangleElementForce((b3SoftBodyTriangleElementForceDef*)def);
		break;
	}
	case e_softBodyTetrahedronElementForce:
	{
		void* block = b3Alloc(sizeof(b3SoftBodyTetrahedronElementForce));
		force = new (block) b3SoftBodyTetrahedronElementForce((b3SoftBodyTetrahedronElementForceDef*)def);
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
	return force;
}

void b3SoftBodyForce::Destroy(b3SoftBodyForce* force)
{
	B3_ASSERT(force);

	b3SoftBodyForceType type = force->GetType();
	switch (type)
	{
	case e_softBodyStretchForce:
	{
		b3SoftBodyStretchForce* o = (b3SoftBodyStretchForce*)force;
		o->~b3SoftBodyStretchForce();
		b3Free(force);
		break;
	}
	case e_softBodyShearForce:
	{
		b3SoftBodyShearForce* o = (b3SoftBodyShearForce*)force;
		o->~b3SoftBodyShearForce();
		b3Free(force);
		break;
	}
	case e_softBodySpringForce:
	{
		b3SoftBodySpringForce* o = (b3SoftBodySpringForce*)force;
		o->~b3SoftBodySpringForce();
		b3Free(force);
		break;
	}
	case e_softBodyMouseForce:
	{
		b3SoftBodyMouseForce* o = (b3SoftBodyMouseForce*)force;
		o->~b3SoftBodyMouseForce();
		b3Free(force);
		break;
	}
	case e_softBodyTriangleElementForce:
	{
		b3SoftBodyTriangleElementForce* o = (b3SoftBodyTriangleElementForce*)force;
		o->~b3SoftBodyTriangleElementForce();
		b3Free(force);
		break;
	}
	case e_softBodyTetrahedronElementForce:
	{
		b3SoftBodyTetrahedronElementForce* o = (b3SoftBodyTetrahedronElementForce*)force;
		o->~b3SoftBodyTetrahedronElementForce();
		b3Free(force);
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	};
}