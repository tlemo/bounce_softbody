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

#include <bounce/dynamics/contacts/softbody_sphere_shape_contact.h>
#include <bounce/dynamics/shapes/softbody_sphere_shape.h>
#include <bounce/dynamics/shapes/softbody_world_shape.h>
#include <bounce/dynamics/softbody_particle.h>
#include <bounce/collision/geometry/sphere.h>
#include <bounce/common/memory/block_allocator.h>
#include <bounce/sparse/sparse_force_solver.h>
#include <bounce/sparse/sparse_mat33.h>
#include <bounce/sparse/dense_vec3.h>

b3SoftBodySphereAndShapeContact* b3SoftBodySphereAndShapeContact::Create(b3SoftBodySphereShape* s1, b3SoftBodyWorldShape* s2, b3BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(b3SoftBodySphereAndShapeContact));
	return new(mem) b3SoftBodySphereAndShapeContact(s1, s2);
}

void b3SoftBodySphereAndShapeContact::Destroy(b3SoftBodySphereAndShapeContact* contact, b3BlockAllocator* allocator)
{
	contact->~b3SoftBodySphereAndShapeContact();
	allocator->Free(contact, sizeof(b3SoftBodySphereAndShapeContact));
}

b3SoftBodySphereAndShapeContact::b3SoftBodySphereAndShapeContact(b3SoftBodySphereShape* s1, b3SoftBodyWorldShape* s2)
{
	m_s1 = s1;
	m_s2 = s2;
	m_normalForce = scalar(0);
	m_active = false;
}

void b3SoftBodySphereAndShapeContact::Update()
{
	m_normalForce = scalar(0);
	m_active = false;

	b3Sphere sphere;
	sphere.vertex = m_s1->m_p->m_position;
	sphere.radius = m_s1->m_radius;

	b3SphereManifold manifold;
	if (m_s2->CollideSphere(&manifold, sphere) == false)
	{
		return;
	}

	// The friction solver uses initial tangents.
	m_tangent1 = b3Perp(manifold.normal);
	m_tangent2 = b3Cross(m_tangent1, manifold.normal);
	m_active = true;
}

void b3SoftBodySphereAndShapeContact::ComputeForces(const b3SparseForceSolverData* data)
{
	const b3DenseVec3& x = *data->x;
	const b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdx = *data->dfdx;
	b3SparseMat33& dfdv = *data->dfdv;

	b3SoftBodyParticle* p1 = m_s1->m_p;

	u32 i1 = p1->m_solverId;

	b3Vec3 x1 = x[i1];
	b3Vec3 v1 = v[i1];

	scalar r1 = m_s1->m_radius;
	scalar r2 = m_s2->m_shape->m_radius;

	b3Sphere sphere1;
	sphere1.vertex = x1;
	sphere1.radius = r1;

	// Evaluate the contact manifold.
	b3SphereManifold manifold2;
	if (m_s2->CollideSphere(&manifold2, sphere1) == false)
	{
		return;
	}

	b3Vec3 x2 = manifold2.point;
	b3Vec3 n2 = manifold2.normal;

	b3Vec3 c1 = x1 - r1 * n2;
	b3Vec3 c2 = x2 + r2 * n2;

	// Force computation requires normal direction from shape 1 to shape 2.
	b3Vec3 n1 = -n2;

	// Theodore Kim and David Eberle:
	// "Dynamic Deformables: Implementation and Production Practicalities", page 143.

	// Apply normal force.
	if (B3_CONTACT_STIFFNESS > scalar(0))
	{
		// There is no spring rest lenght.
		// Therefore, there is no compression force.
		scalar C = b3Length(c2 - c1);

		// Clamp correction to prevent large forces.
		C = b3Min(C, B3_MAX_CONTACT_LINEAR_CORRECTION);

		// Spring force
		b3Vec3 f1 = -B3_CONTACT_STIFFNESS * C * n1;

		b3Mat33 I = b3Mat33_identity;

		// Jacobian
		b3Mat33 K11 = -B3_CONTACT_STIFFNESS * (b3Outer(n1, n1) + C * (I - b3Outer(n1, n1)));

		// Apply 
		f[i1] += f1;
		dfdx(i1, i1) += K11;

		// Accumulate normal force magnitude for friction.
		m_normalForce += b3Length(f1);
	}

	// Apply damping force.
	if (B3_CONTACT_DAMPING_STIFFNESS > scalar(0))
	{
		scalar dCdt = b3Dot(v1, n1);

		// Damping force
		b3Vec3 f1 = -B3_CONTACT_DAMPING_STIFFNESS * dCdt * n1;

		// Jacobian
		b3Mat33 K11 = -B3_CONTACT_DAMPING_STIFFNESS * b3Outer(n1, n1);

		// Apply force and Jacobian
		f[i1] += f1;
		dfdv(i1, i1) += K11;
	}
}