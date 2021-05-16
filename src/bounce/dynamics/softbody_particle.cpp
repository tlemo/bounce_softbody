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

#include <bounce/dynamics/softbody_particle.h>
#include <bounce/dynamics/shapes/softbody_sphere_shape.h>
#include <bounce/dynamics/shapes/softbody_capsule_shape.h>
#include <bounce/dynamics/shapes/softbody_triangle_shape.h>
#include <bounce/dynamics/softbody.h>
#include <bounce/dynamics/forces/softbody_force.h>
#include <bounce/sparse/sparse_force_solver.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/sparse_mat33.h>

b3SoftBodyParticle::b3SoftBodyParticle(const b3SoftBodyParticleDef& def, b3SoftBody* body)
{
	m_body = body;
	m_type = def.type;
	m_position = def.position;
	m_velocity = def.velocity;
	m_force.SetZero();
	m_translation.SetZero();
	m_massDamping = def.massDamping;

	if (m_type == e_dynamicSoftBodyParticle)
	{
		m_mass = scalar(1);
		m_invMass = scalar(1);
	}
	else
	{
		m_mass = scalar(0);
		m_invMass = scalar(0);
	}

	m_meshIndex = def.meshIndex;
	m_userData = def.userData;
}

b3SoftBodyParticle::~b3SoftBodyParticle()
{

}

void b3SoftBodyParticle::SetType(b3SoftBodyParticleType type)
{
	if (m_type == type)
	{
		return;
	}

	m_type = type;
	
	if (m_type == e_staticSoftBodyParticle || m_type == e_kinematicSoftBodyParticle)
	{
		m_mass = scalar(0);
		m_invMass = scalar(0);
	}
	else
	{
		m_body->ResetMass();
	}

	m_force.SetZero();
	m_translation.SetZero();

	if (type == e_staticSoftBodyParticle)
	{
		m_velocity.SetZero();
		SynchronizeShapes();
	}

	DestroyContacts();

	// Move the proxies so new contacts can be created.
	TouchProxies();
}

void b3SoftBodyParticle::DestroyShapes()
{
	{
		// Destroy spheres
		b3SoftBodySphereShape* s = m_body->m_sphereShapeList.m_head;
		while (s)
		{
			b3SoftBodySphereShape* s0 = s;
			s = s->m_next;

			if (s0->m_p == this)
			{
				m_body->DestroySphereShape(s0);
			}
		}
	}

	{
		// Destroy capsules
		b3SoftBodyCapsuleShape* c = m_body->m_capsuleShapeList.m_head;
		while (c)
		{
			b3SoftBodyCapsuleShape* c0 = c;
			c = c->m_next;

			if (c0->m_p1 == this || c0->m_p2 == this)
			{
				m_body->DestroyCapsuleShape(c0);
			}
		}
	}

	{
		// Destroy triangles
		b3SoftBodyTriangleShape* t = m_body->m_triangleShapeList.m_head;
		while (t)
		{
			b3SoftBodyTriangleShape* t0 = t;
			t = t->m_next;

			if (t0->m_p1 == this || t0->m_p2 == this || t0->m_p3 == this)
			{
				m_body->DestroyTriangleShape(t0);
			}
		}
	}
}

void b3SoftBodyParticle::DestroyForces()
{
	b3SoftBodyForce* f = m_body->m_forceList.m_head;
	while (f)
	{
		b3SoftBodyForce* f0 = f;
		f = f->m_next;

		if (f0->HasParticle(this))
		{
			m_body->DestroyForce(f0);
		}
	}
}

void b3SoftBodyParticle::DestroyContacts()
{
	{
		// Destroy shape contacts
		b3SoftBodySphereAndShapeContact* c = m_body->m_contactManager.m_sphereAndShapeContactList.m_head;
		while (c)
		{
			if (c->m_s1->m_p == this)
			{
				b3SoftBodySphereAndShapeContact* quack = c;
				c = c->m_next;
				m_body->m_contactManager.Destroy(quack);
				continue;
			}

			c = c->m_next;
		}
	}

	{
		// Destroy capsule contacts
		b3SoftBodyCapsuleAndCapsuleContact* c = m_body->m_contactManager.m_capsuleAndCapsuleContactList.m_head;
		while (c)
		{
			if (c->m_s1->m_p1 == this ||
				c->m_s1->m_p2 == this ||
				c->m_s2->m_p1 == this ||
				c->m_s2->m_p2 == this)
			{
				b3SoftBodyCapsuleAndCapsuleContact* quack = c;
				c = c->m_next;
				m_body->m_contactManager.Destroy(quack);
				continue;
			}

			c = c->m_next;
		}
	}

	{
		// Destroy triangle contacts
		b3SoftBodySphereAndTriangleContact* c = m_body->m_contactManager.m_sphereAndTriangleContactList.m_head;
		while (c)
		{
			if (c->m_s1->m_p == this ||
				c->m_s2->m_p1 == this ||
				c->m_s2->m_p2 == this ||
				c->m_s2->m_p3 == this)
			{
				b3SoftBodySphereAndTriangleContact* quack = c;
				c = c->m_next;
				m_body->m_contactManager.Destroy(quack);
				continue;
			}

			c = c->m_next;
		}
	}
}

void b3SoftBodyParticle::SynchronizeShapes()
{
	// Synchronize spheres
	for (b3SoftBodySphereShape* s = m_body->m_sphereShapeList.m_head; s; s = s->m_next)
	{
		if (s->m_p == this)
		{
			s->Synchronize(b3Vec3_zero);
		}
	}

	// Synchronize capsules
	for (b3SoftBodyCapsuleShape* c = m_body->m_capsuleShapeList.m_head; c; c = c->m_next)
	{
		if (c->m_p1 == this || c->m_p2 == this)
		{
			c->Synchronize(b3Vec3_zero);
		}
	}

	// Synchronize triangles
	for (b3SoftBodyTriangleShape* t = m_body->m_triangleShapeList.m_head; t; t = t->m_next)
	{
		if (t->m_p1 == this || t->m_p2 == this || t->m_p3 == this)
		{
			t->Synchronize(b3Vec3_zero);
		}
	}
}

void b3SoftBodyParticle::TouchProxies()
{
	// Touch spheres
	for (b3SoftBodySphereShape* s = m_body->m_sphereShapeList.m_head; s; s = s->m_next)
	{
		if (s->m_p == this)
		{
			s->TouchProxy();
		}
	}

	// Touch capsules
	for (b3SoftBodyCapsuleShape* c = m_body->m_capsuleShapeList.m_head; c; c = c->m_next)
	{
		if (c->m_p1 == this || c->m_p2 == this)
		{
			c->TouchProxy();
		}
	}

	// Touch triangles
	for (b3SoftBodyTriangleShape* t = m_body->m_triangleShapeList.m_head; t; t = t->m_next)
	{
		if (t->m_p1 == this || t->m_p2 == this || t->m_p3 == this)
		{
			t->TouchProxy();
		}
	}
}

void b3SoftBodyParticle::ComputeForces(const b3SparseForceSolverData* data)
{
	const b3DenseVec3& v = *data->v;
	b3DenseVec3& f = *data->f;
	b3SparseMat33& dfdv = *data->dfdv;

	u32 i = m_solverId;

	if (m_massDamping > scalar(0))
	{
		b3Vec3 fd = -m_massDamping * m_mass * v[i];

		// Mass damping force
		f[i] += fd;

		// Jacobian
		dfdv(i, i) += b3Mat33Diagonal(-m_massDamping * m_mass);
	}
}