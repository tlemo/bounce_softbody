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

#include <bounce_softbody/dynamics/body.h>
#include <bounce_softbody/dynamics/particle.h>
#include <bounce_softbody/dynamics/time_step.h>
#include <bounce_softbody/dynamics/body_solver.h>
#include <bounce_softbody/dynamics/forces/force.h>
#include <bounce_softbody/dynamics/fixtures/sphere_fixture.h>
#include <bounce_softbody/dynamics/fixtures/triangle_fixture.h>
#include <bounce_softbody/dynamics/fixtures/tetrahedron_fixture.h>
#include <bounce_softbody/dynamics/fixtures/world_fixture.h>
#include <bounce_softbody/common/draw.h>

b3Body::b3Body()
{
	m_contactManager.m_body = this;
	m_contactManager.m_allocator = &m_blockAllocator;
	
	m_gravity.SetZero();
}

b3Body::~b3Body()
{
	// None of the objects use b3Alloc.
}

b3Particle* b3Body::CreateParticle(const b3ParticleDef& def)
{
	void* mem = m_blockAllocator.Allocate(sizeof(b3Particle));
	b3Particle* p = new(mem) b3Particle(def, this);

	// Add to body list.
	m_particleList.PushFront(p);

	return p;
}

void b3Body::DestroyParticle(b3Particle* particle)
{
	// Delete the attached objects.
	particle->DestroyFixtures();
	particle->DestroyForces();
	particle->DestroyContacts();

	// Remove from body list.
	m_particleList.Remove(particle);
	
	particle->~b3Particle();
	m_blockAllocator.Free(particle, sizeof(b3Particle));
}

b3SphereFixture* b3Body::CreateSphere(const b3SphereFixtureDef& def)
{
	// Check if the fixture exists.
	for (b3SphereFixture* s = m_sphereList.m_head; s; s = s->m_next)
	{
		if (s->m_p == def.p)
		{
			return s;
		}
	}
	
	void* mem = m_blockAllocator.Allocate(sizeof(b3SphereFixture));
	b3SphereFixture* s = new (mem)b3SphereFixture(def, this);
	
	// Add to body list.
	m_sphereList.PushFront(s);

	return s;
}

void b3Body::DestroySphere(b3SphereFixture* fixture)
{
	// Destroy attached objects.
	fixture->DestroyContacts();

	// Remove from body list.
	m_sphereList.Remove(fixture);
	
	fixture->~b3SphereFixture();
	m_blockAllocator.Free(fixture, sizeof(b3SphereFixture));
}

b3TriangleFixture* b3Body::CreateTriangle(const b3TriangleFixtureDef& def)
{
	// Check if the fixture exists.
	b3Particle * p1 = def.p1, * p2 = def.p2, * p3 = def.p3;
	for (b3TriangleFixture* t = m_triangleList.m_head; t; t = t->m_next)
	{
		bool hasP1 = t->m_p1 == p1 || t->m_p2 == p1 || t->m_p3 == p1;
		bool hasP2 = t->m_p1 == p2 || t->m_p2 == p2 || t->m_p3 == p2;
		bool hasP3 = t->m_p1 == p3 || t->m_p2 == p3 || t->m_p3 == p3;

		if (hasP1 && hasP2 && hasP3)
		{
			return t;
		}
	}
	
	void* mem = m_blockAllocator.Allocate(sizeof(b3TriangleFixture));
	b3TriangleFixture* t = new (mem)b3TriangleFixture(def, this);

	// Create tree proxy.
	b3AABB aabb = t->ComputeAABB();
	t->m_proxyId = m_tree.CreateProxy(aabb, t);

	// Add to body list.
	m_triangleList.PushFront(t);

	// Reset the body mass
	ResetMass();

	return t;
}

void b3Body::DestroyTriangle(b3TriangleFixture* fixture)
{
	// Destroy tree proxy.
	m_tree.DestroyProxy(fixture->m_proxyId);

	// Remove from body list.
	m_triangleList.Remove(fixture);
	
	fixture->~b3TriangleFixture();
	m_blockAllocator.Free(fixture, sizeof(b3TriangleFixture));

	// Reset the body mass
	ResetMass();
}

b3TetrahedronFixture* b3Body::CreateTetrahedron(const b3TetrahedronFixtureDef& def)
{
	// Check if the fixture exists.
	b3Particle * p1 = def.p1, * p2 = def.p2, * p3 = def.p3, * p4 = def.p4;
	for (b3TetrahedronFixture* t = m_tetrahedronList.m_head; t; t = t->m_next)
	{
		bool hasP1 = t->m_p1 == p1 || t->m_p2 == p1 || t->m_p3 == p1 || t->m_p4 == p1;
		bool hasP2 = t->m_p1 == p2 || t->m_p2 == p2 || t->m_p3 == p2 || t->m_p4 == p2;
		bool hasP3 = t->m_p1 == p3 || t->m_p2 == p3 || t->m_p3 == p3 || t->m_p4 == p3;
		bool hasP4 = t->m_p1 == p4 || t->m_p2 == p4 || t->m_p3 == p4 || t->m_p4 == p4;

		if (hasP1 && hasP2 && hasP3 && hasP4)
		{
			return t;
		}
	}

	void* mem = m_blockAllocator.Allocate(sizeof(b3TetrahedronFixture));
	b3TetrahedronFixture* t = new (mem)b3TetrahedronFixture(def, this);

	// Add to body list.
	m_tetrahedronList.PushFront(t);

	// Reset the body mass.
	ResetMass();

	return t;
}

void b3Body::DestroyTetrahedron(b3TetrahedronFixture* fixture)
{
	// Remove from body list.
	m_tetrahedronList.Remove(fixture);
	
	fixture->~b3TetrahedronFixture();
	m_blockAllocator.Free(fixture, sizeof(b3TetrahedronFixture));

	// Reset the body mass
	ResetMass();
}

b3Force* b3Body::CreateForce(const b3ForceDef& def)
{
	// Call the factory.
	b3Force* f = b3Force::Create(&def, &m_blockAllocator);
	
	// Add to body list.
	m_forceList.PushFront(f);
	return f;
}

void b3Body::DestroyForce(b3Force* force)
{
	// Remove from body list.
	m_forceList.Remove(force);
	
	// Call the factory
	b3Force::Destroy(force, &m_blockAllocator);
}

b3WorldFixture* b3Body::CreateFixture(const b3WorldFixtureDef& def)
{
	void* mem = m_blockAllocator.Allocate(sizeof(b3WorldFixture));
	b3WorldFixture* fixture = new (mem) b3WorldFixture;
	fixture->Create(&m_blockAllocator, this, def);

	// Add to the body list
	m_fixtureList.PushFront(fixture);

	return fixture;
}

void b3Body::DestroyFixture(b3WorldFixture* fixture)
{
	// Destroy attached contacts.
	fixture->DestroyContacts();

	// Remove from the body list.
	m_fixtureList.Remove(fixture);

	fixture->Destroy(&m_blockAllocator);
	fixture->~b3WorldFixture();
	m_blockAllocator.Free(fixture, sizeof(b3WorldFixture));
}

scalar b3Body::GetEnergy() const
{
	scalar E = scalar(0);
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		E += p->m_mass * b3Dot(p->m_velocity, p->m_velocity);
	}
	return scalar(0.5) * E;
}

void b3Body::ResetMass()
{
	// Clear masses. 
	// Only touch fixture masses because there can be external particles.
	for (b3TriangleFixture* t = m_triangleList.m_head; t; t = t->m_next)
	{
		t->m_p1->m_mass = scalar(0);
		t->m_p2->m_mass = scalar(0);
		t->m_p3->m_mass = scalar(0);
	}

	for (b3TetrahedronFixture* t = m_tetrahedronList.m_head; t; t = t->m_next)
	{
		t->m_p1->m_mass = scalar(0);
		t->m_p2->m_mass = scalar(0);
		t->m_p3->m_mass = scalar(0);
		t->m_p4->m_mass = scalar(0);
	}

	// Accumulate contribution of each fixture.
	const scalar inv3 = scalar(1) / scalar(3);
	for (b3TriangleFixture* t = m_triangleList.m_head; t; t = t->m_next)
	{
		b3Particle* p1 = t->m_p1;
		b3Particle* p2 = t->m_p2;
		b3Particle* p3 = t->m_p3;

		scalar mass = t->m_density * t->m_area;

		p1->m_mass += inv3 * mass;
		p2->m_mass += inv3 * mass;
		p3->m_mass += inv3 * mass;
	}

	const scalar inv4 = scalar(1) / scalar(4);
	for (b3TetrahedronFixture* t = m_tetrahedronList.m_head; t; t = t->m_next)
	{
		b3Particle* p1 = t->m_p1;
		b3Particle* p2 = t->m_p2;
		b3Particle* p3 = t->m_p3;
		b3Particle* p4 = t->m_p4;

		scalar mass = t->m_density * t->m_volume;

		p1->m_mass += inv4 * mass;
		p2->m_mass += inv4 * mass;
		p3->m_mass += inv4 * mass;
		p4->m_mass += inv4 * mass;
	}

	// Invert
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		// Static and kinematic particles have zero mass.
		if (p->m_type == e_staticParticle || p->m_type == e_kinematicParticle)
		{
			p->m_mass = scalar(0);
			p->m_invMass = scalar(0);
			continue;
		}

		if (p->m_mass > scalar(0))
		{
			p->m_invMass = scalar(1) / p->m_mass;
		}
		else
		{
			// Force all dynamic particles to have non-zero mass.
			p->m_mass = scalar(1);
			p->m_invMass = scalar(1);
		}
	}
}

struct b3BodyRayCastSingleWrapper
{
	scalar Report(const b3RayCastInput& input, u32 proxyId)
	{
		// Get fixture associated with the proxy.
		void* userData = tree->GetUserData(proxyId);
		b3TriangleFixture* triangle = (b3TriangleFixture*)userData;

		b3RayCastOutput subOutput;
		if (triangle->RayCast(&subOutput, input))
		{
			// Ray hits triangle.
			if (subOutput.fraction < output0.fraction)
			{
				triangle0 = triangle;
				output0.fraction = subOutput.fraction;
				output0.normal = subOutput.normal;
			}
		}

		// Continue search from where we stopped.
		return input.maxFraction;
	}

	const b3DynamicTree* tree;
	b3TriangleFixture* triangle0;
	b3RayCastOutput output0;
};

bool b3Body::RayCastSingle(b3BodyRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3BodyRayCastSingleWrapper wrapper;
	wrapper.tree = &m_tree;
	wrapper.triangle0 = nullptr;
	wrapper.output0.fraction = B3_MAX_SCALAR;
	
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = scalar(1);

	m_tree.RayCast(&wrapper, input);

	if (wrapper.triangle0 != nullptr)
	{
		output->triangle = wrapper.triangle0;
		output->fraction = wrapper.output0.fraction;
		output->normal = wrapper.output0.normal;

		return true;
	}

	return false;
}

void b3Body::Solve(const b3TimeStep& step)
{
	b3BodySolverDef solverDef;
	solverDef.stack = &m_stackAllocator;
	solverDef.particleCapacity = m_particleList.m_count;
	solverDef.forceCapacity = m_forceList.m_count;
	solverDef.shapeContactCapacity = m_contactManager.m_shapeContactList.m_count;
	
	b3BodySolver solver(solverDef);

	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		solver.Add(p);
	}

	for (b3Force* f = m_forceList.m_head; f; f = f->m_next)
	{
		solver.Add(f);
	}

	for (b3SphereAndShapeContact* c = m_contactManager.m_shapeContactList.m_head; c; c = c->m_next)
	{
		solver.Add(c);
	}

	// Solve
	solver.Solve(step, m_gravity);
}

void b3Body::Step(scalar dt, u32 forceIterations, u32 forceSubIterations)
{
	// Time step parameters
	b3TimeStep step;
	step.dt = dt;
	step.forceIterations = forceIterations;
	step.forceSubIterations = forceSubIterations;
	step.inv_dt = dt > scalar(0) ? scalar(1) / dt : scalar(0);
	
	// Update contacts. This is where some contacts are ceased.
	m_contactManager.UpdateContacts();

	// Clear internal forces before accumulating them inside the solver.
	for (b3Force* f = m_forceList.m_head; f; f = f->m_next)
	{
		f->ClearForces();
	}

	// Integrate state, solve constraints. 
	if (step.dt > scalar(0))
	{
		Solve(step);
	}

	// Clear external forces and translations.
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		p->m_force.SetZero();
		p->m_translation.SetZero();
	}

	// Synchronize triangles.
	for (b3TriangleFixture* t = m_triangleList.m_head; t; t = t->m_next)
	{
		b3Particle* p1 = t->m_p1;
		b3Particle* p2 = t->m_p2;
		b3Particle* p3 = t->m_p3;

		b3Vec3 v1 = p1->m_velocity;
		b3Vec3 v2 = p2->m_velocity;
		b3Vec3 v3 = p3->m_velocity;

		// Center velocity
		b3Vec3 velocity = (v1 + v2 + v3) / scalar(3);

		b3Vec3 displacement = dt * velocity;

		t->Synchronize(displacement);
	}

	// Find new contacts
	m_contactManager.FindNewContacts();
}

void b3Body::Draw(b3Draw* draw) const
{
	for (b3Particle* p = m_particleList.m_head; p; p = p->m_next)
	{
		if (p->m_type == e_staticParticle)
		{
			draw->DrawPoint(p->m_position, 4.0, b3Color_white);	
		}

		if (p->m_type == e_kinematicParticle)
		{
			draw->DrawPoint(p->m_position, 4.0, b3Color_blue);
		}

		if (p->m_type == e_dynamicParticle)
		{
			draw->DrawPoint(p->m_position, 4.0, b3Color_green);
		}
	}

	for (b3TriangleFixture* t = m_triangleList.m_head; t; t = t->m_next)
	{
		b3Particle* p1 = t->m_p1;
		b3Particle* p2 = t->m_p2;
		b3Particle* p3 = t->m_p3;

		b3Vec3 v1 = p1->m_position;
		b3Vec3 v2 = p2->m_position;
		b3Vec3 v3 = p3->m_position;

		b3Vec3 c = (v1 + v2 + v3) / scalar(3);

		const scalar s(0.9);

		v1 = s * (v1 - c) + c;
		v2 = s * (v2 - c) + c;
		v3 = s * (v3 - c) + c;

		b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
		n.Normalize();

		// Solid radius
		const scalar rs(0.05);

		// Frame radius plus a small tolerance to prevent z-fighting
		const scalar rf = rs + scalar(0.005);

		b3Color frontSolidColor(scalar(0), scalar(0), scalar(1));
		b3Color frontFrameColor(scalar(0), scalar(0), scalar(0.5));

		b3Color backSolidColor(scalar(0.5), scalar(0.5), scalar(0.5));
		b3Color backFrameColor(scalar(0.25), scalar(0.25), scalar(0.25));

		{
			b3Vec3 x1 = v1 + rf * n;
			b3Vec3 x2 = v2 + rf * n;
			b3Vec3 x3 = v3 + rf * n;

			draw->DrawTriangle(x1, x2, x3, frontFrameColor);
		}

		{
			b3Vec3 x1 = v1 - rf * n;
			b3Vec3 x2 = v2 - rf * n;
			b3Vec3 x3 = v3 - rf * n;

			draw->DrawTriangle(x1, x2, x3, backFrameColor);
		}

		{
			b3Vec3 x1 = v1 + rs * n;
			b3Vec3 x2 = v2 + rs * n;
			b3Vec3 x3 = v3 + rs * n;

			draw->DrawSolidTriangle(n, x1, x2, x3, frontSolidColor);
		}

		{
			b3Vec3 x1 = v1 - rs * n;
			b3Vec3 x2 = v2 - rs * n;
			b3Vec3 x3 = v3 - rs * n;

			draw->DrawSolidTriangle(-n, x3, x2, x1, backSolidColor);
		}
	}

	for (b3TetrahedronFixture* t = m_tetrahedronList.m_head; t; t = t->m_next)
	{
		b3Particle* p1 = t->m_p1;
		b3Particle* p2 = t->m_p2;
		b3Particle* p3 = t->m_p3;
		b3Particle* p4 = t->m_p4;

		b3Vec3 v1 = p1->m_position;
		b3Vec3 v2 = p2->m_position;
		b3Vec3 v3 = p3->m_position;
		b3Vec3 v4 = p4->m_position;

		b3Vec3 c = (v1 + v2 + v3 + v4) / scalar(4);

		const scalar s(0.9);

		v1 = s * (v1 - c) + c;
		v2 = s * (v2 - c) + c;
		v3 = s * (v3 - c) + c;
		v4 = s * (v4 - c) + c;

		// v1, v2, v3
		draw->DrawTriangle(v1, v2, v3, b3Color_black);

		b3Vec3 n1 = b3Cross(v2 - v1, v3 - v1);
		n1.Normalize();
		draw->DrawSolidTriangle(n1, v1, v2, v3, b3Color_blue);

		// v1, v3, v4
		draw->DrawTriangle(v1, v3, v4, b3Color_black);

		b3Vec3 n2 = b3Cross(v3 - v1, v4 - v1);
		n2.Normalize();
		draw->DrawSolidTriangle(n2, v1, v3, v4, b3Color_blue);

		// v1, v4, v2
		draw->DrawTriangle(v1, v4, v2, b3Color_black);

		b3Vec3 n3 = b3Cross(v4 - v1, v2 - v1);
		n3.Normalize();
		draw->DrawSolidTriangle(n3, v1, v4, v2, b3Color_blue);

		// v2, v4, v3
		draw->DrawTriangle(v2, v4, v3, b3Color_black);

		b3Vec3 n4 = b3Cross(v4 - v2, v3 - v2);
		n4.Normalize();
		draw->DrawSolidTriangle(n4, v2, v4, v3, b3Color_blue);
	}

	for (b3WorldFixture* s = m_fixtureList.m_head; s; s = s->GetNext())
	{
		s->Draw(draw);
	}
}
