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

#include <bounce/dynamics/softbody.h>
#include <bounce/dynamics/softbody_particle.h>
#include <bounce/dynamics/softbody_time_step.h>
#include <bounce/dynamics/softbody_solver.h>
#include <bounce/dynamics/forces/softbody_force.h>
#include <bounce/dynamics/shapes/softbody_sphere_shape.h>
#include <bounce/dynamics/shapes/softbody_triangle_shape.h>
#include <bounce/dynamics/shapes/softbody_tetrahedron_shape.h>
#include <bounce/dynamics/shapes/softbody_world_shape.h>
#include <bounce/common/draw.h>

b3SoftBody::b3SoftBody()
{
	m_contactManager.m_body = this;
	m_contactManager.m_allocator = &m_blockAllocator;

	m_gravity.SetZero();
	
	m_inv_dt0 = scalar(0);
}

b3SoftBody::~b3SoftBody()
{
	// None of the objects use b3Alloc.
}

b3SoftBodyParticle* b3SoftBody::CreateParticle(const b3SoftBodyParticleDef& def)
{
	void* mem = m_blockAllocator.Allocate(sizeof(b3SoftBodyParticle));
	b3SoftBodyParticle* p = new(mem) b3SoftBodyParticle(def, this);

	// Add to body list.
	m_particleList.PushFront(p);

	return p;
}

void b3SoftBody::DestroyParticle(b3SoftBodyParticle* particle)
{
	// Delete the attached objects.
	particle->DestroyShapes();
	particle->DestroyForces();
	particle->DestroyContacts();

	// Remove from body list.
	m_particleList.Remove(particle);
	
	particle->~b3SoftBodyParticle();
	m_blockAllocator.Free(particle, sizeof(b3SoftBodyParticle));
}

b3SoftBodySphereShape* b3SoftBody::CreateSphereShape(const b3SoftBodySphereShapeDef& def)
{
	// Check if the shape exists.
	for (b3SoftBodySphereShape* s = m_sphereShapeList.m_head; s; s = s->m_next)
	{
		if (s->m_p == def.p)
		{
			return s;
		}
	}
	
	void* mem = m_blockAllocator.Allocate(sizeof(b3SoftBodySphereShape));
	b3SoftBodySphereShape* s = new (mem)b3SoftBodySphereShape(def, this);
	
	// Add to body list.
	m_sphereShapeList.PushFront(s);

	return s;
}

void b3SoftBody::DestroySphereShape(b3SoftBodySphereShape* shape)
{
	// Destroy attached objects.
	shape->DestroyContacts();

	// Remove from body list.
	m_sphereShapeList.Remove(shape);
	
	shape->~b3SoftBodySphereShape();
	m_blockAllocator.Free(shape, sizeof(b3SoftBodySphereShape));
}

b3SoftBodyTriangleShape* b3SoftBody::CreateTriangleShape(const b3SoftBodyTriangleShapeDef& def)
{
	// Check if the shape exists.
	b3SoftBodyParticle * p1 = def.p1, * p2 = def.p2, * p3 = def.p3;
	for (b3SoftBodyTriangleShape* t = m_triangleShapeList.m_head; t; t = t->m_next)
	{
		bool hasP1 = t->m_p1 == p1 || t->m_p2 == p1 || t->m_p3 == p1;
		bool hasP2 = t->m_p1 == p2 || t->m_p2 == p2 || t->m_p3 == p2;
		bool hasP3 = t->m_p1 == p3 || t->m_p2 == p3 || t->m_p3 == p3;

		if (hasP1 && hasP2 && hasP3)
		{
			return t;
		}
	}
	
	void* mem = m_blockAllocator.Allocate(sizeof(b3SoftBodyTriangleShape));
	b3SoftBodyTriangleShape* t = new (mem)b3SoftBodyTriangleShape(def, this);

	// Create tree proxy.
	b3AABB aabb = t->ComputeAABB();
	t->m_proxyId = m_tree.CreateProxy(aabb, t);

	// Add to body list.
	m_triangleShapeList.PushFront(t);

	// Reset the body mass
	ResetMass();

	return t;
}

void b3SoftBody::DestroyTriangleShape(b3SoftBodyTriangleShape* shape)
{
	// Destroy tree proxy.
	m_tree.DestroyProxy(shape->m_proxyId);

	// Remove from body list.
	m_triangleShapeList.Remove(shape);
	
	shape->~b3SoftBodyTriangleShape();
	m_blockAllocator.Free(shape, sizeof(b3SoftBodyTriangleShape));

	// Reset the body mass
	ResetMass();
}

b3SoftBodyTetrahedronShape* b3SoftBody::CreateTetrahedronShape(const b3SoftBodyTetrahedronShapeDef& def)
{
	// Check if the shape exists.
	b3SoftBodyParticle * p1 = def.p1, * p2 = def.p2, * p3 = def.p3, * p4 = def.p4;
	for (b3SoftBodyTetrahedronShape* t = m_tetrahedronShapeList.m_head; t; t = t->m_next)
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

	void* mem = m_blockAllocator.Allocate(sizeof(b3SoftBodyTetrahedronShape));
	b3SoftBodyTetrahedronShape* t = new (mem)b3SoftBodyTetrahedronShape(def, this);

	// Add to body list.
	m_tetrahedronShapeList.PushFront(t);

	// Reset the body mass.
	ResetMass();

	return t;
}

void b3SoftBody::DestroyTetrahedronShape(b3SoftBodyTetrahedronShape* shape)
{
	// Remove from body list.
	m_tetrahedronShapeList.Remove(shape);
	
	shape->~b3SoftBodyTetrahedronShape();
	m_blockAllocator.Free(shape, sizeof(b3SoftBodyTetrahedronShape));

	// Reset the body mass
	ResetMass();
}

b3SoftBodyForce* b3SoftBody::CreateForce(const b3SoftBodyForceDef& def)
{
	b3SoftBodyForce* f = b3SoftBodyForce::Create(&def, &m_blockAllocator);
	m_forceList.PushFront(f);
	return f;
}

void b3SoftBody::DestroyForce(b3SoftBodyForce* force)
{
	m_forceList.Remove(force);
	b3SoftBodyForce::Destroy(force, &m_blockAllocator);
}

b3SoftBodyWorldShape* b3SoftBody::CreateWorldShape(const b3SoftBodyWorldShapeDef& def)
{
	void* mem = m_blockAllocator.Allocate(sizeof(b3SoftBodyWorldShape));
	b3SoftBodyWorldShape* shape = new (mem) b3SoftBodyWorldShape;
	shape->Create(&m_blockAllocator, this, def);

	// Add to the body list
	m_worldShapeList.PushFront(shape);

	return shape;
}

void b3SoftBody::DestroyWorldShape(b3SoftBodyWorldShape* shape)
{
	// Destroy attached contacts.
	shape->DestroyContacts();

	// Remove from the body list.
	m_worldShapeList.Remove(shape);

	shape->Destroy(&m_blockAllocator);
	shape->~b3SoftBodyWorldShape();
	m_blockAllocator.Free(shape, sizeof(b3SoftBodyWorldShape));
}

scalar b3SoftBody::GetEnergy() const
{
	scalar E = scalar(0);
	for (b3SoftBodyParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		E += p->m_mass * b3Dot(p->m_velocity, p->m_velocity);
	}
	return scalar(0.5) * E;
}

void b3SoftBody::ResetMass()
{
	for (b3SoftBodyTriangleShape* t = m_triangleShapeList.m_head; t; t = t->m_next)
	{
		t->m_p1->m_mass = scalar(0);
		t->m_p2->m_mass = scalar(0);
		t->m_p3->m_mass = scalar(0);
	}

	for (b3SoftBodyTetrahedronShape* t = m_tetrahedronShapeList.m_head; t; t = t->m_next)
	{
		t->m_p1->m_mass = scalar(0);
		t->m_p2->m_mass = scalar(0);
		t->m_p3->m_mass = scalar(0);
		t->m_p4->m_mass = scalar(0);
	}

	const scalar inv3 = scalar(1) / scalar(3);
	for (b3SoftBodyTriangleShape* t = m_triangleShapeList.m_head; t; t = t->m_next)
	{
		b3SoftBodyParticle* p1 = t->m_p1;
		b3SoftBodyParticle* p2 = t->m_p2;
		b3SoftBodyParticle* p3 = t->m_p3;

		scalar mass = t->m_density * t->m_area;

		p1->m_mass += inv3 * mass;
		p2->m_mass += inv3 * mass;
		p3->m_mass += inv3 * mass;
	}

	const scalar inv4 = scalar(1) / scalar(4);
	for (b3SoftBodyTetrahedronShape* t = m_tetrahedronShapeList.m_head; t; t = t->m_next)
	{
		b3SoftBodyParticle* p1 = t->m_p1;
		b3SoftBodyParticle* p2 = t->m_p2;
		b3SoftBodyParticle* p3 = t->m_p3;
		b3SoftBodyParticle* p4 = t->m_p4;

		scalar mass = t->m_density * t->m_volume;

		p1->m_mass += inv4 * mass;
		p2->m_mass += inv4 * mass;
		p3->m_mass += inv4 * mass;
		p4->m_mass += inv4 * mass;
	}

	// Invert
	for (b3SoftBodyParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		// Static and kinematic particles have zero mass.
		if (p->m_type == e_staticSoftBodyParticle || p->m_type == e_kinematicSoftBodyParticle)
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

struct b3SoftBodyRayCastSingleWrapper
{
	scalar Report(const b3RayCastInput& input, u32 proxyId)
	{
		// Get shape associated with the proxy.
		void* userData = tree->GetUserData(proxyId);
		b3SoftBodyTriangleShape* triangleShape = (b3SoftBodyTriangleShape*)userData;

		b3RayCastOutput subOutput;
		if (triangleShape->RayCast(&subOutput, input))
		{
			// Ray hits triangle.
			if (subOutput.fraction < output0.fraction)
			{
				triangle0 = triangleShape;
				output0.fraction = subOutput.fraction;
				output0.normal = subOutput.normal;
			}
		}

		// Continue search from where we stopped.
		return input.maxFraction;
	}

	const b3DynamicTree* tree;
	b3SoftBodyTriangleShape* triangle0;
	b3RayCastOutput output0;
};

bool b3SoftBody::RayCastSingle(b3SoftBodyRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3SoftBodyRayCastSingleWrapper wrapper;
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

void b3SoftBody::Solve(const b3SoftBodyTimeStep& step)
{
	b3SoftBodySolverDef solverDef;
	solverDef.stack = &m_stackAllocator;
	solverDef.particleCapacity = m_particleList.m_count;
	solverDef.forceCapacity = m_forceList.m_count;
	solverDef.shapeContactCapacity = m_contactManager.m_shapeContactList.m_count;
	
	b3SoftBodySolver solver(solverDef);

	for (b3SoftBodyParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		solver.Add(p);
	}

	for (b3SoftBodyForce* f = m_forceList.m_head; f; f = f->m_next)
	{
		solver.Add(f);
	}

	for (b3SoftBodySphereAndShapeContact* c = m_contactManager.m_shapeContactList.m_head; c; c = c->m_next)
	{
		if (c->m_active)
		{
			solver.Add(c);
		}
	}

	// Solve
	solver.Solve(step, m_gravity);
}

void b3SoftBody::Step(scalar dt, u32 forceIterations, u32 forceSubIterations)
{
	// Time step parameters
	b3SoftBodyTimeStep step;
	step.dt = dt;
	step.forceIterations = forceIterations;
	step.forceSubIterations = forceSubIterations;
	step.inv_dt = dt > scalar(0) ? scalar(1) / dt : scalar(0);
	step.dt_ratio = m_inv_dt0 * dt;

	// Update contacts. This is where some contacts are ceased.
	m_contactManager.UpdateContacts();

	// Integrate state, solve constraints. 
	if (step.dt > scalar(0))
	{
		Solve(step);
	}

	if (step.dt > scalar(0))
	{
		m_inv_dt0 = step.inv_dt;
	}

	// Clear forces and translations
	for (b3SoftBodyParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		p->m_force.SetZero();
		p->m_translation.SetZero();
	}

	// Synchronize triangle shapes
	for (b3SoftBodyTriangleShape* t = m_triangleShapeList.m_head; t; t = t->m_next)
	{
		b3SoftBodyParticle* p1 = t->m_p1;
		b3SoftBodyParticle* p2 = t->m_p2;
		b3SoftBodyParticle* p3 = t->m_p3;

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

void b3SoftBody::Draw(b3Draw* draw) const
{
	for (b3SoftBodyParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		if (p->m_type == e_staticSoftBodyParticle)
		{
			draw->DrawPoint(p->m_position, 4.0, b3Color_white);	
		}

		if (p->m_type == e_kinematicSoftBodyParticle)
		{
			draw->DrawPoint(p->m_position, 4.0, b3Color_blue);
		}

		if (p->m_type == e_dynamicSoftBodyParticle)
		{
			draw->DrawPoint(p->m_position, 4.0, b3Color_green);
		}
	}

	for (b3SoftBodyTriangleShape* t = m_triangleShapeList.m_head; t; t = t->m_next)
	{
		b3SoftBodyParticle* p1 = t->m_p1;
		b3SoftBodyParticle* p2 = t->m_p2;
		b3SoftBodyParticle* p3 = t->m_p3;

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

	for (b3SoftBodyTetrahedronShape* t = m_tetrahedronShapeList.m_head; t; t = t->m_next)
	{
		b3SoftBodyParticle* p1 = t->m_p1;
		b3SoftBodyParticle* p2 = t->m_p2;
		b3SoftBodyParticle* p3 = t->m_p3;
		b3SoftBodyParticle* p4 = t->m_p4;

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

	for (b3SoftBodyWorldShape* s = m_worldShapeList.m_head; s; s = s->GetNext())
	{
		s->Draw(draw);
	}
}
