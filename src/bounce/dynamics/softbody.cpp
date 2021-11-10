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
#include <bounce/dynamics/forces/softbody_force.h>
#include <bounce/dynamics/softbody_time_step.h>
#include <bounce/dynamics/softbody_solver.h>
#include <bounce/dynamics/shapes/softbody_sphere_shape.h>
#include <bounce/dynamics/shapes/softbody_capsule_shape.h>
#include <bounce/dynamics/shapes/softbody_triangle_shape.h>
#include <bounce/dynamics/shapes/softbody_tetrahedron_shape.h>
#include <bounce/dynamics/shapes/softbody_world_shape.h>
#include <bounce/common/draw.h>

b3SoftBody::b3SoftBody() : m_particleBlocks(sizeof(b3SoftBodyParticle))
{
	m_contactManager.m_body = this;
	m_gravity.SetZero();
	m_inv_dt0 = scalar(0);
	m_enableSelfCollision = false;
}

b3SoftBody::~b3SoftBody()
{
	b3SoftBodyForce* f = m_forceList.m_head;
	while (f)
	{
		b3SoftBodyForce* boom = f;
		f = f->m_next;
		b3SoftBodyForce::Destroy(boom);
	}

	b3SoftBodySphereShape* s = m_sphereShapeList.m_head;
	while (s)
	{
		b3SoftBodySphereShape* boom = s;
		s = s->m_next;
		boom->~b3SoftBodySphereShape();
		b3Free(boom);
	}

	b3SoftBodyCapsuleShape* c = m_capsuleShapeList.m_head;
	while (c)
	{
		b3SoftBodyCapsuleShape* boom = c;
		c = c->m_next;
		boom->~b3SoftBodyCapsuleShape();
		b3Free(boom);
	}

	b3SoftBodyTriangleShape* t = m_triangleShapeList.m_head;
	while (t)
	{
		b3SoftBodyTriangleShape* boom = t;
		t = t->m_next;
		boom->~b3SoftBodyTriangleShape();
		b3Free(boom);
	}

	b3SoftBodyTetrahedronShape* h = m_tetrahedronShapeList.m_head;
	while (h)
	{
		b3SoftBodyTetrahedronShape* boom = h;
		h = h->m_next;
		boom->~b3SoftBodyTetrahedronShape();
		b3Free(boom);
	}

	b3SoftBodyWorldShape* w = m_worldShapeList.m_head;
	while (w)
	{
		b3SoftBodyWorldShape* boom = w;
		w = w->m_next;
		b3SoftBodyWorldShape::Destroy(boom);
	}
}

b3SoftBodyParticle* b3SoftBody::CreateParticle(const b3SoftBodyParticleDef& def)
{
	void* mem = m_particleBlocks.Allocate();
	b3SoftBodyParticle* p = new(mem) b3SoftBodyParticle(def, this);

	m_particleList.PushFront(p);

	return p;
}

void b3SoftBody::DestroyParticle(b3SoftBodyParticle* particle)
{
	// Destroy shapes
	particle->DestroyShapes();

	// Destroy forces
	particle->DestroyForces();

	// Destroy contacts
	particle->DestroyContacts();

	m_particleList.Remove(particle);
	particle->~b3SoftBodyParticle();
	m_particleBlocks.Free(particle);
}

b3SoftBodySphereShape* b3SoftBody::CreateSphereShape(const b3SoftBodySphereShapeDef& def)
{
#if 0
	// Check if the shape exists.
	for (b3SoftBodySphereShape* s = m_sphereShapeList.m_head; s; s = s->m_next)
	{
		if (s->m_p == def.p)
		{
			return s;
		}
	}
#endif
	void* mem = b3Alloc(sizeof(b3SoftBodySphereShape));
	b3SoftBodySphereShape* s = new (mem)b3SoftBodySphereShape(def, this);
	s->m_radius = def.radius;
	s->m_friction = def.friction;
	s->m_density = def.density;
	s->m_meshIndex = def.meshIndex;

	b3AABB aabb = s->ComputeAABB();
	s->m_proxy.shape = s;
	s->m_proxy.proxyId = m_contactManager.m_broadPhase.CreateProxy(aabb, &s->m_proxy);

	m_sphereShapeList.PushFront(s);

	return s;
}

void b3SoftBody::DestroySphereShape(b3SoftBodySphereShape* shape)
{
	// Destroy contacts
	shape->DestroyContacts();

	// Remove shape from broadphase
	m_contactManager.m_broadPhase.DestroyProxy(shape->m_proxy.proxyId);

	m_sphereShapeList.Remove(shape);
	shape->~b3SoftBodySphereShape();
	b3Free(shape);
	}

b3SoftBodyCapsuleShape* b3SoftBody::CreateCapsuleShape(const b3SoftBodyCapsuleShapeDef& def)
{
#if 0
	// Check if the shape exists.
	for (b3ClothCapsuleShape* c = m_capsuleShapeList.m_head; c; c = c->m_next)
	{
		if (c->m_p1 == def.p1 && c->m_p2 == def.p2)
		{
			return c;
		}

		if (c->m_p1 == def.p2 && c->m_p2 == def.p1)
		{
			return c;
		}
	}
#endif
	void* mem = b3Alloc(sizeof(b3SoftBodyCapsuleShape));
	b3SoftBodyCapsuleShape* c = new (mem)b3SoftBodyCapsuleShape(def, this);

	c->m_radius = def.radius;
	c->m_friction = def.friction;
	c->m_density = def.density;
	c->m_meshIndex = def.meshIndex;

	b3AABB aabb = c->ComputeAABB();
	c->m_proxy.shape = c;
	c->m_proxy.proxyId = m_contactManager.m_broadPhase.CreateProxy(aabb, &c->m_proxy);

	m_capsuleShapeList.PushFront(c);

	return c;
}

void b3SoftBody::DestroyCapsuleShape(b3SoftBodyCapsuleShape* shape)
{
	// Destroy contacts
	shape->DestroyContacts();

	// Remove shape from broadphase
	m_contactManager.m_broadPhase.DestroyProxy(shape->m_proxy.proxyId);

	m_capsuleShapeList.Remove(shape);
	shape->~b3SoftBodyCapsuleShape();
	b3Free(shape);
}

b3SoftBodyTriangleShape* b3SoftBody::CreateTriangleShape(const b3SoftBodyTriangleShapeDef& def)
{
#if 0
	b3SoftBodyParticle * p1 = def.p1;
	b3SoftBodyParticle* p2 = def.p2;
	b3SoftBodyParticle* p3 = def.p3;

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
#endif
	void* mem = b3Alloc(sizeof(b3SoftBodyTriangleShape));
	b3SoftBodyTriangleShape* t = new (mem)b3SoftBodyTriangleShape(def, this);

	t->m_radius = def.radius;
	t->m_friction = def.friction;
	t->m_density = def.density;
	t->m_meshIndex = def.meshIndex;

	b3AABB aabb = t->ComputeAABB();
	t->m_proxy.shape = t;
	t->m_proxy.proxyId = m_contactManager.m_broadPhase.CreateProxy(aabb, &t->m_proxy);

	m_triangleShapeList.PushFront(t);

	// Reset the body mass
	ResetMass();

	return t;
}

void b3SoftBody::DestroyTriangleShape(b3SoftBodyTriangleShape* shape)
{
	// Destroy contacts
	shape->DestroyContacts();

	// Destroy broadphase proxy
	m_contactManager.m_broadPhase.DestroyProxy(shape->m_proxy.proxyId);

	// Destroy memory
	m_triangleShapeList.Remove(shape);
	shape->~b3SoftBodyTriangleShape();
	b3Free(shape);

	// Reset the body mass
	ResetMass();
}

b3SoftBodyTetrahedronShape* b3SoftBody::CreateTetrahedronShape(const b3SoftBodyTetrahedronShapeDef& def)
{
#if 0
	b3SoftBodyParticle * p1 = def.p1;
	b3SoftBodyParticle* p2 = def.p2;
	b3SoftBodyParticle* p3 = def.p3;
	b3SoftBodyParticle* p4 = def.p4;

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
#endif
	void* mem = b3Alloc(sizeof(b3SoftBodyTetrahedronShape));
	b3SoftBodyTetrahedronShape* t = new (mem)b3SoftBodyTetrahedronShape(def, this);

	t->m_radius = def.radius;
	t->m_friction = def.friction;
	t->m_density = def.density;
	t->m_meshIndex = def.meshIndex;

	// For now don't push tetrahedrons to broadphase.
	// b3AABB aabb = t->ComputeAABB();
	// t->m_proxy.shape = t;
	// t->m_proxy.proxyId = m_contactManager.m_broadPhase.CreateProxy(aabb, &t->m_proxy);

	m_tetrahedronShapeList.PushFront(t);

	// Reset the body mass
	ResetMass();

	return t;
}

void b3SoftBody::DestroyTetrahedronShape(b3SoftBodyTetrahedronShape* shape)
{
	// Destroy contacts
	shape->DestroyContacts();

	// Destroy broadphase proxy
	// m_contactManager.m_broadPhase.DestroyProxy(shape->m_proxy.proxyId);

	// Destroy memory
	m_tetrahedronShapeList.Remove(shape);
	shape->~b3SoftBodyTetrahedronShape();
	b3Free(shape);

	// Reset the body mass
	ResetMass();
}

b3SoftBodyForce* b3SoftBody::CreateForce(const b3SoftBodyForceDef& def)
{
	b3SoftBodyForce* f = b3SoftBodyForce::Create(&def);
	m_forceList.PushFront(f);
	return f;
}

void b3SoftBody::DestroyForce(b3SoftBodyForce* force)
{
	m_forceList.Remove(force);
	b3SoftBodyForce::Destroy(force);
}

b3SoftBodyWorldShape* b3SoftBody::CreateWorldShape(const b3SoftBodyWorldShapeDef& def)
{
	// Create clone
	b3SoftBodyWorldShape* s = b3SoftBodyWorldShape::Create(def);
	s->m_friction = def.friction;
	s->m_body = this;

	// Create broadphase proxy.
	b3AABB aabb = s->ComputeAABB();
	s->m_proxy.shape = s;
	s->m_proxy.proxyId = m_contactManager.m_broadPhase.CreateProxy(aabb, &s->m_proxy);

	// Push to the body.
	m_worldShapeList.PushFront(s);

	return s;
}

void b3SoftBody::DestroyWorldShape(b3SoftBodyWorldShape* shape)
{
	// Destroy contacts
	shape->DestroyContacts();

	// Destroy broadphase proxy
	m_contactManager.m_broadPhase.DestroyProxy(shape->m_proxy.proxyId);

	// Remove from the body list of collision shapes.
	m_worldShapeList.Remove(shape);

	// Destroy memory.
	b3SoftBodyWorldShape::Destroy(shape);
}

void b3SoftBody::EnableSelfCollision(bool flag)
{
	if (m_enableSelfCollision == true && flag == false)
	{
		{
			// Destroy triangle contacts
			b3SoftBodySphereAndTriangleContact* c = m_contactManager.m_sphereAndTriangleContactList.m_head;
			while (c)
			{
				b3SoftBodySphereAndTriangleContact* boom = c;
				c = c->m_next;
				m_contactManager.Destroy(boom);
			}
		}

		{
			// Destroy capsule contacts
			b3SoftBodyCapsuleAndCapsuleContact* c = m_contactManager.m_capsuleAndCapsuleContactList.m_head;
			while (c)
			{
				b3SoftBodyCapsuleAndCapsuleContact* boom = c;
				c = c->m_next;
				m_contactManager.Destroy(boom);
			}
		}
	}

	m_enableSelfCollision = flag;
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

struct b3SoftBodyRayCastSingleCallback
{
	scalar Report(const b3RayCastInput& input, u32 proxyId)
	{
		// Get shape associated with the proxy.
		void* userData = broadPhase->GetUserData(proxyId);
		b3SoftBodyShapeBaseProxy* proxy = (b3SoftBodyShapeBaseProxy*)userData;
		b3SoftBodyShapeBase* proxyShape = proxy->shape;

		if (proxyShape->GetBaseType() != e_softBodyShapeBase)
		{
			// Continue search from where we stopped.
			return input.maxFraction;
		}

		b3SoftBodyShape* shape = (b3SoftBodyShape*)proxyShape;
		if (shape->GetShapeType() != e_softBodyTriangleShape)
		{
			// Continue search from where we stopped.
			return input.maxFraction;
		}

		b3SoftBodyTriangleShape* triangleShape = (b3SoftBodyTriangleShape*)shape;

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

	const b3BroadPhase* broadPhase;
	b3SoftBodyTriangleShape* triangle0;
	b3RayCastOutput output0;
};

bool b3SoftBody::RayCastSingle(b3SoftBodyRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = scalar(1);

	b3SoftBodyRayCastSingleCallback callback;
	callback.broadPhase = &m_contactManager.m_broadPhase;
	callback.triangle0 = nullptr;
	callback.output0.fraction = B3_MAX_SCALAR;

	m_contactManager.m_broadPhase.RayCast(&callback, input);

	if (callback.triangle0 != nullptr)
	{
		output->triangle = callback.triangle0;
		output->fraction = callback.output0.fraction;
		output->normal = callback.output0.normal;

		return true;
	}

	return false;
}

void b3SoftBody::Solve(const b3SoftBodyTimeStep& step)
{
	B3_PROFILE("Soft Body Solve");

	// Solve
	b3SoftBodySolverDef solverDef;
	solverDef.stack = &m_stackAllocator;
	solverDef.particleCapacity = m_particleList.m_count;
	solverDef.forceCapacity = m_forceList.m_count;
	solverDef.shapeContactCapacity = m_contactManager.m_sphereAndShapeContactList.m_count;
	solverDef.triangleContactCapacity = m_contactManager.m_sphereAndTriangleContactList.m_count;
	solverDef.capsuleContactCapacity = m_contactManager.m_capsuleAndCapsuleContactList.m_count;

	b3SoftBodySolver solver(solverDef);

	for (b3SoftBodyParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		solver.Add(p);
	}

	for (b3SoftBodyForce* f = m_forceList.m_head; f; f = f->m_next)
	{
		solver.Add(f);
	}

	for (b3SoftBodySphereAndTriangleContact* c = m_contactManager.m_sphereAndTriangleContactList.m_head; c; c = c->m_next)
	{
		if (c->m_active)
		{
			solver.Add(c);
		}
	}

	for (b3SoftBodyCapsuleAndCapsuleContact* c = m_contactManager.m_capsuleAndCapsuleContactList.m_head; c; c = c->m_next)
	{
		if (c->m_active)
		{
			solver.Add(c);
		}
	}

	for (b3SoftBodySphereAndShapeContact* c = m_contactManager.m_sphereAndShapeContactList.m_head; c; c = c->m_next)
	{
		if (c->m_active)
		{
			solver.Add(c);
		}
	}

	// Solve	
	solver.Solve(step, m_gravity);
}

void b3SoftBody::Step(scalar dt, u32 velocityIterations, u32 positionIterations, u32 forceIterations, u32 forceSubIterations)
{
	B3_PROFILE("Soft Body Step");

	// Time step parameters
	b3SoftBodyTimeStep step;
	step.dt = dt;
	step.velocityIterations = velocityIterations;
	step.positionIterations = positionIterations;
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

	// Clear external applied forces and translations
	for (b3SoftBodyParticle* p = m_particleList.m_head; p; p = p->m_next)
	{
		p->m_force.SetZero();
		p->m_translation.SetZero();
	}

	// Synchronize sphere shapes
	for (b3SoftBodySphereShape* s = m_sphereShapeList.m_head; s; s = s->m_next)
	{
		b3SoftBodyParticle* p = s->m_p;

		// Synchronize unconditionally  because all particles can be translated.
		b3Vec3 displacement = dt * p->m_velocity;

		s->Synchronize(displacement);
	}

	// Synchronize capsule shapes
	for (b3SoftBodyCapsuleShape* c = m_capsuleShapeList.m_head; c; c = c->m_next)
	{
		b3SoftBodyParticle* p1 = c->m_p1;
		b3SoftBodyParticle* p2 = c->m_p2;

		b3Vec3 v1 = p1->m_velocity;
		b3Vec3 v2 = p2->m_velocity;

		// Center velocity
		b3Vec3 velocity = scalar(0.5) * (v1 + v2);

		b3Vec3 displacement = dt * velocity;

		c->Synchronize(displacement);
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

	// For now don't push tetrahedrons to broadphase.
#if 0
	// Synchronize triangle shapes
	for (b3SoftBodyTetrahedronShape* t = m_tetrahedronShapeList.m_head; t; t = t->m_next)
	{
		b3SoftBodyParticle* p1 = t->m_p1;
		b3SoftBodyParticle* p2 = t->m_p2;
		b3SoftBodyParticle* p3 = t->m_p3;
		b3SoftBodyParticle* p4 = t->m_p4;

		b3Vec3 v1 = p1->m_velocity;
		b3Vec3 v2 = p2->m_velocity;
		b3Vec3 v3 = p3->m_velocity;
		b3Vec3 v4 = p4->m_velocity;

		// Center velocity
		b3Vec3 velocity = (v1 + v2 + v3 + v4) / scalar(4);

		b3Vec3 displacement = dt * velocity;

		t->Synchronize(displacement);
	}
#endif

	// Synchronize world shapes
	for (b3SoftBodyWorldShape* s = m_worldShapeList.m_head; s; s = s->m_next)
	{
		s->Synchronize(b3Vec3_zero);
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

	for (b3SoftBodyCapsuleShape* c = m_capsuleShapeList.m_head; c; c = c->m_next)
	{
		b3SoftBodyParticle* p1 = c->m_p1;
		b3SoftBodyParticle* p2 = c->m_p2;

		draw->DrawSegment(p1->m_position, p2->m_position, b3Color_black);
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
