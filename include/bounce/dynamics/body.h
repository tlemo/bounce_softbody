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

#ifndef B3_BODY_H
#define B3_BODY_H

#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/memory/block_allocator.h>
#include <bounce/common/template/list.h>
#include <bounce/collision/trees/dynamic_tree.h>
#include <bounce/dynamics/contact_manager.h>

class b3Draw;

struct b3ParticleDef;
class b3Particle;

struct b3ForceDef;
class b3Force;

struct b3BodySphereShapeDef;
class b3BodySphereShape;

struct b3BodyTriangleShapeDef;
class b3BodyTriangleShape;

struct b3BodyTetrahedronShapeDef;
class b3BodyTetrahedronShape;

struct b3BodyWorldShapeDef;
class b3BodyWorldShape;

struct b3RayCastInput;
struct b3RayCastOutput;

struct b3TimeStep;

struct b3BodyRayCastSingleOutput
{
	b3BodyTriangleShape* triangle;
	scalar fraction;
	b3Vec3 normal;
};

// A body represents a deformable body as a collection of particles.
// Particles may be connected with each other by forces.
class b3Body
{
public:
	b3Body();
	virtual ~b3Body();

	// Create a particle.
	b3Particle* CreateParticle(const b3ParticleDef& def);

	// Destroy a given particle.
	void DestroyParticle(b3Particle* particle);

	// Return the list of particles in this body.
	const b3List<b3Particle>& GetParticleList() const;

	// Create a force.
	b3Force* CreateForce(const b3ForceDef& def);

	// Destroy a given force.
	void DestroyForce(b3Force* force);

	// Return the list of forces in this body.
	const b3List<b3Force>& GetForceList() const;

	// Create a sphere shape.
	b3BodySphereShape* CreateSphereShape(const b3BodySphereShapeDef& def);

	// Destroy a given sphere shape.
	void DestroySphereShape(b3BodySphereShape* shape);
	
	// Return the list of sphere shapes in this body.
	const b3List<b3BodySphereShape>& GetSphereShapeList() const;
	
	// Create a triangle shape.
	b3BodyTriangleShape* CreateTriangleShape(const b3BodyTriangleShapeDef& def);

	// Destroy a given triangle shape.
	void DestroyTriangleShape(b3BodyTriangleShape* shape);

	// Return the list of triangle shapes in this body.
	const b3List<b3BodyTriangleShape>& GetTriangleShapeList() const;

	// Create a tetrahedron shape.
	b3BodyTetrahedronShape* CreateTetrahedronShape(const b3BodyTetrahedronShapeDef& def);

	// Destroy a given tetrahedron shape.
	void DestroyTetrahedronShape(b3BodyTetrahedronShape* shape);

	// Return the list of tetrahedron shapes in this body.
	const b3List<b3BodyTetrahedronShape>& GetTetrahedronShapeList() const;

	// Create a new world shape.
	b3BodyWorldShape* CreateWorldShape(const b3BodyWorldShapeDef& def);

	// Destroy a given world shape.
	void DestroyWorldShape(b3BodyWorldShape* shape);

	// Return the list of world shapes in this body.
	const b3List<b3BodyWorldShape>& GetWorldShapeList() const;

	// Set the acceleration of gravity.
	void SetGravity(const b3Vec3& gravity);

	// Get the acceleration of gravity.
	b3Vec3 GetGravity() const;

	// Perform a time step. 
	void Step(scalar dt, u32 forceIterations, u32 forceSubIterations);

	// Perform a ray cast with the body.
	bool RayCastSingle(b3BodyRayCastSingleOutput* output, const b3Vec3& p1, const b3Vec3& p2) const;

	// Return the kinetic energy in this system.
	scalar GetEnergy() const;

	// Debug draw the body entities.
	void Draw(b3Draw* draw) const;
protected:
	friend class b3Particle;
	friend class b3BodySphereShape;
	friend class b3BodyTriangleShape;
	friend class b3BodyTetrahedronShape;
	friend class b3BodyWorldShape;
	friend class b3SpringForce;
	friend class b3StretchForce;
	friend class b3ShearForce;
	friend class b3MouseForce;
	friend class b3ContactManager;
	
	// Rest the mass data of the body.
	void ResetMass();

	// Solve
	void Solve(const b3TimeStep& step);

	// Stack allocator
	b3StackAllocator m_stackAllocator;

	// Block allocator
	b3BlockAllocator m_blockAllocator;

	// Gravity acceleration
	b3Vec3 m_gravity;

	// List of particles
	b3List<b3Particle> m_particleList;

	// List of forces
	b3List<b3Force> m_forceList;

	// List of sphere shapes
	b3List<b3BodySphereShape> m_sphereShapeList;
	
	// List of triangle shapes
	b3List<b3BodyTriangleShape> m_triangleShapeList;
	
	// List of tetrahedron shapes
	b3List<b3BodyTetrahedronShape> m_tetrahedronShapeList;
	
	// List of world shapes
	b3List<b3BodyWorldShape> m_worldShapeList;

	// Contact manager
	b3ContactManager m_contactManager;

	// Dynamic tree.
	b3DynamicTree m_tree;
};

inline void b3Body::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline b3Vec3 b3Body::GetGravity() const
{
	return m_gravity;
}

inline const b3List<b3Force>& b3Body::GetForceList() const
{
	return m_forceList;
}

inline const b3List<b3Particle>& b3Body::GetParticleList() const
{
	return m_particleList;
}

inline const b3List<b3BodySphereShape>& b3Body::GetSphereShapeList() const
{
	return m_sphereShapeList;
}

inline const b3List<b3BodyTriangleShape>& b3Body::GetTriangleShapeList() const
{
	return m_triangleShapeList;
}

inline const b3List<b3BodyTetrahedronShape>& b3Body::GetTetrahedronShapeList() const
{
	return m_tetrahedronShapeList;
}

inline const b3List<b3BodyWorldShape>& b3Body::GetWorldShapeList() const
{
	return m_worldShapeList;
}

#endif