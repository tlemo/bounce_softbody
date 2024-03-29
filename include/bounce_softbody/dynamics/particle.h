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

#ifndef B3_PARTICLE_H
#define B3_PARTICLE_H

#include <bounce_softbody/common/template/list.h>
#include <bounce_softbody/common/math/vec3.h>

class b3Body;

struct b3SparseForceSolverData;

// Static particle: Zero mass. Can be moved manually.
// Kinematic particle: Zero mass. Non-zero velocity, can be moved by the solver.
// Dynamic particle: Non-zero mass. Non-zero velocity determined by force, can be moved by the solver.
enum b3ParticleType
{
	e_staticParticle,
	e_kinematicParticle,
	e_dynamicParticle
};

// Particle definition.
struct b3ParticleDef
{
	b3ParticleDef()
	{
		type = e_staticParticle;
		position.SetZero();
		velocity.SetZero();
		massDamping = scalar(0);
		meshIndex = B3_MAX_U32;
		userData = nullptr;
	}

	// The particle type.
	b3ParticleType type;

	// Initial position of the particle.
	b3Vec3 position;

	// Initial velocity of the particle.
	b3Vec3 velocity;

	// Coefficient of mass damping of the particle.
	scalar massDamping;

	// Optional vertex index in the mesh.
	u32 meshIndex;

	// User data pointer to anything.
	void* userData;
};

// A particle.
class b3Particle
{
public:
	// Set the particle type.
	void SetType(b3ParticleType type);

	// Get the particle type.
	b3ParticleType GetType() const;

	// Set the particle position. 
	// If the particle is dynamic changing the position directly might lead 
	// to physically incorrect simulation behaviour.
	void SetPosition(const b3Vec3& position);

	// Get the particle position.
	const b3Vec3& GetPosition() const;

	// Set the particle velocity.
	void SetVelocity(const b3Vec3& velocity);

	// Get the particle velocity.
	const b3Vec3& GetVelocity() const;

	// Get the particle mass.
	scalar GetMass() const;

	// Get the applied force.
	const b3Vec3& GetForce() const;

	// Apply a force.
	void ApplyForce(const b3Vec3& force);

	// Get the applied translation.
	const b3Vec3& GetTranslation() const;

	// Apply a translation.
	void ApplyTranslation(const b3Vec3& translation);

	// Set the coefficient of mass damping.
	void SetMassDamping(scalar massDamping);

	// Get the coefficient of mass damping.
	scalar GetMassDamping() const;

	// Get the mesh index.
	u32 GetMeshIndex() const;
	
	// Set the user data.
	void SetUserData(void* userData);

	// Get the user data.
	const void* GetUserData() const;
	void* GetUserData();

	// Get the next particle in the body list of particles.
	b3Particle* GetNext();
	const b3Particle* GetNext() const;
private:
	friend class b3List<b3Particle>;
	friend class b3Body;
	friend class b3ContactManager;
	friend class b3BodySolver;
	friend class b3ForceSolver;
	friend class b3FrictionSolver;
	friend class b3ForceModel;
	friend class b3SphereFixture;
	friend class b3TriangleFixture;
	friend class b3TetrahedronFixture;
	friend class b3SphereAndShapeContact;
	friend class b3Force;
	friend class b3StretchForce;
	friend class b3ShearForce;
	friend class b3SpringForce;
	friend class b3MouseForce;
	friend class b3TriangleElementForce;
	friend class b3TetrahedronElementForce;

	b3Particle(const b3ParticleDef& def, b3Body* body);
	
	// Synchronize fixtures
	void SynchronizeFixtures();

	// Destroy fixtures.
	void DestroyFixtures();

	// Destroy forces.
	void DestroyForces();

	// Destroy contacts.
	void DestroyContacts();
	
	// Compute forces due to particle.
	void ComputeForces(const b3SparseForceSolverData* data);

	// Type
	b3ParticleType m_type;

	// Position
	b3Vec3 m_position;

	// Velocity
	b3Vec3 m_velocity;

	// Applied external force
	b3Vec3 m_force;

	// Applied translation
	b3Vec3 m_translation;

	// Mass
	scalar m_mass;

	// Inverse mass
	scalar m_invMass;

	// Coefficient of mass damping.
	scalar m_massDamping;

	// Mesh index. 
	u32 m_meshIndex;

	// Solver temp identifier
	u32 m_solverId;

	// User data
	void* m_userData;

	// Body
	b3Body* m_body;

	// Links to the body particle list.
	b3Particle* m_prev;
	b3Particle* m_next;
};

inline b3ParticleType b3Particle::GetType() const
{
	return m_type;
}

inline void b3Particle::SetPosition(const b3Vec3& position)
{
	m_position = position;
	m_translation.SetZero();
	SynchronizeFixtures();
}

inline const b3Vec3& b3Particle::GetPosition() const
{
	return m_position;
}

inline void b3Particle::SetVelocity(const b3Vec3& velocity)
{
	if (m_type == e_staticParticle)
	{
		return;
	}
	m_velocity = velocity;
}

inline const b3Vec3& b3Particle::GetVelocity() const
{
	return m_velocity;
}

inline scalar b3Particle::GetMass() const
{
	return m_mass;
}

inline const b3Vec3& b3Particle::GetForce() const
{
	return m_force;
}

inline void b3Particle::ApplyForce(const b3Vec3& force)
{
	if (m_type != e_dynamicParticle)
	{
		return;
	}
	m_force += force;
}

inline const b3Vec3& b3Particle::GetTranslation() const
{
	return m_translation;
}

inline void b3Particle::ApplyTranslation(const b3Vec3& translation)
{
	m_translation += translation;
}

inline void b3Particle::SetMassDamping(scalar damping)
{
	B3_ASSERT(damping >= scalar(0));
	m_massDamping = damping;
}

inline scalar b3Particle::GetMassDamping() const
{
	return m_massDamping;
}

inline u32 b3Particle::GetMeshIndex() const
{
	return m_meshIndex;
}

inline void b3Particle::SetUserData(void* userData)
{
	m_userData = userData;
}

inline const void* b3Particle::GetUserData() const
{
	return m_userData;
}

inline void* b3Particle::GetUserData()
{
	return m_userData;
}

inline b3Particle* b3Particle::GetNext()
{
	return m_next;
}

inline const b3Particle* b3Particle::GetNext() const
{
	return m_next;
}

#endif