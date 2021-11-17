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

#include <bounce/dynamics/force_solver.h>
#include <bounce/dynamics/particle.h>
#include <bounce/dynamics/forces/force.h>
#include <bounce/dynamics/contacts/sphere_shape_contact.h>
#include <bounce/sparse/sparse_force_solver.h>
#include <bounce/sparse/dense_vec3.h>
#include <bounce/sparse/diag_mat33.h>
#include <bounce/sparse/sparse_mat33.h>
#include <bounce/common/memory/stack_allocator.h>

// Number of non-linear iterations.
u32 b3_forceSolverIterations = 0;

// Min/max number of inner iterations.
u32 b3_forceSolverMinSubIterations = B3_MAX_U32;
u32 b3_forceSolverMaxSubIterations = 0;

b3ForceSolver::b3ForceSolver(const b3ForceSolverDef& def)
{
	m_step = def.step;
	m_stack = def.stack;

	m_particleCount = def.particleCount;
	m_particles = def.particles;

	m_forceCount = def.forceCount;
	m_forces = def.forces;

	m_shapeContactCount = def.shapeContactCount;
	m_shapeContacts = def.shapeContacts;
}

b3ForceSolver::~b3ForceSolver()
{
}

class b3ForceModel : public b3SparseForceModel
{
public:
	void ComputeForces(const b3SparseForceSolverData* data)
	{
		for (u32 i = 0; i < m_particleCount; ++i)
		{
			m_particles[i]->ComputeForces(data);
		}
		
		for (u32 i = 0; i < m_forceCount; ++i)
		{
			m_forces[i]->ComputeForces(data);
		}

		for (u32 i = 0; i < m_shapeContactCount; ++i)
		{
			m_shapeContacts[i]->ComputeForces(data);
		}
	}

	u32 m_particleCount;
	b3Particle** m_particles;

	u32 m_forceCount;
	b3Force** m_forces;

	b3SphereAndShapeContact** m_shapeContacts;
	u32 m_shapeContactCount;
};

void b3ForceSolver::Solve(const b3Vec3& gravity)
{
	b3DenseVec3 x0(m_particleCount);
	b3DenseVec3 v0(m_particleCount);
	b3DenseVec3 fe(m_particleCount);
	b3DenseVec3 y(m_particleCount);
	b3DenseVec3 x(m_particleCount);
	b3DenseVec3 v(m_particleCount);
	b3DiagMat33 M(m_particleCount);
	u32 fixedDofCount = 0;

	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];

		x0[i] = p->m_position;
		v0[i] = p->m_velocity;
		fe[i] = p->m_force;
		y[i] = p->m_translation;

		if (p->m_type == e_dynamicParticle)
		{
			B3_ASSERT(p->m_mass > scalar(0));
			M[i] = b3Mat33Diagonal(p->m_mass);

			// Apply weight
			fe[i] += p->m_mass * gravity;
		}
		else
		{
			// Ensure a non-zero mass because zero masses 
			// can make the system unsolvable.
			M[i] = b3Mat33Diagonal(scalar(1));
			++fixedDofCount;
		}
	}
	
	// Collect the constrained degrees of freedom.
	u32* fixedDofs = (u32*)m_stack->Allocate(fixedDofCount * sizeof(u32));

	fixedDofCount = 0;
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		b3Particle* p = m_particles[i];
		if (p->m_type != e_dynamicParticle)
		{
			fixedDofs[fixedDofCount] = i;
			++fixedDofCount;
		}
	}

	// Prepare the force model.
	b3ForceModel forceModel;
	forceModel.m_particleCount = m_particleCount;
	forceModel.m_particles = m_particles;
	forceModel.m_forceCount = m_forceCount;
	forceModel.m_forces = m_forces;
	forceModel.m_shapeContactCount = m_shapeContactCount;
	forceModel.m_shapeContacts = m_shapeContacts;

	// Prepare input.
	b3SolveBEInput solverInput;
	solverInput.forceModel = &forceModel;
	solverInput.h = m_step.dt;
	solverInput.inv_h = m_step.inv_dt;
	solverInput.dofCount = m_particleCount;
	solverInput.x0 = &x0; 
	solverInput.v0 = &v0;
	solverInput.fe = &fe;
	solverInput.M = &M;
	solverInput.y = &y;
	solverInput.fixedDofCount = fixedDofCount;
	solverInput.fixedDofs = fixedDofs;
	solverInput.maxIterations = m_step.forceIterations;
	solverInput.maxSubIterations = m_step.forceSubIterations;
	
	// Prepare output.
	b3SolveBEOutput solverOutput;
	solverOutput.x = &x;
	solverOutput.v = &v;
	solverOutput.minSubIterations = b3_forceSolverMinSubIterations;
	solverOutput.maxSubIterations = b3_forceSolverMaxSubIterations;

	// Integrate F = ma.
	b3SparseSolveBE(&solverOutput, &solverInput);

	// Free temporary memory.
	m_stack->Free(fixedDofs);

	// Track non-linear iterations.
	b3_forceSolverIterations = solverOutput.iterations;
	
	// Track min-max sub-iterations.
	b3_forceSolverMinSubIterations = solverOutput.minSubIterations;
	b3_forceSolverMaxSubIterations = solverOutput.maxSubIterations;

	// Copy buffers back to the particles.
	for (u32 i = 0; i < m_particleCount; ++i)
	{
		m_particles[i]->m_position = x[i];
		m_particles[i]->m_velocity = v[i];
	}
}