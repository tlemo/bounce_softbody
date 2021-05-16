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

#include "softbody.h"

UniformSoftBody::UniformSoftBody()
{
	m_particles = nullptr;
}

UniformSoftBody::UniformSoftBody(const ClothDef& def)
{
	const SoftBodyMesh* mesh = def.mesh;
	m_mesh = mesh;

	// Create particles
	m_particles = (b3SoftBodyParticle**)malloc(mesh->vertexCount * sizeof(b3SoftBodyParticle*));
	for (int i = 0; i < mesh->vertexCount; ++i)
	{
		b3Vec3 v = mesh->GetVertexPosition(i);

		b3SoftBodyParticleDef pd;
		pd.type = e_dynamicSoftBodyParticle;
		pd.position = v;
		pd.meshIndex = i;

		b3SoftBodyParticle* p = CreateParticle(pd);
		m_particles[i] = p;

		b3SoftBodySphereShapeDef sd;
		sd.p = p;
		sd.radius = def.thickness;
		sd.friction = def.friction;
		sd.meshIndex = i;
		CreateSphereShape(sd);
	}

	// Create triangles
	for (int i = 0; i < mesh->triangleCount; ++i)
	{
		SoftBodyMeshTriangle triangle = mesh->GetTriangle(i);

		int vi1 = triangle.v1;
		int vi2 = triangle.v2;
		int vi3 = triangle.v3;

		b3SoftBodyParticle* p1 = m_particles[vi1];
		b3SoftBodyParticle* p2 = m_particles[vi2];
		b3SoftBodyParticle* p3 = m_particles[vi3];

		b3Vec3 v1 = p1->GetPosition();
		b3Vec3 v2 = p2->GetPosition();
		b3Vec3 v3 = p3->GetPosition();

		b3SoftBodyTriangleShapeDef td;
		td.p1 = p1;
		td.p2 = p2;
		td.p3 = p3;
		td.v1 = v1;
		td.v2 = v2;
		td.v3 = v3;
		td.density = def.density;
		td.radius = def.thickness;
		td.friction = def.friction;
		td.meshIndex = i;

		CreateTriangleShape(td);

		if (def.createElements)
		{
			b3SoftBodyTriangleElementForceDef fd;
			fd.p1 = p1;
			fd.p2 = p2;
			fd.p3 = p3;
			fd.v1 = v1;
			fd.v2 = v2;
			fd.v3 = v3;
			fd.youngModulusX = def.elementYoungModulus;
			fd.youngModulusY = def.elementYoungModulus;
			fd.shearModulus = def.elementShearModulus;
			fd.poissonRationXY = def.elementPoissonRatio;
			fd.poissonRationYX = def.elementPoissonRatio;
			fd.stiffnessDamping = def.elementStiffnessDamping;
			fd.meshIndex = i;

			CreateForce(fd);
		}
		else
		{
			if (def.stretchingStiffness > scalar(0))
			{
				b3SoftBodyStretchForceDef fd;
				fd.Initialize(v1, v2, v3);

				fd.p1 = p1;
				fd.p2 = p2;
				fd.p3 = p3;
				fd.stretching_stiffness_u = def.stretchingStiffness;
				fd.damping_stiffness_u = def.stretchStiffnessDamping;
				fd.b_u = scalar(1);
				fd.stretching_stiffness_v = def.stretchingStiffness;
				fd.damping_stiffness_v = def.stretchStiffnessDamping;
				fd.b_v = scalar(1);
				fd.meshIndex = i;

				CreateForce(fd);
			}
		}
	}
}

UniformSoftBody::UniformSoftBody(const TetDef& def)
{
	const SoftBodyMesh* mesh = def.mesh;
	m_mesh = mesh;

	// Create particles
	m_particles = (b3SoftBodyParticle**)malloc(mesh->vertexCount * sizeof(b3SoftBodyParticle*));
	for (int i = 0; i < mesh->vertexCount; ++i)
	{
		b3Vec3 v = mesh->GetVertexPosition(i);

		b3SoftBodyParticleDef pd;
		pd.type = e_dynamicSoftBodyParticle;
		pd.position = v;
		pd.meshIndex = i;

		b3SoftBodyParticle* p = CreateParticle(pd);

		m_particles[i] = p;

		b3SoftBodySphereShapeDef sd;
		sd.p = p;
		sd.radius = def.thickness;
		sd.friction = def.friction;
		sd.meshIndex = i;

		CreateSphereShape(sd);
	}

	// Create triangles
	for (int i = 0; i < mesh->triangleCount; ++i)
	{
		SoftBodyMeshTriangle triangle = mesh->GetTriangle(i);

		int v1 = triangle.v1;
		int v2 = triangle.v2;
		int v3 = triangle.v3;

		b3SoftBodyParticle* p1 = m_particles[v1];
		b3SoftBodyParticle* p2 = m_particles[v2];
		b3SoftBodyParticle* p3 = m_particles[v3];

		b3SoftBodyTriangleShapeDef td;
		td.p1 = p1;
		td.p2 = p2;
		td.p3 = p3;
		td.v1 = p1->GetPosition();
		td.v2 = p2->GetPosition();
		td.v3 = p3->GetPosition();
		td.radius = def.thickness;
		td.friction = def.friction;
		td.meshIndex = i;

		// Zero mass contribution
		td.density = scalar(0);

		CreateTriangleShape(td);
	}

	// Create tetrahedrons
	for (int i = 0; i < mesh->tetrahedronCount; ++i)
	{
		SoftBodyMeshTetrahedron tet = mesh->GetTetrahedron(i);

		int vi1 = tet.v1;
		int vi2 = tet.v2;
		int vi3 = tet.v3;
		int vi4 = tet.v4;

		b3SoftBodyParticle* p1 = m_particles[vi1];
		b3SoftBodyParticle* p2 = m_particles[vi2];
		b3SoftBodyParticle* p3 = m_particles[vi3];
		b3SoftBodyParticle* p4 = m_particles[vi4];

		b3Vec3 v1 = p1->GetPosition();
		b3Vec3 v2 = p2->GetPosition();
		b3Vec3 v3 = p3->GetPosition();
		b3Vec3 v4 = p4->GetPosition();

		b3SoftBodyTetrahedronShapeDef td;
		td.p1 = p1;
		td.p2 = p2;
		td.p3 = p3;
		td.p4 = p4;
		td.v1 = v1;
		td.v2 = v2;
		td.v3 = v3;
		td.v4 = v4;
		td.density = def.density;
		td.radius = def.thickness;
		td.friction = def.friction;
		td.meshIndex = i;

		CreateTetrahedronShape(td);

		// Create element
		b3SoftBodyTetrahedronElementForceDef fd;
		fd.p1 = p1;
		fd.p2 = p2;
		fd.p3 = p3;
		fd.p4 = p4;
		fd.v1 = v1;
		fd.v2 = v2;
		fd.v3 = v3;
		fd.v4 = v4;
		fd.youngModulus = def.elementYoungModulus;
		fd.poissonRatio = def.elementPoissonRatio;
		fd.elasticStrainYield = def.elementElasticStrainYield;
		fd.creepRate = def.elementCreepRate;
		fd.maxPlasticStrain = def.elementMaxPlasticStrain;
		fd.stiffnessDamping = def.elementStiffnessDamping;
		fd.meshIndex = i;

		CreateForce(fd);
	}
}

UniformSoftBody::~UniformSoftBody()
{
	free(m_particles);
}
