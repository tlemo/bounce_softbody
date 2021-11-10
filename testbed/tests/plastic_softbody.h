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

#ifndef PLASTIC_SOFTBODY_H
#define PLASTIC_SOFTBODY_H

class PlasticSoftBody : public SoftBody
{
public:
	PlasticSoftBody()
	{
		// Create soft body
		TetDef def;
		def.mesh = &m_mesh;
		def.density = 0.2f;
		def.elementYoungModulus = 1000.0f;
		def.elementPoissonRatio = 0.33f;		
		def.elementElasticStrainYield = 0.1f;
		def.elementCreepRate = 0.5f;
		def.elementMaxPlasticStrain = 1.0f;
		def.massDamping = 0.2f;

		m_body = new UniformSoftBody(def);

		// Up center vertex
		int i = m_mesh.GetRowVertexCount() - 1;
		int j = m_mesh.GetColumnVertexCount() / 2;
		int k = m_mesh.GetDepthVertexCount() / 2;

		int pinIndex = m_mesh.GetVertex(i, j, k);
		m_body->GetParticle(pinIndex)->SetType(e_staticSoftBodyParticle);

		b3Vec3 gravity(0.0f, -9.8f, 0.0f);
		m_body->SetGravity(gravity);

		m_bodyDragger = new SoftBodyDragger(&m_ray, m_body);
	}

	static Test* Create()
	{
		return new PlasticSoftBody;
	}

	GridTetMesh<2, 2, 2> m_mesh;
};

#endif
