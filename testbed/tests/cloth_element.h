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

#ifndef CLOTH_ELEMENT_H
#define CLOTH_ELEMENT_H

class ClothElement : public SoftBody
{
public:
	ClothElement(const TestDef& testDef) : SoftBody(testDef)
	{
		ClothDef def;
		def.mesh = &m_mesh;
		def.createElements = true;
		m_body = new UniformSoftBody(def);

		for (int i = 0; i < m_mesh.GetColumnVertexCount(); ++i)
		{
			int vertex = m_mesh.GetVertex(0, i);
			m_body->GetParticle(vertex)->SetType(e_staticSoftBodyParticle);
		}

		m_body->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));
		
		m_bodyDragger = new b3SoftBodyDragger(&m_ray, m_body);
		m_bodyDragger->SetStaticDrag(false);
	}

	static Test* Create(const TestDef& def)
	{
		return new ClothElement(def);
	}

	GridClothMesh<10, 10> m_mesh;
};

#endif
