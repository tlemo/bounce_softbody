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

#ifndef CLOTH_TEARING_H
#define CLOTH_TEARING_H

#include <bounce/common/template/array.h>
#include <bounce/collision/geometry/plane.h>

class ClothTearing : public SoftBody
{
public:
	ClothTearing()	
	{
		m_body = new UniformSoftBody();

		GridClothMesh<10, 10> m;

		b3SoftBodyParticle** particles = (b3SoftBodyParticle * *)malloc(m.vertexCount * sizeof(b3SoftBodyParticle*));
		for (int i = 0; i < m.vertexCount; ++i)
		{
			b3SoftBodyParticleDef pd;
			pd.type = e_dynamicSoftBodyParticle;
			pd.position = m.GetVertexPosition(i);

			b3SoftBodyParticle* p = m_body->CreateParticle(pd);
			particles[i] = p;

			b3SoftBodySphereShapeDef sd;
			sd.p = p;
			sd.radius = 0.2f;
			sd.friction = 0.4f;

			m_body->CreateSphereShape(sd);
		}

		for (int i = 0; i < m.triangleCount; ++i)
		{
			SoftBodyMeshTriangle triangle = m.GetTriangle(i);
			int v1 = triangle.v1;
			int v2 = triangle.v2;
			int v3 = triangle.v3;

			b3SoftBodyParticle* p1 = particles[v1];
			b3SoftBodyParticle* p2 = particles[v2];
			b3SoftBodyParticle* p3 = particles[v3];

			b3SoftBodyTriangleShapeDef tsd;
			tsd.p1 = p1;
			tsd.p2 = p2;
			tsd.p3 = p3;
			tsd.v1 = p1->GetPosition();
			tsd.v2 = p2->GetPosition();
			tsd.v3 = p3->GetPosition();
			tsd.density = 0.1f;

			m_body->CreateTriangleShape(tsd);

			{
				b3SoftBodySpringForceDef sfd;
				sfd.Initialize(p1, p2, 1000.0f, 10.0f);

				CreateSpringForce(sfd);
			}

			{
				b3SoftBodySpringForceDef sfd;
				sfd.Initialize(p2, p3, 1000.0f, 10.0f);

				CreateSpringForce(sfd);
			}

			{
				b3SoftBodySpringForceDef sfd;
				sfd.Initialize(p3, p1, 1000.0f, 10.0f);

				CreateSpringForce(sfd);
			}
		}

		for (int i = 0; i < m.GetColumnVertexCount(); ++i)
		{
			int vertex = m.GetVertex(0, i);
			particles[vertex]->SetType(e_staticSoftBodyParticle);
		}

		free(particles);

		m_body->SetGravity(b3Vec3(0.0f, -9.8f, 0.0f));

		m_bodyDragger = new SoftBodyDragger(&m_ray, m_body);
		m_bodyDragger->SetStaticDrag(false);
	}

	b3SoftBodySpringForce* FindSpringForce(b3SoftBodyParticle* p1, b3SoftBodyParticle* p2)
	{
		for (b3SoftBodyForce* f = m_body->GetForceList().m_head; f; f = f->GetNext())
		{
			if (f->GetType() != e_softBodySpringForce)
			{
				continue;
			}

			b3SoftBodySpringForce* sf = (b3SoftBodySpringForce*)f;

			b3SoftBodyParticle* sp1 = sf->GetParticle1();
			b3SoftBodyParticle* sp2 = sf->GetParticle2();

			if (sp1 == p1 && sp2 == p2)
			{
				return sf;
			}

			if (sp1 == p2 && sp2 == p1)
			{
				return sf;
			}
		}

		return nullptr;
	}

	b3SoftBodySpringForce* CreateSpringForce(const b3SoftBodySpringForceDef& def)
	{
		b3SoftBodySpringForce* sf = FindSpringForce(def.p1, def.p2);
		if (sf != nullptr)
		{
			return sf;
		}

		return (b3SoftBodySpringForce*)m_body->CreateForce(def);
	}

	void DrawSpringForces()
	{
		for (b3SoftBodyForce* f = m_body->GetForceList().m_head; f; f = f->GetNext())
		{
			if (f->GetType() != e_softBodySpringForce)
			{
				continue;
			}

			b3SoftBodySpringForce* s = (b3SoftBodySpringForce*)f;

			b3SoftBodyParticle* p1 = s->GetParticle1();
			b3SoftBodyParticle* p2 = s->GetParticle2();

			b3DrawSegment(g_debugDrawData, p1->GetPosition(), p2->GetPosition(), b3Color_black);
		}
	}

	void Partition(b3SoftBodyParticle* p, const b3Plane& plane,
		b3Array<b3SoftBodyTriangleShape*>& above,
		b3Array<b3SoftBodyTriangleShape*>& below)
	{
		for (b3SoftBodyTriangleShape* t = m_body->GetTriangleShapeList().m_head; t; t = t->GetNext())
		{
			b3SoftBodyParticle* p1 = t->GetParticle1();
			b3SoftBodyParticle* p2 = t->GetParticle2();
			b3SoftBodyParticle* p3 = t->GetParticle3();

			if (p1 != p && p2 != p && p3 != p)
			{
				continue;
			}

			b3Vec3 x1 = p1->GetPosition();
			b3Vec3 x2 = p2->GetPosition();
			b3Vec3 x3 = p3->GetPosition();

			b3Vec3 center = (x1 + x2 + x3) / 3.0f;

			scalar distance = b3Distance(center, plane);
			if (distance > 0.0f)
			{
				above.PushBack(t);
			}
			else
			{
				below.PushBack(t);
			}
		}
	}

	bool HasSpring(const b3Array<b3SoftBodyTriangleShape*>& triangles,
		b3SoftBodyParticle* pSplit, b3SoftBodyParticle* pOther)
	{
		for (u32 i = 0; i < triangles.Count(); ++i)
		{
			b3SoftBodyTriangleShape* triangle = triangles[i];

			b3SoftBodyParticle* tp1 = triangle->GetParticle1();
			b3SoftBodyParticle* tp2 = triangle->GetParticle2();
			b3SoftBodyParticle* tp3 = triangle->GetParticle3();

			// 1, 2
			if (tp1 == pSplit && tp2 == pOther)
			{
				return true;
			}

			// 2, 1
			if (tp2 == pSplit && tp1 == pOther)
			{
				return true;
			}

			// 2, 3
			if (tp2 == pSplit && tp3 == pOther)
			{
				return true;
			}

			// 3, 2
			if (tp3 == pSplit && tp2 == pOther)
			{
				return true;
			}

			// 3, 1
			if (tp3 == pSplit && tp1 == pOther)
			{
				return true;
			}

			// 1, 3
			if (tp1 == pSplit && tp3 == pOther)
			{
				return true;
			}
		}

		return false;
	}

	bool SplitParticle(b3SoftBodyParticle* pSplit, const b3Plane& plane)
	{
		// Collect triangles.
		b3StackArray<b3SoftBodyTriangleShape*, 32> trianglesAbove, trianglesBelow;
		Partition(pSplit, plane, trianglesAbove, trianglesBelow);

		// There must be at least one triangle on each side of the plane.
		if (trianglesAbove.Count() == 0 || trianglesBelow.Count() == 0)
		{
			return false;
		}

		b3SoftBodyParticleDef pdNew;
		pdNew.type = pSplit->GetType();
		pdNew.position = pSplit->GetPosition() - 0.2f * plane.normal;

		b3SoftBodyParticle* pNew = m_body->CreateParticle(pdNew);

		b3SoftBodySphereShapeDef ssdNew;
		ssdNew.p = pNew;
		ssdNew.radius = 0.2f;
		ssdNew.friction = 0.4f;
		
		m_body->CreateSphereShape(ssdNew);

		for (u32 i = 0; i < trianglesBelow.Count(); ++i)
		{
			b3SoftBodyTriangleShape* triangle = trianglesBelow[i];

			b3SoftBodyParticle* p1 = triangle->GetParticle1();
			b3SoftBodyParticle* p2 = triangle->GetParticle2();
			b3SoftBodyParticle* p3 = triangle->GetParticle3();

			m_body->DestroyTriangleShape(triangle);

			if (p1 == pSplit)
			{
				b3SoftBodyTriangleShapeDef tdNew;
				tdNew.p1 = pNew;
				tdNew.p2 = p2;
				tdNew.p3 = p3;
				tdNew.v1 = pNew->GetPosition();
				tdNew.v2 = p2->GetPosition();
				tdNew.v3 = p3->GetPosition();

				m_body->CreateTriangleShape(tdNew);

				b3SoftBodySpringForce* sf1 = FindSpringForce(p1, p2);
				if (sf1)
				{
					b3SoftBodySpringForceDef sNew;
					sNew.p1 = pNew;
					sNew.p2 = p2;
					sNew.restLength = sf1->GetRestLenght();
					sNew.stiffness = sf1->GetStiffness();
					sNew.dampingStiffness = sf1->GetDampingStiffness();

					m_body->CreateForce(sNew);

					if (HasSpring(trianglesAbove, p1, p2) == false)
					{
						m_body->DestroyForce(sf1);
					}
				}

				b3SoftBodySpringForce* sf2 = FindSpringForce(p3, p1);
				if (sf2)
				{
					b3SoftBodySpringForceDef sNew;
					sNew.p1 = p3;
					sNew.p2 = pNew;
					sNew.restLength = sf2->GetRestLenght();
					sNew.stiffness = sf2->GetStiffness();
					sNew.dampingStiffness = sf2->GetDampingStiffness();
					
					m_body->CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p3, p1) == false)
					{
						m_body->DestroyForce(sf2);
					}
				}
			}
			
			if (p2 == pSplit)
			{
				b3SoftBodyTriangleShapeDef tdNew;
				tdNew.p1 = p1;
				tdNew.p2 = pNew;
				tdNew.p3 = p3;
				tdNew.v1 = p1->GetPosition();
				tdNew.v2 = pNew->GetPosition();
				tdNew.v3 = p3->GetPosition();

				m_body->CreateTriangleShape(tdNew);

				b3SoftBodySpringForce* sf1 = FindSpringForce(p1, p2);
				if (sf1)
				{
					b3SoftBodySpringForceDef sNew;
					sNew.p1 = p1;
					sNew.p2 = pNew;
					sNew.restLength = sf1->GetRestLenght();
					sNew.stiffness = sf1->GetStiffness();
					sNew.dampingStiffness = sf1->GetDampingStiffness();

					m_body->CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p1, p2) == false)
					{
						m_body->DestroyForce(sf1);
					}
				}

				b3SoftBodySpringForce* sf2 = FindSpringForce(p2, p3);
				if (sf2)
				{
					b3SoftBodySpringForceDef sNew;
					sNew.p1 = pNew;
					sNew.p2 = p3;
					sNew.restLength = sf2->GetRestLenght();
					sNew.stiffness = sf2->GetStiffness();
					sNew.dampingStiffness = sf2->GetDampingStiffness();
					
					m_body->CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p2, p3) == false)
					{
						m_body->DestroyForce(sf2);
					}
				}
			}
			
			if (p3 == pSplit)
			{
				b3SoftBodyTriangleShapeDef tdNew;
				tdNew.p1 = p1;
				tdNew.p2 = p2;
				tdNew.p3 = pNew;
				tdNew.v1 = p1->GetPosition();
				tdNew.v2 = p2->GetPosition();
				tdNew.v3 = pNew->GetPosition();

				m_body->CreateTriangleShape(tdNew);
				
				b3SoftBodySpringForce* sf1 = FindSpringForce(p2, p3);
				if (sf1)
				{
					b3SoftBodySpringForceDef sNew;
					sNew.p1 = p2;
					sNew.p2 = pNew;
					sNew.restLength = sf1->GetRestLenght();
					sNew.stiffness = sf1->GetStiffness();
					sNew.dampingStiffness = sf1->GetDampingStiffness();
					
					m_body->CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p2, p3) == false)
					{
						m_body->DestroyForce(sf1);
					}
				}

				b3SoftBodySpringForce* sf2 = FindSpringForce(p3, p1);
				if (sf2)
				{
					b3SoftBodySpringForceDef sNew;
					sNew.p1 = pNew;
					sNew.p2 = p1;
					sNew.restLength = sf2->GetRestLenght();
					sNew.stiffness = sf2->GetStiffness();
					sNew.dampingStiffness = sf2->GetDampingStiffness();
					
					m_body->CreateForce(sNew);
					
					if (HasSpring(trianglesAbove, p3, p1) == false)
					{
						m_body->DestroyForce(sf2);
					}
				}
			}
		}

		return true;
	}

	bool Tear()
	{
		b3SoftBodyForce* f = m_body->GetForceList().m_head;
		while (f)
		{
			if (f->GetType() != e_softBodySpringForce)
			{
				f = f->GetNext();
				continue;
			}

			b3SoftBodySpringForce* s = (b3SoftBodySpringForce*)f;
			f = f->GetNext();

			b3Vec3 tension = s->GetActionForce();

			const scalar kMaxTension = 1000.0f;

			if (b3LengthSquared(tension) <= kMaxTension * kMaxTension)
			{
				continue;
			}

			b3SoftBodyParticle* p1 = s->GetParticle1();
			b3SoftBodyParticle* p2 = s->GetParticle2();

			b3Vec3 x1 = p1->GetPosition();
			b3Vec3 x2 = p2->GetPosition();

			if (p1->GetType() == e_dynamicSoftBodyParticle)
			{
				b3Vec3 n = b3Normalize(x2 - x1);
				b3Plane plane(n, x1);

				bool wasSplit = SplitParticle(p1, plane);
				if (wasSplit)
				{
					return true;
				}
			}

			if (p2->GetType() == e_dynamicSoftBodyParticle)
			{
				b3Vec3 n = b3Normalize(x1 - x2);
				b3Plane plane(n, x2);

				bool wasSplit = SplitParticle(p2, plane);
				if (wasSplit)
				{
					return true;
				}
			}
		}

		return false;
	}

	void Step()
	{
		SoftBody::Step();

		while (Tear());

		DrawSpringForces();
	}

	static Test* Create()
	{
		return new ClothTearing;
	}
};

#endif
