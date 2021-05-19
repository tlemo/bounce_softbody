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

#include <bounce/dynamics/shapes/softbody_world_shape.h>
#include <bounce/dynamics/softbody.h>
#include <bounce/common/template/array.h>
#include <bounce/draw.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

b3SoftBodyWorldShape::b3SoftBodyWorldShape()
{
	m_baseType = e_softBodyWorldShapeBase;
	m_radius = scalar(0);
}

void b3SoftBodyWorldShape::Synchronize(const b3Vec3& displacement)
{
	b3AABB aabb = ComputeAABB();
	m_body->m_contactManager.m_broadPhase.MoveProxy(m_proxy.proxyId, aabb, displacement);
}

void b3SoftBodyWorldShape::DestroyContacts()
{
	b3SoftBodySphereAndShapeContact* c = m_body->m_contactManager.m_sphereAndShapeContactList.m_head;
	while (c)
	{
		b3SoftBodySphereAndShapeContact* c0 = c;
		c = c->m_next;

		if (c0->m_s2 == this)
		{
			m_body->m_contactManager.Destroy(c0);
		}
	}
}

b3SoftBodyWorldShape* b3SoftBodyWorldShape::Create(const b3SoftBodyWorldShapeDef& def)
{
	b3SoftBodyWorldShape* shape = nullptr;
	switch (def.shape->GetShapeType())
	{
	case e_softBodySphereWorldShape:
	{
		// Grab pointer to the specific memory.
		b3SoftBodySphereWorldShape* sphere1 = (b3SoftBodySphereWorldShape*)def.shape;
		void* mem = b3Alloc(sizeof(b3SoftBodySphereWorldShape));
		b3SoftBodySphereWorldShape* sphere2 = new (mem)b3SoftBodySphereWorldShape();
		sphere2->Clone(*sphere1);
		shape = sphere2;
		break;
	}
	case e_softBodyCapsuleWorldShape:
	{
		// Grab pointer to the specific memory.
		b3SoftBodyCapsuleWorldShape* caps1 = (b3SoftBodyCapsuleWorldShape*)def.shape;
		void* block = b3Alloc(sizeof(b3SoftBodyCapsuleWorldShape));
		b3SoftBodyCapsuleWorldShape* caps2 = new (block)b3SoftBodyCapsuleWorldShape();
		caps2->Clone(*caps1);
		shape = caps2;
		break;
	}
	case e_softBodyBoxWorldShape:
	{
		// Grab pointer to the specific memory.
		b3SoftBodyBoxWorldShape* box1 = (b3SoftBodyBoxWorldShape*)def.shape;
		void* block = b3Alloc(sizeof(b3SoftBodyBoxWorldShape));
		b3SoftBodyBoxWorldShape* box2 = new (block)b3SoftBodyBoxWorldShape();
		box2->Clone(*box1);
		shape = box2;
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}

	return shape;
}

void b3SoftBodyWorldShape::Destroy(b3SoftBodyWorldShape* shape)
{
	// Free the shape from the memory.
	switch (shape->GetShapeType())
	{
	case e_softBodySphereWorldShape:
	{
		b3SoftBodySphereWorldShape* sphere = (b3SoftBodySphereWorldShape*)shape;
		sphere->~b3SoftBodySphereWorldShape();
		b3Free(shape);
		break;
	}
	case e_softBodyCapsuleWorldShape:
	{
		b3SoftBodyCapsuleWorldShape* caps = (b3SoftBodyCapsuleWorldShape*)shape;
		caps->~b3SoftBodyCapsuleWorldShape();
		b3Free(shape);
		break;
	}
	case e_softBodyBoxWorldShape:
	{
		b3SoftBodyBoxWorldShape* box = (b3SoftBodyBoxWorldShape*)shape;
		box->~b3SoftBodyBoxWorldShape();
		b3Free(shape);
		break;
	}
	default:
	{
		B3_ASSERT(false);
	}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

b3SoftBodySphereWorldShape::b3SoftBodySphereWorldShape()
{
	m_worldShapeType = e_softBodySphereWorldShape;
	m_center.SetZero();
	m_radius = scalar(0);
}

b3SoftBodySphereWorldShape::~b3SoftBodySphereWorldShape()
{

}

void b3SoftBodySphereWorldShape::Clone(const b3SoftBodySphereWorldShape& other)
{
	m_center = other.m_center;
	m_radius = other.m_radius;
}

b3AABB b3SoftBodySphereWorldShape::ComputeAABB() const
{
	b3Vec3 center = m_center;
	b3Vec3 r(m_radius, m_radius, m_radius);
	
	b3AABB aabb;
	aabb.lowerBound = center - r;
	aabb.upperBound = center + r;
	return aabb;
}

bool b3SoftBodySphereWorldShape::CollideSphere(b3SoftBodySphereManifold* manifold, const b3Sphere& sphere) const
{
	b3Vec3 center = m_center;
	scalar radius = m_radius + sphere.radius;
	scalar rr = radius * radius;
	b3Vec3 d = sphere.vertex - center;
	scalar dd = b3Dot(d, d);

	if (dd <= rr)
	{
		scalar distance = b3Sqrt(dd);

		manifold->point = center;

		if (distance > B3_EPSILON)
		{
			manifold->normal = d / distance;
		}
		else
		{
			manifold->normal.Set(0, 1, 0);
		}

		return true;
	}

	return false;
}

void b3SoftBodySphereWorldShape::Draw() const
{
	b3Draw_draw->DrawPoint(m_center, scalar(4), b3Color_black);
	b3Draw_draw->DrawSolidSphere(b3Vec3_y, m_center, m_radius, b3Color_gray);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

b3SoftBodyCapsuleWorldShape::b3SoftBodyCapsuleWorldShape()
{
	m_worldShapeType = e_softBodyCapsuleWorldShape;
	m_center1.Set(0, 1, 0);
	m_center1.Set(0, -1, 0);
	m_radius = scalar(0);
}

b3SoftBodyCapsuleWorldShape::~b3SoftBodyCapsuleWorldShape()
{

}

void b3SoftBodyCapsuleWorldShape::Clone(const b3SoftBodyCapsuleWorldShape& other)
{
	m_center1 = other.m_center1;
	m_center2 = other.m_center2;
	m_radius = other.m_radius;
}

b3AABB b3SoftBodyCapsuleWorldShape::ComputeAABB() const
{
	b3Vec3 c1 = m_center1;
	b3Vec3 c2 = m_center2;
	b3Vec3 r(m_radius, m_radius, m_radius);
	
	b3AABB aabb;
	aabb.lowerBound = b3Min(c1, c2) - r;
	aabb.upperBound = b3Max(c1, c2) + r;
	return aabb;
}

bool b3SoftBodyCapsuleWorldShape::CollideSphere(b3SoftBodySphereManifold* manifold, const b3Sphere& sphere) const
{
	b3Vec3 Q = sphere.vertex;

	b3Vec3 A = m_center1;
	b3Vec3 B = m_center2;
	b3Vec3 AB = B - A;

	// Barycentric coordinates for Q
	scalar u = b3Dot(B - Q, AB);
	scalar v = b3Dot(Q - A, AB);

	scalar radius = m_radius + sphere.radius;

	if (v <= scalar(0))
	{
		// A
		b3Vec3 P = A;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		b3Vec3 n(scalar(0), scalar(1), scalar(0));
		scalar len = b3Sqrt(dd);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold->point = P;
		manifold->normal = n;

		return true;
	}

	if (u <= scalar(0))
	{
		// B
		b3Vec3 P = B;
		b3Vec3 d = Q - P;
		scalar dd = b3Dot(d, d);
		if (dd > radius * radius)
		{
			return false;
		}

		b3Vec3 n(scalar(0), scalar(1), scalar(0));
		scalar len = b3Sqrt(dd);
		if (len > B3_EPSILON)
		{
			n = d / len;
		}

		manifold->point = P;
		manifold->normal = n;

		return true;
	}

	// AB
	scalar s = b3Dot(AB, AB);
	//B3_ASSERT(s > scalar(0));
	b3Vec3 P = (u * A + v * B) / s;

	b3Vec3 d = Q - P;
	scalar dd = b3Dot(d, d);
	if (dd > radius * radius)
	{
		return false;
	}

	b3Vec3 AQ = Q - A;
	b3Vec3 AB_x_AQ = b3Cross(AB, AQ);
	b3Vec3 n = b3Cross(AB_x_AQ, AB);
	if (b3Dot(n, AQ) < scalar(0))
	{
		n = -n;
	}
	n.Normalize();

	manifold->point = P;
	manifold->normal = n;
	return true;
}

void b3SoftBodyCapsuleWorldShape::Draw() const
{
	b3Draw_draw->DrawPoint(m_center1, scalar(4), b3Color_black);
	b3Draw_draw->DrawPoint(m_center2, scalar(4), b3Color_black);
	b3Draw_draw->DrawSegment(m_center1, m_center2, b3Color_black);

	b3Draw_draw->DrawSolidCapsule(b3Vec3_y, m_center1, m_center2, m_radius, b3Color_gray);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

b3SoftBodyBoxWorldShape::b3SoftBodyBoxWorldShape()
{
	m_worldShapeType = e_softBodyBoxWorldShape;
	m_extents.Set(scalar(1), scalar(1), scalar(1));
	m_xf.SetIdentity();
	m_radius = scalar(0);
}

b3SoftBodyBoxWorldShape::~b3SoftBodyBoxWorldShape()
{

}

void b3SoftBodyBoxWorldShape::Clone(const b3SoftBodyBoxWorldShape& other)
{
	m_extents = other.m_extents;
	m_xf = other.m_xf;
	m_radius = other.m_radius;
}

b3AABB b3SoftBodyBoxWorldShape::ComputeAABB() const
{
	b3Vec3 e = m_extents;

	b3Vec3 vertices[8] = 
	{
		b3Vec3(-e.x, -e.y, -e.z),
		b3Vec3(-e.x, -e.y,  e.z),
		b3Vec3(-e.x,  e.y, -e.z),
		b3Vec3(-e.x,  e.y,  e.z),
		b3Vec3(e.x, -e.y, -e.z),
		b3Vec3(e.x, -e.y,  e.z),
		b3Vec3(e.x,  e.y, -e.z),
		b3Vec3(e.x,  e.y,  e.z)
	};

	b3AABB aabb;
	aabb.Set(vertices, 8, m_xf);
	
	aabb.Extend(m_radius);

	return aabb;
}

bool b3SoftBodyBoxWorldShape::CollideSphere(b3SoftBodySphereManifold* manifold, const b3Sphere& sphere) const
{
	b3Vec3 e = m_extents;

	scalar radius = m_radius + sphere.radius;

	// Sphere center in the frame of the box.
	b3Vec3 cLocal = b3MulT(m_xf, sphere.vertex);

	// Box planes
	b3Plane planes[6] =
	{
		b3Plane(b3Vec3(scalar(1), scalar(0), scalar(0)), e.x),
		b3Plane(b3Vec3(scalar(-1), scalar(0), scalar(0)), e.x),
		b3Plane(b3Vec3(scalar(0), scalar(1), scalar(0)), e.y),
		b3Plane(b3Vec3(scalar(0), scalar(-1), scalar(0)), e.y),
		b3Plane(b3Vec3(scalar(0), scalar(0), scalar(1)), e.z),
		b3Plane(b3Vec3(scalar(0), scalar(0), scalar(-1)), e.z),
	};

	// Find the minimum separation face.	
	u32 faceIndex = 0;
	scalar separation = -B3_MAX_SCALAR;

	for (u32 i = 0; i < 6; ++i)
	{
		b3Plane plane = planes[i];
		scalar s = b3Distance(cLocal, plane);

		if (s > radius)
		{
			// Early out.
			return false;
		}

		if (s > separation)
		{
			faceIndex = i;
			separation = s;
		}
	}
	
	if (separation < scalar(0))
	{
		// The sphere center is inside the box.
		b3Plane pLocal = planes[faceIndex];

		b3Vec3 cBox = b3ClosestPointOnPlane(cLocal, pLocal);

		manifold->point = b3Mul(m_xf, cBox);
		manifold->normal = b3Mul(m_xf.rotation, pLocal.normal);

		return true;
	}

	b3Vec3 lo(-e.x, -e.y, -e.z);

	// Closest point on box to sphere center
	b3Vec3 cBox = b3Max(lo, b3Min(cLocal, e));

	scalar distance = b3Length(cLocal - cBox);

	if (distance > radius)
	{
		return false;
	}

	if (distance > scalar(0))
	{
		b3Vec3 normal = (cLocal - cBox) / distance;

		manifold->point = b3Mul(m_xf, cBox);
		manifold->normal = b3Mul(m_xf.rotation, normal);

		return true;
	}

	return false;
}

void b3SoftBodyBoxWorldShape::Draw() const
{
	b3Vec3 e = m_extents;

	b3Vec3 vertices[8] = 
	{
		b3Vec3(-e.x, -e.y, -e.z),
		b3Vec3(-e.x, -e.y,  e.z),
		b3Vec3(-e.x,  e.y, -e.z),
		b3Vec3(-e.x,  e.y,  e.z),
		b3Vec3(e.x, -e.y, -e.z),
		b3Vec3(e.x, -e.y,  e.z),
		b3Vec3(e.x,  e.y, -e.z),
		b3Vec3(e.x,  e.y,  e.z)
	};

	u32 indices[36] =
	{
		0, 6, 4,
		0, 2, 6,
		0, 3, 2,
		0, 1, 3,
		2, 7, 6,
		2, 3, 7,
		4, 6, 7,
		4, 7, 5,
		0, 4, 5,
		0, 5, 1,
		1, 5, 7,
		1, 7, 3
	};

	for (u32 i = 0; i < 36; i += 3)
	{
		b3Vec3 A = m_xf * vertices[indices[i]];
		b3Vec3 B = m_xf * vertices[indices[i + 1]];
		b3Vec3 C = m_xf * vertices[indices[i + 2]];

		b3Vec3 N = b3Normalize(b3Cross(B - A, C - A));

		b3Draw_draw->DrawSolidTriangle(N, A, B, C, b3Color_gray);
	}
}
