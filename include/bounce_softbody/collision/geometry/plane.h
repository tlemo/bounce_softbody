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

#ifndef B3_PLANE_H
#define B3_PLANE_H

#include <bounce_softbody/common/math/math.h>
#include <bounce_softbody/common/math/transform.h>

// A plane in constant normal form.
// dot(n, p) - d = 0.
struct b3Plane
{
	// Does nothing for performance.
	b3Plane() { }

	// Set this plane from a normal and a signed distance from its origin.
	b3Plane(const b3Vec3& _normal, scalar _offset)
	{
		normal = _normal;
		offset = _offset;
	}

	// Set this plane from a normal and a point on the plane.
	b3Plane(const b3Vec3& _normal, const b3Vec3& _point)
	{
		normal = _normal;
		offset = b3Dot(_normal, _point);
	}

	// Compute this plane from three non-colinear points.
	b3Plane(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C)
	{
		b3Vec3 N = b3Cross(B - A, C - A);
		normal = b3Normalize(N);
		offset = b3Dot(normal, A);
	}

	b3Vec3 normal;
	scalar offset;
};

// Transform a plane by a given frame.
inline b3Plane b3Mul(const b3Transform& T, const b3Plane& plane)
{
	b3Vec3 normal = b3Mul(T.rotation, plane.normal);
	return b3Plane(normal, plane.offset + b3Dot(normal, T.translation));
}

// Transform a plane by a given frame.
inline b3Plane operator*(const b3Transform& T, const b3Plane& plane)
{
	return b3Mul(T, plane);
}

// Compute the distance between a point and a plane.
inline scalar b3Distance(const b3Vec3& P, const b3Plane& plane)
{
	return b3Dot(plane.normal, P) - plane.offset;
}

// Project a point onto a normal plane.
inline b3Vec3 b3ClosestPointOnPlane(const b3Vec3& P, const b3Plane& plane)
{
	scalar fraction = b3Distance(P, plane);
	return P - fraction * plane.normal;
}

#endif
