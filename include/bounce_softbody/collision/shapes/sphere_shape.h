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

#ifndef B3_SPHERE_SHAPE_H
#define B3_SPHERE_SHAPE_H

#include <bounce_softbody/collision/shapes/shape.h>

// Sphere collision shape.
class b3SphereShape : public b3Shape
{
public:
	b3SphereShape();

	b3Shape* Clone(b3BlockAllocator* allocator) const;
	
	b3AABB ComputeAABB() const;

	bool CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const;
	
	void Draw(b3Draw* draw) const;

	// Center
	b3Vec3 m_center;
};

#endif
