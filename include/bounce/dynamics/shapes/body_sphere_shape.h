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

#ifndef B3_BODY_SPHERE_SHAPE_H
#define B3_BODY_SPHERE_SHAPE_H

#include <bounce/dynamics/shapes/body_shape.h>
#include <bounce/collision/geometry/aabb.h>
#include <bounce/common/template/list.h>

class b3Particle;

// Body sphere shape definition.
struct b3BodySphereShapeDef : public b3BodyShapeDef
{
	b3BodySphereShapeDef()
	{
		type = e_bodySphereShape;
	}

	// Particle
	b3Particle* p = nullptr;
};

// Sphere shape.
class b3BodySphereShape : public b3BodyShape
{
public:
	// Return the particle.
	b3Particle* GetParticle() { return m_p; }
	const b3Particle* GetParticle() const { return m_p; }

	// Return the next sphere in the body.
	b3BodySphereShape* GetNext() { return m_next; };
	const b3BodySphereShape* GetNext() const { return m_next; };
private:
	friend class b3Body;
	friend class b3Particle;
	friend class b3ContactManager;
	friend class b3SphereAndShapeContact;
	friend class b3BodySolver;
	friend class b3FrictionSolver;
	friend class b3List<b3BodySphereShape>;

	b3BodySphereShape(const b3BodySphereShapeDef& def, b3Body* body);
	
	// Compute AABB
	b3AABB ComputeAABB() const;

	// Destroy contacts
	void DestroyContacts();

	// Particle
	b3Particle* m_p;

	// Links to the body list.
	b3BodySphereShape* m_prev;
	b3BodySphereShape* m_next;
};

#endif