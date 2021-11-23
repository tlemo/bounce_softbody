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

#ifndef B3_BODY_TETRAHEDRON_SHAPE_H
#define B3_BODY_TETRAHEDRON_SHAPE_H

#include <bounce/dynamics/shapes/body_shape.h>
#include <bounce/collision/geometry/aabb.h>
#include <bounce/common/template/list.h>

class b3Particle;

// Body tetrahedron shape definition.
struct b3BodyTetrahedronShapeDef : public b3BodyShapeDef
{
	b3BodyTetrahedronShapeDef()
	{
		type = e_bodyTetrahedronShape;
	}

	// Particles
	b3Particle* p1 = nullptr;
	b3Particle* p2 = nullptr;
	b3Particle* p3 = nullptr;
	b3Particle* p4 = nullptr;

	// Rest vertices
	b3Vec3 v1, v2, v3, v4;
};

// Tetrahedron shape. Used for computing the mass of the particles in a body.
class b3BodyTetrahedronShape : public b3BodyShape
{
public:
	// Return the particle 1.
	b3Particle* GetParticle1() { return m_p1; }
	const b3Particle* GetParticle1() const { return m_p1; }

	// Return the particle 2.
	b3Particle* GetParticle2() { return m_p2; }
	const b3Particle* GetParticle2() const { return m_p2; }

	// Return the particle 3.
	b3Particle* GetParticle3() { return m_p3; }
	const b3Particle* GetParticle3() const { return m_p3; }

	// Return the particle 4.
	b3Particle* GetParticle4() { return m_p4; }
	const b3Particle* GetParticle4() const { return m_p4; }
	
	// Return the next tetrahedron shape in the body list of shapes.
	b3BodyTetrahedronShape* GetNext() { return m_next; }
	const b3BodyTetrahedronShape* GetNext() const { return m_next; }
private:
	friend class b3Body;
	friend class b3Particle;
	friend class b3List<b3BodyTetrahedronShape>;

	b3BodyTetrahedronShape(const b3BodyTetrahedronShapeDef& def, b3Body* body);

	// Particles
	b3Particle* m_p1;
	b3Particle* m_p2;
	b3Particle* m_p3;
	b3Particle* m_p4;

	// Rest volume. Used for computing the mass of the particles.
	scalar m_volume;

	// Links to the body list.
	b3BodyTetrahedronShape* m_prev;
	b3BodyTetrahedronShape* m_next;
};

#endif