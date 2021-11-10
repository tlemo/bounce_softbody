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

#ifndef B3_SOFTBODY_WORLD_SHAPE_H
#define B3_SOFTBODY_WORLD_SHAPE_H

#include <bounce/collision/shapes/shape.h>
#include <bounce/common/template/list.h>

class b3Draw;
class b3SoftBody;

// Collision shape definition.
// The given shape will be cloned and can be a temporary object.
struct b3SoftBodyWorldShapeDef
{
	b3SoftBodyWorldShapeDef()
	{
		shape = nullptr;
		friction = scalar(0.5);
	}

	// Shape to be cloned.
	b3Shape* shape;
	
	// Coefficient of friction in the range [0, 1].
	scalar friction;
};

// Body collision shape in static environment.
class b3SoftBodyWorldShape
{
public:
	// Default dtor.
	virtual ~b3SoftBodyWorldShape() { }
	
	// Return the shape type.
	b3Shape::Type GetType() const;
	
	// Compute AABB.
	b3AABB ComputeAABB() const;

	// Generate the contact manifold for a given sphere.
	// Return true if the given sphere is colliding with this shape, false otherwise.
	bool CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const;

	// Draw this shape.
	void Draw(b3Draw* draw) const;

	// Set the coefficient of friction of this shape.
	void SetFriction(scalar friction);

	// Get the coefficient of friction of this shape.
	// This represents both static and dynamic friction.
	scalar GetFriction() const;

	// Return the next world shape in the body list of world shapes.
	b3SoftBodyWorldShape* GetNext() { return m_next; }
	const b3SoftBodyWorldShape* GetNext() const { return m_next; }
private:
	friend class b3SoftBody;
	friend class b3SoftBodyContactManager;
	friend class b3SoftBodySphereAndShapeContact;
	friend class b3SoftBodyContactSolver;
	friend class b3List<b3SoftBodyWorldShape>;

	// Ctor.
	b3SoftBodyWorldShape();
	
	// Create/destroy this shape. 
	void Create(b3BlockAllocator* allocator, b3SoftBody* body, const b3SoftBodyWorldShapeDef& def);
	void Destroy(b3BlockAllocator* allocator);

	// Destroy contacts.
	void DestroyContacts();

	// The collision shape.
	b3Shape* m_shape;

	// Coefficient of friction.
	scalar m_friction;

	// Body.
	b3SoftBody* m_body;

	// Body list links.
	b3SoftBodyWorldShape* m_prev;
	b3SoftBodyWorldShape* m_next;
};

inline b3Shape::Type b3SoftBodyWorldShape::GetType() const
{
	return m_shape->m_type;
}

inline b3AABB b3SoftBodyWorldShape::ComputeAABB() const
{
	return m_shape->ComputeAABB();
}

inline bool b3SoftBodyWorldShape::CollideSphere(b3SphereManifold* manifold, const b3Sphere& sphere) const
{
	return m_shape->CollideSphere(manifold, sphere);
}

inline void b3SoftBodyWorldShape::Draw(b3Draw* draw) const
{
	m_shape->Draw(draw);
}

inline void b3SoftBodyWorldShape::SetFriction(scalar friction)
{
	m_friction = friction;
}

inline scalar b3SoftBodyWorldShape::GetFriction() const
{
	return m_friction;
}

#endif
