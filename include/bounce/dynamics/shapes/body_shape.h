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

#ifndef B3_BODY_SHAPE_H
#define B3_BODY_SHAPE_H

#include <bounce/common/math/math.h>

class b3Body;

enum b3BodyShapeType
{
	e_bodySphereShape,
	e_bodyTriangleShape,
	e_bodyTetrahedronShape,
	e_maxBodyShapes
};

// Body shape definition.
struct b3BodyShapeDef
{
	b3BodyShapeDef()
	{
		radius = scalar(0);
		friction = scalar(0);
		density = scalar(0);
		meshIndex = B3_MAX_U32;
	}

	// Type.
	b3BodyShapeType type;
	
	// Skin.
	scalar radius;

	// Coefficient of friction.
	scalar friction;

	// Density.
	scalar density;
	
	// Feature index into mesh.
	u32 meshIndex;
};

// This is an internal body shape.
class b3BodyShape
{
public:
	// Get the shape type.
	b3BodyShapeType GetType() const;
	
	// Get the body.
	b3Body* GetBody();
	const b3Body* GetBody() const;

	// Get the shape radius.
	scalar GetRadius() const;

	// Get the shape density.
	scalar GetDensity() const;

	// Set the coefficient of friction of this shape.
	// This represents both static and dynamic friction.
	void SetFriction(scalar friction);

	// Get the coefficient of friction of this shape.
	scalar GetFriction() const;
protected:
	friend class b3Body;
	friend class b3Particle;
	friend class b3ContactManager;

	b3BodyShape(const b3BodyShapeDef& def, b3Body* body);
	virtual ~b3BodyShape() { }

	// Type
	b3BodyShapeType m_type;

	// Body
	b3Body* m_body;

	// Radius
	scalar m_radius;

	// Coefficient of friction
	scalar m_friction;

	// Density
	scalar m_density;

	// Mesh index
	u32 m_meshIndex;
};

inline b3BodyShape::b3BodyShape(const b3BodyShapeDef& def, b3Body* body)
{
	m_body = body;
	m_radius = def.radius;
	m_friction = def.friction;
	m_density = def.density;
	m_meshIndex = def.meshIndex;
}

inline b3BodyShapeType b3BodyShape::GetType() const
{
	return m_type;
}

inline b3Body* b3BodyShape::GetBody()
{
	return m_body;
}

inline const b3Body* b3BodyShape::GetBody() const
{
	return m_body;
}

inline scalar b3BodyShape::GetRadius() const
{
	return m_radius;
}

inline void b3BodyShape::SetFriction(scalar friction)
{
	m_friction = friction;
}

inline scalar b3BodyShape::GetFriction() const
{
	return m_friction;
}

inline scalar b3BodyShape::GetDensity() const
{
	return m_density;
}

#endif