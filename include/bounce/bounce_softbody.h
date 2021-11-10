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

#ifndef BOUNCE_SOFTBODY_H
#define BOUNCE_SOFTBODY_H

// Include this file header in your project to directly access Bounce Softbody objects.

#include <bounce/common/settings.h>
#include <bounce/common/draw.h>

#include <bounce/common/math/math.h>

#include <bounce/dynamics/softbody.h>
#include <bounce/dynamics/softbody_particle.h>

#include <bounce/dynamics/shapes/softbody_sphere_shape.h>
#include <bounce/dynamics/shapes/softbody_capsule_shape.h>
#include <bounce/dynamics/shapes/softbody_triangle_shape.h>
#include <bounce/dynamics/shapes/softbody_tetrahedron_shape.h>
#include <bounce/dynamics/shapes/softbody_world_shape.h>

#include <bounce/dynamics/forces/softbody_stretch_force.h>
#include <bounce/dynamics/forces/softbody_shear_force.h>
#include <bounce/dynamics/forces/softbody_spring_force.h>
#include <bounce/dynamics/forces/softbody_mouse_force.h>
#include <bounce/dynamics/forces/softbody_triangle_element_force.h>
#include <bounce/dynamics/forces/softbody_tetrahedron_element_force.h>

#endif
