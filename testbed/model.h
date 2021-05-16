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

#ifndef MODEL_H
#define MODEL_H

#include "gl_debugdraw.h"
#include <bounce/common/graphics/camera.h>
#include <bounce/common/graphics/debugdraw.h>

class Test;

struct Properties;
struct TestProperties;

class Model
{
public:
	Model();
	~Model();
	
	void EnablePause(bool flag) { m_pause = flag; } 
	bool IsPaused() const { return m_pause; }
	
	void EnableSetTest(bool flag) { m_setTest = flag; }
	bool WillSetTest() { return m_setTest; }
	
	bool WillSinglePlay() { return m_singlePlay; }

	void SinglePlay();
	
	void ResetCamera();
	
	void Update();
private:
	friend class ViewModel;
	
	Properties* m_properties;
	TestProperties* m_testProperties;
	b3Camera m_camera;
	b3DebugDraw m_debugDraw;
	GLDebugDraw m_glDebugDraw;
	Test* m_test;
	bool m_setTest;
	bool m_singlePlay;
	bool m_pause;
};

inline void Model::SinglePlay()
{
	m_pause = true;
	m_singlePlay = true;
}

inline void Model::ResetCamera()
{
	m_camera.SetAzimuthalAngle(0.15f * B3_PI);
	m_camera.SetPolarAngle(0.35f * B3_PI);	
	m_camera.SetRadius(50.0f);
	m_camera.SetCenter(b3Vec3_zero);
}

#endif
