/*
Copyright (c) 2012 Advanced Micro Devices, Inc.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

#ifndef GPU_PARTICLE_SYSTEM_SOLVER_H
#define GPU_PARTICLE_SYSTEM_SOLVER_H



//#define MAX_CONVEX_BODIES_CL 8*1024
//#define MAX_CONVEX_BODIES_CL 128*1024
//#define MAX_PAIRS_PER_BODY_CL 16
//#define MAX_CONVEX_SHAPES_CL 8192
//#define MAX_BROADPHASE_COLLISION_CL (MAX_CONVEX_BODIES_CL*MAX_PAIRS_PER_BODY_CL)

/*
#define MAX_CONVEX_BODIES_CL 1024
#define MAX_PAIRS_PER_BODY_CL 32
#define MAX_CONVEX_SHAPES_CL 8192
#define MAX_BROADPHASE_COLLISION_CL (MAX_CONVEX_BODIES_CL*MAX_PAIRS_PER_BODY_CL)
*/

struct	CustomDispatchData;
struct  ParticleSystemParams
{
    uint32 maxParticles;
}

#include "../basic_initialize/btOpenCLInclude.h"

/*enum
{
	BT_SOLVER_N_SPLIT = 16,
	BT_SOLVER_N_BATCHES = 4,
	BT_SOLVER_N_OBJ_PER_SPLIT = 10,
	BT_SOLVER_N_TASKS_PER_BATCH = BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT,
};*/

class btParticleSystem
{
protected:
	CustomDispatchData*	m_internalData;
	ParticleSystemParams params;
	//int m_acceleratedCompanionShapeIndex;
	//int m_planeBodyIndex;
	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_queue;
    bool m_interop;

public:
	btParticleSystem(cl_context ctx, cl_device_id dev, cl_command_queue q, ParticleSystemParams params,  bool useCLGLInterop = true);
	virtual ~btParticleSystem(void);
	void	writeAllParticlesToGpu();
	virtual void computeCollisions();
	cl_mem	getParticlesGpu();
	cl_mem	getParticlesInertiasGpu();

};

#endif //GPU_NARROWPHASE_SOLVER_H
