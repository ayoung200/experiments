/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2009 Sony Computer Entertainment Inc.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT3DGRIDSPHERECL_H
#define BT3DGRIDSPHERECL_H

#ifdef __APPLE__
#ifdef USE_MINICL
	#include <MiniCL/cl.h>
#else
	#include <MiniCL/cl.h>
#endif
//CL_PLATFORM_MINI_CL could be defined in build system
#else
//#include <GL/glew.h>
// standard utility and system includes
#ifdef USE_MINICL
	#include <MiniCL/cl.h>
#else
	#include <CL/cl.h>
#endif
// Extra CL/GL include
//#include <CL/cl_gl.h>
#endif //__APPLE__

#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "btGpu3DGridBroadphaseSharedTypes.h"
#include "btGpu3DGridBroadphase.h"


#define GRID3DOCL_CHECKERROR(a, b) if((a)!=(b)) { printf("3D GRID CL Error : %d\n", (a)); btAssert((a) == (b)); }

enum
{
	GRID3DSPHERECL_KERNEL_CALC_HASH = 0,
	GRID3DSPHERECL_KERNEL_CLEAR_CELL_START,
	GRID3DSPHERECL_KERNEL_FIND_CELL_START,
	GRID3DSPHERECL_KERNEL_TOTAL
};

struct bt3dGridSphereCLKernelInfo
{
	int			m_Id;
	cl_kernel	m_kernel;
	char*		m_name;
	int			m_workgroupSize;
};


///The bt3dGridSphereOCL uses OpenCL-capable GPU to compute hash for Spheres(particles)

class bt3dGridSphereCL
{
protected:
	int						m_hashSize;
	unsigned int	        m_numCells;
	unsigned int            m_maxNumSpheres;
	cl_context				m_cxMainContext;
	cl_device_id			m_cdDevice;
	cl_command_queue		m_cqCommandQue;
	cl_program				m_cpProgram;
	bt3dGridSphereOCLKernelInfo	m_kernels[GRID3DSPHERECL_KERNEL_TOTAL];
	// data buffers
	cl_mem					m_dSphereHash;
	cl_mem					m_dCellStart;
public:
	cl_mem					m_centers;

public:

	bt3dGridSphereCL(const btVector3& cellSize,
							int gridSizeX, int gridSizeY, int gridSizeZ,
							cl_context context = NULL,
							cl_device_id device = NULL,
							cl_command_queue queue = NULL
							);
	virtual ~bt3dGridSphereCL();

protected:
	void initCL(cl_context context, cl_device_id device, cl_command_queue queue);
	void initKernels();
	void allocateBuffers();
	void prefillBuffers();
	void initKernel(int kernelId, char* pName);
	void allocateArray(void** devPtr, unsigned int size);
	void freeArray(void* devPtr);
	void runKernelWithWorkgroupSize(int kernelId, int globalSize);
	void setKernelArg(int kernelId, int argNum, int argSize, void* argPtr);
	void copyArrayToDevice(cl_mem device, const void* host, unsigned int size, int devOffs = 0, int hostOffs = 0);
	void copyArrayFromDevice(void* host, const cl_mem device, unsigned int size, int hostOffs = 0, int devOffs = 0);

// overrides
	virtual void setParameters(bt3DGridSphereParams* hostParams);
	virtual void prepareAABB();
	virtual void calcHash();
	virtual void sortHash();
	virtual void findCellStart();
	virtual void resetPool(btDispatcher* dispatcher);
};

#endif //BT3DGRIDBROADPHASEOCL_H
