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

#include "btParticleSystem.h"

//#include "CustomConvexShape.h"
//#include "CustomConvexPairCollision.h"
#include "LinearMath/btQuickprof.h"
#include "../broadphase_benchmark/btLauncherCL.h"
#include "../broadphase_benchmark/btRadixSort32CL.h"
#include "btConvexUtility.h"
#include "../gpu_rigidbody_pipeline2/ConvexHullContact.h"

ATTRIBUTE_ALIGNED16(struct) btParticle
{
    BT_DECLARE_ALIGNED_ALLOCATOR();

    float4 position;
    float4 color;
    float3 velocity;
    float mass;
    float lifetime;
};
struct	CustomDispatchData
{
    //should also allow for DirectX?
    unsigned int m_numParticles;
    GLuint m_particlesVBO;
    btOpenCLGLInteropBuffer<btParticle>* m_particlesGL;
    btOpenCLArray<btParticle> m_particles;
};



	btParticleSystem::btParticleSystem(cl_context ctx, cl_device_id dev, cl_command_queue q, ParticleSystemParams param,  bool useCLGLInterop)
	:m_internalData(0),
        m_context(ctx),
        m_device(dev),
        m_queue(q)
	{
        m_internalData = new CustomDispatchData();
        memset(m_internalData,0,sizeof(CustomDispatchData));
        if(useCLGLInterop)
        {
            glGenBuffers(1, &m_particlesVBO);
            glBindBuffer(GL_ARRAY_BUFFER, m_particlesVBO);
            size_t size = (sizeof(btParticle)/sizeof(float))*param.maxParticles;
            GLfloat* tmp = new GLfloat[size];
            memset(tmp,0,size);
            glBufferData(GL_ARRAY_BUFFER, size, tmp, GL_DYNAMIC_DRAW);
            m_internalData->m_particlesGL = new btOpenCLGLInteropBuffer<btParticle>(ctx,q,m->m_particlesVBO);
            glBindBuffer(GL_ARRAY_BUFFER,0);
        }
        else
        {
            m_internalData->m_particles = new btOpenCLArray<btParticle>(ctx,q,param.maxParticles,false);
        }
        m_internalData->m_numParticles=0;
	}
	virtual ~btParticleSystem(void)
	{
	    if(m_internalData)
        {
            delete m_internalData->m_particles;
            delete m_internalData->m_particlesGL;
            glDeleteBuffers(1,&m_particlesVBO);
            delete m_internalData;
        }

	}
	void	writeAllParticlesToGpu();
	virtual void computeCollisions();
	cl_mem	getParticlesGpu()
    {
        if(useCLGLInterop)
            return (cl_mem)m_internalData->m_particlesGL->getCLBuffer();
        else
            return (cl_mem)m_internalData->m_particles->getBufferCL();
    }
    GLuint	getParticlesGL()
    {
        return m_particlesVBO;
    }
    void btPushParticles(btAlignedObjectArray<btParticle>& pArray, bool blocking)
    {
        cl_mem buf;
        if(useCLGLInterop)
        {
            buf = (cl_mem)m_internalData->m_particlesGL->getCLBuffer();
            ciErrNum = clEnqueueAcquireGLObjects(m_queue, 1, &buf, 0, 0, NULL);
            clFinish(m_queue);
        }
        else
        {
            buf = (cl_mem)m_internalData->m_particles->getBufferCL();
        }
        ciErrNum = clEnqueueWriteBuffer (	m_queue,
                buf,
 				blocking,
 				0,
 				pArray->size()*sizeof(btParticle),
 				&pArray[0],0,0,0
			);
        clFinish(m_queue);
        m_internalData->m_numParticles+=pArray.size();
    }

void btParticleSystem::computeParticles()
{
	BT_PROFILE("computeParticles");
	bool bGPU = (m_internalData != 0);
	int maxParticleIndex = m_internalData->m_numParticles;

	if (!maxParticleIndex)
		return;
	//ChNarrowphaseBase::Config cfgNP;
	//cfgNP.m_collisionMargin = 0.01f;
	//int nContactOut = 0;
	//printf("convexPairsOut.m_size = %d\n",m_internalData->m_convexPairsOutGPU->m_size);


	//btOpenCLArray<int2> broadphasePairsGPU(m_context,m_queue);
	//broadphasePairsGPU.setFromOpenCLBuffer(broadphasePairs,numBroadphasePairs);
	{
	    BT_PROFILE("btParticleSystem::forceComputation");
        {

            //btOpenCLArray<RigidBodyBase::Body>* bodyNative = btOpenCLArrayUtils::map<adl::TYPE_CL, true>( data->m_device, bodyBuf );
            //btOpenCLArray<Contact4>* contactNative = btOpenCLArrayUtils::map<adl::TYPE_CL, true>( data->m_device, contactsIn );

            const int sortAlignment = 512; // todo. get this out of sort

            int sortSize = NEXTMULTIPLEOF( nParticles, sortAlignment );

            btOpenCLArray<u32>* countsNative = m_internalData->m_solverGPU->m_numConstraints;
            btOpenCLArray<u32>* offsetsNative = m_internalData->m_solverGPU->m_offsets;

                {	//	2. set cell idx
                    BT_PROFILE("GPU set cell idx");
                    struct CB
                    {
                        int m_nContacts;
                        int m_staticIdx;
                        float m_scale;
                        int m_nSplit;
                    };

                    ADLASSERT( sortSize%64 == 0 );
                    CB cdata;
                    cdata.m_nContacts = nContacts;
                    cdata.m_staticIdx = csCfg.m_staticIdx;
                    cdata.m_scale = 1.f/(BT_SOLVER_N_OBJ_PER_SPLIT*csCfg.m_averageExtent);
                    cdata.m_nSplit = BT_SOLVER_N_SPLIT;

                    m_internalData->m_solverGPU->m_sortDataBuffer->resize(nContacts);


                    btBufferInfoCL bInfo[] = { btBufferInfoCL( contactNative->getBufferCL() ), btBufferInfoCL( bodyBuf->getBufferCL()), btBufferInfoCL( m_internalData->m_solverGPU->m_sortDataBuffer->getBufferCL()) };
                    btLauncherCL launcher(m_queue, m_internalData->m_solverGPU->m_setSortDataKernel );
                    launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
                    launcher.setConst( cdata );
                    launcher.launch1D( sortSize, 64 );
                }
                bool gpuRadixSort=true;
                if (gpuRadixSort)
                {	//	3. sort by cell idx
                    BT_PROFILE("gpuRadixSort");
                    int n = BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT;
                    int sortBit = 32;
                    //if( n <= 0xffff ) sortBit = 16;
                    //if( n <= 0xff ) sortBit = 8;
                    //adl::RadixSort<adl::TYPE_CL>::execute( data->m_sort, *data->m_sortDataBuffer, sortSize );
                    //adl::RadixSort32<adl::TYPE_CL>::execute( data->m_sort32, *data->m_sortDataBuffer, sortSize );
                    btOpenCLArray<btSortData>& keyValuesInOut = *(m_internalData->m_solverGPU->m_sortDataBuffer);
                    this->m_internalData->m_solverGPU->m_sort32->execute(keyValuesInOut);

                    /*btAlignedObjectArray<btSortData> hostValues;
                    keyValuesInOut.copyToHost(hostValues);
                    printf("hostValues.size=%d\n",hostValues.size());
                    */

                }




                {
                    //	4. find entries
                    BT_PROFILE("gpuBoundSearch");

                    m_internalData->m_solverGPU->m_search->execute(*m_internalData->m_solverGPU->m_sortDataBuffer,nContacts,*countsNative,
                        BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT,btBoundSearchCL::COUNT);


                    //adl::BoundSearch<adl::TYPE_CL>::execute( data->m_search, *data->m_sortDataBuffer, nContacts, *countsNative,
                    //	BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT, adl::BoundSearchBase::COUNT );

                    //unsigned int sum;
                    m_internalData->m_solverGPU->m_scan->execute(*countsNative,*offsetsNative, BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT);//,&sum );
                    //printf("sum = %d\n",sum);
                }


                {	//	5. sort constraints by cellIdx
                    {
                        BT_PROFILE("gpu m_reorderContactKernel");

                        btInt4 cdata;
                        cdata.x = nContacts;

                        btBufferInfoCL bInfo[] = { btBufferInfoCL( contactNative->getBufferCL() ), btBufferInfoCL( m_internalData->m_solverGPU->m_contactBuffer->getBufferCL())
                            , btBufferInfoCL( m_internalData->m_solverGPU->m_sortDataBuffer->getBufferCL()) };
                        btLauncherCL launcher(m_queue,m_internalData->m_solverGPU->m_reorderContactKernel);
                        launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
                        launcher.setConst( cdata );
                        launcher.launch1D( nContacts, 64 );
                    }
                }

            }

        }

		if (1)
		{
			BT_PROFILE("GPU solveContactConstraint");
			m_internalData->m_solverGPU->m_nIterations = 4;//10
			m_internalData->m_solverGPU->solveContactConstraint(m_internalData->m_bodyBufferGPU,
				m_internalData->m_inertiaBufferGPU,
				m_internalData->m_contactCGPU,
				0,
				nContactOut );

			clFinish(m_queue);
		}
	}
}
