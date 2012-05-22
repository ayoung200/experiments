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
//Originally written by Takahiro Harada


#ifndef __ADL_SOLVER_H
#define __ADL_SOLVER_H

#include "../../../opencl/broadphase_benchmark/btOpenCLArray.h"
#include "AdlConstraint4.h"
typedef btOpenCLArray<Constraint4>* SolverData;


//#include <AdlPhysics/TypeDefinition.h>
#include "AdlRigidBody.h"
#include "AdlContact4.h"

//#include "AdlPhysics/Batching/Batching.h>


#define MYF4 float4
#define MAKE_MYF4 make_float4

//#define MYF4 float4sse
//#define MAKE_MYF4 make_float4sse

#include "AdlConstraint4.h"
#include "../../../opencl/broadphase_benchmark/btPrefixScanCL.h"
#include "../../../opencl/broadphase_benchmark/btRadixSort32CL.h"
#include "../../../opencl/broadphase_benchmark/btBoundSearchCL.h"

#include "../../../opencl/basic_initialize/btOpenCLUtils.h"



class SolverBase
{
	public:
		

		struct ConstraintData
		{
			ConstraintData(): m_b(0.f), m_appliedRambdaDt(0.f) {}

			float4 m_linear; // have to be normalized
			float4 m_angular0;
			float4 m_angular1;
			float m_jacCoeffInv;
			float m_b;
			float m_appliedRambdaDt;

			u32 m_bodyAPtr;
			u32 m_bodyBPtr;

			bool isInvalid() const { return ((u32)m_bodyAPtr+(u32)m_bodyBPtr) == 0; }
			float getFrictionCoeff() const { return m_linear.w; }
			void setFrictionCoeff(float coeff) { m_linear.w = coeff; }
		};

		struct ConstraintCfg
		{
			ConstraintCfg( float dt = 0.f ): m_positionDrift( 0.005f ), m_positionConstraintCoeff( 0.2f ), m_dt(dt), m_staticIdx(-1) {}

			float m_positionDrift;
			float m_positionConstraintCoeff;
			float m_dt;
			bool m_enableParallelSolve;
			float m_averageExtent;
			int m_staticIdx;
		};

		

		enum
		{
			N_SPLIT = 16,
			N_BATCHES = 4,
			N_OBJ_PER_SPLIT = 10,
			N_TASKS_PER_BATCH = N_SPLIT*N_SPLIT,
		};
};

class Solver : public SolverBase
{
	public:

		cl_context m_context;
		cl_device_id m_device;
		cl_command_queue m_queue;
				

		btOpenCLArray<u32>* m_numConstraints;
		btOpenCLArray<u32>* m_offsets;
		
		
		int m_nIterations;
		cl_kernel m_batchingKernel;
		cl_kernel m_batchSolveKernel;
		cl_kernel m_contactToConstraintKernel;
		cl_kernel m_setSortDataKernel;
		cl_kernel m_reorderContactKernel;
		cl_kernel m_copyConstraintKernel;

		class btRadixSort32CL*	m_sort32;
		class btBoundSearchCL*	m_search;
		class btPrefixScanCL*	m_scan;

		btOpenCLArray<btSortData>* m_sortDataBuffer;
		btOpenCLArray<Contact4>* m_contactBuffer;

		enum
		{
			DYNAMIC_CONTACT_ALLOCATION_THRESHOLD = 2000000,
		};

		

		
		Solver(cl_context ctx, cl_device_id device, cl_command_queue queue, int pairCapacity);

		virtual ~Solver();
		
		void reorderConvertToConstraints( const btOpenCLArray<RigidBodyBase::Body>* bodyBuf, 
		const btOpenCLArray<RigidBodyBase::Inertia>* shapeBuf, 
			btOpenCLArray<Contact4>* contactsIn, SolverData contactCOut, void* additionalData, 
			int nContacts, const ConstraintCfg& cfg );

		
		void solveContactConstraint( const btOpenCLArray<RigidBodyBase::Body>* bodyBuf, const btOpenCLArray<RigidBodyBase::Inertia>* inertiaBuf, 
			SolverData constraint, void* additionalData, int n );



		void convertToConstraints( const btOpenCLArray<RigidBodyBase::Body>* bodyBuf, 
			const btOpenCLArray<RigidBodyBase::Inertia>* shapeBuf, 
			btOpenCLArray<Contact4>* contactsIn, SolverData contactCOut, void* additionalData, 
			int nContacts, const ConstraintCfg& cfg );

		void sortContacts( const btOpenCLArray<RigidBodyBase::Body>* bodyBuf, 
			btOpenCLArray<Contact4>* contactsIn, void* additionalData, 
			int nContacts, const ConstraintCfg& cfg );

		void batchContacts( btOpenCLArray<Contact4>* contacts, int nContacts, btOpenCLArray<u32>* n, btOpenCLArray<u32>* offsets, int staticIdx );

};


#undef MYF4
#undef MAKE_MYF4

#endif //__ADL_SOLVER_H
