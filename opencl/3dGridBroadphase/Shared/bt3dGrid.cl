
MSTRINGIFY(

int getPosHash(int4 gridPos, __global float4* pParams)
{
	int4 gridDim = *((__global int4*)(pParams + 1));
	gridPos.x &= gridDim.x - 1;
	gridPos.y &= gridDim.y - 1;
	gridPos.z &= gridDim.z - 1;
	int hash = gridPos.z * gridDim.y * gridDim.x + gridPos.y * gridDim.x + gridPos.x;
	return hash;
}

int4 getGridPos(float4 worldPos, __global float4* pParams)
{
    int4 gridPos;
	int4 gridDim = *((__global int4*)(pParams + 1));
    gridPos.x = (int)floor(worldPos.x * pParams[0].x) & (gridDim.x - 1);
    gridPos.y = (int)floor(worldPos.y * pParams[0].y) & (gridDim.y - 1);
    gridPos.z = (int)floor(worldPos.z * pParams[0].z) & (gridDim.z - 1);
    return gridPos;
}

// calculate grid hash value for each body using its AABB
__kernel void kCalcHashParticle(int numObjects, __global float4* pos, __global int2* pHash, __global float4* pParams GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numObjects)
	{
		return;
	}
    // get address in grid
    int4 gridPos = getGridPos(pos[index], pParams);
    int gridHash = getPosHash(gridPos, pParams);
    // store grid hash and body index
    int2 hashVal;
    hashVal.x = gridHash;
    hashVal.y = index;
    pHash[index] = hashVal;
}

__kernel void kClearCellStart(	int numCells,
								__global int* pCellStart GUID_ARG)
{
    int index = get_global_id(0);
    if(index >= numCells)
	{
		return;
	}
	pCellStart[index] = -1;
}

__kernel void kFindCellStart(int numObjects, __global int2* pHash, __global int* cellStart GUID_ARG)
{
	__local int sharedHash[513];
    int index = get_global_id(0);
	int2 sortedData;
    if(index < numObjects)
	{
		sortedData = pHash[index];
		// Load hash data into shared memory so that we can look
		// at neighboring body's hash value without loading
		// two hash values per thread
		sharedHash[get_local_id(0) + 1] = sortedData.x;
		if((index > 0) && (get_local_id(0) == 0))
		{
			// first thread in block must load neighbor body hash
			sharedHash[0] = pHash[index-1].x;
		}
	}
    barrier(CLK_LOCAL_MEM_FENCE);
    if(index < numObjects)
	{
		if((index == 0) || (sortedData.x != sharedHash[get_local_id(0)]))
		{
			cellStart[sortedData.x] = index;
		}
	}
}

__inline float SQ(float x)
{
    return x*x;
}

int testAABBSphereOverlap(float4 min0, float4 max0, float4 center, float r)
{
    return (SQ(max(min0.x-center.x,0.0f)+max(max0.x-center.x,0.0f))+
    SQ(max(min0.y-center.y,0.0f)+max(max0.y-center.y,0.0f))+
    SQ(max(min0.z-center.z,0.0f)+max(max0.z-center.z,0.0f)))<=SQ(r);
}
ATTRIBUTE_ALIGNED16(struct) btParticle;
void computeInteraction(btParticle* a, btParticle* b);

void forNeighborParticles(int numObjects,
						int4    gridPos,
						int    index,
						__global int2*  pHash,
						__global int*   pCellStart,
						__global btParticle* particles,
						__global float4* pParams)
{
	int4 pGridDim = *((__global int4*)(pParams + 1));
	int maxBodiesPerCell = pGridDim.w;
    int gridHash = getPosHash(gridPos, pParams);
    // get start of bucket for this cell
    int bucketStart = pCellStart[gridHash];
    if (bucketStart == -1)
	{
        return;   // cell empty
	}
	// iterate over bodies in this cell
    int2 sortedData = pHash[index];
	int unsorted_indx = sortedData.y;
	int bucketEnd = bucketStart + maxBodiesPerCell;
	bucketEnd = (bucketEnd > numObjects) ? numObjects : bucketEnd;
	for(int index2 = bucketStart; index2 < bucketEnd; index2++)
	{
        int2 cellData = pHash[index2];
        if (cellData.x != gridHash)
        {
			break;   // no longer in same bucket
		}
		int unsorted_indx2 = cellData.y;
        if (unsorted_indx2 < unsorted_indx) // check not colliding with self
        {
            float d = pCenter[unsorted_indx2]-pCenter[unsorted_indx];
			if(SQ(d)<)
			{
                computeIteraction()
			}
		}
	}
	int2 newStartCurr;
	newStartCurr.x = start;
	newStartCurr.y = curr;
	pPairBuffStartCurr[handleIndex] = newStartCurr;
    return;
}

__kernel void kProcessNeighbors(	int numObjects,
										__global float4* pos,
										__global int2* pHash,
										__global int* pCellStart,
										__global btParticle* particles,
										__global float4* pParams GUID_ARG)

{
    int index = get_global_id(0);
    if(index >= numObjects)
	{
		return;
	}
    int2 sortedData = pHash[index];
	int unsorted_indx = sortedData.y;

    btParticle* a = particles[unsorted_indx];
    // get address in grid
    int4 gridPosA = getGridPos(a->position, pParams);
    int4 gridPosB;
    // examine only neighbouring cells
    preprocess(a);
    for(int z=-1; z<=1; z++)
    {
		gridPosB.z = gridPosA.z + z;
        for(int y=-1; y<=1; y++)
        {
			gridPosB.y = gridPosA.y + y;
            for(int x=-1; x<=1; x++)
            {
				gridPosB.x = gridPosA.x + x;
                forNeighbors(numObjects, gridPosB, index, pHash, pCellStart,particles,a, pParams);
            }
        }
    }
}

);
