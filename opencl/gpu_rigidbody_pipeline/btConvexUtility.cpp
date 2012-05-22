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


#include "btConvexUtility.h"
#include "LinearMath/btConvexHullComputer.h"
#include "LinearMath/btGrahamScan2dConvexHull.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btHashMap.h"

#include "../gpu_rigidbody_pipeline2/ConvexPolyhedronCL.h"

btConvexUtility::~btConvexUtility()
{
}

bool	btConvexUtility::initializePolyhedralFeatures(const btAlignedObjectArray<btVector3>& orgVertices, bool mergeCoplanarTriangles)
{
	
	

	btConvexHullComputer conv;
	conv.compute(&orgVertices[0].getX(), sizeof(btVector3),orgVertices.size(),0.f,0.f);

	btAlignedObjectArray<btVector3> faceNormals;
	int numFaces = conv.faces.size();
	faceNormals.resize(numFaces);
	btConvexHullComputer* convexUtil = &conv;

	
	btAlignedObjectArray<btFace>	tmpFaces;
	tmpFaces.resize(numFaces);

	int numVertices = convexUtil->vertices.size();
	m_vertices.resize(numVertices);
	for (int p=0;p<numVertices;p++)
	{
		m_vertices[p] = convexUtil->vertices[p];
	}


	for (int i=0;i<numFaces;i++)
	{
		int face = convexUtil->faces[i];
		//printf("face=%d\n",face);
		const btConvexHullComputer::Edge*  firstEdge = &convexUtil->edges[face];
		const btConvexHullComputer::Edge*  edge = firstEdge;

		btVector3 edges[3];
		int numEdges = 0;
		//compute face normals

		btScalar maxCross2 = 0.f;
		int chosenEdge = -1;

		do
		{
			
			int src = edge->getSourceVertex();
			tmpFaces[i].m_indices.push_back(src);
			int targ = edge->getTargetVertex();
			btVector3 wa = convexUtil->vertices[src];

			btVector3 wb = convexUtil->vertices[targ];
			btVector3 newEdge = wb-wa;
			newEdge.normalize();
			if (numEdges<2)
				edges[numEdges++] = newEdge;

			edge = edge->getNextEdgeOfFace();
		} while (edge!=firstEdge);

		btScalar planeEq = 1e30f;

		
		if (numEdges==2)
		{
			faceNormals[i] = edges[0].cross(edges[1]);
			faceNormals[i].normalize();
			tmpFaces[i].m_plane[0] = faceNormals[i].getX();
			tmpFaces[i].m_plane[1] = faceNormals[i].getY();
			tmpFaces[i].m_plane[2] = faceNormals[i].getZ();
			tmpFaces[i].m_plane[3] = planeEq;

		}
		else
		{
			btAssert(0);//degenerate?
			faceNormals[i].setZero();
		}

		for (int v=0;v<tmpFaces[i].m_indices.size();v++)
		{
			btScalar eq = m_vertices[tmpFaces[i].m_indices[v]].dot(faceNormals[i]);
			if (planeEq>eq)
			{
				planeEq=eq;
			}
		}
		tmpFaces[i].m_plane[3] = -planeEq;
	}

	//merge coplanar faces

	btScalar faceWeldThreshold= 0.999f;
	btAlignedObjectArray<int> todoFaces;
	for (int i=0;i<tmpFaces.size();i++)
		todoFaces.push_back(i);

	while (todoFaces.size())
	{
		btAlignedObjectArray<int> coplanarFaceGroup;
		int refFace = todoFaces[todoFaces.size()-1];

		coplanarFaceGroup.push_back(refFace);
		btFace& faceA = tmpFaces[refFace];
		todoFaces.pop_back();

		btVector3 faceNormalA(faceA.m_plane[0],faceA.m_plane[1],faceA.m_plane[2]);
		for (int j=todoFaces.size()-1;j>=0;j--)
		{
			int i = todoFaces[j];
			btFace& faceB = tmpFaces[i];
			btVector3 faceNormalB(faceB.m_plane[0],faceB.m_plane[1],faceB.m_plane[2]);
			if (faceNormalA.dot(faceNormalB)>faceWeldThreshold)
			{
				coplanarFaceGroup.push_back(i);
				todoFaces.remove(i);
			}
		}


		bool did_merge = false;
		if (mergeCoplanarTriangles && coplanarFaceGroup.size()>1)
		{
			//do the merge: use Graham Scan 2d convex hull

			btAlignedObjectArray<GrahamVector2> orgpoints;

			for (int i=0;i<coplanarFaceGroup.size();i++)
			{

				btFace& face = tmpFaces[coplanarFaceGroup[i]];
				btVector3 faceNormal(face.m_plane[0],face.m_plane[1],face.m_plane[2]);
				btVector3 xyPlaneNormal(0,0,1);

				btQuaternion rotationArc = shortestArcQuat(faceNormal,xyPlaneNormal);
				
				for (int f=0;f<face.m_indices.size();f++)
				{
					int orgIndex = face.m_indices[f];
					btVector3 pt = m_vertices[orgIndex];
					btVector3 rotatedPt =  quatRotate(rotationArc,pt);
					rotatedPt.setZ(0);
					bool found = false;

					for (int i=0;i<orgpoints.size();i++)
					{
						//if ((orgpoints[i].m_orgIndex == orgIndex) || ((rotatedPt-orgpoints[i]).length2()<0.0001))
						if (orgpoints[i].m_orgIndex == orgIndex)
						{
							found=true;
							break;
						}
					}
					if (!found)
						orgpoints.push_back(GrahamVector2(rotatedPt,orgIndex));
				}
			}

			btFace combinedFace;
			for (int i=0;i<4;i++)
				combinedFace.m_plane[i] = tmpFaces[coplanarFaceGroup[0]].m_plane[i];

			btAlignedObjectArray<GrahamVector2> hull;
			GrahamScanConvexHull2D(orgpoints,hull);

			for (int i=0;i<hull.size();i++)
			{
				combinedFace.m_indices.push_back(hull[i].m_orgIndex);
				for(int k = 0; k < orgpoints.size(); k++) {
					if(orgpoints[k].m_orgIndex == hull[i].m_orgIndex) {
						orgpoints[k].m_orgIndex = -1; // invalidate...
						break;
			}
				}
			}
			// are there rejected vertices?
			bool reject_merge = false;
			for(int i = 0; i < orgpoints.size(); i++) {
				if(orgpoints[i].m_orgIndex == -1)
					continue; // this is in the hull...
				// this vertex is rejected -- is anybody else using this vertex?
				for(int j = 0; j < tmpFaces.size(); j++) {
					btFace& face = tmpFaces[j];
					// is this a face of the current coplanar group?
					bool is_in_current_group = false;
					for(int k = 0; k < coplanarFaceGroup.size(); k++) {
						if(coplanarFaceGroup[k] == j) {
							is_in_current_group = true;
							break;
						}
					}
					if(is_in_current_group) // ignore this face...
						continue;
					// does this face use this rejected vertex?
					for(int v = 0; v < face.m_indices.size(); v++) {
						if(face.m_indices[v] == orgpoints[i].m_orgIndex) {
							// this rejected vertex is used in another face -- reject merge
							reject_merge = true;
							break;
						}
					}
					if(reject_merge)
						break;
				}
				if(reject_merge)
					break;
			}
			if(!reject_merge) {
				// do this merge!
				did_merge = true;
			m_faces.push_back(combinedFace);
			}
		}
		if(!did_merge)
		{
			for (int i=0;i<coplanarFaceGroup.size();i++)
			{
				m_faces.push_back(tmpFaces[coplanarFaceGroup[i]]);
			}
		}

	}

	initialize();

	return true;
}






inline bool IsAlmostZero(const btVector3& v)
{
	if(fabsf(v.x())>1e-6 || fabsf(v.y())>1e-6 || fabsf(v.z())>1e-6)	return false;
	return true;
}

struct btInternalVertexPair
{
	btInternalVertexPair(short int v0,short int v1)
		:m_v0(v0),
		m_v1(v1)
	{
		if (m_v1>m_v0)
			btSwap(m_v0,m_v1);
	}
	short int m_v0;
	short int m_v1;
	int getHash() const
	{
		return m_v0+(m_v1<<16);
	}
	bool equals(const btInternalVertexPair& other) const
	{
		return m_v0==other.m_v0 && m_v1==other.m_v1;
	}
};

struct btInternalEdge
{
	btInternalEdge()
		:m_face0(-1),
		m_face1(-1)
	{
	}
	short int m_face0;
	short int m_face1;
};

//

#ifdef TEST_INTERNAL_OBJECTS
bool btConvexUtility::testContainment() const
{
	for(int p=0;p<8;p++)
	{
		btVector3 LocalPt;
		if(p==0)		LocalPt = m_localCenter + btVector3(m_extents[0], m_extents[1], m_extents[2]);
		else if(p==1)	LocalPt = m_localCenter + btVector3(m_extents[0], m_extents[1], -m_extents[2]);
		else if(p==2)	LocalPt = m_localCenter + btVector3(m_extents[0], -m_extents[1], m_extents[2]);
		else if(p==3)	LocalPt = m_localCenter + btVector3(m_extents[0], -m_extents[1], -m_extents[2]);
		else if(p==4)	LocalPt = m_localCenter + btVector3(-m_extents[0], m_extents[1], m_extents[2]);
		else if(p==5)	LocalPt = m_localCenter + btVector3(-m_extents[0], m_extents[1], -m_extents[2]);
		else if(p==6)	LocalPt = m_localCenter + btVector3(-m_extents[0], -m_extents[1], m_extents[2]);
		else if(p==7)	LocalPt = m_localCenter + btVector3(-m_extents[0], -m_extents[1], -m_extents[2]);

		for(int i=0;i<m_faces.size();i++)
		{
			const btVector3 Normal(m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2]);
			const btScalar d = LocalPt.dot(Normal) + m_faces[i].m_plane[3];
			if(d>0.0f)
				return false;
		}
	}
	return true;
}
#endif

void	btConvexUtility::initialize()
{

	btHashMap<btInternalVertexPair,btInternalEdge> edges;

	btScalar TotalArea = 0.0f;
	
	m_localCenter.setValue(0, 0, 0);
	for(int i=0;i<m_faces.size();i++)
	{
		int numVertices = m_faces[i].m_indices.size();
		int NbTris = numVertices;
		for(int j=0;j<NbTris;j++)
		{
			int k = (j+1)%numVertices;
			btInternalVertexPair vp(m_faces[i].m_indices[j],m_faces[i].m_indices[k]);
			btInternalEdge* edptr = edges.find(vp);
			btVector3 edge = m_vertices[vp.m_v1]-m_vertices[vp.m_v0];
			edge.normalize();

			bool found = false;

			for (int p=0;p<m_uniqueEdges.size();p++)
			{
				
				if (IsAlmostZero(m_uniqueEdges[p]-edge) || 
					IsAlmostZero(m_uniqueEdges[p]+edge))
				{
					found = true;
					break;
				}
			}

			if (!found)
			{
				m_uniqueEdges.push_back(edge);
			}

			if (edptr)
			{
				btAssert(edptr->m_face0>=0);
				btAssert(edptr->m_face1<0);
				edptr->m_face1 = i;
			} else
			{
				btInternalEdge ed;
				ed.m_face0 = i;
				edges.insert(vp,ed);
			}
		}
	}

#ifdef USE_CONNECTED_FACES
	for(int i=0;i<m_faces.size();i++)
	{
		int numVertices = m_faces[i].m_indices.size();
		m_faces[i].m_connectedFaces.resize(numVertices);

		for(int j=0;j<numVertices;j++)
		{
			int k = (j+1)%numVertices;
			btInternalVertexPair vp(m_faces[i].m_indices[j],m_faces[i].m_indices[k]);
			btInternalEdge* edptr = edges.find(vp);
			btAssert(edptr);
			btAssert(edptr->m_face0>=0);
			btAssert(edptr->m_face1>=0);

			int connectedFace = (edptr->m_face0==i)?edptr->m_face1:edptr->m_face0;
			m_faces[i].m_connectedFaces[j] = connectedFace;
		}
	}
#endif//USE_CONNECTED_FACES

	for(int i=0;i<m_faces.size();i++)
	{
		int numVertices = m_faces[i].m_indices.size();
		int NbTris = numVertices-2;
		
		const btVector3& p0 = m_vertices[m_faces[i].m_indices[0]];
		for(int j=1;j<=NbTris;j++)
		{
			int k = (j+1)%numVertices;
			const btVector3& p1 = m_vertices[m_faces[i].m_indices[j]];
			const btVector3& p2 = m_vertices[m_faces[i].m_indices[k]];
			btScalar Area = ((p0 - p1).cross(p0 - p2)).length() * 0.5f;
			btVector3 Center = (p0+p1+p2)/3.0f;
			m_localCenter += Area * Center;
			TotalArea += Area;
		}
	}
	m_localCenter /= TotalArea;




#ifdef TEST_INTERNAL_OBJECTS
	if(1)
	{
		m_radius = FLT_MAX;
		for(int i=0;i<m_faces.size();i++)
		{
			const btVector3 Normal(m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2]);
			const btScalar dist = btFabs(m_localCenter.dot(Normal) + m_faces[i].m_plane[3]);
			if(dist<m_radius)
				m_radius = dist;
		}

	
		btScalar MinX = FLT_MAX;
		btScalar MinY = FLT_MAX;
		btScalar MinZ = FLT_MAX;
		btScalar MaxX = -FLT_MAX;
		btScalar MaxY = -FLT_MAX;
		btScalar MaxZ = -FLT_MAX;
		for(int i=0; i<m_vertices.size(); i++)
		{
			const btVector3& pt = m_vertices[i];
			if(pt.x()<MinX)	MinX = pt.x();
			if(pt.x()>MaxX)	MaxX = pt.x();
			if(pt.y()<MinY)	MinY = pt.y();
			if(pt.y()>MaxY)	MaxY = pt.y();
			if(pt.z()<MinZ)	MinZ = pt.z();
			if(pt.z()>MaxZ)	MaxZ = pt.z();
		}
		mC.setValue(MaxX+MinX, MaxY+MinY, MaxZ+MinZ);
		mE.setValue(MaxX-MinX, MaxY-MinY, MaxZ-MinZ);



//		const btScalar r = m_radius / sqrtf(2.0f);
		const btScalar r = m_radius / sqrtf(3.0f);
		const int LargestExtent = mE.maxAxis();
		const btScalar Step = (mE[LargestExtent]*0.5f - r)/1024.0f;
		m_extents[0] = m_extents[1] = m_extents[2] = r;
		m_extents[LargestExtent] = mE[LargestExtent]*0.5f;
		bool FoundBox = false;
		for(int j=0;j<1024;j++)
		{
			if(testContainment())
			{
				FoundBox = true;
				break;
			}

			m_extents[LargestExtent] -= Step;
		}
		if(!FoundBox)
		{
			m_extents[0] = m_extents[1] = m_extents[2] = r;
		}
		else
		{
			// Refine the box
			const btScalar Step = (m_radius - r)/1024.0f;
			const int e0 = (1<<LargestExtent) & 3;
			const int e1 = (1<<e0) & 3;

			for(int j=0;j<1024;j++)
			{
				const btScalar Saved0 = m_extents[e0];
				const btScalar Saved1 = m_extents[e1];
				m_extents[e0] += Step;
				m_extents[e1] += Step;

				if(!testContainment())
				{
					m_extents[e0] = Saved0;
					m_extents[e1] = Saved1;
					break;
				}
			}
		}
	}
#endif
}
