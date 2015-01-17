// Copyright (c) bminortx

#ifndef SIMBA_MODELGRAPH_BULLET_SHAPES_BULLET_HEIGHTMAP_H_
#define SIMBA_MODELGRAPH_BULLET_SHAPES_BULLET_HEIGHTMAP_H_

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h>
#include <bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.h>
#include <bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>

#include <memory>
#include <vector>

#include "ModelGraph/Bullet_shapes/bullet_shape.h"


// Constructs a Bullet btHeightfieldTerrainShape.

class bullet_heightmap : public bullet_shape {
 public:
  // constructor
  explicit bullet_heightmap(const std::shared_ptr<ModelNode>& mnMap) {
    std::shared_ptr<HeightmapShape> pMap =
        std::dynamic_pointer_cast<HeightmapShape>(mnMap);
    int row_count = pMap->row_count_;
    int col_count = pMap->col_count_;
    std::vector<double> X = pMap->x_data_;
    std::vector<double> Y = pMap->y_data_;
    std::vector<double> Z = pMap->z_data_;

    /////////////
    // Algorithm for populating BVHTriangleMeshShape taken from VehicleDemo.cpp

    int vertStride = sizeof(btVector3);
    int indexStride = 3*sizeof(row_count);
    const int NUM_VERTS_X = row_count;
    const int NUM_VERTS_Y = col_count;
    const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
    const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);
    btVector3* m_vertices = new btVector3[totalVerts];
    int* gIndices = new int[totalTriangles*3];
    for (int i = 0; i < NUM_VERTS_X; i++) {
      for (int j = 0; j < NUM_VERTS_Y; j++) {
        double width = X[i+j*NUM_VERTS_X];
        double length = Y[i+j*NUM_VERTS_X];
        double height = Z[i+j*NUM_VERTS_X];
        m_vertices[i+j*NUM_VERTS_X].setValue(width, length, height);
      }
    }

    int index = 0;
    for (int i = 0; i < NUM_VERTS_X-1; i++) {
      for (int j = 0; j < NUM_VERTS_Y-1; j++) {
        gIndices[index++] = j*NUM_VERTS_X+i;
        gIndices[index++] = j*NUM_VERTS_X+i+1;
        gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

        gIndices[index++] = j*NUM_VERTS_X+i;
        gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
        gIndices[index++] = (j+1)*NUM_VERTS_X+i;
      }
    }

    btTriangleIndexVertexArray* m_indexVertexArrays =
        new btTriangleIndexVertexArray(totalTriangles,
                                       gIndices,
                                       indexStride,
                                       totalVerts,
                                       (btScalar*) &m_vertices[0].x(),
                                       vertStride);
    ////////////
    bulletShape = std::make_shared<btBvhTriangleMeshShape>(
        m_indexVertexArrays, true);
    bulletMotionState = std::make_shared<NodeMotionState>(mnMap);
    btRigidBody::btRigidBodyConstructionInfo cInfo(0, bulletMotionState.get(),
                                                   bulletShape.get(),
                                                   btVector3(0, 0, 0));
    bulletBody = std::make_shared<btRigidBody>(cInfo);
  }
};

#endif  // SIMBA_MODELGRAPH_BULLET_SHAPES_BULLET_HEIGHTMAP_H_
