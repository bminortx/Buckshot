#ifndef BULLET_HEIGHTMAP_H
#define BULLET_HEIGHTMAP_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h>
#include <bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.h>
#include <bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>

//Constructs a Bullet btHeightfieldTerrainShape.

class bullet_heightmap : public bullet_shape {

public:
  //constructor
  bullet_heightmap(int row_count, int col_count, double grad, double min_ht,
                   double max_ht, double* X, double* Y, double* Z,
                   double* normal){
    if(max_ht<=1){
      //Just make a flat plain
      bulletShape = new btStaticPlaneShape(
            btVector3(normal[0], normal[1], normal[2]), 0);
      bulletMotionState = new btDefaultMotionState(btTransform::getIdentity());
      btRigidBody::btRigidBodyConstructionInfo cInfo(0, bulletMotionState,
                                                     bulletShape,
                                                     btVector3(0, 0, 0));
      bulletBody = new btRigidBody(cInfo);
    }
    else{
      //////////////
      //Algorithm for populating BVHTriangleMeshShape taken from VehicleDemo.cpp
      int vertStride = sizeof(btVector3);
      int indexStride = 3*sizeof(int);
      const int NUM_VERTS_X = row_count;
      const int NUM_VERTS_Y = col_count;
      const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
      const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);
      btVector3* m_vertices = new btVector3[totalVerts];
      int* gIndices = new int[totalTriangles*3];
      for (int i=0;i<NUM_VERTS_X;i++){
        for (int j=0;j<NUM_VERTS_Y;j++){
          float width = X[i+j*NUM_VERTS_X];
          float length = Y[i+j*NUM_VERTS_X];
          float height = Z[i+j*NUM_VERTS_X];
          m_vertices[i+j*NUM_VERTS_X].setValue(width, length, height);
        }
      }

      int index=0;
      for (int i=0;i<NUM_VERTS_X-1;i++)
      {
        for (int j=0;j<NUM_VERTS_Y-1;j++)
        {
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
      /////////////
      bulletShape = new btBvhTriangleMeshShape(m_indexVertexArrays, true);
      bulletMotionState = new btDefaultMotionState(btTransform::getIdentity());
      btRigidBody::btRigidBodyConstructionInfo cInfo(0, bulletMotionState,
                                                     bulletShape,
                                                   btVector3(0, 0, 0));
      bulletBody = new btRigidBody(cInfo);
    }
  }

  void set_vertex_data() {}

  //////////////////////////

  ///getters
  btCollisionShape* getBulletShapePtr(){
    return bulletShape;
  }

  btRigidBody* getBulletBodyPtr(){
    return bulletBody;
  }

  btDefaultMotionState* getBulletMotionStatePtr(){
    return bulletMotionState;
  }

private:
  btCollisionShape* bulletShape;
  btRigidBody* bulletBody;
  btDefaultMotionState* bulletMotionState;

};




#endif // BULLET_HEIGHTMAP_H
