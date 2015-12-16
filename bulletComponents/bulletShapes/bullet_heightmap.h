#pragma once

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include <iostream>

//Constructs a Bullet btHeightfieldTerrainShape.

class bullet_heightmap : public bullet_shape {

public:
  //constructor
  bullet_heightmap(int row_count, int col_count, double grad, double min_ht,
                   double max_ht, double* X, double* Y, double* Z,
                   double* normal){
    _max_ht = max_ht;
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
      NUM_VERTS_X = row_count;
      NUM_VERTS_Y = col_count;
      totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
      totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);
      btVector3* m_vertices = new btVector3[totalVerts];
      vertices = new float[totalVerts * 3];
      gIndices = new int[totalTriangles * 3];
      for (int i=0;i<NUM_VERTS_X;i++){
        for (int j=0;j<NUM_VERTS_Y;j++){
          float width = X[i+j*NUM_VERTS_X];
          float length = Y[i+j*NUM_VERTS_X];
          float height = Z[i+j*NUM_VERTS_X];
          m_vertices[i+j*NUM_VERTS_X].setValue(width, length, height);
          // TODO:  FIGURE THIS OUT 
          vertices[3*i + j*NUM_VERTS_X + 0] = width;
          vertices[3*i + j*NUM_VERTS_X + 1] = length;
          vertices[3*i + j*NUM_VERTS_X + 2] = height;
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
      
      bulletShape = new btBvhTriangleMeshShape(m_indexVertexArrays, true);
      bulletMotionState = new btDefaultMotionState(btTransform::getIdentity());
      btRigidBody::btRigidBodyConstructionInfo cInfo(0, bulletMotionState,
                                                     bulletShape,
                                                     btVector3(0, 0, 0));
      bulletBody = new btRigidBody(cInfo);
    }
  }

  void getDrawData() {
    if (_max_ht <= 10) {
      glLineWidth(2); 
      glColor3f(1.0, 0.0, 0.0);
      glBegin(GL_LINES);
      for (int i = -10; i < 10; i++) {
        for (int j = -10; j < 10; j++) {
          glVertex3f(i, j, 0.0);
          glVertex3f(j, i, 0.0);
        }
      }
      glEnd();
    } else {
      glEnableClientState( GL_VERTEX_ARRAY );
      glVertexPointer( 3, GL_FLOAT, 0, vertices );
      glDrawElements( GL_TRIANGLE_STRIP, totalTriangles * 3, GL_UNSIGNED_INT, gIndices );
      glDisableClientState( GL_VERTEX_ARRAY );
    }
  }

  int _max_ht;
  float* vertices;
  int* gIndices;
  int totalTriangles;
  int totalVerts;
  int NUM_VERTS_X;
  int NUM_VERTS_Y;

};
