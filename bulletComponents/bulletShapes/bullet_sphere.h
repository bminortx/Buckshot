#ifndef BULLET_SPHERE_H
#define BULLET_SPHERE_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btSphereShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet_shape.h"

class bullet_sphere : public bullet_shape{

public:
  //constructor
  bullet_sphere(double dRadius, double dMass, double dRestitution,
                double *position, double* rotation){
    bulletShape = new btSphereShape(dRadius);

    bulletMotionState = new btDefaultMotionState(btTransform::getIdentity());
    bool isDynamic = ( dMass != 0.f );
    btVector3 localInertia( 0, 0, 0 );
    if( isDynamic ){
        bulletShape->calculateLocalInertia( dMass, localInertia );
    }

    btRigidBody::btRigidBodyConstructionInfo  cInfo(dMass, bulletMotionState,
                                                    bulletShape, localInertia);
    bulletBody = new btRigidBody(cInfo);
    double dContactProcessingThreshold = 0.001;
    bulletBody->setContactProcessingThreshold( dContactProcessingThreshold );
    bulletBody->setRestitution( dRestitution );
    SetPose(position, rotation);
  }

  /// OpenGL stuff

  void createIsocahedron(std::vector<float>& iso_points,
                         std::vector<float>& iso_colors,
                         std::vector<float>& iso_normals) {
    // Construction function modified from http://bit.ly/1Ywq7wE
		// Distance = 1.0 here
		float X = .525731112119133606;
		float Z = .850650808352039932;
		float vdata[] = {
      -X,  0.0, Z,   X,   0.0, Z,   -X,  0.0, -Z,  X,   0.0, -Z, 
      0.0, Z,   X,   0.0, Z,   -X,  0.0, -Z,  X,   0.0, -Z,  -X,     
      Z,   X,   0.0, -Z,  X,   0.0, Z,   -X,  0.0, -Z,  -X,  0.0, 

    };
		int tindices[] = { 
      0,4,1, 0,9,4, 9,5,4, 4,5,8, 4,8,1,    
      8,10,1, 8,3,10, 5,3,8, 5,2,3, 2,7,3,    
      7,10,3, 7,6,10, 7,11,6, 11,0,6, 0,1,6, 
      6,1,10, 9,0,11, 9,11,2, 9,2,5, 7,2,11,
    };
    
		int i;
    int numFaces = 20;
    /// POINTS
		for (i = 0; i < numFaces; i++) {
      iso_points.push_back(vdata[3 * tindices[3 * i + 0] + 0]);
      iso_points.push_back(vdata[3 * tindices[3 * i + 0] + 1]);
      iso_points.push_back(vdata[3 * tindices[3 * i + 0] + 2]);
      iso_points.push_back(1);
      iso_points.push_back(vdata[3 * tindices[3 * i + 1] + 0]);
      iso_points.push_back(vdata[3 * tindices[3 * i + 1] + 1]);
      iso_points.push_back(vdata[3 * tindices[3 * i + 1] + 2]);
      iso_points.push_back(1);
      iso_points.push_back(vdata[3 * tindices[3 * i + 2] + 0]);
      iso_points.push_back(vdata[3 * tindices[3 * i + 2] + 1]);
      iso_points.push_back(vdata[3 * tindices[3 * i + 2] + 2]);
      iso_points.push_back(1);
		}

    /// COLORS
    int numColorChannels = numFaces * 3;
		for (i = 0; i < numColorChannels; i++) {
      iso_colors.push_back(1.0);
      iso_colors.push_back(i / 60);
      iso_colors.push_back(0.0 * i);
		}

		// Geometrically, the normals of the vertices point the same way as the
		// vertices themselves. However, in case we want to scale our
		// points, the normals are better calculated here
    iso_normals = iso_points;
  }

  void set_vertex_data() {
    createIsocahedron(vertex_data_, color_data_, normal_data_);
  }
};


#endif // BULLET_SPHERE_H
