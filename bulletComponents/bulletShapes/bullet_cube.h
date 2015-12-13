#pragma once

#include "bullet_shape.h"
#include <bullet/BulletCollision/CollisionShapes/btBoxShape.h>

class bullet_cube: public bullet_shape {

 public:
  //constructor
  bullet_cube(double x_length, double y_length, double z_length, double dMass,
              double dRestitution, double* position, double* rotation)
  {
    vertex_color_ = {0.5, 0.5, 0.5};
    vertex_data_size_ = 36;
    btVector3 bounds = btVector3(x_length*.5, y_length*.5, z_length*.5);
    bulletShape = std::make_shared<btBoxShape>(bounds);

    bulletMotionState = std::make_shared<btDefaultMotionState>(btTransform::getIdentity());
    bool isDynamic = ( dMass != 0.f );
    btVector3 localInertia( 0, 0, 0 );
    if( isDynamic ){
      bulletShape->calculateLocalInertia( dMass, localInertia );
    }

    btRigidBody::btRigidBodyConstructionInfo  cInfo(dMass, bulletMotionState.get(),
                                                    bulletShape.get(), localInertia);
    bulletBody = std::make_shared<btRigidBody>(cInfo);
    //    double dContactProcessingThreshold = 0.1;
    //    bulletBody->setContactProcessingThreshold( dContactProcessingThreshold );
    bulletBody->setRestitution( dRestitution );
    SetPose(position, rotation);

    // OpenGL setup
    set_vertex_data();
  }

  void set_vertex_data() {
    // TODO: Set the color here
    vertex_data_ = {
      //  X  Y  Z  W   Nx Ny Nz    R G B   s t
      //  Front
      +1,+1,+1,+1,   0, 0,+1,   1,0,0,  1,1,
      -1,+1,+1,+1,   0, 0,+1,   1,0,0,  0,1,
      +1,-1,+1,+1,   0, 0,+1,   1,0,0,  1,0,
      -1,+1,+1,+1,   0, 0,+1,   1,0,0,  0,1,
      +1,-1,+1,+1,   0, 0,+1,   1,0,0,  1,0,
      -1,-1,+1,+1,   0, 0,+1,   1,0,0,  0,0,
      //  Back
      -1,-1,-1,+1,   0, 0,-1,   0,0,1,  1,0,
      +1,-1,-1,+1,   0, 0,-1,   0,0,1,  0,0,
      -1,+1,-1,+1,   0, 0,-1,   0,0,1,  1,1,
      +1,-1,-1,+1,   0, 0,-1,   0,0,1,  0,0,
      -1,+1,-1,+1,   0, 0,-1,   0,0,1,  1,1,
      +1,+1,-1,+1,   0, 0,-1,   0,0,1,  0,1,
      //  Right
      +1,+1,+1,+1,  +1, 0, 0,   1,1,0,  0,1,
      +1,-1,+1,+1,  +1, 0, 0,   1,1,0,  0,0,
      +1,+1,-1,+1,  +1, 0, 0,   1,1,0,  1,1,
      +1,-1,+1,+1,  +1, 0, 0,   1,1,0,  0,0,
      +1,+1,-1,+1,  +1, 0, 0,   1,1,0,  1,1,
      +1,-1,-1,+1,  +1, 0, 0,   1,1,0,  1,0,
      //  Left
      -1,+1,+1,+1,  -1, 0, 0,   0,1,0,  1,1,
      -1,+1,-1,+1,  -1, 0, 0,   0,1,0,  0,1,
      -1,-1,+1,+1,  -1, 0, 0,   0,1,0,  1,0,
      -1,+1,-1,+1,  -1, 0, 0,   0,1,0,  0,1,
      -1,-1,+1,+1,  -1, 0, 0,   0,1,0,  1,0,
      -1,-1,-1,+1,  -1, 0, 0,   0,1,0,  0,0,
      //  Top
      +1,+1,+1,+1,   0,+1, 0,   0,1,1,  1,0,
      +1,+1,-1,+1,   0,+1, 0,   0,1,1,  1,1,
      -1,+1,+1,+1,   0,+1, 0,   0,1,1,  0,0,
      +1,+1,-1,+1,   0,+1, 0,   0,1,1,  1,1,
      -1,+1,+1,+1,   0,+1, 0,   0,1,1,  0,0,
      -1,+1,-1,+1,   0,+1, 0,   0,1,1,  0,1,
      //  Bottom
      -1,-1,-1,+1,   0,-1, 0,   1,0,1,  0,0,
      +1,-1,-1,+1,   0,-1, 0,   1,0,1,  1,0,
      -1,-1,+1,+1,   0,-1, 0,   1,0,1,  0,1,
      +1,-1,-1,+1,   0,-1, 0,   1,0,1,  1,0,
      -1,-1,+1,+1,   0,-1, 0,   1,0,1,  0,1,
      +1,-1,+1,+1,   0,-1, 0,   1,0,1,  1,1,
    };
  }

};
