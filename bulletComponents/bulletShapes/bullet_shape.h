#pragma once

/////////////////////////////////////////
/// \brief The bullet_shape class
/// Is a superclass holding all of the essential functions and members of our
/// Bullet Shapes.
/////////////////////////////////////////

#include <memory>
#include <vector>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/LinearMath/btAlignedAllocator.h>

class bullet_shape{
 public:

  //Set the object to the pose specified through MATLAB.
  int SetPose(double* position, double* rotation){
    btVector3 pos (position[0], position[1], position[2]);
    btMatrix3x3 rot (rotation[0], rotation[1], rotation[3],
                     rotation[3], rotation[4], rotation[5],
                     rotation[6], rotation[7], rotation[8]);
    btTransform bullet_trans(rot, pos);
    bulletBody->setCenterOfMassTransform(bullet_trans);
    return 0;
  }

  // Getters
  btCollisionShape* collisionShapePtr(){
    return bulletShape;
  }

  btRigidBody* rigidBodyPtr(){
    return bulletBody;
  }

  btMotionState* motionStatePtr(){
    return bulletMotionState;
  }

  // OpenGL functions
  void set_vertex_buffer(int buffer_int) {
    vertex_buffer_ = buffer_int;
  }

  unsigned int vertex_buffer() {
    return vertex_buffer_;
  };

  virtual void set_vertex_data(){}

 protected:
  btCollisionShape* bulletShape;
  btRigidBody* bulletBody;
  btMotionState* bulletMotionState;
  // Graphics functions
  std::vector<float> vertex_color_;
  int vertex_data_size_;
  unsigned int vertex_buffer_;  // Vertex buffer object
  std::vector<float> vertex_data_;   // Vertex data

};
