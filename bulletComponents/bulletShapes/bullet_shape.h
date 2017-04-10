#pragma once

/////////////////////////////////////////
/// \brief The bullet_shape class
/// Is a superclass holding all of the essential functions and members of our
/// Bullet Shapes.
/////////////////////////////////////////

#ifdef USEGLEW
#include <GL/glew.h>
#endif
#define GL_GLEXT_PROTOTYPES

#include <GL/freeglut.h>

/// OpenGL Headers
// #define GLM_FORCE_RADIANS
// #include <glm/glm.hpp>
// #include <glm/vec3.hpp> // glm::vec3
// #include <glm/vec4.hpp> // glm::vec4
// #include <glm/mat4x4.hpp> // glm::mat4
// #include <glm/gtc/type_ptr.hpp>
// #include <glm/gtc/matrix_transform.hpp>

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
    _startingPose = bullet_trans;
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

  btTransform startingPose() {
    return _startingPose;
  }

  // OpenGL functions
  virtual void getDrawData(){}

 protected:
  btCollisionShape* bulletShape;
  btRigidBody* bulletBody;
  btMotionState* bulletMotionState;
  btTransform _startingPose;
};
