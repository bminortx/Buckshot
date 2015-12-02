#ifndef BULLET_SHAPE_H
#define BULLET_SHAPE_H

/////////////////////////////////////////
/// \brief The bullet_shape class
/// Is a superclass holding all of the essential functions and members of our
/// Bullet Shapes.
/////////////////////////////////////////

// Our Eigen Helpers
#include <URDFParser/EigenHelpers.h>

#include <ModelGraph/PhysicsEngineHelpers.h>

class bullet_shape {
public:

  // Methods

  // Set the object to the pose specified through MATLAB.
  int SetPose(double* position, double* rotation) {
    btVector3 pos;
    pos.setX(position[0]);
    pos.setY(position[1]);
    pos.setZ(position[2]);
    btMatrix3x3 rot;
    rot[0].setX(rotation[0]);
    rot[0].setY(rotation[1]);
    rot[0].setZ(rotation[2]);
    rot[1].setX(rotation[3]);
    rot[1].setY(rotation[4]);
    rot[1].setZ(rotation[5]);
    rot[2].setX(rotation[6]);
    rot[2].setY(rotation[7]);
    rot[2].setZ(rotation[8]);
    btTransform bullet_trans(rot, pos);
    bulletBody->setCenterOfMassTransform(bullet_trans);
    return 0;
  }

  void SetPose(Eigen::Matrix4d World_pose) {
    //  This may not work... may have to cast.
    btTransform bullet_trans = toBullet(World_pose);
    bulletBody->setCenterOfMassTransform(bullet_trans);
  }

  ///////////////////////////////////
  ///getters
  CollisionShapePtr getBulletShapePtr() {
    return bulletShape;
  }

  RigidBodyPtr getBulletBodyPtr() {
    return bulletBody;
  }

  MotionStatePtr getBulletMotionStatePtr() {
    return bulletMotionState;
  }

 protected:
  //A compound shape to hold all of our collision shapes.
  CollisionShapePtr bulletShape;
  RigidBodyPtr bulletBody;
  MotionStatePtr bulletMotionState;
};



#endif //  BULLET_SHAPE_H
