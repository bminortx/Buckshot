#pragma once

#include <memory>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/LinearMath/btAlignedAllocator.h>

typedef btCollisionShape* CollisionShapePtr;
typedef btMotionState* MotionStatePtr;
typedef btRigidBody* RigidBodyPtr;
typedef btRaycastVehicle* VehiclePtr;

// Useless enum until we get more compounds
enum Compounds{
  VEHICLE = 0
};

/**
 * The Shape Entity class: Holds our bullet shapes and terrain.
 */

class Shape_Entity {
public:
  Shape_Entity() {
  }

  Shape_Entity(CollisionShapePtr  pShape, MotionStatePtr  pMotionState,
         RigidBodyPtr pRigidBody) {
    shape_       = pShape;
    motionstate_ = pMotionState;
    rigidbody_   = pRigidBody;
  }
  // member variables
  CollisionShapePtr       shape_;
  MotionStatePtr          motionstate_;
  RigidBodyPtr            rigidbody_;
};

/**
 * The Compound_Entity class: Holds all of our compound shapes and constraints,
 * as well as the type of compound we have.
 */
class Compound_Entity {
public:
  Compound_Entity() {

  }

  Compound_Entity(double* Shape_ids,
                  double* Con_ids,
                  Compounds type) {
    shapeid_ = Shape_ids;
    constraintid_ = Con_ids;
    type_ = type;
  }

  double* shapeid_;
  double* constraintid_;
  Compounds type_;
};

/**
 * The Vehicle_Entity class: Holds all of our RaycastVehicles
 */
class Vehicle_Entity {
public:
  Vehicle_Entity() {

  }

  Vehicle_Entity(CollisionShapePtr  pShape, MotionStatePtr  pMotionState,
                 RigidBodyPtr pRigidBody, VehiclePtr pVehicle):
      shape_(pShape), motionstate_(pMotionState), rigidbody_(pRigidBody),
      vehicle_(pVehicle) {

  }
  // member variables
  CollisionShapePtr shape_;
  MotionStatePtr motionstate_;
  RigidBodyPtr rigidbody_;
  VehiclePtr vehicle_;
};
