// Copyright (c) bminortx

#ifndef SIMBA_MODELGRAPH_PHYSICSENGINEHELPERS_H_
#define SIMBA_MODELGRAPH_PHYSICSENGINEHELPERS_H_

#include <math.h>

// Bullet libraries
#include <bullet/LinearMath/btIDebugDraw.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include <bullet/btBulletDynamicsCommon.h>

// Our ModelNode Objects
#include <BulletStructs/Shape.h>
#include <BulletStructs/SimRaycastVehicle.h>
#include <BulletStructs/Constraint.h>

// Our Controllers
#include <SimDevices/SimDevices.h>

// SceneGraphMotionState (for tracking our shapes)
#include <SceneGraph/SceneGraph.h>
#include <pangolin/pangolin.h>
#include <ModelGraph/GLDebugDrawer.h>

// Assimp to import our meshes into the Physics system
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

// Eigen to Bullet converters
#include <ModelGraph/EigenToBullet.h>

enum Compounds {
  VEHICLE = 0
};

//////////////////////////////////////////////////////////
///
/// NodeMotionState class
///
//////////////////////////////////////////////////////////

class NodeMotionState : public btMotionState {
 public:
  explicit NodeMotionState(const std::shared_ptr<ModelNode>& obj)
      : object(obj) {
  }

  virtual void getWorldTransform(btTransform &worldTrans) const {
    worldTrans = toBullet(object->GetPoseMatrix());
  }

  virtual void setWorldTransform(const btTransform &worldTrans) {
    object->SetPose(toEigen(worldTrans));
  }

  std::shared_ptr<ModelNode> object;
};

/// We declare these typedefs to shorten the name, basically.

typedef std::shared_ptr<btCollisionShape> CollisionShapePtr;
typedef std::shared_ptr<btRigidBody> RigidBodyPtr;
typedef std::shared_ptr<NodeMotionState> MotionStatePtr;
typedef std::shared_ptr<btRaycastVehicle> VehiclePtr;

//////////////////////////
/// The Entity class
/// Holds our bullet shapes and terrain.

class Entity {
 public:
  Entity() {
  }

  Entity(const CollisionShapePtr& pShape, const MotionStatePtr& pMotionState,
         const RigidBodyPtr& pRigidShape, const std::string& sName)
      : name_(sName), shape_(pShape), motion_state_(pMotionState),
        rigid_body_(pRigidShape) {
    name_ = sName;
    shape_ = pShape;
    motion_state_ = pMotionState;
    rigid_body_ = pRigidShape;
  }

  // member variables
  std::string name_;
  CollisionShapePtr shape_;
  MotionStatePtr motion_state_;
  RigidBodyPtr rigid_body_;
};

//////////////////////////
/// The Compound_Entity class
/// Holds all of our compound shapes and constraints, as well as the type of
/// compound we have.

class Compound_Entity {
 public:
  Compound_Entity() { }

  Compound_Entity(const std::vector<double> Shape_ids,
                  const std::vector<double> Con_ids,
                  const Compounds& type, const std::string& sName)
      : name_(sName), shape_ids_(Shape_ids),
        constraint_ids_(Con_ids), type_(type) { }

  std::string name_;
  std::vector<double> shape_ids_;
  std::vector<double> constraint_ids_;
  Compounds type_;
};

//////////////////////////
/// The Vehicle_Entity class
/// Holds all of our RaycastVehicles

class Vehicle_Entity {
 public:
  Vehicle_Entity() { }

  Vehicle_Entity(const CollisionShapePtr& pShape,
                 const MotionStatePtr& pMotionState,
                 const RigidBodyPtr& pRigidShape, const VehiclePtr& pVehicle,
                 const std::string& sName)
      : name_(sName), shape_(pShape), motion_state_(pMotionState),
        rigid_body_(pRigidShape), vehicle_(pVehicle)  { }

  // member variables
  std::string name_;
  CollisionShapePtr shape_;
  MotionStatePtr motion_state_;
  RigidBodyPtr rigid_body_;
  VehiclePtr vehicle_;
};

#endif  // SIMBA_MODELGRAPH_PHYSICSENGINEHELPERS_H_
