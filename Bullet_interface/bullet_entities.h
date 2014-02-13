#ifndef BULLET_ENTITIES_H
#define BULLET_ENTITIES_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/LinearMath/btAlignedAllocator.h>

typedef  boost::shared_ptr<btCollisionShape>            CollisionShapePtr;
typedef  boost::shared_ptr<btMotionState>               MotionStatePtr;
typedef  boost::shared_ptr<btRigidBody>                 RigidBodyPtr;
typedef  boost::shared_ptr<btRaycastVehicle>            VehiclePtr;

enum Compounds{
  VEHICLE = 0
};

///////////////////////////////////////////////////////
///
/// The Entity class
/// Holds our bullet shapes and terrain.
///
///////////////////////////////////////////////////////

class Shape_Entity
{
public:
  Shape_Entity()
  {

  }
  Shape_Entity(CollisionShapePtr  pShape, MotionStatePtr  pMotionState,
         RigidBodyPtr pRigidBody){
    m_pShape       = pShape;
    m_pMotionState = pMotionState;
    m_pRigidBody   = pRigidBody;
  }
  //member variables
  CollisionShapePtr       m_pShape;
  MotionStatePtr          m_pMotionState;
  RigidBodyPtr            m_pRigidBody;
};

///////////////////////////////////////////////////////
///
/// The Compound_Entity class
/// Holds all of our compound shapes and constraints, as well as the type of
/// compound we have.
///
///////////////////////////////////////////////////////

class Compound_Entity
{
public:
  Compound_Entity()
  {

  }
  Compound_Entity(double* Shape_ids,
                  double* Con_ids,
                  Compounds type){
    m_vShape_ids = Shape_ids;
    m_vCon_ids = Con_ids;
    m_Type = type;
  }
  double* m_vShape_ids;
  double* m_vCon_ids;
  Compounds m_Type;
};

///////////////////////////////////////////////////////
///
/// The Vehicle_Entity class
/// Holds all of our RaycastVehicles
///
///////////////////////////////////////////////////////

class Vehicle_Entity
{
public:
  Vehicle_Entity()
  {

  }
  Vehicle_Entity(CollisionShapePtr  pShape, MotionStatePtr  pMotionState,
                 RigidBodyPtr pRigidBody, VehiclePtr pVehicle){
    m_pShape       = pShape;
    m_pMotionState = pMotionState;
    m_pRigidBody   = pRigidBody;
    m_pVehicle     = pVehicle;
  }
  //member variables
  CollisionShapePtr       m_pShape;
  MotionStatePtr          m_pMotionState;
  RigidBodyPtr            m_pRigidBody;
  VehiclePtr              m_pVehicle;
};

#endif // BULLET_ENTITIES_H
