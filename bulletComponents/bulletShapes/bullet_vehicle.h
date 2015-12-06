#ifndef BULLET_VEHICLE_H
#define BULLET_VEHICLE_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btBoxShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet/BulletDynamics/Vehicle/btRaycastVehicle.h"

/////////////////////////////////////////
/// \brief The bullet_vehicle class
/// A great tutorial on this construction is found at the below link:
/// http://bit.ly/1eeaHoK
/// *and*
/// http://www.gamedev.net/topic/644280-bullet-physics-vehicle/
/// Nima Keivan did most of the legwork on this car in his
/// BulletCarModel.cpp. Way to go, champ.
/////////////////////////////////////////

class bullet_vehicle{
public:

  //constructor
  bullet_vehicle(double* parameters,
                 double* position,
                 double* rotation,
                 btDynamicsWorld* m_pDynamicsWorld){

    ///////
    //Create our Collision Objects
    ///////
    bulletShape = new btBoxShape(btVector3(parameters[WheelBase]/2,
                                           parameters[Width]/2,
                                           parameters[Height]/2));
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(0,0,1));
    bool isDynamic = (parameters[Mass] != 0.f);
    btVector3 localInertia(0,0,0);
    if (isDynamic){
      bulletShape->calculateLocalInertia(parameters[Mass],localInertia);
    }
    bulletMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(parameters[Mass],
                                                   bulletMotionState,
                                                   bulletShape,
                                                   localInertia);
    bulletBody = new btRigidBody(cInfo);
    bulletBody->setContactProcessingThreshold(BT_LARGE_FLOAT);
    m_pDynamicsWorld->addRigidBody( bulletBody );

    ///////
    //Create our Vehicle
    ///////

    btVector3 vWheelDirectionCS0(0,0,-1);
    btVector3 vWheelAxleCS(0,-1,0);
    btRaycastVehicle::btVehicleTuning	tuning;
    tuning.m_frictionSlip = parameters[TractionFriction];
    tuning.m_suspensionCompression = parameters[CompDamping];
    tuning.m_suspensionStiffness = parameters[Stiffness];
    tuning.m_suspensionDamping = parameters[ExpDamping];
    tuning.m_maxSuspensionForce = parameters[MaxSuspForce];
    tuning.m_maxSuspensionTravelCm = parameters[MaxSuspTravel]*100.0;
    btVehicleRaycaster* VehicleRaycaster =
        new btDefaultVehicleRaycaster(m_pDynamicsWorld);
    bulletVehicle = new btRaycastVehicle(tuning, bulletBody,
                                         VehicleRaycaster);
    // Never deactivate the vehicle
    bulletBody->forceActivationState(DISABLE_DEACTIVATION);
    bulletVehicle->setCoordinateSystem(1, 2, 0);
    m_pDynamicsWorld->addAction( bulletVehicle );

    bool bIsFrontWheel=true;
    double con_length = parameters[WheelBase]/2;
    double con_width = parameters[Width]/2-(0.3*parameters[WheelWidth]);
    double con_height = parameters[SuspConnectionHeight];

    btVector3 connectionPointCS0(con_length,
                                 con_width,
                                 con_height);
    bulletVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,
                            parameters[SuspRestLength],parameters[WheelRadius],
                            tuning,bIsFrontWheel);
    connectionPointCS0 = btVector3(-con_length,
                                   con_width,
                                   con_height);
    bulletVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,
                            parameters[SuspRestLength],parameters[WheelRadius],
                            tuning,bIsFrontWheel);
    bIsFrontWheel = false;
    connectionPointCS0 = btVector3(-con_length,
                                   -con_width,
                                   con_height);
    bulletVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,
                            parameters[SuspRestLength],parameters[WheelRadius],
                            tuning,bIsFrontWheel);
    connectionPointCS0 = btVector3(con_length,
                                   -con_width,
                                   con_height);
    bulletVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,
                            parameters[SuspRestLength],parameters[WheelRadius],
                            tuning,bIsFrontWheel);

    for (int i=0;i<bulletVehicle->getNumWheels();i++)
    {
      btWheelInfo& wheel = bulletVehicle->getWheelInfo(i);
      wheel.m_rollInfluence = parameters[RollInfluence];
    }

    //reset all parameters
    bulletBody->setLinearVelocity(btVector3(0,0,0));
    bulletBody->setAngularVelocity(btVector3(0,0,0));
    m_pDynamicsWorld->getBroadphase()->
        getOverlappingPairCache()->cleanProxyFromPairs(
          bulletBody->getBroadphaseHandle(),
          m_pDynamicsWorld->getDispatcher());
    if (bulletVehicle)
    {
      bulletVehicle->resetSuspension();
      //synchronize the wheels with the (interpolated) chassis worldtransform
      for (size_t i=0;i<bulletVehicle->getNumWheels();i++)
      {
        bulletVehicle->updateWheelTransform(i,false);
      }
    }
    SetVehiclePose(position, rotation);
  }

  void set_vertex_data() {}

  ///////////////////////////

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

  btRaycastVehicle* getBulletRaycastVehicle(){
    return bulletVehicle;
  }


private:
  //A compound shape to hold all of our collision shapes.
  btCollisionShape* bulletShape;
  btRigidBody* bulletBody;
  btDefaultMotionState* bulletMotionState;
  btRaycastVehicle* bulletVehicle;


  enum{
    WheelBase = 0,                //< Wheel base of the car
    Width = 1,                    //< Width of the car
    Height = 2,                   //< Height of the CG above the vehicle datum
    DynamicFrictionCoef = 3,      //< Friction coefficient, which slows the car
    /// down.
    StaticSideFrictionCoef = 4,   //< Static side friction coefficient,
    /// which enforces nonholonomity
    SlipCoefficient = 5,
    ControlDelay = 6,             //< Control delay of the vehicle (in seconds)
    Mass = 7,
    //WHEEL OPTIONS
    WheelRadius = 8,              //< Radius of the wheels
    WheelWidth = 9,               //< Thickness of the wheels
    TractionFriction = 10,        //< Friction value of the wheels to the ground
    //SUSPENSION OPTIONS
    SuspConnectionHeight = 11,    //< Height of the suspension connection point
    /// above the vehicle datum
    Stiffness = 12,               //< Suspension spring stiffness
    MaxSuspForce = 13,            //< Maximum allowable susp. force, in Newtons
    MaxSuspTravel = 14,           //< Maximum allowable travel distance for the
    /// suspension from the rest length
    SuspRestLength = 15,          //< Rest length of the suspension. This should
    /// be the point where the suspension lies when
    /// there is no contact
    CompDamping = 16,             //< The damping coefficient for when the
    /// suspension is compressed. Set to k * 2.0 *
    /// btSqrt(m_suspensionStiffness)
    /// so k is proportional to critical damping.
    /// k = 0.0 undamped & bouncy, k = 1.0 critical
    /// damping. k = 0.1 to 0.3 are good values.
    ExpDamping = 17,              //< The damping coefficient for when the
    /// suspension is expanding. See the comments
    /// for m_wheelsDampingCompression for how to
    /// set k.
    RollInfluence = 18,
    SteeringCoef = 19,            //< Multiplier coefficient for steering
    MaxSteering = 20,
    MaxSteeringRate = 21,
    AccelOffset = 22,             //< Offset from the center for acceleration
    SteeringOffset = 23,          //< Offset from the center for steering
    ///DC MOTOR COEFFICIENTS
    StallTorqueCoef = 24,
    TorqueSpeedSlope = 25,
    //MAGIC FORMULA PARAMS
    MagicFormula_B  = 26,
    MagicFormula_C  = 27,
    MagicFormula_E = 28
  };

  int SetVehiclePose(double* position, double* rotation){
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


};



#endif // BULLET_VEHICLE_H
