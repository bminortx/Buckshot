#ifndef BULLET_VEHICLE_H
#define BULLET_VEHICLE_H

#include <vector>

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btBoxShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet/BulletDynamics/Vehicle/btRaycastVehicle.h"

#include "ModelGraph/Bullet_shapes/bullet_shape.h"

/////////////////////////////////////////
/// \brief The bullet_vehicle class
/// A great tutorial on this construction is found at the below link:
/// http://bit.ly/1eeaHoK
/// *and*
/// http://www.gamedev.net/topic/644280-bullet-physics-vehicle/
/// Nima Keivan did most of the legwork on this car in his
/// BulletCarModel.cpp. Way to go, champ.
/////////////////////////////////////////

class bullet_vehicle {
public:

  // constructor
  bullet_vehicle(const std::shared_ptr<ModelNode>& mnVehicle,
                 std::shared_ptr<btDefaultVehicleRaycaster>& VehicleRaycaster,
                 std::shared_ptr<btDiscreteDynamicsWorld>& m_pDynamicsWorld) {
    std::shared_ptr<SimRaycastVehicle> pVehicle =
        std::dynamic_pointer_cast<SimRaycastVehicle>(mnVehicle);
    std::vector<double> parameters = pVehicle->GetParameters();
    Eigen::Matrix4d dPose;
    dPose = pVehicle->GetPoseMatrix();

    //////
    // Create our Collision Objects
    //////
    bulletShape = std::make_shared<btBoxShape>(
        btVector3(parameters[WheelBase]/2,
                  parameters[Width]/2,
                  parameters[Height]/2));
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(0, 0, 0));
    bool isDynamic = (parameters[Mass] != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic) {
      bulletShape->calculateLocalInertia(parameters[Mass], localInertia);
    }
    bulletMotionState = std::make_shared<NodeMotionState>(mnVehicle);
    btRigidBody::btRigidBodyConstructionInfo cInfo(parameters[Mass],
                                                   bulletMotionState.get(),
                                                   bulletShape.get(),
                                                   localInertia);
    bulletBody = std::make_shared<btRigidBody>(cInfo);
    bulletBody->setContactProcessingThreshold(BT_LARGE_FLOAT);
    m_pDynamicsWorld->addRigidBody(bulletBody.get());

    //////
    // Create our Vehicle
    //////

    btVector3 vWheelDirectionCS0(0, 0, -1);
    btVector3 vWheelAxleCS(0, -1, 0);
    btRaycastVehicle::btVehicleTuning tuning;
    tuning.m_frictionSlip = parameters[TractionFriction];
    tuning.m_suspensionCompression = parameters[CompDamping];
    tuning.m_suspensionStiffness = parameters[Stiffness];
    tuning.m_suspensionDamping = parameters[ExpDamping];
    tuning.m_maxSuspensionForce = parameters[MaxSuspForce];
    tuning.m_maxSuspensionTravelCm = parameters[MaxSuspTravel] * 100.0;
    bulletVehicle = std::make_shared<btRaycastVehicle>(tuning, bulletBody.get(),
                                                       VehicleRaycaster.get());
    //  Never deactivate the vehicle
    bulletBody->forceActivationState(DISABLE_DEACTIVATION);
    bulletVehicle->setCoordinateSystem(1, 2, 0);
    m_pDynamicsWorld->addAction(bulletVehicle.get());

    bool bIsFrontWheel = true;
    double con_length = parameters[WheelBase]/2;
    double con_width = parameters[Width]/2-(0.3*parameters[WheelWidth]);
    double con_height = parameters[SuspConnectionHeight];

    btVector3 connectionPointCS0(con_length,
                                 con_width,
                                 con_height);
    bulletVehicle->addWheel(connectionPointCS0, vWheelDirectionCS0, vWheelAxleCS,
                            parameters[SuspRestLength], parameters[WheelRadius],
                            tuning, bIsFrontWheel);
    connectionPointCS0 = btVector3(con_length,
                                   -con_width,
                                   con_height);
    bulletVehicle->addWheel(connectionPointCS0, vWheelDirectionCS0, vWheelAxleCS,
                            parameters[SuspRestLength], parameters[WheelRadius],
                            tuning, bIsFrontWheel);
    bIsFrontWheel = false;
    connectionPointCS0 = btVector3(-con_length,
                                   -con_width,
                                   con_height);
    bulletVehicle->addWheel(connectionPointCS0, vWheelDirectionCS0, vWheelAxleCS,
                            parameters[SuspRestLength], parameters[WheelRadius],
                            tuning, bIsFrontWheel);
    connectionPointCS0 = btVector3(-con_length,
                                   con_width,
                                   con_height);
    bulletVehicle->addWheel(connectionPointCS0, vWheelDirectionCS0, vWheelAxleCS,
                            parameters[SuspRestLength], parameters[WheelRadius],
                            tuning, bIsFrontWheel);

    for (int i = 0; i<bulletVehicle->getNumWheels(); i++) {
      btWheelInfo& wheel = bulletVehicle->getWheelInfo(i);
      wheel.m_rollInfluence = parameters[RollInfluence];
      btTransform wheelTrans = bulletVehicle->getWheelTransformWS(i);
      pVehicle->SetWheelPose(i, _T2Cart(toEigen(wheelTrans)));
    }

    // reset all parameters
    bulletBody->setLinearVelocity(btVector3(0, 0, 0));
    bulletBody->setAngularVelocity(btVector3(0, 0, 0));
    m_pDynamicsWorld->getBroadphase()->
        getOverlappingPairCache()->cleanProxyFromPairs(
          bulletBody->getBroadphaseHandle(),
          m_pDynamicsWorld->getDispatcher());
    if (bulletVehicle) {
      bulletVehicle->resetSuspension();
      // synchronize the wheels with the (interpolated) chassis worldtransform
      for (int i = 0; i<bulletVehicle->getNumWheels(); i++) {
        bulletVehicle->updateWheelTransform(i, false);
      }
    }
    SetVehiclePose(dPose);
  }

  //////////////////////////

  //getters
  CollisionShapePtr getBulletShapePtr() {
    return bulletShape;
  }

  RigidBodyPtr getBulletBodyPtr() {
    return bulletBody;
  }

  MotionStatePtr getBulletMotionStatePtr() {
    return bulletMotionState;
  }

  VehiclePtr getBulletRaycastVehicle() {
    return bulletVehicle;
  }

 private:
  // A compound shape to hold all of our collision shapes.
  CollisionShapePtr bulletShape;
  RigidBodyPtr bulletBody;
  MotionStatePtr bulletMotionState;
  VehiclePtr bulletVehicle;

  int SetVehiclePose(double* position, double* rotation) {
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

  int SetVehiclePose(Eigen::Matrix4d World_pose) {
    //  This may not work... may have to cast.
    btTransform bullet_trans = toBullet(World_pose);
    bulletBody->setCenterOfMassTransform(bullet_trans);
  }


};



#endif //  BULLET_VEHICLE_H
