#ifndef __MATLAB_BULLET_WRAPPER_
#define __MATLAB_BULLET_WRAPPER_

/*
 * File:   matlab_bullet_wrapper.h
 * Author: bminortx
 * This wrapper is designed to facilitate the use of Bullet Physics
 * in MATLAB. Graphics are handled by MATLAB, as well as parameterization of
 * objects and environmental variables. Bullet takes care of all calculations
 * and returns those results back to MATLAB.
 */

// Suppress Bullet warnings in GCC and Clang
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic pop

#include <map>
#include <boost/shared_ptr.hpp>
//#include <pangolin/pangolin.h>
//#include <SceneGraph/SceneGraph.h>
#include "bulletEntities.h"
// Our shapes
#include "../bulletShapes/bullet_vehicle.h"
#include "../bulletShapes/bullet_cube.h"
#include "../bulletShapes/bullet_sphere.h"
#include "../bulletShapes/bullet_cylinder.h"
#include "../bulletShapes/bullet_heightmap.h"

///////////////////////////////////////////////////////
/// The Sim class
/// The Simulator class holds all of the methods that can be called from
/// bullet_interface_mex.cpp. Any method called from there MUST be in this
/// class.

class Sim {
 public:
  Sim() {
    ///Bullet settings
    m_dTimeStep = 1.0/30.0;
    m_dGravity = -9.8;
    m_nMaxSubSteps = 10; //  bullet -- for stepSimulation
    m_pDispatcher
        = boost::shared_ptr<btCollisionDispatcher>(
            new btCollisionDispatcher(&m_CollisionConfiguration));
    m_pBroadphase
        = boost::shared_ptr<btDbvtBroadphase>(new btDbvtBroadphase);
    m_pSolver
        = boost::shared_ptr<btSequentialImpulseConstraintSolver>(
            new btSequentialImpulseConstraintSolver);
    m_pDynamicsWorld = boost::shared_ptr<btDiscreteDynamicsWorld>(
        new btDiscreteDynamicsWorld(m_pDispatcher.get(),
                                    m_pBroadphase.get(),
                                    m_pSolver.get(),
                                    &m_CollisionConfiguration
                                    ));
    m_pDynamicsWorld->setGravity(btVector3(0, 0, m_dGravity));
  }

  /*********************************************************************
   *ADDING OBJECTS
   **********************************************************************/

  int AddCube(double x_length, double y_length, double z_length,
              double dMass, double dRestitution,
              double* position, double* rotation) {
    bullet_cube btBox(x_length, y_length, z_length, dMass, dRestitution,
                      position, rotation);
    CollisionShapePtr pShape(btBox.getBulletShapePtr());
    MotionStatePtr pMotionState(btBox.getBulletMotionStatePtr());
    RigidBodyPtr body(btBox.getBulletBodyPtr());
    m_pDynamicsWorld->addRigidBody(body.get());
    boost::shared_ptr<Shape_Entity> pEntity(new Shape_Entity);
    pEntity->rigidbody_ = body;
    pEntity->shape_ = pShape;
    pEntity->motionstate_ = pMotionState;
    int id = m_mShapes.size();
    m_mShapes[id] = pEntity;
    return id;
  }

  int AddSphere(double radius, double dMass, double dRestitution,
                double* position, double* rotation) {
    bullet_sphere btSphere(radius, dMass, dRestitution,
                           position, rotation);
    CollisionShapePtr pShape(btSphere.getBulletShapePtr());
    btAssert((!pShape || pShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
    MotionStatePtr pMotionState(btSphere.getBulletMotionStatePtr());
    RigidBodyPtr body(btSphere.getBulletBodyPtr());
    m_pDynamicsWorld->addRigidBody(body.get());
    boost::shared_ptr<Shape_Entity> pEntity(new Shape_Entity);
    pEntity->rigidbody_ = body;
    pEntity->shape_ = pShape;
    pEntity->motionstate_ = pMotionState;
    int id = m_mShapes.size();
    m_mShapes[id] = pEntity;
    return id;
  }

  int AddCylinder(double radius, double height, double dMass,
                  double dRestitution, double* position, double* rotation) {
    bullet_cylinder btCylinder(radius, height, dMass, dRestitution,
                               position, rotation);
    CollisionShapePtr pShape(btCylinder.getBulletShapePtr());
    btAssert((!pShape || pShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
    MotionStatePtr pMotionState(btCylinder.getBulletMotionStatePtr());
    RigidBodyPtr body(btCylinder.getBulletBodyPtr());
    m_pDynamicsWorld->addRigidBody(body.get());
    boost::shared_ptr<Shape_Entity> pEntity(new Shape_Entity);
    pEntity->rigidbody_ = body;
    pEntity->shape_ = pShape;
    pEntity->motionstate_ = pMotionState;
    int id = m_mShapes.size();
    m_mShapes[id] = pEntity;
    return id;
  }

  int AddTerrain(int row_count, int col_count, double grad,
                 double min_ht, double max_ht,
                 double* X, double *Y, double* Z,
                 double* normal) {
    // Let's see if that works... if not, try something else.
    bullet_heightmap btTerrain(row_count, col_count, grad, min_ht,
                               max_ht, X, Y, Z, normal);
    CollisionShapePtr pShape(btTerrain.getBulletShapePtr());
    MotionStatePtr pMotionState(btTerrain.getBulletMotionStatePtr());
    RigidBodyPtr body(btTerrain.getBulletBodyPtr());
    m_pDynamicsWorld->addCollisionObject(body.get());
    boost::shared_ptr<Shape_Entity> pEntity(new Shape_Entity);
    pEntity->rigidbody_ = body;
    pEntity->shape_ = pShape;
    pEntity->motionstate_ = pMotionState;
    int id = m_mShapes.size();
    m_mShapes[id] = pEntity;
    return id;
  }

  int AddCompound(double* Shape_ids, double* Con_ids,
                  const char* CompoundType) {
    boost::shared_ptr<Compound_Entity> pCompound(new Compound_Entity);
    pCompound->shapeid_ = Shape_ids;
    pCompound->constraintid_ = Con_ids;
    if (!std::strcmp(CompoundType, "Vehicle")) {
      pCompound->type_ = VEHICLE;
    }
    int id = m_mCompounds.size();
    m_mCompounds[id] = pCompound;
    return id;
  }

  int AddRaycastVehicle(double* parameters, double* position,
                        double* rotation) {
    bullet_vehicle btRayVehicle(parameters, position, rotation,
                                m_pDynamicsWorld.get());
    CollisionShapePtr pShape(btRayVehicle.getBulletShapePtr());
    MotionStatePtr pMotionState(btRayVehicle.getBulletMotionStatePtr());
    RigidBodyPtr body(btRayVehicle.getBulletBodyPtr());
    VehiclePtr vehicle(btRayVehicle.getBulletRaycastVehicle());
    boost::shared_ptr<Vehicle_Entity> pEntity(new Vehicle_Entity);
    pEntity->rigidbody_ = body;
    pEntity->shape_ = pShape;
    pEntity->motionstate_ = pMotionState;
    pEntity->vehicle_ = vehicle;
    int id = m_mRayVehicles.size();
    m_mRayVehicles[id] = pEntity;
    return id;
  }

  /*********************************************************************
   *RUNNING THE SIMULATION
   **********************************************************************/

  void StepSimulation() {
    m_pDynamicsWorld->stepSimulation(m_dTimeStep,  m_nMaxSubSteps);
  }

  /*********************************************************************
   *COMPOUND METHODS
   **********************************************************************/

  void CommandVehicle(double id, double steering_angle, double force) {
    boost::shared_ptr<Compound_Entity> Vehicle = m_mCompounds[id];
    double* Shape_ids = Vehicle->shapeid_;
    double* Con_ids = Vehicle->constraintid_;
    btHinge2Constraint* wheel_fl = m_Hinge2.at(int(Con_ids[0]));
    btHinge2Constraint* wheel_fr = m_Hinge2.at(int(Con_ids[1]));
    boost::shared_ptr<Shape_Entity> boost_wheel_bl =
        m_mShapes.at(int(Shape_ids[3]));
    boost::shared_ptr<Shape_Entity> boost_wheel_br =
        m_mShapes.at(int(Shape_ids[4]));
    // Turn the front wheels. This requires manipulation of the constraints.
    wheel_fl->setUpperLimit(steering_angle);
    wheel_fl->setLowerLimit(steering_angle);
    wheel_fr->setUpperLimit(steering_angle);
    wheel_fr->setLowerLimit(steering_angle);
    // Power to the back wheels. Requires torque on the back tires. They're
    // rotated the same way, so torque should be applied in the same direction
    btVector3 torque(0, 0, force);
    boost_wheel_bl->rigidbody_->applyTorque(torque);
    boost_wheel_br->rigidbody_->applyTorque(torque);
  }

  /*********************************************************************
   *RAYCAST VEHICLE METHODS
   **********************************************************************/

  void CommandRaycastVehicle(double id, double steering_angle, double force) {
    VehiclePtr Vehicle = m_mRayVehicles[id]->vehicle_;
    Vehicle->setSteeringValue(steering_angle, 0);
    Vehicle->setSteeringValue(steering_angle, 1);
    Vehicle->applyEngineForce(force, 2);
    Vehicle->applyEngineForce(force, 3);
  }

  // Holds the steering, engine force, and current velocity
  double* GetRaycastMotionState(double id) {
    double* pose = new double[9];
    VehiclePtr Vehicle = m_mRayVehicles[id]->vehicle_;
    RigidBodyPtr VehicleBody = m_mRayVehicles[id]->rigidbody_;
    pose[0] = Vehicle->getSteeringValue(0);
    btWheelInfo wheel = Vehicle->getWheelInfo(2);
    pose[1] = wheel.m_engineForce;
    pose[2] = VehicleBody->getLinearVelocity().getX();
    pose[3] = VehicleBody->getLinearVelocity().getY();
    pose[4] = VehicleBody->getLinearVelocity().getZ();
    pose[5] = VehicleBody->getAngularVelocity().getX();
    pose[6] = VehicleBody->getAngularVelocity().getY();
    pose[7] = VehicleBody->getAngularVelocity().getZ();
    pose[8] = OnTheGround(id);
    return pose;
  }

  double* RaycastToGround(double id, double x, double y) {
    double* pose = new double[3];
    VehiclePtr Vehicle = m_mRayVehicles[id]->vehicle_;
    //  Move our vehicle out of the way...
    btVector3 point(x+50, y+50, -100);
    btMatrix3x3 rot = Vehicle->getChassisWorldTransform().getBasis();
    btTransform bullet_trans(rot, point);
    m_mRayVehicles[id]->rigidbody_->setCenterOfMassTransform(bullet_trans);
    //  Now shoot our ray...
    btVector3 ray_start(x, y, 100);
    btVector3 ray_end(x, y, -100);
    btCollisionWorld::ClosestRayResultCallback ray_callback(ray_start,
                                                            ray_end);
    m_pDynamicsWorld->rayTest(ray_start, ray_end, ray_callback);
    btVector3 hitpoint = Vehicle->getChassisWorldTransform().getOrigin();
    if (ray_callback.hasHit()) {
      hitpoint = ray_callback.m_hitPointWorld;
      btWheelInfo wheel = Vehicle->getWheelInfo(2);
      double radius = wheel.m_wheelsRadius;
      //  Find a way to access the height of the car.
      hitpoint.setZ(hitpoint.getZ()+(3*radius));
    }
    //  Now move our car!
    btTransform bullet_move(rot, hitpoint);
    m_mRayVehicles[id]->rigidbody_->setCenterOfMassTransform(bullet_move);

    // Now make sure none of our wheels are in the ground.
    // Kind of a nasty oop, but keep it for now.
    double hit = -1;
    double count = 0;
    while (hit == -1 && count<20) {
      for (int i = 0; i<4; i++) {
        hit = Vehicle->rayCast(Vehicle->getWheelInfo(i));
        if (hit!= -1) {
          break;
        }
      }
      // If we're still in the ground, lift us up!
      hitpoint.setZ(hitpoint.getZ()+.1);
      btTransform bullet_move(rot, hitpoint);
      m_mRayVehicles[id]->
          rigidbody_->setCenterOfMassTransform(bullet_move);
      if (hit!= -1) {
        break;
      }
      count++;
      if (count == 20) {
        break;
      }
    }
    int on = false;
    btVector3 VehiclePose = Vehicle->getChassisWorldTransform().getOrigin();
    while (on == 0) {
      on = OnTheGround(id);
      StepSimulation();
      VehiclePose = Vehicle->getChassisWorldTransform().getOrigin();
    }
    pose[0] = VehiclePose.getX();
    pose[1] = VehiclePose.getY();
    pose[2] = VehiclePose.getZ();
    return pose;
  }

  //  This just drops us off on the surface...
  int OnTheGround(double id) {
    VehiclePtr Vehicle = m_mRayVehicles[id]->vehicle_;
    int OnGround = 0;
    int hit = 0;
    for (int i = 0; i<4; i++) {
      hit = hit + Vehicle->rayCast(Vehicle->getWheelInfo(i));
    }
    if (hit == 0) {
      OnGround = 1;
    }
    return OnGround;
  }

  void SetVehicleVels(double id, double* lin_vel, double* ang_vel) {
    RigidBodyPtr VehicleBody = m_mRayVehicles[id]->rigidbody_;
    VehiclePtr Vehicle = m_mRayVehicles[id]->vehicle_;
    btVector3 Lin(lin_vel[0], lin_vel[1], lin_vel[2]);
    btVector3 Ang(ang_vel[0], ang_vel[1], ang_vel[2]);
    VehicleBody->setLinearVelocity(Lin);
    VehicleBody->setAngularVelocity(Ang);
    Vehicle->resetSuspension();
  }

  void ResetVehicle(double id, double* start_pose, double* start_rot) {
    //  Move the vehicle into start position
    btMatrix3x3 rot(start_rot[0], start_rot[3], start_rot[6],
                    start_rot[1], start_rot[4], start_rot[7],
                    start_rot[2], start_rot[5], start_rot[8]);
    btVector3 pose(start_pose[0], start_pose[1], start_pose[2]);
    btTransform bullet_trans(rot, pose);
    //  Reset our car to its initial state.
    m_mRayVehicles[id]->rigidbody_->setCenterOfMassTransform(bullet_trans);
  }

  double* SpeedSim(double id, double* start_pose, double* start_rot,
                   double* start_lin_vel, double* start_ang_vel,
                   double* forces, double* steering_angles,
                   double command_length) {
    int state_size = (command_length*3)+22;
    double* states = new double[state_size];
    VehiclePtr Vehicle = m_mRayVehicles[id]->vehicle_;
    ResetVehicle(id, start_pose, start_rot);
    SetVehicleVels(id, start_lin_vel, start_ang_vel);

    //  Run our commands through
    for (int i = 0; i < command_length; i++) {
      CommandRaycastVehicle(id, steering_angles[i], forces[i]);
      StepSimulation();
      btVector3 VehiclePose =
          Vehicle->getChassisWorldTransform().getOrigin();
      states[3*i] = VehiclePose.getX();
      states[3*i+1] = VehiclePose.getY();
      states[3*i+2] = VehiclePose.getZ();

      //  Get our whole state on the last step.

      if (i == command_length-1) {
        btVector3 VehiclePose =
            Vehicle->getChassisWorldTransform().getOrigin();
        btMatrix3x3 VehicleRot =
            Vehicle->getChassisWorldTransform().getBasis();
        states[3*i+3] = VehiclePose.getX();
        states[3*i+4] = VehiclePose.getY();
        states[3*i+5] = VehiclePose.getZ();
        states[3*i+6] = VehicleRot[0].getX();
        states[3*i+7] = VehicleRot[1].getX();
        states[3*i+8] = VehicleRot[2].getX();
        states[3*i+9] = VehicleRot[0].getY();
        states[3*i+10] = VehicleRot[1].getY();
        states[3*i+11] = VehicleRot[2].getY();
        states[3*i+12] = VehicleRot[0].getZ();
        states[3*i+13] = VehicleRot[1].getZ();
        states[3*i+14] = VehicleRot[2].getZ();
        double* motionstate = GetRaycastMotionState(id);
        states[3*i+15] = motionstate[2];
        states[3*i+16] = motionstate[3];
        states[3*i+17] = motionstate[4];
        states[3*i+18] = motionstate[5];
        states[3*i+19] = motionstate[6];
        states[3*i+20] = motionstate[7];
        states[3*i+21] = motionstate[8];
      }
    }

    // Reset our vehicle again (just in case this is our last iteration)
    ResetVehicle(id, start_pose, start_rot);
    SetVehicleVels(id, start_lin_vel, start_ang_vel);
    CommandRaycastVehicle(id, 0, 0);
    return states;
  }

  /*********************************************************************
   *CONSTRAINT METHODS
   *All of the constructors for our constraints.
   **********************************************************************/

  ///////
  // Point-to-Point
  int PointToPoint_one(double id_A, double* pivot_in_A) {
    boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
    btVector3 pivot_A(pivot_in_A[0], pivot_in_A[1], pivot_in_A[2]);
    btPoint2PointConstraint* PToP =
        new btPoint2PointConstraint(*Shape_A->rigidbody_.get(), pivot_A);
    m_pDynamicsWorld->addConstraint(PToP);
    int id = m_PtoP.size();
    m_PtoP.push_back(PToP);
    return id;
  }

  int PointToPoint_two(double id_A, double id_B,
                       double* pivot_in_A, double* pivot_in_B) {
    boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
    boost::shared_ptr<Shape_Entity> Shape_B = m_mShapes.at(id_B);
    btVector3 pivot_A(pivot_in_A[0], pivot_in_A[1], pivot_in_A[2]);
    btVector3 pivot_B(pivot_in_B[0], pivot_in_B[1], pivot_in_B[2]);
    btPoint2PointConstraint* PToP =
        new btPoint2PointConstraint(*Shape_A->rigidbody_.get(),
                                    *Shape_B->rigidbody_.get(),
                                    pivot_A, pivot_B);
    m_pDynamicsWorld->addConstraint(PToP);
    int id = m_PtoP.size();
    m_PtoP.push_back(PToP);
    return id;
  }

  ///////
  // Hinge
  int Hinge_one_transform(double id_A, double* transform_A, double* limits) {
    boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
    int id = 0;
    return id;
  }

  int Hinge_two_transform(double id_A, double id_B,
                          double* transform_A, double* transform_B,
                          double* limits) {
    boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
    boost::shared_ptr<Shape_Entity> Shape_B = m_mShapes.at(id_B);
    int id = 0;
    return id;

  }

  int Hinge_one_pivot(double id_A, double* pivot_in_A,
                      double* axis_in_A, double* limits) {
    boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
    btVector3 pivot_A(pivot_in_A[0], pivot_in_A[1], pivot_in_A[2]);
    btVector3 axis_A(axis_in_A[0], axis_in_A[1], axis_in_A[2]);
    btHingeConstraint* Hinge =
        new btHingeConstraint(*Shape_A->rigidbody_.get(), pivot_A,
                              axis_A, true);
    Hinge->setLimit(limits[0], limits[1], limits[2], limits[3], limits[4]);
    m_pDynamicsWorld->addConstraint(Hinge);
    int id = m_Hinge.size();
    m_Hinge.push_back(Hinge);
    return id;
  }

  int Hinge_two_pivot(double id_A, double id_B,
                      double* pivot_in_A, double* pivot_in_B,
                      double* axis_in_A, double* axis_in_B,
                      double* limits) {
    boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
    boost::shared_ptr<Shape_Entity> Shape_B = m_mShapes.at(id_B);
    btVector3 pivot_A(pivot_in_A[0], pivot_in_A[1], pivot_in_A[2]);
    btVector3 axis_A(axis_in_A[0], axis_in_A[1], axis_in_A[2]);
    btVector3 pivot_B(pivot_in_B[0], pivot_in_B[1], pivot_in_B[2]);
    btVector3 axis_B(axis_in_B[0], axis_in_B[1], axis_in_B[2]);
    btHingeConstraint* Hinge =
        new btHingeConstraint(*Shape_A->rigidbody_.get(),
                              *Shape_B->rigidbody_.get(),
                              pivot_A, pivot_B,
                              axis_A, axis_B,
                              true);
    Hinge->setLimit(limits[0], limits[1], limits[2], limits[3], limits[4]);
    m_pDynamicsWorld->addConstraint(Hinge);
    int id = m_Hinge.size();
    m_Hinge.push_back(Hinge);
    return id;
  }

  ///////
  // Hinge2
  int Hinge2(double id_A, double id_B, double* Anchor, double* Axis_1,
             double* Axis_2, double damping, double stiffness,
             double steering_angle) {
    boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
    boost::shared_ptr<Shape_Entity> Shape_B = m_mShapes.at(id_B);
    btVector3 btAnchor(Anchor[0], Anchor[1], Anchor[2]);
    btVector3 btAxis_1(Axis_1[0], Axis_1[1], Axis_1[2]);
    btVector3 btAxis_2(Axis_2[0], Axis_2[1], Axis_2[2]);
    btHinge2Constraint* Hinge2 =
        new btHinge2Constraint(*Shape_A->rigidbody_.get(),
                               *Shape_B->rigidbody_.get(),
                               btAnchor, btAxis_1, btAxis_2);
    Hinge2->setUpperLimit(steering_angle);
    Hinge2->setLowerLimit(steering_angle);
    Hinge2->enableSpring(3, true);
    Hinge2->setStiffness(3, stiffness);
    Hinge2->setDamping(3, damping);
    m_pDynamicsWorld->addConstraint(Hinge2);
    int id = m_Hinge2.size();
    m_Hinge2.push_back(Hinge2);
    return id;
  }

  ///////
  // Six DOF
  int SixDOF_one(double id_A, double* transform_A, double* limits) {
    boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
    btQuaternion quat_A(transform_A[3], transform_A[4],
                        transform_A[5], transform_A[6]);
    btVector3 pos_A(transform_A[0], transform_A[1], transform_A[2]);
    btTransform trans_A(quat_A, pos_A);
    btGeneric6DofConstraint* SixDOF =
        new btGeneric6DofConstraint(*Shape_A->rigidbody_.get(),
                                    trans_A,
                                    true);
    btVector3 max_lin_limits(limits[0], limits[1],  limits[2]);
    btVector3 min_lin_limits(limits[3], limits[4],  limits[5]);
    btVector3 max_ang_limits(limits[6], limits[7],  limits[8]);
    btVector3 min_ang_limits(limits[9], limits[10], limits[11]);
    SixDOF->setLinearLowerLimit(min_lin_limits);
    SixDOF->setLinearUpperLimit(max_lin_limits);
    SixDOF->setAngularLowerLimit(min_ang_limits);
    SixDOF->setAngularUpperLimit(max_ang_limits);
    m_pDynamicsWorld->addConstraint(SixDOF);
    int id = m_SixDOF.size();
    m_SixDOF.push_back(SixDOF);
    return id;
  }

  int SixDOF_two(double id_A, double id_B,
                 double* transform_A, double* transform_B,
                 double* limits) {
    boost::shared_ptr<Shape_Entity> Shape_A = m_mShapes.at(id_A);
    boost::shared_ptr<Shape_Entity> Shape_B = m_mShapes.at(id_B);
    btQuaternion quat_A(transform_A[3], transform_A[4],
                        transform_A[5], transform_A[6]);
    btVector3 pos_A(transform_A[0], transform_A[1], transform_A[2]);
    btTransform trans_A(quat_A, pos_A);
    btQuaternion quat_B(transform_B[3], transform_B[4],
                        transform_B[5], transform_B[6]);
    btVector3 pos_B(transform_B[0], transform_B[1], transform_B[2]);
    btTransform trans_B(quat_B, pos_B);
    btGeneric6DofConstraint* SixDOF =
        new btGeneric6DofConstraint(*Shape_A->rigidbody_.get(),
                                    *Shape_B->rigidbody_.get(),
                                    trans_A, trans_B,
                                    true);
    btVector3 max_lin_limits(limits[0], limits[1],  limits[2]);
    btVector3 min_lin_limits(limits[3], limits[4],  limits[5]);
    btVector3 max_ang_limits(limits[6], limits[7],  limits[8]);
    btVector3 min_ang_limits(limits[9], limits[10], limits[11]);
    SixDOF->setLinearLowerLimit(min_lin_limits);
    SixDOF->setLinearUpperLimit(max_lin_limits);
    SixDOF->setAngularLowerLimit(min_ang_limits);
    SixDOF->setAngularUpperLimit(max_ang_limits);
    m_pDynamicsWorld->addConstraint(SixDOF);
    int id = m_SixDOF.size();
    m_SixDOF.push_back(SixDOF);
    return id;
  }

  /*********************************************************************
   *GETTERS FOR OBJECT POSES
   **********************************************************************/

  double* GetShapeTransform(double id) {
    // Returns:
    // 1. The positon of the object in the world
    // 2. The rotation matrix that's used in motion.
    int n_id = (int)id;
    double* pose = new double[12];
    boost::shared_ptr<Shape_Entity> boost_Entity = m_mShapes.at(n_id);
    btTransform world_transform = boost_Entity->
        rigidbody_->getCenterOfMassTransform();
    btMatrix3x3 rotation = world_transform.getBasis();
    btVector3 position = world_transform.getOrigin();
    pose[0] = position.getX();
    pose[1] = position.getY();
    pose[2] = position.getZ();
    pose[3] =  rotation[0].getX();
    pose[4] =  rotation[1].getX();
    pose[5] =  rotation[2].getX();
    pose[6] =  rotation[0].getY();
    pose[7] =  rotation[1].getY();
    pose[8] =  rotation[2].getY();
    pose[9] =  rotation[0].getZ();
    pose[10] = rotation[1].getZ();
    pose[11] = rotation[2].getZ();
    return pose;
  }

  double* GetConstraintTransform(double id) {
    int n_id = (int)id;
    btHinge2Constraint* constraint = m_Hinge2.at(n_id);
    btVector3 position = constraint->getAnchor();
    double* pose = new double[3];
    pose[0] = position.getX();
    pose[1] = position.getY();
    pose[2] = position.getZ();
    return pose;
  }

  //////////
  // For RaycastVehicles
  // Returns:
  // 1. The position and rotation of the body of the vehicle.
  // 2. The position and rotations for each of the wheels.
  //////////

  std::vector< btTransform > GetVehiclePoses(Vehicle_Entity* Vehicle) {
    std::vector<btTransform> VehiclePoses;
    btTransform VehiclePose;
    VehiclePose.setIdentity();
    VehiclePose = Vehicle->vehicle_->getChassisWorldTransform();
    VehiclePoses.push_back(VehiclePose);
    for (int i = 0; i<Vehicle->vehicle_->getNumWheels(); i++) {
      Vehicle->vehicle_->updateWheelTransform(i, false);
      btTransform WheelPose;
      WheelPose.setIdentity();
      WheelPose = Vehicle->vehicle_->getWheelTransformWS(i);
      VehiclePoses.push_back(WheelPose);
    }
    return VehiclePoses;
  }

  double* GetVehicleTransform(double id) {
    int n_id = (int)id;
    boost::shared_ptr<Vehicle_Entity> boost_Vehicle = m_mRayVehicles.at(n_id);
    Vehicle_Entity* Vehicle = boost_Vehicle.get();
    std::vector<btTransform> Transforms = GetVehiclePoses(Vehicle);
    double* pose = new double[12*5];
    for (unsigned int i = 0; i<5; i++) {
      btMatrix3x3 rotation = Transforms.at(i).getBasis();
      btVector3 position = Transforms.at(i).getOrigin();
      pose[i*12+0] = position.getX();
      pose[i*12+1] = position.getY();
      pose[i*12+2] = position.getZ();
      pose[i*12+3] =  rotation[0].getX();
      pose[i*12+4] =  rotation[1].getX();
      pose[i*12+5] =  rotation[2].getX();
      pose[i*12+6] =  rotation[0].getY();
      pose[i*12+7] =  rotation[1].getY();
      pose[i*12+8] =  rotation[2].getY();
      pose[i*12+9] =  rotation[0].getZ();
      pose[i*12+10] = rotation[1].getZ();
      pose[i*12+11] = rotation[2].getZ();
    }
    return pose;
  }

  //////////////////////////////////////////////////////////////////

  std::map<int, boost::shared_ptr<Shape_Entity> > m_mShapes;
  std::map<int, boost::shared_ptr<Compound_Entity> > m_mCompounds;
  std::vector<btPoint2PointConstraint*> m_PtoP;
  std::vector<btHingeConstraint*> m_Hinge;
  std::vector<btHinge2Constraint*> m_Hinge2;
  std::vector<btGeneric6DofConstraint*> m_SixDOF;
  std::map<int, boost::shared_ptr<Vehicle_Entity> > m_mRayVehicles;

 private:
  btDefaultCollisionConfiguration m_CollisionConfiguration;
  boost::shared_ptr<btCollisionDispatcher> m_pDispatcher;
  boost::shared_ptr<btDbvtBroadphase> m_pBroadphase;
  boost::shared_ptr<btSequentialImpulseConstraintSolver> m_pSolver;
  boost::shared_ptr<btDiscreteDynamicsWorld> m_pDynamicsWorld;
  double m_dTimeStep;
  double m_dGravity;
  int m_nMaxSubSteps;
  //  SceneGraph::GLSceneGraph                               m_GLGraph;
};

#endif
