#include "bulletWorld.h"
#include <iostream>
#include <cstring>

// See http://bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World
BulletWorld::BulletWorld() :
  timestep_(1.0/30.0), gravity_(-9.8), max_sub_steps_(10),
  use_opengl_(false)
{
  bt_dispatcher_ = std::unique_ptr<btCollisionDispatcher>(
      new btCollisionDispatcher(&collision_configuration_));
  bt_broadphase_.reset(new btDbvtBroadphase);
  bt_solver_ = std::unique_ptr<btSequentialImpulseConstraintSolver>(
      new btSequentialImpulseConstraintSolver());
  dynamics_world_ = std::shared_ptr<btDiscreteDynamicsWorld>(
      new btDiscreteDynamicsWorld(bt_dispatcher_.get(),
                                  bt_broadphase_.get(),
                                  bt_solver_.get(),
                                  &collision_configuration_));
  dynamics_world_->setGravity(btVector3(0, 0, gravity_));
}

BulletWorld::~BulletWorld() {
  glutDestroyWindow(window);
}

void BulletWorld::Reset() {
  int i = 0;
  for (std::unique_ptr<bullet_shape>& shape : shapes_) {;
    if (i == 0) {
      i++;
    } else {
      dynamics_world_->removeRigidBody(shape->rigidBodyPtr());
      shape->rigidBodyPtr()->clearForces();
      btVector3 zeroVector(0,0,0);
      shape->rigidBodyPtr()->setLinearVelocity(zeroVector);
      shape->rigidBodyPtr()->setAngularVelocity(zeroVector);
      shape->rigidBodyPtr()->setWorldTransform(shape->startingPose());
      dynamics_world_->addRigidBody(shape->rigidBodyPtr());
    }
  }
  for (std::unique_ptr<bullet_vehicle>& shape : vehicles_) {
    dynamics_world_->removeRigidBody(shape->rigidBodyPtr());
    shape->rigidBodyPtr()->clearForces();
    btVector3 zeroVector(0,0,0);
    shape->rigidBodyPtr()->setLinearVelocity(zeroVector);
    shape->rigidBodyPtr()->setAngularVelocity(zeroVector);
    shape->rigidBodyPtr()->setWorldTransform(shape->startingPose());
    dynamics_world_->addRigidBody(shape->rigidBodyPtr());
  }
}

void BulletWorld::UseOpenGL() {
  use_opengl_ = true;
  Init();
}

/*********************************************************************
 *ADDING OBJECTS
 **********************************************************************/

int BulletWorld::AddCube(double x_length, double y_length, double z_length,
                         double dMass, double dRestitution,
                         double* position, double* rotation) {
  int id = shapes_.size();
  shapes_.emplace_back(new bullet_cube(x_length, y_length, z_length, dMass,
                                       dRestitution, position, rotation));
  dynamics_world_->addRigidBody(shapes_[id]->rigidBodyPtr());
  return id;
}

int BulletWorld::AddSphere(double radius, double dMass, double dRestitution,
                           double* position, double* rotation) {
  int id = shapes_.size();
  shapes_.emplace_back(
      new bullet_sphere(radius, dMass, dRestitution, position, rotation));
  dynamics_world_->addRigidBody(shapes_[id]->rigidBodyPtr());
  return id;
}

int BulletWorld::AddCylinder(double radius, double height, double dMass,
                             double dRestitution, double* position,
                             double* rotation) {
  int id = shapes_.size();
  shapes_.emplace_back(new bullet_cylinder(radius, height, dMass, dRestitution,
                                           position, rotation));
  dynamics_world_->addRigidBody(shapes_[id]->rigidBodyPtr());
  return id;
}

int BulletWorld::AddTerrain(int row_count, int col_count, double grad,
                            double min_ht, double max_ht,
                            double* X, double *Y, double* Z,
                            double* normal) {
  int id = shapes_.size();
  shapes_.emplace_back(new bullet_heightmap (row_count, col_count, grad,
                                             min_ht, max_ht, X, Y, Z, normal));
  dynamics_world_->addRigidBody(shapes_[id]->rigidBodyPtr());
  return id;
}

int BulletWorld::AddCompound(double* Shape_ids, double* Con_ids,
                             const char* CompoundType) {
  if (!std::strcmp(CompoundType, "Vehicle")) {
    int id = compounds_.size();
    compounds_.emplace_back(new Compound(Shape_ids, Con_ids, VEHICLE));
    return id;
  }
  return -1;
}

int BulletWorld::AddRaycastVehicle(double* parameters, double* position,
                                   double* rotation) {
  int id = vehicles_.size();
  vehicles_.emplace_back(new bullet_vehicle (parameters, position, rotation,
                                             dynamics_world_.get()));
  return id;
}

/*********************************************************************
 *RUNNING THE SIMULATION
 **********************************************************************/

void BulletWorld::StepSimulation() {
  dynamics_world_->stepSimulation(timestep_,  max_sub_steps_);
}

void BulletWorld::StepGUI() {
  if (use_opengl_) {
    glutMainLoopEvent();
  }
}

void BulletWorld::RunSimulation() {
  if (is_running_) {
    StepSimulation();
  } else if (is_iterating_) {
    StepSimulation();
    is_iterating_ = false;
  } else if (is_reset_) {
    is_running_ = false;
    is_iterating_ = false;
    Reset();
    is_reset_ = false;
  }
  StepGUI();
}

/*********************************************************************
 *COMPOUND METHODS
 **********************************************************************/

void BulletWorld::CommandVehicle(double id, double steering_angle,
                                 double force) {
  std::unique_ptr<Compound>& Vehicle = compounds_[id];
  double* Shape_ids = Vehicle->shapeid_;
  double* Con_ids = Vehicle->constraintid_;
  btHinge2Constraint* wheel_fl = static_cast<btHinge2Constraint*>(
      constraints_.at(int(Con_ids[0])));
  btHinge2Constraint* wheel_fr = static_cast<btHinge2Constraint*>(
      constraints_.at(int(Con_ids[1])));
  std::unique_ptr<bullet_shape>& wheel_bl =
      shapes_.at(int(Shape_ids[3]));
  std::unique_ptr<bullet_shape>& wheel_br =
      shapes_.at(int(Shape_ids[4]));
  // Turn the front wheels. This requires manipulation of the constraints.
  wheel_fl->setUpperLimit(steering_angle);
  wheel_fl->setLowerLimit(steering_angle);
  wheel_fr->setUpperLimit(steering_angle);
  wheel_fr->setLowerLimit(steering_angle);
  // Power to the back wheels. Requires torque on the back tires. They're
  // rotated the same way, so torque should be applied in the same direction
  btVector3 torque(0, 0, force);
  wheel_bl->rigidBodyPtr()->applyTorque(torque);
  wheel_br->rigidBodyPtr()->applyTorque(torque);
}

/*********************************************************************
 *RAYCAST VEHICLE METHODS
 **********************************************************************/

void BulletWorld::CommandRaycastVehicle(double id, double steering_angle,
                                        double force) {
  btRaycastVehicle* Vehicle = vehicles_[id]->vehiclePtr();
  Vehicle->setSteeringValue(steering_angle, 0);
  Vehicle->setSteeringValue(steering_angle, 1);
  Vehicle->applyEngineForce(force, 2);
  Vehicle->applyEngineForce(force, 3);
}

// Holds the steering, engine force, and current velocity
double* BulletWorld::GetRaycastMotionState(double id) {
  double* pose = new double[9];
  btRaycastVehicle* Vehicle = vehicles_[id]->vehiclePtr();
  btRigidBody* VehicleBody = vehicles_[id]->rigidBodyPtr();
  pose[0] = Vehicle->getSteeringValue(0);
  btWheelInfo wheel = Vehicle->getWheelInfo(2);
  pose[1] = wheel.m_engineForce;
  pose[2] = VehicleBody->getLinearVelocity()[0];
  pose[3] = VehicleBody->getLinearVelocity()[1];
  pose[4] = VehicleBody->getLinearVelocity()[2];
  pose[5] = VehicleBody->getAngularVelocity()[0];
  pose[6] = VehicleBody->getAngularVelocity()[1];
  pose[7] = VehicleBody->getAngularVelocity()[2];
  pose[8] = OnTheGround(id);
  return pose;
}

double* BulletWorld::RaycastToGround(double id, double x, double y) {
  double* pose = new double[3];
  btRaycastVehicle* Vehicle = vehicles_[id]->vehiclePtr();
  //  Move our vehicle out of the way...
  btVector3 point(x+50, y+50, -100);
  btMatrix3x3 rot = Vehicle->getChassisWorldTransform().getBasis();
  btTransform bullet_trans(rot, point);
  vehicles_[id]->rigidBodyPtr()->setCenterOfMassTransform(bullet_trans);
  //  Now shoot our ray...
  btVector3 ray_start(x, y, 100);
  btVector3 ray_end(x, y, -100);
  btCollisionWorld::ClosestRayResultCallback ray_callback(ray_start,
                                                          ray_end);
  dynamics_world_->rayTest(ray_start, ray_end, ray_callback);
  btVector3 hitpoint = Vehicle->getChassisWorldTransform().getOrigin();
  if (ray_callback.hasHit()) {
    hitpoint = ray_callback.m_hitPointWorld;
    btWheelInfo wheel = Vehicle->getWheelInfo(2);
    double radius = wheel.m_wheelsRadius;
    //  Find a way to access the height of the car.
    hitpoint.setZ(hitpoint[2]+(3*radius));
  }
  //  Now move our car!
  btTransform bullet_move(rot, hitpoint);
  vehicles_[id]->rigidBodyPtr()->setCenterOfMassTransform(bullet_move);

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
    hitpoint.setZ(hitpoint[2]+.1);
    btTransform bullet_move(rot, hitpoint);
    vehicles_[id]->rigidBodyPtr()->setCenterOfMassTransform(bullet_move);
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
  pose[0] = VehiclePose[0];
  pose[1] = VehiclePose[1];
  pose[2] = VehiclePose[2];
  return pose;
}

//  This just drops us off on the surface...
int BulletWorld::OnTheGround(double id) {
  btRaycastVehicle* Vehicle = vehicles_[id]->vehiclePtr();
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

void BulletWorld::SetVehicleVels(double id, double* lin_vel, double* ang_vel) {
  btRigidBody* VehicleBody = vehicles_[id]->rigidBodyPtr();
  btRaycastVehicle* Vehicle = vehicles_[id]->vehiclePtr();
  btVector3 Lin(lin_vel[0], lin_vel[1], lin_vel[2]);
  btVector3 Ang(ang_vel[0], ang_vel[1], ang_vel[2]);
  VehicleBody->setLinearVelocity(Lin);
  VehicleBody->setAngularVelocity(Ang);
  Vehicle->resetSuspension();
}

void BulletWorld::ResetVehicle(double id, double* start_pose, double* start_rot) {
  //  Move the vehicle into start position
  btMatrix3x3 rot(start_rot[0], start_rot[3], start_rot[6],
                  start_rot[1], start_rot[4], start_rot[7],
                  start_rot[2], start_rot[5], start_rot[8]);
  btVector3 pose(start_pose[0], start_pose[1], start_pose[2]);
  btTransform bullet_trans(rot, pose);
  //  Reset our car to its initial state.
  vehicles_[id]->rigidBodyPtr()->setCenterOfMassTransform(bullet_trans);
}

/*********************************************************************
 *CONSTRAINT METHODS
 *All of the constructors for our constraints.
 **********************************************************************/

inline int BulletWorld::AddConstraintToWorld(btTypedConstraint* constraint) {
  int id = constraints_.size();
  constraints_.push_back(constraint);
  dynamics_world_->addConstraint(constraints_.at(id));
  return id;
}

///////
// Point-to-Point
int BulletWorld::PointToPoint_one(double id_A, double* pivot_in_A) {
  std::unique_ptr<bullet_shape>& Shape_A = shapes_.at(id_A);
  btVector3 pivot_A(pivot_in_A[0], pivot_in_A[1], pivot_in_A[2]);
  btPoint2PointConstraint* constraint =
      new btPoint2PointConstraint(*Shape_A->rigidBodyPtr(), pivot_A);
  return AddConstraintToWorld(constraint);
}

int BulletWorld::PointToPoint_two(double id_A, double id_B,
                                  double* pivot_in_A, double* pivot_in_B) {
  std::unique_ptr<bullet_shape>& Shape_A = shapes_.at(id_A);
  std::unique_ptr<bullet_shape>& Shape_B = shapes_.at(id_B);
  btVector3 pivot_A(pivot_in_A[0], pivot_in_A[1], pivot_in_A[2]);
  btVector3 pivot_B(pivot_in_B[0], pivot_in_B[1], pivot_in_B[2]);
  btPoint2PointConstraint* constraint =
      new btPoint2PointConstraint(*Shape_A->rigidBodyPtr(),
                                  *Shape_B->rigidBodyPtr(),
                                  pivot_A, pivot_B);
  return AddConstraintToWorld(constraint);
}

///////
// Hinge
int BulletWorld::Hinge_one_transform(double id_A, double* transform_A, double* limits) {
  std::unique_ptr<bullet_shape>& Shape_A = shapes_.at(id_A);
  int id = 0;
  return id;
}

int BulletWorld::Hinge_two_transform(double id_A, double id_B,
                                     double* transform_A, double* transform_B,
                                     double* limits) {
  std::unique_ptr<bullet_shape>& Shape_A = shapes_.at(id_A);
  std::unique_ptr<bullet_shape>& Shape_B = shapes_.at(id_B);
  int id = 0;
  return id;
}

int BulletWorld::Hinge_one_pivot(double id_A, double* pivot_in_A,
                                 double* axis_in_A, double* limits) {
  std::unique_ptr<bullet_shape>& Shape_A = shapes_.at(id_A);
  btVector3 pivot_A(pivot_in_A[0], pivot_in_A[1], pivot_in_A[2]);
  btVector3 axis_A(axis_in_A[0], axis_in_A[1], axis_in_A[2]);

  btHingeConstraint* Hinge = new btHingeConstraint(*Shape_A->rigidBodyPtr(),
                                                  pivot_A, axis_A, true);
  Hinge->setLimit(limits[0], limits[1], limits[2], limits[3], limits[4]);
  return AddConstraintToWorld(Hinge);
}

int BulletWorld::Hinge_two_pivot(double id_A, double id_B,
                                 double* pivot_in_A, double* pivot_in_B,
                                 double* axis_in_A, double* axis_in_B,
                                 double* limits) {
  std::unique_ptr<bullet_shape>& Shape_A = shapes_.at(id_A);
  std::unique_ptr<bullet_shape>& Shape_B = shapes_.at(id_B);
  btVector3 pivot_A(pivot_in_A[0], pivot_in_A[1], pivot_in_A[2]);
  btVector3 axis_A(axis_in_A[0], axis_in_A[1], axis_in_A[2]);
  btVector3 pivot_B(pivot_in_B[0], pivot_in_B[1], pivot_in_B[2]);
  btVector3 axis_B(axis_in_B[0], axis_in_B[1], axis_in_B[2]);
  btHingeConstraint* Hinge = new btHingeConstraint(*Shape_A->rigidBodyPtr(),
                                                   *Shape_B->rigidBodyPtr(),
                                                   pivot_A, pivot_B,
                                                   axis_A, axis_B,
                                                   true);
  Hinge->setLimit(limits[0], limits[1], limits[2], limits[3], limits[4]);
  return AddConstraintToWorld(Hinge);
}

///////
// Hinge2
int BulletWorld::Hinge2(double id_A, double id_B, double* Anchor, double* Axis_1,
                        double* Axis_2, double damping, double stiffness,
                        double steering_angle) {
  std::unique_ptr<bullet_shape>& Shape_A = shapes_.at(id_A);
  std::unique_ptr<bullet_shape>& Shape_B = shapes_.at(id_B);
  btVector3 btAnchor(Anchor[0], Anchor[1], Anchor[2]);
  btVector3 btAxis_1(Axis_1[0], Axis_1[1], Axis_1[2]);
  btVector3 btAxis_2(Axis_2[0], Axis_2[1], Axis_2[2]);
  btHinge2Constraint* Hinge2 = new btHinge2Constraint(*Shape_A->rigidBodyPtr(),
                                                      *Shape_B->rigidBodyPtr(),
                                                      btAnchor, btAxis_1,
                                                      btAxis_2);
  Hinge2->setUpperLimit(steering_angle);
  Hinge2->setLowerLimit(steering_angle);
  Hinge2->enableSpring(3, true);
  Hinge2->setStiffness(3, stiffness);
  Hinge2->setDamping(3, damping);
  return AddConstraintToWorld(Hinge2);
}

///////
// Six DOF

/////////
//// TODO: DEBUG CONSTRAINT
/////////
int BulletWorld::SixDOF_one(double id_A, double* transform_A, double* limits) {
  std::unique_ptr<bullet_shape>& Shape_A = shapes_.at(id_A);
  btQuaternion quat_A(transform_A[3], transform_A[4],
                      transform_A[5], transform_A[6]);
  btVector3 pos_A(transform_A[0], transform_A[1], transform_A[2]);
  btTransform trans_A(quat_A, pos_A);
  btGeneric6DofConstraint* SixDOF =
      new btGeneric6DofConstraint(*Shape_A->rigidBodyPtr(),
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
  return AddConstraintToWorld(SixDOF);
}

/*********************************************************************
 *GETTERS FOR OBJECT POSES
 **********************************************************************/

std::vector<double> BulletWorld::GetShapeTransform(double id) {
  // Returns:
  // 1. The positon of the object in the world
  // 2. The rotation matrix that's used in motion.
  int n_id = (int)id;
  std::unique_ptr<bullet_shape>& entity = shapes_.at(n_id);
  btTransform world_transform =
      entity->rigidBodyPtr()->getCenterOfMassTransform();
  btMatrix3x3 rotation = world_transform.getBasis();
  btVector3 position = world_transform.getOrigin();
  double pose[] = {
    position[0], position[1], position[2],
    rotation[0][0], rotation[1][0], rotation[2][0],
    rotation[0][1], rotation[1][1], rotation[2][1],
    rotation[0][2], rotation[1][2], rotation[2][2]
  };
  std::vector<double> vecpose (pose, pose + sizeof(pose) / sizeof(double));
  return vecpose;
}

std::vector<double> BulletWorld::GetConstraintTransform(double id) {
  int n_id = (int)id;
  btHinge2Constraint* constraint =
      static_cast<btHinge2Constraint*>(constraints_.at(n_id));
  btVector3 position = constraint->getAnchor();
  double pose[] = { position[0], position[1], position[2] };
  std::vector<double> vecpose (pose, pose + sizeof(pose) / sizeof(double));
  return vecpose;
}

//////////
// For RaycastVehicles
// Returns:
// 1. The position and rotation of the body of the vehicle.
// 2. The position and rotations for each of the wheels.
//////////

std::vector< btTransform > BulletWorld::GetVehiclePoses(bullet_vehicle& Vehicle) {
  std::vector<btTransform> VehiclePoses;
  btTransform VehiclePose;
  VehiclePose.setIdentity();
  VehiclePose = Vehicle.vehiclePtr()->getChassisWorldTransform();
  VehiclePoses.push_back(VehiclePose);
  for (int i = 0; i<Vehicle.vehiclePtr()->getNumWheels(); i++) {
    Vehicle.vehiclePtr()->updateWheelTransform(i, false);
    btTransform WheelPose;
    WheelPose.setIdentity();
    WheelPose = Vehicle.vehiclePtr()->getWheelTransformWS(i);
    VehiclePoses.push_back(WheelPose);
  }
  return VehiclePoses;
}

double* BulletWorld::GetVehicleTransform(double id) {
  int n_id = (int)id;
  std::unique_ptr<bullet_vehicle>& Vehicle = vehicles_.at(n_id);
  std::vector<btTransform> Transforms = GetVehiclePoses(*Vehicle);
  double* pose = new double[12*5];
  for (unsigned int i = 0; i<5; i++) {
    btMatrix3x3 rotation = Transforms.at(i).getBasis();
    btVector3 position = Transforms.at(i).getOrigin();
    pose[i*12+0] = position[0];
    pose[i*12+1] = position[1];
    pose[i*12+2] = position[2];
    pose[i*12+3] =  rotation[0][0];
    pose[i*12+4] =  rotation[1][0];
    pose[i*12+5] =  rotation[2][0];
    pose[i*12+6] =  rotation[0][1];
    pose[i*12+7] =  rotation[1][1];
    pose[i*12+8] =  rotation[2][1];
    pose[i*12+9] =  rotation[0][2];
    pose[i*12+10] = rotation[1][2];
    pose[i*12+11] = rotation[2][2];
  }
  return pose;
}
