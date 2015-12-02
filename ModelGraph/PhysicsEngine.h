// Copyright (c) bminortx

#ifndef SIMBA_MODELGRAPH_PHYSICSENGINE_H_
#define SIMBA_MODELGRAPH_PHYSICSENGINE_H_

// All of our Bullet Objects
// bullet_shape holds the header files Shapes.h and RaycastVehicle.h
#include <ModelGraph/Bullet_shapes/bullet_shape.h>
#include <ModelGraph/Bullet_shapes/bullet_cube.h>
#include <ModelGraph/Bullet_shapes/bullet_cylinder.h>
#include <ModelGraph/Bullet_shapes/bullet_sphere.h>
#include <ModelGraph/Bullet_shapes/bullet_vehicle.h>
#include <ModelGraph/Bullet_shapes/bullet_plane.h>
#include <ModelGraph/Bullet_shapes/bullet_heightmap.h>
#include <ModelGraph/Bullet_shapes/bullet_mesh.h>
#include "ModelGraph/PhysicsEngineHelpers.h"


//////////////////////////////////////////////////////////
///
/// PhysicsEngine class
/// PhysicsEngine encapsulates all of the Physics engine (Bullet) into one
/// class. It initializes the physics environment, and allows for the addition
/// and deletion of objects. It must also be called to run the physics sim.
///
//////////////////////////////////////////////////////////


class PhysicsEngine {
 public:
  /// CONSTRUCTOR
  PhysicsEngine();

  /// Initializer
  bool Init(double dGravity = -9.8, double dTimeStep = 1.0/30.0,
            double nMaxSubSteps = 10);

  /// ADDING OBJECTS TO THE PHYSICS ENGINE
  /// RegisterObject adds shapes, constraints, and vehicles.
  void RegisterObject(const std::shared_ptr<ModelNode>& pItem);
  void RegisterDevice(SimDeviceInfo* pDevice);
  bool isVehicle(std::string Shape);

  /// RUNNING THE SIMULATION
  void DebugDrawWorld();
  void RunDevices();
  void StepSimulation();

  /// PRINT AND DRAW FUNCTIONS
  void PrintAllShapes();

  /// RAYCAST VEHICLE METHODS
  Eigen::Vector6d SwitchYaw(Eigen::Vector6d bad_yaw);
  Eigen::Vector6d SwitchWheelYaw(Eigen::Vector6d bad_yaw);
  std::vector<Eigen::Matrix4d> GetVehiclePoses(Vehicle_Entity* Vehicle);
  std::vector<Eigen::Matrix4d> GetVehicleTransform(std::string sVehicleName);
  bool RayCast(const Eigen::Vector3d& dSource,
               const Eigen::Vector3d& dRayVector,
               Eigen::Vector3d& dIntersect, const bool& biDirectional);
  bool RayCastNormal(const Eigen::Vector3d& dSource,
                     const Eigen::Vector3d& dRayVector,
                     Eigen::Vector3d& dNormal);

  /// PUBLIC MEMBER VARIABLES
  GLDebugDrawer debug_drawer_;
  std::map<std::string, std::shared_ptr<Vehicle_Entity> >    ray_vehicles_map_;
  std::map<std::string, std::shared_ptr<Entity> >            shapes_map_;
  std::map<std::string, std::shared_ptr<Compound_Entity> >   compounds_map_;
  std::map<std::string, btHingeConstraint*>                  hinge_map_;
  std::map<std::string, btHinge2Constraint*>                 hinge2_map_;
  std::map<std::string, btGeneric6DofConstraint*>            sixdof_map_;
  std::map<std::string, btPoint2PointConstraint*>            ptop_map_;
  std::vector<SimDeviceInfo*>                                sim_devices_;
  std::shared_ptr<btDiscreteDynamicsWorld>              dynamics_world_;
  std::shared_ptr<btDefaultVehicleRaycaster> vehicle_raycaster_;

 private:
  /// PRIVATE MEMBER VARIABLES
  btDefaultCollisionConfiguration  collision_configuration_;
  std::shared_ptr<btCollisionDispatcher> bt_dispatcher_;
  std::shared_ptr<btDbvtBroadphase> bt_broadphase_;
  std::shared_ptr<btSequentialImpulseConstraintSolver> bt_solver_;
  double timestep_;
  double gravity_acc_;
  int time_max_substeps_;
};

#endif  // SIMBA_MODELGRAPH_PHYSICSENGINE_H_
