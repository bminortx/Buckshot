#pragma once

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

#include "../bulletShapes/bullet_cube.h"
#include "../bulletShapes/bullet_cylinder.h"
#include "../bulletShapes/bullet_heightmap.h"
#include "../bulletShapes/bullet_sphere.h"
#include "../bulletShapes/bullet_vehicle.h"
#include "Compound.h"
#include "../Graphics/graphicsWorld.h"
#include <map>
#include <vector>
#include <memory>

///////////////////////////////////////////////////////
/// The BulletWorld class
/// The Simulator class holds all of the methods that can be called from
/// bullet_interface_mex.cpp. Any method called from there MUST be in this
/// class.

class BulletWorld {
 public:
  BulletWorld();
  ~BulletWorld();

  /*********************************************************************
   *ADDING OBJECTS
   **********************************************************************/
  int AddCube(double x_length, double y_length, double z_length,
                     double dMass, double dRestitution,
                     double* position, double* rotation);
  int AddSphere(double radius, double dMass, double dRestitution,
                double* position, double* rotation);
  int AddCylinder(double radius, double height, double dMass,
                  double dRestitution, double* position, double* rotation);
  int AddTerrain(int row_count, int col_count, double grad,
                 double min_ht, double max_ht,
                 double* X, double *Y, double* Z,
                 double* normal);

  int AddCompound(double* Shape_ids, double* Con_ids,
                  const char* CompoundType);

  int AddRaycastVehicle(double* parameters, double* position,
                        double* rotation);

  /*********************************************************************
   *RUNNING THE SIMULATION
   **********************************************************************/

  void StepSimulation();

  /*********************************************************************
   *COMPOUND METHODS
   **********************************************************************/

  void CommandVehicle(double id, double steering_angle, double force);

  /*********************************************************************
   *RAYCAST VEHICLE METHODS
   **********************************************************************/
  void CommandRaycastVehicle(double id, double steering_angle, double force);
  // Holds the steering, engine force, and current velocity
  double* GetRaycastMotionState(double id);

  double* RaycastToGround(double id, double x, double y);
  //  This just drops us off on the surface...
  int OnTheGround(double id);
  void SetVehicleVels(double id, double* lin_vel, double* ang_vel);
  void ResetVehicle(double id, double* start_pose, double* start_rot);

  /*********************************************************************
   *CONSTRAINT METHODS
   *All of the constructors for our constraints.
   **********************************************************************/
  int AddConstraintToWorld(btTypedConstraint* constraint);
  int PointToPoint_one(double id_A, double* pivot_in_A);
  int PointToPoint_two(double id_A, double id_B,
                       double* pivot_in_A, double* pivot_in_B);
  int Hinge_one_transform(double id_A, double* transform_A, double* limits);
  int Hinge_two_transform(double id_A, double id_B,
                          double* transform_A, double* transform_B,
                          double* limits);
  int Hinge_one_pivot(double id_A, double* pivot_in_A,
                      double* axis_in_A, double* limits);
  int Hinge_two_pivot(double id_A, double id_B,
                      double* pivot_in_A, double* pivot_in_B,
                      double* axis_in_A, double* axis_in_B,
                      double* limits);
  int Hinge2(double id_A, double id_B, double* Anchor, double* Axis_1,
             double* Axis_2, double damping, double stiffness,
             double steering_angle);
  int SixDOF_one(double id_A, double* transform_A, double* limits);

    /*********************************************************************
   *GETTERS FOR OBJECT POSES
   **********************************************************************/

  std::vector<double> GetShapeTransform(double id);
  std::vector<double> GetConstraintTransform(double id);
  std::vector< btTransform > GetVehiclePoses(bullet_vehicle& Vehicle);
  double* GetVehicleTransform(double id);

 private:
  
  // Physics Bodies
  std::vector<std::unique_ptr<Compound> > compounds_;
  std::vector<std::unique_ptr<bullet_shape> > shapes_;
  std::vector<std::unique_ptr<bullet_vehicle> > vehicles_;
  std::vector<btTypedConstraint*> constraints_;
  
  // Physics Engine setup
  double timestep_;
  double gravity_;
  int max_sub_steps_;
  btDefaultCollisionConfiguration  collision_configuration_;
  std::unique_ptr<btCollisionDispatcher> bt_dispatcher_;
  std::unique_ptr<btDbvtBroadphase> bt_broadphase_;
  std::unique_ptr<btSequentialImpulseConstraintSolver> bt_solver_;
  
  // Physics and Graphics worlds
  std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world_;
  std::shared_ptr<GraphicsWorld> graphics_world_;
};

