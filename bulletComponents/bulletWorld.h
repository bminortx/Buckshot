/*
 * File:   matlab_bullet_wrapper.h
 * Author: bminortx
 * This wrapper is designed to facilitate the use of Bullet Physics
 * in MATLAB. Graphics are handled by MATLAB, as well as parameterization of
 * objects and environmental variables. Bullet takes care of all calculations
 * and returns those results back to MATLAB.
 */

#pragma once

// Suppress Bullet warnings in GCC and Clang
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic pop

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

static std::vector<std::unique_ptr<Compound> > compounds_;
static std::vector<std::unique_ptr<bullet_shape> > shapes_;
static std::vector<std::unique_ptr<bullet_vehicle> > vehicles_; 
static std::vector<btTypedConstraint*> constraints_;

/// OPENGL STUFF
static int window = 0;
static std::vector<int> buffers_;
static int view_angle_ = 0;
static int view_elevation_ = 0;
static float fov_ = 55;
static float aspect_ratio_ = 1;
static float world_dim_ = 3.0;
static int light_move_ = 1;
static int light_angle_ = 90;
static float light_elevation_ = 2;
static int shader_program_;
/// OPENGL STUFF

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

/////////////////////////
// OPENGL STUFF
/////////////////////////

///////////////
////// Most of these are used to Init OpenGL

inline void gwDisplay(){
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glLoadIdentity();

  // TAKEN FROM HW 4
  const double len=2.0;  //  Length of axes
  //  Light position and colors
  float Ambient[]   = {1.0, 1.0, 1.0, 1.0};
  float Diffuse[]   = {0.1, 0.1, 0.1, 1.0};
  float Specular[]  = {0.6, 0.6, 0.6, 1.0};
  float Position[]  = {(float)(2 * std::cos(light_angle_)),
                       light_elevation_,
                       (float)(2*std::sin(light_angle_)),
                       1.0};
  //  Perspective - set eye position
  float Ex = -2 * world_dim_ * std::sin(view_angle_) * std::cos(view_elevation_);
  float Ey = +2 * world_dim_ * std::sin(view_elevation_);
  float Ez = +2 * world_dim_ * std::cos(view_angle_) * std::cos(view_elevation_);
  gluLookAt(Ex,Ey,Ez , 0,0,0 , 0, std::cos(view_elevation_),0);
  glColor3f(1,1,1);
  glPushMatrix();
  glTranslated(Position[0],Position[1],Position[2]);
  glutSolidSphere(0.03,10,10);
  glPopMatrix();

  // Use our Shader
  glUseProgram(shader_program_);

  /// Define our view matrices
  int loc;
  glm::mat4 ProjectionMatrix = glm::perspective<float>(fov_, aspect_ratio_,
                                                       world_dim_/16,
                                                       world_dim_*16);
  Ex = -2 * world_dim_ * std::sin(view_angle_)
    * std::cos(view_elevation_);
  Ey = +2 * world_dim_ * std::sin(view_elevation_);
  Ez = +2 * world_dim_ * std::cos(view_angle_) * std::cos(view_elevation_);
  glm::mat4 ViewMatrix = glm::lookAt(
                                     glm::vec3(Ex, Ey, Ez),
                                     glm::vec3(0, 0, 0),
                                     glm::vec3(0, std::cos(view_elevation_), 0));
  glm::mat4 NormalMatrix = glm::transpose(glm::inverse(ViewMatrix));

  loc = glGetUniformLocation(shader_program_, "light_position");
  if (loc>=0) glUniform3fv(loc, 1, Position);
  loc = glGetUniformLocation(shader_program_, "light_ambient");
  if (loc>=0) glUniform4fv(loc, 1, Ambient);
  loc = glGetUniformLocation(shader_program_, "light_diffuse");
  if (loc>=0) glUniform4fv(loc, 1, Diffuse);
  loc = glGetUniformLocation(shader_program_, "light_specular");
  if (loc>=0) glUniform4fv(loc, 1, Specular);
  loc = glGetUniformLocation(shader_program_, "ProjectionMatrix");
  if (loc>=0) glUniformMatrix4fv(loc, 1, GL_FALSE,
                                 glm::value_ptr(ProjectionMatrix));
  loc = glGetUniformLocation(shader_program_, "NormalMatrix");
  if (loc>=0) glUniformMatrix4fv(loc, 1, GL_FALSE,
                                 glm::value_ptr(NormalMatrix));

  // FOR ALL OF OUR OBJECTS
  // Hand-calculate View Matrices FOR EACH OBJECT
  // Get transformation matrix from bullet
  // Set to model matrix
  int numModels = 1;
  for (int i = 1; i < shapes_.size(); i++) {
    // TODO: SET CURRENT SHAPE
    std::unique_ptr<bullet_shape>& currentShape = shapes_[i];
    glm::mat4 ModelMatrix = glm::translate(glm::mat4(1.0f),
                                           glm::vec3(-5.0f, 0.0f, 0.0f));
    loc = glGetUniformLocation(shader_program_, "ModelViewMatrix");
    if (loc>=0) glUniformMatrix4fv(loc, 1, GL_FALSE,
                                   glm::value_ptr(ViewMatrix * ModelMatrix));

    /////////
    // DRAWING OUR SHAPES
    //  Select cube buffer
    // glBindBuffer(GL_ARRAY_BUFFER, cube_buffer);
    // Vertex array
    // DO THIS WHEN WE GET A NEW SHAPE

    glBindBuffer(GL_ARRAY_BUFFER, currentShape->vertex_buffer_);
    int XYZW = glGetAttribLocation(shader_program_,"Vertex");
    glEnableVertexAttribArray(XYZW);
    glVertexAttribPointer(XYZW, 4, GL_FLOAT, GL_FALSE, 0, 0);
    // Color array
    glBindBuffer(GL_ARRAY_BUFFER, currentShape->color_buffer_);

    int RGB = glGetAttribLocation(shader_program_,"Color");
    glEnableVertexAttribArray(RGB);
    glVertexAttribPointer(RGB, 3, GL_FLOAT, GL_FALSE, 0, 0);
    // Normal array
    glBindBuffer(GL_ARRAY_BUFFER, currentShape->normal_buffer_);
    int NORMAL = glGetAttribLocation(shader_program_,"Normal");
    glEnableVertexAttribArray(NORMAL);
    glVertexAttribPointer(NORMAL,3, GL_FLOAT, GL_FALSE, 0, 0);
    // Texture array? Not yet, but it goes here
    ///
    ///

    // Draw the cube
    // TODO: SET THIS BASED ON CURRENT SHAPE
    int numVertices = currentShape->vertex_data_.size();
    glDrawArrays(GL_TRIANGLES, 0, numVertices);

    //  Disable vertex arrays
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);

    //  Unbind this buffer
    glBindBuffer(GL_ARRAY_BUFFER,0);
  }


  // Back to fixed pipeline
  glUseProgram(0);

  //  Display parameters
  glWindowPos2i(5,5);
  //  Render the scene and make it visible
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glFlush();
  glutSwapBuffers();
  glutPostWindowRedisplay(window);
}

inline void gwReshape(int width, int height){
  //  Ratio of the width to the height of the window
  aspect_ratio_ = (height > 0) ? (double) width / height : 1;
  glViewport(0,0, width, height);
  //  Set projection
  Project(fov_, aspect_ratio_, world_dim_);
}

inline void gwSpecial(int key, int x, int y){
  //  Tell GLUT it is necessary to redisplay the scene
  glutPostRedisplay();
}

inline void gwKeyboard(unsigned char ch,int x,int y){}

inline void gwIdle(){
  double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
  if (light_move_) light_angle_ = fmod(90 * t, 360.0);
  glutPostRedisplay();
}
