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
#define PI 3.14159265359
#define TWOPI 6.28318530718

#include "Compound.h"
#include "../Graphics/graphicsWorld.h"
#include <map>
#include <vector>
#include <memory>
#include <iostream>
#include <thread>
#include <chrono>

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
static int window;
static float view_angle_ = 0;
static float view_elevation_ = 3;
static float fov_ = 55;
static float aspect_ratio_ = 1;
static float world_dim_ = 7.0;
static int light_move_ = 1;
static float light_angle_ = PI / 4;
static float light_elevation_ = 2;
#define MODE 2
static int shader_program_[MODE] = {0, 0};
static int mode = 0;
const float CrystalDensity=5.0;
const float CrystalSize=.15;

/// Key accessors to BulletWorld
static bool is_running_ = false;
static bool is_reset_ = false;
static bool is_iterating_ = false;
static bool is_drawing_constraints_ = false;
static bool quit_now_ = false;

/// OPENGL STUFF

class BulletWorld {
 public:
  BulletWorld();
  ~BulletWorld();

  void Reset();
  void UseOpenGL();

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
  void StepGUI();
  void RunSimulation();

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
  bool use_opengl_;
  btDefaultCollisionConfiguration  collision_configuration_;
  std::unique_ptr<btCollisionDispatcher> bt_dispatcher_;
  std::unique_ptr<btDbvtBroadphase> bt_broadphase_;
  std::unique_ptr<btSequentialImpulseConstraintSolver> bt_solver_;

  // Physics and Graphics worlds
  std::shared_ptr<btDiscreteDynamicsWorld> dynamics_world_;
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
  float Emission[]  = {0.6,0.6,0.6,1.0};
  float Ambient[]   = {0.8,0.8,0.8,1.0};
  float Diffuse[]   = {1.0,1.0,1.0,1.0};
  float Specular[]  = {1.0,1.0,1.0,1.0};
  float Shinyness[] = {16};
  float Position[]  = {(float)(2 * std::cos(light_angle_)),
                       light_elevation_,
                       (float)(2*std::sin(light_angle_)),
                       1.0};
  //  Perspective - set eye position
  float Ex = -2 * world_dim_ * std::sin(view_angle_) * std::cos(view_elevation_);
  float Ey = +2 * world_dim_ * std::sin(view_elevation_);
  float Ez = +2 * world_dim_ * std::cos(view_angle_) * std::cos(view_elevation_);
  gluLookAt(Ex, Ey, Ez, 0,0,0 , 0, std::cos(view_elevation_),0);
  glColor3f(1,1,1);
  glPushMatrix();
  glTranslated(Position[0],Position[1],Position[2]);
  glutSolidSphere(0.03,10,10);
  glPopMatrix();

  //  OpenGL should normalize normal vectors
  glEnable(GL_NORMALIZE);
  //  Enable lighting
  glEnable(GL_LIGHTING);
  //  glColor sets ambient and diffuse color materials
  glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  //  Enable light 0
  glEnable(GL_LIGHT0);

  if (mode != 0) {
    //  Set ambient, diffuse, specular components and position of light 0
    glLightfv(GL_LIGHT0,GL_AMBIENT ,Ambient);
    glLightfv(GL_LIGHT0,GL_DIFFUSE ,Diffuse);
    glLightfv(GL_LIGHT0,GL_SPECULAR,Specular);
    glLightfv(GL_LIGHT0,GL_POSITION,Position);
    //  Set materials
    glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS,Shinyness);
    glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,Specular);
    glMaterialfv(GL_FRONT_AND_BACK,GL_EMISSION,Emission);
  }
  
  // Use our Shader
  glUseProgram(shader_program_[mode]);
  int id;
  id = glGetUniformLocation(shader_program_[mode],"CrystalDensity");
  if (id>=0) glUniform1f(id,CrystalDensity);
  id = glGetUniformLocation(shader_program_[mode],"CrystalSize");
  if (id>=0) glUniform1f(id,CrystalSize);

  /////////
  // DRAWING OUR SHAPES
  for (std::unique_ptr<bullet_shape>& currentShape: shapes_) {
    btTransform world_transform =
        currentShape->rigidBodyPtr()->getCenterOfMassTransform();
    btMatrix3x3 rotation = world_transform.getBasis();
    btVector3 position = world_transform.getOrigin();
    float pose[] = {
      (float)rotation[0][0], (float)rotation[1][0], (float)rotation[2][0], 0,
      (float)rotation[0][1], (float)rotation[1][1], (float)rotation[2][1], 0,
      (float)rotation[0][2], (float)rotation[1][2], (float)rotation[2][2], 0,
      (float)position[0], (float)position[1], (float)position[2], 1
    };
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glMultMatrixf(pose);
    currentShape->getDrawData();
    glPopMatrix();
  }

  if (is_drawing_constraints_) {
    for (btTypedConstraint* cons: constraints_) {
      // TODO(bminortx): investigate this. Have to cast down for now...
      btHinge2Constraint* constraint =
        static_cast<btHinge2Constraint*>(cons);
      btVector3 position = constraint->getAnchor();
      glColor4f(0.2, 1, 0.2, 0.7);
      glPushMatrix();
      glTranslated(position[0], position[1], position[2]);
      glutSolidSphere(0.03,10,10);
      glPopMatrix();
    }
  }

  glUseProgram(0);
  glutPostRedisplay();

  //  Display parameters
  glWindowPos2i(5,5);
  glFlush();
  glutSwapBuffers();
}

inline void gwReshape(int width, int height){
  //  Ratio of the width to the height of the window
  aspect_ratio_ = (height > 0) ? (double) width / height : 1;
  glViewport(0,0, width, height);
  //  Set projection
  Project(fov_, aspect_ratio_, world_dim_);
}

inline void gwSpecial(int key, int x, int y){
  //  Right arrow key - increase angle by 5 degrees
  if (key == GLUT_KEY_RIGHT)
    view_angle_ += .05;
  //  Left arrow key - decrease angle by 5 degrees
  else if (key == GLUT_KEY_LEFT)
    view_angle_ -= .05;
  //  Up arrow key - increase elevation by 5 degrees
  else if (key == GLUT_KEY_UP)
    view_elevation_ += .05;
  //  Down arrow key - decrease elevation by 5 degrees
  else if (key == GLUT_KEY_DOWN)
    view_elevation_ -= .05;
  //  PageUp key - increase dim
  else if (key == GLUT_KEY_PAGE_DOWN)
    world_dim_ += 0.1;
  //  PageDown key - decrease dim
  else if (key == GLUT_KEY_PAGE_UP && world_dim_ > 1)
    world_dim_ -= 0.1;
  //  Keep angles to +/-360 degrees
  view_angle_ = fmod(view_angle_, TWOPI);
  view_elevation_ = fmod(view_elevation_, TWOPI);
  //  Update projection
  Project(fov_, aspect_ratio_, world_dim_);
  glutPostRedisplay();
}

inline void gwKeyboard(unsigned char ch,int x,int y){
  //  Exit on ESC
  if (ch == 27 || ch == 'q') {
    glutDestroyWindow(window);
    exit(0);
  }
  //  Reset view angle
  else if (ch == '0') {
    view_angle_ = 0;
    view_elevation_ = 3;
  } else if (ch == 'i') {
    is_iterating_ = true;
  } else if (ch == 32) {
    is_running_ = !is_running_;
    is_iterating_ = false;
  } else if (ch == 'r') {
    is_reset_ = true;
  } else if (ch == 'c') {
    is_drawing_constraints_ = !is_drawing_constraints_;
  }
  else if (ch == 'm' || ch == 'M')
    mode = ((mode + 1) % MODE);
  else if (ch == '+')
    light_elevation_ += 0.1;
  else if (ch == '-')
    light_elevation_ -= 0.1;
  else if (ch == '[')
    light_angle_ = fmod(light_angle_ - .05, TWOPI);
  else if (ch == ']')
    light_angle_ = fmod(light_angle_ + .05, TWOPI);
  Project(fov_, aspect_ratio_, world_dim_);
  glutPostRedisplay();
}

inline void gwIdle(){
  double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
  if (light_move_) light_angle_ = fmod(90 * t, 360.0);
  glutPostRedisplay();
}

// http://bit.ly/1NwkgiQ
inline void mouse(int button, int state, int x, int y)
{
  // Wheel reports as button 3(scroll up) and button 4(scroll down)
  if ((button == 3) || (button == 4)) // It's a wheel event
    {
      // Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
      if (state == GLUT_UP) return; // Disregard redundant GLUT_UP events
      if (button == 3){
        world_dim_ += 0.1;
      } else {
        if (world_dim_ > 1) world_dim_ -= 0.1;
      }
    } else {  // normal button event

  }
}

inline void Init() {
  char *argv [1];
  int argc = 1;
  argv[0] = strdup ("Buckshot");
  glutInit(&argc,argv);
  //  Request double buffered, true color window with Z buffering at 600x600
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(600,600);
  window = glutCreateWindow("Buckshot GUI");
#ifdef USEGLEW
  //  Initialize GLEW
  if (glewInit()!=GLEW_OK) Fatal("Error initializing GLEW\n");
  if (!GLEW_VERSION_4_3) Fatal("OpenGL 4.3 not supported\n");
#endif

  //  Set callbacks
  glutDisplayFunc(gwDisplay);
  glutReshapeFunc(gwReshape);
  glutSpecialFunc(gwSpecial);
  glutKeyboardFunc(gwKeyboard);
  glutMouseFunc(mouse);
  glutIdleFunc(gwIdle);
  // Load our shader programs
  shader_program_[1] = CreateShaderProg("bulletComponents/Graphics/gl430.vert",
                                        "bulletComponents/Graphics/gl430.frag");
}
