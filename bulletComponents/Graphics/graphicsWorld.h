/**
 * GraphicsEngine: The home of all OpenGL implementation
 * Some Inspiration from Adv. Graphics class at CU Boulder
 */
#pragma once

#ifdef USEGLEW
#include <GL/glew.h>
#endif
#define GL_GLEXT_PROTOTYPES
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// #include <glm/glm.hpp>
// #include <glm/vec3.hpp> // glm::vec3
// #include <glm/vec4.hpp> // glm::vec4
// #include <glm/mat4x4.hpp> // glm::mat4
// #include <glm/gtc/type_ptr.hpp>
// #include <glm/gtc/matrix_transform.hpp>

class GraphicsWorld {
 public:
  GraphicsWorld();

  void Init();

  int AddShapes();
  int AddConstraints();

 private:
  // Most of these are used to Init OpenGL
  static void gwDisplay(){
    glFlush();
  }

  static void gwReshape(int width, int height){
    //  Ratio of the width to the height of the window
    // aspect_ratio_ = (height > 0) ? (double) width / height : 1;
    // glViewport(0,0, width, height);
    // // //  Set projection
    // Project(fov_, aspect_ratio_, world_dim_);
  }

  static void gwSpecial(int key, int x, int y){
    //  Tell GLUT it is necessary to redisplay the scene
    glutPostRedisplay();
  }

  static void gwKeyboard(unsigned char ch,int x,int y){}

  static void gwIdle(){
    // double t = glutGet(GLUT_ELAPSED_TIME)/1000.0;
    // if (move) zh = fmod(90*t,360.0);
    // glutPostRedisplay();
  }

  int view_angle_;
  int view_elevation_;
  float fov_;
  float aspect_ratio_;
  float world_dim_;
  int light_move_;
  int light_angle_;
  float light_elevation_;
  int shader_program_;
};
