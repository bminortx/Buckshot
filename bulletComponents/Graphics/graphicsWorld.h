/**
 * GraphicsEngine: The home of all OpenGL implementation
 * Some Inspiration from Adv. Graphics class at CU Boulder
 */
#pragma once

#ifdef USEGLEW
#include <GL/glew.h>
#endif
#define GL_GLEXT_PROTOTYPES

/// TODO: Make this optional
#include <GL/freeglut.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <stdio.h>
#include <vector>


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

class GraphicsWorld {
 public:
  GraphicsWorld();

  void Init();
  int AddShapes();
  int AddConstraints();
  void stepSimulation();
};

// Borrowed from Advanced Graphics
inline void Project(double fov,double asp,double dim)
{
  //  Tell OpenGL we want to manipulate the projection matrix
  glMatrixMode(GL_PROJECTION);
  //  Undo previous transformations
  glLoadIdentity();
  //  Perspective transformation
  if (fov)
    gluPerspective(fov,asp,dim/16,16*dim);
  //  Orthogonal transformation
  else
    glOrtho(-asp*dim,asp*dim,-dim,+dim,-dim,+dim);
  //  Switch to manipulating the model matrix
  glMatrixMode(GL_MODELVIEW);
  //  Undo previous transformations
  glLoadIdentity();
}


//////////
// LOADING SHADERS

static char* ReadText(const char *file)
{
  int   n;
  char* buffer;
  //  Open file
  FILE* f = fopen(file,"rt");
  // if (!f) Fatal("Cannot open text file %s\n",file);
  //  Seek to end to determine size, then rewind
  fseek(f,0,SEEK_END);
  n = ftell(f);
  rewind(f);
  //  Allocate memory for the whole file
  buffer = (char*)malloc(n+1);
  // if (!buffer) Fatal("Cannot allocate %d bytes for text file %s\n",n+1,file);
  //  Snarf the file
  // if (fread(buffer,n,1,f)!=1) Fatal("Cannot read %d bytes for text file %s\n",n,file);
  buffer[n] = 0;
  //  Close and return
  fclose(f);
  return buffer;
}

//
//  Print Shader Log
//
static void PrintShaderLog(int obj,const char* file)
{
  int len=0;
  glGetShaderiv(obj,GL_INFO_LOG_LENGTH,&len);
  if (len>1)
  {
    int n=0;
    char* buffer = (char *)malloc(len);
    // if (!buffer) Fatal("Cannot allocate %d bytes of text for shader log\n",len);
    glGetShaderInfoLog(obj,len,&n,buffer);
    fprintf(stderr,"%s:\n%s\n",file,buffer);
    free(buffer);
  }
  glGetShaderiv(obj,GL_COMPILE_STATUS,&len);
  // if (!len) Fatal("Error compiling %s\n",file);
}

//
//  Print Program Log
//
static void PrintProgramLog(int obj)
{
  int len=0;
  glGetProgramiv(obj,GL_INFO_LOG_LENGTH,&len);
  if (len>1)
  {
    int n=0;
    char* buffer = (char *)malloc(len);
    // if (!buffer) Fatal("Cannot allocate %d bytes of text for program log\n",len);
    glGetProgramInfoLog(obj,len,&n,buffer);
    fprintf(stderr,"%s\n",buffer);
  }
  glGetProgramiv(obj,GL_LINK_STATUS,&len);
  // if (!len) Fatal("Error linking program\n");
}

//
//  Create Shader
//
static void CreateShader(int prog,const GLenum type,const char* file)
{
  //  Create the shader
  int shader = glCreateShader(type);
  //  Load source code from file
  char* source = ReadText(file);
  glShaderSource(shader,1,(const char**)&source,NULL);
  free(source);
  //  Compile the shader
  glCompileShader(shader);
  //  Check for errors
  PrintShaderLog(shader,file);
  //  Attach to shader program
  glAttachShader(prog,shader);
}

//
//  Create Shader Program
//
static int CreateShaderProg(const char* VertFile,const char* FragFile)
{
  //  Create program
  int prog = glCreateProgram();
  //  Create and compile vertex shader
  if (VertFile) CreateShader(prog,GL_VERTEX_SHADER,VertFile);
  //  Create and compile fragment shader
  if (FragFile) CreateShader(prog,GL_FRAGMENT_SHADER,FragFile);
  //  Link program
  glLinkProgram(prog);
  //  Check for errors
  PrintProgramLog(prog);
  //  Return name
  return prog;
}


///////////////
////// Most of these are used to Init OpenGL

inline void gwDisplay(){

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

  //  Erase The Window and the depth buffer
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  //  Enable Z-buffering in OpenGL
  glEnable(GL_DEPTH_TEST);

  //  Undo previous transformations
  glLoadIdentity();
  //  Perspective - set eye position
  float Ex = -2 * world_dim_ * std::sin(view_angle_) * std::cos(view_elevation_);
  float Ey = +2 * world_dim_ * std::sin(view_elevation_);
  float Ez = +2 * world_dim_ * std::cos(view_angle_) * std::cos(view_elevation_);
  gluLookAt(Ex,Ey,Ez , 0,0,0 , 0, std::cos(view_elevation_),0);
  //  Draw light position as sphere
  //  This uses the fixed pipeline
  glColor3f(1,1,1);
  glPushMatrix();
  glTranslated(Position[0],Position[1],Position[2]);
  glutSolidSphere(0.03,10,10);
  glPopMatrix();

  // Use our shader_index_
  glUseProgram(shader_program_);

  // Hand-calculate matrices based on project()
  // ModelMatrix is set to Identity, so nothing needed there.
  glm::mat4 ProjectionMatrix;
  ProjectionMatrix = glm::ortho<float>(-aspect_ratio_*world_dim_, aspect_ratio_*world_dim_, -world_dim_, world_dim_, -4 * world_dim_,
                                       4 * world_dim_);
  int loc;
  ProjectionMatrix = glm::perspective<float>(fov_, aspect_ratio_, world_dim_/16, world_dim_*16);
  Ex = -2 * world_dim_ * std::sin(view_angle_)
       * std::cos(view_elevation_);
  Ey = +2 * world_dim_ * std::sin(view_elevation_);
  Ez = +2 * world_dim_ * std::cos(view_angle_) * std::cos(view_elevation_);
  glm::mat4 ViewMatrix = glm::lookAt(glm::vec3(Ex, Ey, Ez),
                                     glm::vec3(0, 0, 0),
                                     glm::vec3(0, std::cos(view_elevation_), 0));
  glm::mat4 NormalMatrix = glm::transpose(glm::inverse(ViewMatrix));
  loc = glGetUniformLocation(shader_program_, "ModelViewMatrix");
  if (loc>=0) glUniformMatrix4fv(loc, 1, GL_FALSE,
                                 glm::value_ptr(ViewMatrix));
  loc = glGetUniformLocation(shader_program_, "ProjectionMatrix");
  if (loc>=0) glUniformMatrix4fv(loc, 1, GL_FALSE,
                                 glm::value_ptr(ProjectionMatrix));
  loc = glGetUniformLocation(shader_program_, "NormalMatrix");
  if (loc>=0) glUniformMatrix4fv(loc, 1, GL_FALSE,
                                 glm::value_ptr(NormalMatrix));
  loc = glGetUniformLocation(shader_program_, "light_position");
  if (loc>=0) glUniform3fv(loc, 1, Position);
  loc = glGetUniformLocation(shader_program_, "light_ambient");
  if (loc>=0) glUniform4fv(loc, 1, Ambient);
  loc = glGetUniformLocation(shader_program_, "light_diffuse");
  if (loc>=0) glUniform4fv(loc, 1, Diffuse);
  loc = glGetUniformLocation(shader_program_, "light_specular");
  if (loc>=0) glUniform4fv(loc, 1, Specular);

  //  Select cube buffer
  // glBindBuffer(GL_ARRAY_BUFFER, cube_buffer);

  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);
  glEnableVertexAttribArray(2);
  glEnableVertexAttribArray(3);
  //   Attribute 0: vertex coordinate (vec4) at offset 0
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 12*sizeof(float),
                        (void*)0);
  //   Attribute 1:  vertex normal (vec3) offset 4 floats
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 12*sizeof(float),
                        (void*)(4*sizeof(float)));
  //   Attribute 2:  vertex color (vec3) offset 7 floats
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 12*sizeof(float),
                        (void*)(7*sizeof(float)));
  //   Attribute 3:  vertex texture (vec2) offset 10 floats
  glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 12*sizeof(float),
                        (void*)(10*sizeof(float)));

  // Draw the cube
  // glDrawArrays(GL_TRIANGLES,0,cube_size);

  //  Disable vertex arrays
  glDisableVertexAttribArray(0);
  glDisableVertexAttribArray(1);
  glDisableVertexAttribArray(2);
  glDisableVertexAttribArray(3);

  //  Unbind this buffer
  glBindBuffer(GL_ARRAY_BUFFER,0);

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
