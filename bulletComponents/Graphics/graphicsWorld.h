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

/// OpenGL Headers
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
// All of our shapes to draw
#include "Compound.h"
// Std lib
#include <cmath>
#include <stdio.h>
#include <vector>

class GraphicsWorld {
 public:
  GraphicsWorld();
  void Init(int& window,
            int& shader_program_,
            void (*gwDisplay)(),
            void (*gwReshape)(int, int),
            void (*gwSpecial)(int, int, int),
            void (*gwKeyboard)(unsigned char, int, int),
            void (*gwIdle)());
  int AddShape(std::unique_ptr<bullet_shape>& currentShape);
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
