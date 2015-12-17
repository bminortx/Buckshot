/**
 * GraphicsEngine: The home of all OpenGL implementation
 * Some Inspiration from Adv. Graphics class at CU Boulder
 */
#pragma once

// All of our shapes to draw
#include "Compound.h"
// Std lib
#include <cmath>
#include <stdio.h>
#include <vector>

inline void glFatal(const char* format , ...)
{
  va_list args;
  va_start(args,format);
  vfprintf(stderr,format,args);
  va_end(args);
  exit(1);
}

// Borrowed from Advanced Graphics
inline void Project(double fov,double asp,double dim)
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(fov,asp,dim/16,16*dim);
  glMatrixMode(GL_MODELVIEW);
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
  if (!f) glFatal("Cannot open text file %s\n",file);
  //  Seek to end to determine size, then rewind
  fseek(f,0,SEEK_END);
  n = ftell(f);
  rewind(f);
  //  Allocate memory for the whole file
  buffer = (char*)malloc(n+1);
  if (!buffer) glFatal("Cannot allocate %d bytes for text file %s\n",n+1,file);
  //  Snarf the file
  if (fread(buffer,n,1,f)!=1) glFatal("Cannot read %d bytes for text file %s\n",n,file);
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
    if (!buffer) glFatal("Cannot allocate %d bytes of text for shader log\n",len);
    glGetShaderInfoLog(obj,len,&n,buffer);
    fprintf(stderr,"%s:\n%s\n",file,buffer);
    free(buffer);
  }
  glGetShaderiv(obj,GL_COMPILE_STATUS,&len);
  if (!len) glFatal("Error compiling %s\n",file);
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
    if (!buffer) glFatal("Cannot allocate %d bytes of text for program log\n",len);
    glGetProgramInfoLog(obj,len,&n,buffer);
    fprintf(stderr,"%s\n",buffer);
  }
  glGetProgramiv(obj,GL_LINK_STATUS,&len);
  if (!len) glFatal("Error linking program\n");
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
  // //  Create and compile fragment shader
  if (FragFile) CreateShader(prog,GL_FRAGMENT_SHADER,FragFile);
  //  Link program
  glLinkProgram(prog);
  //  Check for errors
  PrintProgramLog(prog);
  //  Return name
  return prog;
}
