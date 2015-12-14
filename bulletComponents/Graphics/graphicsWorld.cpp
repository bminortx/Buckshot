#include "graphicsWorld.h"
#include <string.h>

GraphicsWorld::GraphicsWorld() {
}

int GraphicsWorld::AddShape(std::unique_ptr<bullet_shape>& currentShape) {
  unsigned int buffers[3];
  glGenBuffers(3, buffers);
  currentShape->vertex_buffer_ = buffers[0];
  currentShape->color_buffer_ = buffers[1];
  currentShape->normal_buffer_ = buffers[2];
  glBindBuffer(GL_ARRAY_BUFFER, currentShape->vertex_buffer_);
  // NOT SURE THIS BUFFER COPY IS CORRECT...
  glBufferData(GL_ARRAY_BUFFER, sizeof(currentShape->vertex_data_),
               &currentShape->vertex_data_[0], GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, currentShape->color_buffer_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(currentShape->color_data_),
               &currentShape->color_data_[0], GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, currentShape->normal_buffer_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(currentShape->normal_data_),
               &currentShape->normal_data_[0], GL_STATIC_DRAW);
  return 0;
}

void GraphicsWorld::Init(int& window,
                         int& shader_program_,
                         void (*gwDisplay)(),
                         void (*gwReshape)(int, int),
                         void (*gwSpecial)(int, int, int),
                         void (*gwKeyboard)(unsigned char, int, int),
                         void (*gwIdle)()) {
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
  glutIdleFunc(gwIdle);

  // Load our shader programs
  shader_program_ = CreateShaderProg(
      "/home/replica/GitMisc/personal_repos/Buckshot/bulletComponents/Graphics/gl430.vert",
      "/home/replica/GitMisc/personal_repos/Buckshot/bulletComponents/Graphics/gl430.frag");
}

void GraphicsWorld::stepSimulation() {
  glutMainLoopEvent();
}
