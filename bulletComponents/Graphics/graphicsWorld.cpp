#include "graphicsWorld.h"
#include <string.h>

GraphicsWorld::GraphicsWorld()
    : light_move_(1), view_angle_(0), view_elevation_(0), fov_(55),
      aspect_ratio_(1), world_dim_(3.0f), light_angle_(90),
      light_elevation_(2)
{}

void GraphicsWorld::Init() {
  //  Initialize GLUT
  char *argv [1];
  int argc = 1;
  argv[0] = strdup ("Buckshot");
  glutInit(&argc,argv);
  //  Request double buffered, true color window with Z buffering at 600x600
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutInitWindowSize(600,600);
  glutCreateWindow("Buckshot GUI");
#ifdef USEGLEW
  //  Initialize GLEW
  if (glewInit()!=GLEW_OK) Fatal("Error initializing GLEW\n");
  if (!GLEW_VERSION_4_3) Fatal("OpenGL 4.3 not supported\n");
#endif

  // TODO: CREATE FUNCTIONS
  //  Set callbacks
  glutDisplayFunc(GraphicsWorld::gwDisplay);
  glutReshapeFunc(GraphicsWorld::gwReshape);
  glutSpecialFunc(GraphicsWorld::gwSpecial);
  glutKeyboardFunc(GraphicsWorld::gwKeyboard);
  glutIdleFunc(GraphicsWorld::gwIdle);
  //  Load crate
  // crate = LoadTexBMP("pi.bmp");
  // //  Create Shader Programs
  // shader = CreateShaderProg("gl430.vert","gl430.frag");
  // //  Initialize cube
  // InitCube();
  // //  Pass control to GLUT so it can interact with the user
  // ErrCheck("init");

  /// TODO: INVESTIGATE THIS BEHAVIOR
  glutMainLoop();
}
