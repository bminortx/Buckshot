/**
 * GraphicsEngine: The home of all OpenGL implementation
 */

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

#include <bulletWorld.h>

class GraphicsWorld {
 public:
  GraphicsWorld();

  void Init();

  int AddShapes();
  int AddConstraints();

 private:
  // Most of these are used to Init OpenGL
  static void  gwDisplay(){}

  static  void  gwReshape(int width, int height){}

  static  void  gwSpecial(int key, int x, int y){}

  static  void  gwKeyboard(unsigned char ch,int x,int y){}

  static  void  gwIdle(){}
};
