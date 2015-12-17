###################
# USER INPUT
# might have to change these, depending on your system.
###################

set(MEX /usr/local/MATLAB/R2015b/bin/mex)
set(BULLET_PRECISION -DBT_USE_DOUBLE_PRECISION)
set(CMAKE_BUILD_TYPE RELEASE)

###################
# Project boilerplate
###################

project( Buckshot )
set(PROJECT_NAME Buckshot)
set(LIBRARY_NAME ${PROJECT_NAME})
cmake_minimum_required(VERSION 2.8)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 OLD)
endif(COMMAND cmake_policy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

###################
# LINK TO OUR LIBRARIES
###################
find_package( Bullet REQUIRED )
add_definitions(${BULLET_PRECISION})
add_definitions(-DBT_NO_PROFILE)
# TODO: MAKE THIS NOT REQUIRED
find_package( OpenGL )
find_package( GLUT )
find_package( FREEGLUT )


set(USER_INC
  ${BULLET_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}  
  ${FREEGLUT_INCLUDE_DIRS}  
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/bulletShapes
  ${CMAKE_CURRENT_SOURCE_DIR}/Graphics
  ${CMAKE_CURRENT_BINARY_DIR})
set(LINK_DIRS
  /usr/local/lib/)
set(LINK_LIBS
  ${BULLET_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY}
  ${FREEGLUT_LIBRARY}  
  )
include_directories(${USER_INC})
link_directories(${USER_INC})

################
# MATLAB MEX Files
set(HDRS
  class_handle.hpp
  Compound.h
  bulletWorld.h
  Graphics/graphicsWorld.h)
set(SRC
  buckshot.cpp
  bulletWorld.cpp)

################
# Bullet tester files
set(TEST_HDRS
  bulletShapes/bullet_cube.h
  bulletShapes/bullet_cylinder.h
  bulletShapes/bullet_heightmap.h
  bulletShapes/bullet_mesh.h
  bulletShapes/bullet_shape.h
  bulletShapes/bullet_sphere.h
  bulletShapes/bullet_vehicle.h
  Compound.h
  bulletWorld.h
  Graphics/graphicsWorld.h)
set(TEST_SRC
  bulletWorld.cpp)

###################
# BULLET TESTER
###################

add_executable(BulletTester maintest.cpp ${TEST_SRC} ${TEST_HDRS})
target_link_libraries(BulletTester ${LINK_LIBS})

###################
# MEX CONFIGURATION
###################
set(MEX_CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/mexopts.sh)
set(MEX_TARGET buckshot)
# Apple
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(MEX_OUTPUT ${MEX_TARGET}.mexmaci64)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

## 64-bit Linux
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(MEX_OUTPUT ${MEX_TARGET}.mexa64)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")


set(MEXINCLUDES)
set(MEXLIBS)
set(MEXLIBDIRS)
foreach(INC_DIR ${USER_INC})
  set(MEXINCLUDES "${MEXINCLUDES} -I${INC_DIR}")
endforeach()
foreach(INC_DIR_OPT ${USER_INC_OPT})
  set(MEXINCLUDES "${MEXINCLUDES} -I${INC_DIR_OPT}")
endforeach()
separate_arguments(MEXINCLUDES)
# message(${MEXINCLUDES})

foreach(DIR ${LINK_DIRS})
  set(MEXLIBDIRS "${MEXINCLUDES} -L${DIR}")
endforeach()
separate_arguments(MEXLIBDIRS)

foreach(DIR ${LINK_LIBS})
  set(MEXLIBS "${MEXLIBS} ${DIR}")
Endforeach()
foreach(DIR_OPT ${LINK_LIBS_OPT})
  set(MEXLIBS "${MEXLIBS} -L${DIR_OPT}")
Endforeach()
separate_arguments(MEXLIBS)
message(${MEXLIBS})

###################
# MEX COMPILATION
###################

set(MEX_COMMAND ${MEX} -f ${MEX_CONFIG} -silent -cxx ${BULLET_PRECISION} -O ${MEXINCLUDES} -output
  ${MEX_OUTPUT} ${SRC} ${MEXLIBS} ${MEXLIBDIRS})

add_custom_command(OUTPUT ${MEX_OUTPUT}
  COMMAND ${MEX_COMMAND}
  DEPENDS ${HDRS} ${SRC}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_target(${MEX_TARGET} ALL
  DEPENDS ${MEX_OUTPUT})
