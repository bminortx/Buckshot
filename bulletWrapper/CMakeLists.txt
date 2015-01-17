project( SimLAB )
set(PROJECT_NAME SimLAB)
set(LIBRARY_NAME ${PROJECT_NAME})

set(MEX /usr/local/MATLAB/R2014a/bin/mex)

cmake_minimum_required(VERSION 2.8)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 OLD)
endif(COMMAND cmake_policy)

###################
# LINK TO OUR LIBRARIES
###################
add_definitions(-DBT_USE_DOUBLE_PRECISION)
add_definitions(-DBT_NO_PROFILE)
set( SCENEGRAPH_INTERFACE false CACHE BOOL force )

find_package( Eigen3 REQUIRED )
find_package( Bullet REQUIRED )

set(USER_INC
	${EIGEN3_INCLUDE_DIRS}
	${BULLET_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR})

set(LINK_DIRS
  /usr/local/lib/)

set(LINK_LIBS
  ${BULLET_LIBRARIES}
	${EIGEN3_LIBRARIES})

# Are we using SceneGraph?
if(${SCENEGRAPH_INTERFACE})
	find_package(SceneGraph REQUIRED)
	find_package(Pangolin REQUIRED)
	set(USER_INC_OPT
		${SceneGraph_INCLUDE_DIRS}
		${Pangolin_INCLUDE_DIRS})
	set(LINK_LIBS_OPT
		${SceneGraph_LIBRARIES}
		${Pangolin_LIBRARIES})
endif()

include_directories(${USER_INC})
link_directories(${USER_INC})

set(HDRS
	class_handle.hpp
	bullet_entities.h
	matlab_bullet_wrapper.h)
set(SRC
	bullet_interface_mex.cpp)

set(MEX_TARGET bullet_interface_mex)
# Set the right output file
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	set(MEX_OUTPUT ${MEX_TARGET}.mexmaci64)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	set(MEX_OUTPUT ${MEX_TARGET}.mexglx)
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
message(${MEXINCLUDES})

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

set(MEX_COMMAND ${MEX} -silent -cxx -O ${MEXINCLUDES} -output ${MEX_OUTPUT} ${SRC} ${MEXLIBS} ${MEXLIBDIRS})

add_custom_command(OUTPUT ${MEX_OUTPUT}
  COMMAND ${MEX_COMMAND}
  DEPENDS ${HDRS} ${SRC}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  
add_custom_target(${MEX_TARGET} ALL
  DEPENDS ${MEX_OUTPUT})

