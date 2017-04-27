cmake_minimum_required (VERSION 2.8)
project (HexPlanner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
 ########## include_directories ###############
 if(UNIX)
include_directories("/usr/Aris_Dependent" )
 endif(UNIX)

if(WIN32)
include_directories("E:/HexCodes/eigen" )
endif(WIN32)
######### build project ###############
add_executable(main main.cpp Dynamics.cpp DynamicsBase.cpp DynamicsModelBase.cpp  solver.cpp matrix_support.cpp util.cpp ldl.cpp Planner.cpp)
