cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)

set(NLOPT_INCLUDE_DIR "/usr/local/include")  
set(NLOPT_LIBRARY "/usr/local/lib/libnlopt.so")  

find_package(catkin REQUIRED COMPONENTS
  roscpp
  plan_env
  active_perception
  bspline
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline_opt
 CATKIN_DEPENDS roscpp plan_env active_perception bspline
)


include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${NLOPT_INCLUDE_DIR}  
)

add_library( bspline_opt 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES} 
    ${NLOPT_LIBRARY}   
    )
