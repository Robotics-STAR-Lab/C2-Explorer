cmake_minimum_required(VERSION 2.8.3)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  bspline
  bspline_opt
  path_searching
  poly_traj
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_utils
 CATKIN_DEPENDS path_searching bspline bspline_opt poly_traj
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

add_library( traj_utils
    src/planning_visualization.cpp 
    )
target_link_libraries( traj_utils
    ${catkin_LIBRARIES} 
    )  
