cmake_minimum_required(VERSION 2.8.3)
project(exploration_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
# find_package(ortools REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  lkh_tsp_solver
  lkh_mtsp_solver
  plan_manage
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  DroneState.msg
  MeetingOpt.msg
  MeetingOptResponse.msg
  HGrid.msg
  GridTour.msg
  Rendezvous.msg
)

generate_messages(
  DEPENDENCIES
  # std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS plan_env path_searching bspline bspline_opt active_perception traj_utils lkh_tsp_solver lkh_mtsp_solver plan_manage message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# racer node
add_executable(exploration_node
  src/exploration_node.cpp 
  src/c2_exploration_fsm.cpp
  src/c2_exploration_manager.cpp
  )
target_link_libraries(exploration_node 
  ${catkin_LIBRARIES}
  # ortools::ortools
  )
