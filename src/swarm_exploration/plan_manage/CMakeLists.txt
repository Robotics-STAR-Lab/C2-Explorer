cmake_minimum_required(VERSION 2.8.3)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  poly_traj
  plan_env
  path_searching
  bspline
  bspline_opt
  active_perception
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATKIN_DEPENDS plan_env path_searching bspline bspline_opt active_perception)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_library(plan_manage 
  src/planner_manager.cpp
  )
target_link_libraries(plan_manage ${catkin_LIBRARIES})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

# benchmark node
# add_executable(compare_topo
#   src/compare_topo.cpp 
#   src/planner_manager.cpp
#   )
# target_link_libraries(compare_topo 
#   ${catkin_LIBRARIES}
#   )

# add_executable(send_cmd
#     src/send_cmd.cpp
# )
# target_link_libraries( send_cmd
#     ${catkin_LIBRARIES} 
#     )  
