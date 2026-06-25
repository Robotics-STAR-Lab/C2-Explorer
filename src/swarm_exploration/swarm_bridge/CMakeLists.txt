cmake_minimum_required(VERSION 3.0.2)
project(swarm_bridge)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  exploration_manager
  plan_env
  bspline
)

catkin_package()

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(bridge_node_udp
  src/bridge_node_udp.cpp 
)
add_dependencies(bridge_node_udp 
  ${catkin_EXPORTED_TARGETS}
  exploration_manager_generate_messages_cpp
  plan_env_generate_messages_cpp
  bspline_generate_messages_cpp
)
target_link_libraries(bridge_node_udp 
  ${catkin_LIBRARIES}
)
