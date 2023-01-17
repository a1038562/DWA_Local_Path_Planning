cmake_minimum_required(VERSION 3.0.2)
project(dwa_local_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED COMPONENTS system)

add_message_files(
  FILES
  local_path.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dwa_local_planner
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf
)

file(GLOB_RECURSE project_HEADERS
  "*.h"
  "*.hpp"
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME} 
  src/path_planning.cpp
  src/main.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)