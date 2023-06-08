cmake_minimum_required(VERSION 3.0.2)
project(relax_reference_interpolation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  xbot_msgs
)

find_package(xbot2 REQUIRED)
find_package(Eigen3 REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(jointTrajInterpol
  src/joint_traj_interpol.cpp
)

target_link_libraries(jointTrajInterpol
  ${catkin_LIBRARIES}
)

install(TARGETS jointTrajInterpol
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
