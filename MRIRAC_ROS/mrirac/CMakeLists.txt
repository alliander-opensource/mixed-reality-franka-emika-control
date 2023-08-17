cmake_minimum_required(VERSION 3.0.2)
project(mrirac)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  franka_gripper
  moveit_ros_planning_interface
  mrirac_msgs
  mrirac_lib
  franka_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(trajectory_planner_node_Fr3 src/trajectory_planner_node_Fr3.cpp)
add_executable(recovery_node src/recovery_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(trajectory_planner_node_Fr3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(recovery_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(trajectory_planner_node_Fr3
  ${catkin_LIBRARIES}
)
target_link_libraries(recovery_node
  ${catkin_LIBRARIES}
)
