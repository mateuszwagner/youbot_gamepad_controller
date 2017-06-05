cmake_minimum_required(VERSION 2.8.3)
project(youbot_controller)

find_package(catkin REQUIRED COMPONENTS youbot_driver geometry_msgs joy roscpp brics_actuator)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package( CATKIN_DEPENDS youbot_driver roscpp brics_actuator geometry_msgs)


###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

# THIS SHOULD NOT BE NEEDED AS WE INCLUDED THE 'include' FOLDER
#add_executable(youbot_project src/youbot_project.cpp include/arm_controller_node.h include/arm.h)
add_executable(youbot_project src/youbot_project.cpp)
add_dependencies(youbot_project ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(youbot_project ${catkin_LIBRARIES})
