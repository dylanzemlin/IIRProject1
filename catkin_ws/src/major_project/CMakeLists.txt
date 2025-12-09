cmake_minimum_required(VERSION 3.0.2)
project(major_project)

## Use C++11 (optional but recommended)
# add_compile_options(-std=c++11)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    roslib
    rospy
    sensor_msgs
    std_msgs
    actionlib
    actionlib_msgs
    move_base_msgs
)

###########
## Build ##
###########

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## C++ node: src/major_project.cpp -> executable "major_project"
add_executable(major_project
  src/major_project.cpp
)

target_link_libraries(major_project
  ${catkin_LIBRARIES}
)

add_dependencies(major_project
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Install Python node: src/interface.py
## Make sure src/interface.py has a proper shebang and is chmod +x.
catkin_install_python(PROGRAMS
  src/interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

# (Add tests here if needed)
