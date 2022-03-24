cmake_minimum_required(VERSION 2.8.3)
project(multiagent_slam)

set(PROJ_DEPS roscpp std_msgs message_generation roslib rosbag sensor_msgs nav_msgs tf)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS ${PROJ_DEPS})

## Declare ROS messages and services
# add_message_files(FILES mystr.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)


## Declare a catkin package

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}_core
	CATKIN_DEPENDS ${PROJ_DEPS}
	)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_core
	lib/map2D.cpp
)

#target_link_libraries($(PROJECT_NAME}_core
#  ${catkin_LIBRARIES}
#	)
catkin_install_python(PROGRAMS
    scripts/carmen2rosbag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Build executables
add_executable(create_rosbag scripts/create_rosbag.cpp)
target_link_libraries(create_rosbag ${catkin_LIBRARIES})
add_dependencies(create_rosbag ${PROJ_DEPS})

add_executable(publish_lidar scripts/carmen_gaussian.cpp)
target_link_libraries(publish_lidar ${catkin_LIBRARIES} ${PROJECT_NAME}_core)
add_dependencies(publish_lidar ${PROJ_DEPS})




