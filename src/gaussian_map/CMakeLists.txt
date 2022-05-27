cmake_minimum_required(VERSION 2.8.3)
project(gaussian_map)

set(PROJ_DEPS roscpp std_msgs message_generation roslib rosbag sensor_msgs nav_msgs tf)
set(field_type SE_FIELD_TYPE=SDF)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS ${PROJ_DEPS})
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)


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
	lib/gpModel.cpp
	lib/robot2D.cpp
	lib/Gaussian_pub.cpp
	lib/dataloader.cpp
	lib/robot.cpp
)

#catkin_install_python(PROGRAMS
#    scripts/carmen2rosbag.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# For linking libraries  with PROJ_DEPS
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)


## Build libraries
add_library(slam_lib  lib/slam.cpp)
target_link_libraries(slam_lib ${PROJECT_NAME}_core)
target_compile_definitions(slam_lib PUBLIC ${field_type}) 


## Build executables
# add_executable(gp_map scripts/ros_gaussian.cpp)
# target_link_libraries(gp_map ${catkin_LIBRARIES} ${PROJECT_NAME}_core)
# add_dependencies(gp_map ${PROJ_DEPS})

# add_executable(create_rosbag scripts/create_rosbag.cpp)
# target_link_libraries(create_rosbag ${catkin_LIBRARIES})
# add_dependencies(create_rosbag ${PROJ_DEPS})



#add_executable(new_gaussian scripts/carmen_gaussian.cpp)
#target_link_libraries(new_gaussian ${catkin_LIBRARIES} ${PROJECT_NAME}_core)
#add_dependencies(new_gaussian ${PROJ_DEPS})




