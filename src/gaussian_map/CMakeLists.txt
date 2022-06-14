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
	lib/gpModel.cpp
	lib/dataloader.cpp
	lib/robot.cpp
	lib/slam.cpp
)
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${field_type}) 

## Build executables
# add_executable(gp_map scripts/ros_gaussian.cpp)
# target_link_libraries(gp_map ${catkin_LIBRARIES} ${PROJECT_NAME}_core)
# add_dependencies(gp_map ${PROJ_DEPS})

add_executable(gp_octree src/main.cpp)
target_link_libraries(gp_octree  ${catkin_LIBRARIES} ${PROJECT_NAME}_core )
# add_dependencies(gp_octree ${PROJ_DEPS})



#add_executable(new_gaussian scripts/carmen_gaussian.cpp)
#target_link_libraries(new_gaussian ${catkin_LIBRARIES} ${PROJECT_NAME}_core)
#add_dependencies(new_gaussian ${PROJ_DEPS})

install(TARGETS gp_octree
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



