cmake_minimum_required(VERSION 2.8.3)
project(descartes_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  trajectory_msgs
  visualization_msgs
  moveit_ros_planning_interface
  rosbag
)

catkin_package()

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_tutorial1 src/tutorial1.cpp)
 add_executable(${PROJECT_NAME}_robot src/robot.cpp src/trajvis.cpp)
 add_executable(${PROJECT_NAME}_robot_kr120 src/robot_kr120.cpp src/trajvis.cpp)

 target_link_libraries(${PROJECT_NAME}_tutorial1
   ${catkin_LIBRARIES}
 )
 
 target_link_libraries(${PROJECT_NAME}_robot
   ${catkin_LIBRARIES}
   utilities
 )
 
  target_link_libraries(${PROJECT_NAME}_robot_kr120
   ${catkin_LIBRARIES}
   utilities
 )

set_target_properties(${PROJECT_NAME}_tutorial1
  PROPERTIES OUTPUT_NAME tutorial1
  PREFIX "")

set_target_properties(${PROJECT_NAME}_robot
  PROPERTIES OUTPUT_NAME robot
  PREFIX "")
  
set_target_properties(${PROJECT_NAME}_robot_kr120
  PROPERTIES OUTPUT_NAME robot_kr120
  PREFIX "")

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_tutorial1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_robot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

add_library(utilities
  include/descartes_tutorials/utilities.h
  src/utilities.cpp
)
add_dependencies(utilities ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(utilities
  ${catkin_LIBRARIES}
)


add_executable(test_bits
  src/test_bits.cpp
)
add_dependencies(test_bits ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_bits
  ${catkin_LIBRARIES}
)