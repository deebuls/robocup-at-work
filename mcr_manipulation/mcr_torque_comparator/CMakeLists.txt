cmake_minimum_required(VERSION 2.8.3)
project(mcr_torque_comparator)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs mcr_manipulation_utils brics_actuator tf urdf nodelet std_msgs)
find_package(eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS  
    mcr_manipulation_utils
    nodelet
)

include_directories(
 ${PROJECT_SOURCE_DIR}/ros/include 
 ${EIGEN_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${orocos_kdl_LIBRARY_DIRS}
)

add_library(torque_comparator
  ros/src/torque_comparator.cpp)
    
target_link_libraries(torque_comparator
  ${catkin_LIBRARIES} 
  ${orocos_kdl_LIBRARIES}  
)

add_dependencies(torque_comparator 
  ${catkin_EXPORTED_TARGETS}
)

### INSTALL
install(TARGETS torque_comparator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY ros/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

install(FILES nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
