cmake_minimum_required(VERSION 2.8.3)
project(sparkfun_9dof_razor_imu_m0)

## Add support for C++11, supported in ROS Kinetic and newer
#add_definitions(-std=c++11)

find_package(Boost REQUIRED COMPONENTS regex)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  sensor_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet pluginlib roscpp sensor_msgs serial
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# node executable
add_executable(driver_node
  src/driver.cpp
  src/node.cpp)
add_dependencies(driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(driver_node
  ${catkin_LIBRARIES}
)

# nodelet library
add_library(sparkfun_9dof_razor_imu_m0_driver_nodelet
  src/driver.cpp
  src/nodelet.cpp)
add_dependencies(sparkfun_9dof_razor_imu_m0_driver_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(sparkfun_9dof_razor_imu_m0_driver_nodelet
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS sparkfun_9dof_razor_imu_m0_driver_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# TODO
