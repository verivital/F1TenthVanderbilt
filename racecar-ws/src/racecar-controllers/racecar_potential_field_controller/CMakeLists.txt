cmake_minimum_required(VERSION 2.8.3)
project(racecar_potential_field_controller)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  nodelet
  pluginlib
  dynamic_reconfigure
  roscpp
  sensor_msgs
  ackermann_msgs
  visualization_msgs
)

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
  config/RacecarPotentialFieldController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    nodelet
    pluginlib
    dynamic_reconfigure
    roscpp
    sensor_msgs
    ackermann_msgs
    visualization_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# node executable
add_executable(node
  src/node.cpp
  src/controller.cpp
)
add_dependencies(node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(node
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

# nodelet library
add_library(racecar_potential_field_controller_nodelet
  src/nodelet.cpp
  src/controller.cpp
)
add_dependencies(racecar_potential_field_controller_nodelet
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(racecar_potential_field_controller_nodelet
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS racecar_potential_field_controller_nodelet
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
