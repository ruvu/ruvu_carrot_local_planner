# Copyright 2020 RUVU Robotics B.V.

cmake_minimum_required(VERSION 3.0.2)
project(ruvu_carrot_local_planner)

add_compile_options(-std=c++11)
set(EXTRA_FLAGS -Wno-unused-private-field) # required for costmap_2d.h

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  dynamic_reconfigure
  mbf_costmap_core
  nav_msgs
  pluginlib
  roscpp
  ruvu_lint
  tf
)

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/CarrotPlanner.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    dynamic_reconfigure
    nav_msgs
    roscpp
    tf
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/carrot_planner.cpp src/carrot_planner.hpp
  src/carrot_planner_ros.cpp src/carrot_planner_ros.hpp
  src/geometry_utils.hpp
  src/latched_stop_rotate_controller.cpp src/latched_stop_rotate_controller.hpp
  src/local_planner_limits.cpp src/local_planner_limits.hpp
  src/local_planner_util.cpp src/local_planner_util.hpp
  src/parameter_magic.hpp
  src/simulator.cpp src/simulator.hpp
  src/utils.cpp src/utils.hpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_warnings(${PROJECT_NAME} ${EXTRA_FLAGS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-build/include_what_you_use,-whitespace/braces,-runtime/references")
ruvu_lint_add_test()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_utils test/test_utils.cpp)
  target_compile_warnings(test_utils)
  target_link_libraries(test_utils ${catkin_LIBRARIES})

  catkin_add_gtest(geometry_utils test/test_geometry_utils.cpp)
  target_compile_warnings(geometry_utils)
  target_link_libraries(geometry_utils ${catkin_LIBRARIES})
endif()
