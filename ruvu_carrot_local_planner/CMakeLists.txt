cmake_minimum_required(VERSION 2.8.3)
project(ruvu_carrot_local_planner)

add_compile_options(-std=c++11)

# https://stackoverflow.com/questions/5088460/flags-to-enable-thorough-and-verbose-g-warnings
set(FLAGS -Wall -Wextra -Wno-ignored-qualifiers -Wcast-align -Wdisabled-optimization -Wformat=2 -Winit-self -Wnoexcept -Wredundant-decls -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=1 -Wswitch-default -Wtype-limits -Werror -Wno-unused)

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

add_library(ruvu_carrot_local_planner
  src/carrot_planner_ros.cpp src/carrot_planner_ros.h
  src/carrot_planner.cpp src/carrot_planner.h
  src/simulator.cpp src/simulator.h
  src/local_planner_util.cpp src/local_planner_util.h
  src/latched_stop_rotate_controller.cpp src/latched_stop_rotate_controller.h
  src/local_planner_limits.cpp src/local_planner_limits.h
  src/utils.cpp src/utils.h
  src/utils.cpp src/geometry_utils.hpp
  src/parameter_magic.h
)
add_dependencies(ruvu_carrot_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(ruvu_carrot_local_planner PRIVATE ${FLAGS})
target_link_libraries(ruvu_carrot_local_planner ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ruvu_carrot_local_planner
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
  target_compile_options(test_utils PRIVATE ${FLAGS})
  target_link_libraries(test_utils ${catkin_LIBRARIES})

  catkin_add_gtest(geometry_utils test/test_geometry_utils.cpp)
  target_compile_options(geometry_utils PRIVATE ${FLAGS})
  target_link_libraries(geometry_utils ${catkin_LIBRARIES})
endif()
