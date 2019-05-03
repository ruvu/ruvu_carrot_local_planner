cmake_minimum_required(VERSION 2.8.3)
project(ruvu_carrot_local_planner)

add_compile_options(-std=c++11)

# https://stackoverflow.com/questions/5088460/flags-to-enable-thorough-and-verbose-g-warnings
add_compile_options(-Wall -Wextra -Wno-ignored-qualifiers -Wcast-align -Wdisabled-optimization -Wformat=2 -Winit-self -Wnoexcept -Wredundant-decls -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=1 -Wswitch-default -Wtype-limits -Werror -Wno-unused)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  dynamic_reconfigure
  nav_core
  nav_msgs
  pluginlib
  roscpp
  ruvu_lint
  tf
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/CarrotPlanner.cfg
)

catkin_package(
  LIBRARIES ruvu_carrot_local_planner
  CATKIN_DEPENDS
    dynamic_reconfigure
    nav_msgs
    pluginlib
    roscpp
)

add_library(ruvu_carrot_local_planner
  src/carrot_planner_ros.cpp src/carrot_planner_ros.h
  src/local_planner_util.cpp src/local_planner_util.h
  src/utils.cpp src/utils.h
)
add_dependencies(ruvu_carrot_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ruvu_carrot_local_planner ${catkin_LIBRARIES})

install(TARGETS ruvu_carrot_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES blp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-build/include_what_you_use,-whitespace/braces,-runtime/references")
ruvu_lint_add_test()
