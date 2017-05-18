cmake_minimum_required(VERSION 2.8.3)
project(ur_tests)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(move_group_interface_test src/move_group_interface_test.cpp)
target_link_libraries(move_group_interface_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(objects_test src/objects_test.cpp)
target_link_libraries(objects_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS objects_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pick_up src/pick_up.cpp)
target_link_libraries(pick_up ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_up DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(compare_ik src/compare_ik.cpp)
target_link_libraries(compare_ik ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS compare_ik DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(planner_test src/planner_test.cpp)
target_link_libraries(planner_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planner_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(path_length src/path_length_test.cpp)
target_link_libraries(path_length ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS path_length DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(publish_shelf src/publish_shelf.cpp)
target_link_libraries(publish_shelf ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS publish_shelf DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cartesian_line src/cartesian_line.cpp)
target_link_libraries(cartesian_line ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS cartesian_line DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

