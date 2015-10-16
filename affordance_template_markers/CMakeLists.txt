cmake_minimum_required(VERSION 2.8.3)
project(affordance_template_markers)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
    affordance_template_msgs
    affordance_template_library
    interactive_markers
    planner_interface
    moveit_planner
    geometry_msgs
    nav_msgs
    std_msgs
    roscpp
)

catkin_package(
    INCLUDE_DIRS include
	CATKIN_DEPENDS affordance_template_msgs affordance_template_library geometry_msgs interactive_markers nav_msgs roscpp std_msgs planner_interface moveit_planner
    LIBRARIES robot_interface affordance_template
    DEPENDS system_lib yaml-cpp
)

include_directories(
    include
    yaml-cpp
    ${catkin_INCLUDE_DIRS}
)

add_library(robot_interface src/robot_interface.cpp)
target_link_libraries(robot_interface ${catkin_LIBRARIES} yaml-cpp)

add_library(affordance_template src/affordance_template.cpp)
target_link_libraries(affordance_template ${catkin_LIBRARIES} yaml-cpp robot_interface affordance_template_library)

# !!!!!
# use to test various .yaml files, take out when running as lib
# add_executable(ri_test_main src/robot_interface.cpp)
# target_link_libraries(ri_test_main ${catkin_LIBRARIES} yaml-cpp)

add_executable(at_test src/affordance_template.cpp)
target_link_libraries(at_test ${catkin_LIBRARIES} robot_interface affordance_template_library)

# !!!!!