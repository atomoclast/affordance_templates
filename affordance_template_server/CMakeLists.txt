cmake_minimum_required(VERSION 2.8.3)
project(affordance_template_server)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    affordance_template_msgs
    affordance_template_markers
    affordance_template_library
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp affordance_template_msgs affordance_template_markers tf affordance_template_library
    #LIBRARIES ${PROJECT_NAME}
    DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME} src/server.cpp)

# !!!!!
# use to test various .yaml files, take out when running as lib
add_executable(at_test_main src/server.cpp)
target_link_libraries(at_test_main ${catkin_LIBRARIES} robot_interface)
# !!!!!