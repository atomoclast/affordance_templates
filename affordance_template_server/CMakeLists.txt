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
    interactive_markers
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp affordance_template_msgs affordance_template_markers tf affordance_template_library interactive_markers
    LIBRARIES ${PROJECT_NAME}
    DEPENDS system_lib Boost
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/server.cpp src/service_interface.cpp)

add_executable(server src/server.cpp src/service_interface.cpp)
target_link_libraries(server ${catkin_LIBRARIES})