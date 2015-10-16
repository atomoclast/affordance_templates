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

add_library(${PROJECT_NAME} src/interface.cpp src/server.cpp)

add_executable(${PROJECT_NAME}_node src/interface.cpp src/server.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} affordance_template_parser)