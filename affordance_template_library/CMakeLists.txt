cmake_minimum_required(VERSION 2.8.3)
project(affordance_template_library)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
    LIBRARIES ${PROJECT_NAME} affordance_template_parser
    DEPENDS system_lib 
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(affordance_template_parser src/affordance_template_parser.cpp)
target_link_libraries(affordance_template_parser ${catkin_LIBRARIES})


