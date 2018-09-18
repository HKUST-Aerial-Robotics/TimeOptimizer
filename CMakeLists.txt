cmake_minimum_required(VERSION 2.8.3)
project(time_optimizer)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs visualization_msgs tf)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package()

include_directories(${PROJECT_SOURCE_DIR}/third_party/include)
include_directories(
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

link_directories( ${PROJECT_SOURCE_DIR}/third_party/lib/mosek8_1 )
link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_BUILD_TYPE "Release")

add_executable(demo 
    src/demo.cpp
    src/time_optimizer.cpp
    src/trajectory_generator_waypoint.cpp )

target_link_libraries(demo 
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} mosek64 )
