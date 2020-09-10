cmake_minimum_required(VERSION 2.8.3)
project(herd_ros)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
 set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
 set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
 message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use different C++ compiler")
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  roscpp
  rospy
  std_msgs
)


find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
 DEPENDS
  roscpp
  gazebo_ros
  rospy
  )

catkin_install_python(PROGRAMS src/TaskDivisionRos.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(HerderStateRos src/HerderNodeRos.cpp)
target_link_libraries(HerderStateRos ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(TargetStateRos src/TargetNodeRos.cpp)
target_link_libraries(TargetStateRos ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(SaveDataRos src/SaveData.cpp)
target_link_libraries(SaveDataRos ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


