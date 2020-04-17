cmake_minimum_required(VERSION 2.8.3)
project(vl53l1x_scanner)

find_package(catkin REQUIRED COMPONENTS roscpp serial sensor_msgs)

catkin_package(CATKIN_DEPENDS serial sensor_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vl53l1x_scanner_driver_node src/vl53l1x_scanner_driver_node.cpp)
target_link_libraries(vl53l1x_scanner_driver_node ${catkin_LIBRARIES})