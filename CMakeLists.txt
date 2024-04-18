cmake_minimum_required(VERSION 3.0.2)
project(serial_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_demo
#  CATKIN_DEPENDS roscpp serial std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  /usr/include/eigen3
)

add_executable(serial_232 src/serial_232.cpp)
add_executable(serial_485 src/serial_485.cpp)

add_dependencies(serial_232 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(serial_485 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(serial_232
  ${catkin_LIBRARIES}
)
target_link_libraries(serial_485
${catkin_LIBRARIES}
)