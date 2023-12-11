cmake_minimum_required(VERSION 3.0.2)
project(livelybot_serial)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  livelybot_msg
)
set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(serial_SRCS
    # include/hardware/**.h
    src/hardware/motor.cc
    src/serial/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
    src/lively_serial.cc
)
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread
)
list(APPEND serial_SRCS src/serial/impl/unix.cc)
list(APPEND serial_SRCS src/serial/impl/list_ports/list_ports_linux.cc)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_test
 CATKIN_DEPENDS roscpp rospy std_msgs livelybot_msg
#  DEPENDS system_lib
)
add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# test
add_executable(lvbot_test src/serial_t.cpp ${serial_SRCS} )
target_link_libraries(lvbot_test ${catkin_LIBRARIES})