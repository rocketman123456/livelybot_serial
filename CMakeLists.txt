cmake_minimum_required(VERSION 3.0.2)
project(livelybot_serial)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    livelybot_msg  
    serial
    moveit_msgs
    control_msgs
    actionlib
    actionlib_msgs
    moveit_ros_planning_interface
)
set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(serial_SRCS
    include/hardware/robot.h
    include/hardware/motor.h
    src/lively_serial.cc
    src/hardware/motor.cc
)
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread
    CATKIN_DEPENDS roscpp std_msgs livelybot_msg serial actionlib actionlib_msgs moveit_msgs control_msgs
)
add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# test
add_executable(lvbot_test src/serial_t.cpp ${serial_SRCS})
target_link_libraries(lvbot_test ${catkin_LIBRARIES})

add_executable(test_motor_run src/test_motor_run.cpp ${serial_SRCS})
target_link_libraries(test_motor_run ${catkin_LIBRARIES})

add_executable(test_motor src/test_motor.cpp ${serial_SRCS})
target_link_libraries(test_motor ${catkin_LIBRARIES})

add_executable(test_p_sit src/test_p_sit.cpp ${serial_SRCS})
target_link_libraries(test_p_sit ${catkin_LIBRARIES})

add_executable(test_torque src/test_torque.cpp ${serial_SRCS})
target_link_libraries(test_torque ${catkin_LIBRARIES})

add_executable(test_pose_torque src/test_pose_torque.cpp ${serial_SRCS})
target_link_libraries(test_pose_torque ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install headers
install(FILES include/hardware/robot.h include/hardware/motor.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livelybot_serial)
