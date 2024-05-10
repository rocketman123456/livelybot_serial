cmake_minimum_required(VERSION 3.0.2)
project(livelybot_serial)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    livelybot_msg  
    serial
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(serial_SRCS
    src/lively_serial.cpp
    src/hardware/canboard.cpp
    src/hardware/canport.cpp
    src/hardware/motor.cpp
    src/hardware/robot.cpp
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS rt pthread
    CATKIN_DEPENDS roscpp std_msgs livelybot_msg serial
)

add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)

add_executable(test_motor src/test_motor.cpp ${serial_SRCS})
target_link_libraries(test_motor ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install headers
install(FILES include/hardware/robot.h include/hardware/motor.h
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livelybot_serial
)
