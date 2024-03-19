#ifndef _ROBOT_H_
#define _ROBOT_H_
#include "canboard.h"

#include <ros/ros.h>
#include <iostream>
#include <thread>

class Robot
{
private:
    std::string m_robot_name;
    std::string m_serial_type;
    std::string m_can_type;
    std::string m_canboard_type;
    std::string m_serial_allocate;

    int m_arm_dof;
    int m_leg_dof;
    int m_canboard_num;
    int m_seial_baudrate;

    ros::NodeHandle             m_node;
    std::vector<CANBoard>       m_canboards;
    std::vector<std::string>    m_serial_ids;
    std::vector<lively_serial*> m_serials;

public:
    std::vector<Motor*>      m_motors;
    std::vector<CANPort*>    m_canports;
    std::vector<std::thread> m_receive_threads;
    std::vector<std::thread> m_send_threads;

    Robot();
    ~Robot() = default;

    void motor_send();
    void init_ser();
    void test_ser_motor();
};
#endif