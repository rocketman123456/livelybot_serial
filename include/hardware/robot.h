#ifndef _ROBOT_H_
#define _ROBOT_H_
#include "canboard.h"

#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <string>
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

    ros::NodeHandle m_node;

public:
    std::vector<std::string>                    m_serial_ids;
    std::vector<std::shared_ptr<lively_serial>> m_serials;
    std::vector<std::shared_ptr<CANPort>>       m_canports;
    std::vector<std::shared_ptr<CANBoard>>      m_canboards;
    std::vector<std::shared_ptr<Motor>>         m_motors;

    std::vector<std::thread> m_receive_threads;
    std::vector<std::thread> m_send_threads;

    Robot();
    ~Robot() = default;

    void motor_send();
    void init_ser();
};
#endif