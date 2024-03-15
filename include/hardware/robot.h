#ifndef _ROBOT_H_
#define _ROBOT_H_
#include "canboard.h"
#include "ros/ros.h"
#include <iostream>
#include <thread>

class robot
{
private:
    std::string                 robot_name, Serial_Type, CAN_type, CANboard_type, Serial_allocate;
    int                         arm_dof, leg_dof, CANboard_num, Seial_baudrate;
    ros::NodeHandle             n;
    std::vector<canboard>       CANboards;
    std::vector<std::string>    str;
    std::vector<lively_serial*> ser;

public:
    std::vector<motor*> Motors;
    std::vector<canport*>    CANPorts;
    std::vector<std::thread> ser_recv_threads, send_threads;

    robot();
    ~robot() = default;

    void motor_send();
    void init_ser();
    void test_ser_motor();
};
#endif