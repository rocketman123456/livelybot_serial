#ifndef _CANBOARD_H_
#define _CANBOARD_H_

#include "hardware/canport.h"

#include <iostream>
#include <ros/ros.h>
#include <vector>

class canboard
{
private:
    int                   CANport_num;
    ros::NodeHandle       n;
    std::vector<canport*> CANport;
    // std::vector<motor> motor;
    // std::vector<std::shared_ptr<canport>> CANport;

public:
    canboard(int _CANboard_ID, std::vector<lively_serial*>* ser);
    ~canboard() = default;

    int  get_CANport_num();
    void push_CANport(std::vector<canport*>* _CANport);
    void motor_send();
};
#endif