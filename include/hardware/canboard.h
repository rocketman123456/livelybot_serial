#ifndef _CANBOARD_H_
#define _CANBOARD_H_

#include "hardware/canport.h"

#include <iostream>
#include <ros/ros.h>
#include <vector>

class CANBoard
{
private:
    int                   m_canport_num;
    ros::NodeHandle       n;
    std::vector<CANPort*> m_canport;
    // std::vector<motor> motor;
    // std::vector<std::shared_ptr<canport>> CANport;

public:
    CANBoard(int canboard_ID, std::vector<lively_serial*>* ser);
    ~CANBoard() = default;

    int  get_canport_num();
    void push_canport(std::vector<CANPort*>* canport);
    void motor_send();
};
#endif