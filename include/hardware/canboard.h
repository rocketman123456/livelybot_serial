#ifndef _CANBOARD_H_
#define _CANBOARD_H_

#include "hardware/canport.h"

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <memory>

class CANBoard
{
private:
    int                   m_canport_num;
    ros::NodeHandle       m_node;
    std::vector<std::shared_ptr<CANPort>> m_canport;

public:
    CANBoard(int canboard_ID, std::vector<std::shared_ptr<lively_serial>>& serials);
    ~CANBoard();

    int  get_canport_num();
    void push_canport(std::vector<std::shared_ptr<CANPort>>& canport);
    void motor_send();
};
#endif