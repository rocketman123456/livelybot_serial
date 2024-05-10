#pragma once

#include "hardware/canport.h"

#include <ros/ros.h>

#include <memory>
#include <vector>

class CANBoard
{
private:
    int                                   m_canport_num;
    ros::NodeHandle                       m_node;
    std::vector<std::shared_ptr<CANPort>> m_canport;

public:
    CANBoard(int canboard_ID, std::vector<std::shared_ptr<lively_serial>>& serials);
    ~CANBoard() = default;

    int  get_canport_num();
    void push_canport(std::vector<std::shared_ptr<CANPort>>& canport);
    void motor_send();
};
