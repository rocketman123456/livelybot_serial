#include "hardware/canboard.h"

canboard::canboard(int _CANboard_ID, std::vector<lively_serial*>* ser)
{
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_ID) + "_CANboard/CANport_num", CANport_num))
    {
        // ROS_INFO("Got params CANport_num: %d",CANport_num);
    }
    else
    {
        ROS_ERROR("Faile to get params CANport_num");
    }
    for (size_t j = 1; j <= CANport_num; j++) // 一个串口对应一个CANport
    {
        CANport.push_back(new canport(j, _CANboard_ID, (*ser)[(_CANboard_ID - 1) * CANport_num + j - 1]));
    }
}

int canboard::get_CANport_num() { return CANport_num; }

void canboard::push_CANport(std::vector<canport*>* _CANport)
{
    for (canport* c : CANport)
    {
        _CANport->push_back(c);
    }
}

void canboard::motor_send()
{
    for (canport* c : CANport)
    {
        c->motor_send();
    }
}