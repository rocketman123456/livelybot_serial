#include "hardware/canboard.h"

CANBoard::CANBoard(int canboard_ID, std::vector<lively_serial*>* ser)
{
    if (!n.getParam("robot/CANboard/No_" + std::to_string(canboard_ID) + "_CANboard/CANport_num", m_canport_num))
    {
        ROS_ERROR("Faile to get params CANport_num");
    }

    for (size_t j = 1; j <= m_canport_num; j++) // 一个串口对应一个CANport
    {
        m_canport.push_back(new CANPort(j, canboard_ID, (*ser)[(canboard_ID - 1) * m_canport_num + j - 1]));
    }
}

int CANBoard::get_canport_num() { return m_canport_num; }

void CANBoard::push_canport(std::vector<CANPort*>* _CANport)
{
    for (CANPort* c : m_canport)
    {
        _CANport->push_back(c);
    }
}

void CANBoard::motor_send()
{
    for (CANPort* c : m_canport)
    {
        c->motor_send();
    }
}