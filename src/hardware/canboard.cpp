#include "hardware/canboard.h"

CANBoard::CANBoard(int canboard_ID, std::vector<std::shared_ptr<lively_serial>>& serial)
{
    if (!m_node.getParam("robot/CANboard/No_" + std::to_string(canboard_ID) + "_CANboard/CANport_num", m_canport_num))
    {
        ROS_ERROR("Faile to get params CANport_num");
    }

    for (size_t j = 1; j <= m_canport_num; j++) // 一个串口对应一个CANport
    {
        auto port = std::make_shared<CANPort>(j, canboard_ID, serial[(canboard_ID - 1) * m_canport_num + j - 1]);
        m_canport.push_back(port);
    }
}

CANBoard::~CANBoard() { m_canport.clear(); }

int CANBoard::get_canport_num() { return m_canport_num; }

void CANBoard::push_canport(std::vector<std::shared_ptr<CANPort>>& canport)
{
    for (auto& port : m_canport)
    {
        canport.push_back(port);
    }
}

void CANBoard::motor_send()
{
    for (auto& port : m_canport)
    {
        port->motor_send();
    }
}