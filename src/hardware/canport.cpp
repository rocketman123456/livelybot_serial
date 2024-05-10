#include "hardware/canport.h"

#include <iostream>
#include <memory>

CANPort::CANPort(int canport_num, int canboard_num, std::shared_ptr<lively_serial> serial)
    : m_serial(serial)
{
    m_canboard_id = canboard_num;
    m_canport_id  = canport_num;

    std::cout << "canboard_id" << m_canboard_id << ", canport_id" << m_canport_id << std::endl;

    if (!m_node.getParam(
            "robot/CANboard/No_" + std::to_string(m_canboard_id) + "_CANboard/CANport/CANport_" +
                std::to_string(m_canport_id) + "/motor_num",
            m_motor_num
        ))
    {
        ROS_ERROR("Faile to get params motor_num");
    }
    std::cout << "motor_num" << m_motor_num << std::endl;

    for (size_t i = 1; i <= m_motor_num; i++)
    {
        std::cout << "motor_id" << i << std::endl;
        auto motor = std::make_shared<Motor>(i, m_canport_id, m_canboard_id);
        m_motors.push_back(motor);
    }

    for (auto& motor : m_motors)
    {
        m_motor_map.insert(std::pair<int, std::shared_ptr<Motor>>(motor->get_motor_id(), motor));
    }

    m_serial->init_map_motor(m_motor_map);
    m_send_enabled = false;
    // std::thread(&canport::send, this);
}

CANPort::~CANPort()
{
    m_motors.clear();
    m_motor_map.clear();
}

void CANPort::puch_motor(std::vector<std::shared_ptr<Motor>>& motors)
{
    for (auto& m : m_motors)
    {
        motors.push_back(m);
    }
}

void CANPort::motor_send()
{
    for (auto m : m_motors)
    {
        m_serial->send(m->return_cmd_p());
    }
}