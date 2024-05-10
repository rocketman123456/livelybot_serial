#include "hardware/robot.h"
#include <memory>

RobotDriver::RobotDriver()
{
    if (!m_node.getParam("robot/Seial_Baudrate", m_seial_baudrate))
    {
        ROS_ERROR("Faile to get params seial_baudrate");
    }
    if (!m_node.getParam("robot/Robot_Name", m_robot_name))
    {
        ROS_ERROR("Faile to get params m_robot_name");
    }
    if (!m_node.getParam("robot/CANboard_Num", m_canboard_num))
    {
        ROS_ERROR("Faile to get params CANboard_num");
    }
    if (!m_node.getParam("robot/CANboard_Type", m_canboard_type))
    {
        ROS_ERROR("Faile to get params CANboard_type");
    }
    if (!m_node.getParam("robot/Serial_Type", m_serial_type))
    {
        ROS_ERROR("Faile to get params Serial_Type");
    }
    if (!m_node.getParam("robot/Serial_Allocate", m_serial_allocate))
    {
        ROS_ERROR("Faile to get params Serial_allocate");
    }

    ROS_INFO("\033[1;32mThe robot name is %s\033[0m", m_robot_name.c_str());
    ROS_INFO("\033[1;32mThe robot has %d CANboards\033[0m", m_canboard_num);
    ROS_INFO("\033[1;32mThe CANboard type is %s\033[0m", m_canboard_type.c_str());
    ROS_INFO("\033[1;32mThe Serial type is %s\033[0m", m_serial_type.c_str());
    ROS_INFO("\033[1;32mThe Serial allocate type is %s\033[0m", m_serial_allocate.c_str());

    init_serial();
    if (m_serial_allocate == "1for2")
    {
        for (size_t i = 1; i <= m_canboard_num; i++) // 一个CANboard使用两个串口
        {
            auto board = std::make_shared<CANBoard>(i, m_serials);
            m_canboards.push_back(board);
        }
    }

    for (auto& cb : m_canboards)
    {
        cb->push_canport(m_canports);
    }

    for (auto& cp : m_canports)
    {
        cp->puch_motor(m_motors);
    }

    ROS_INFO("\033[1;32m The robot has %ld motors \033[0m", m_motors.size());
}

void RobotDriver::motor_send()
{
    for (auto& cb : m_canboards)
    {
        cb->motor_send();
    }
}

void RobotDriver::init_serial()
{
    for (size_t i = 0; i < 4; i++)
    {
        m_serial_ids.push_back("/dev/ttyUSB" + std::to_string(i));
    }

    for (auto& m_serial_id : m_serial_ids)
    {
        std::cout << "serial_id " << m_serial_id << std::endl;
        // lively_serial *s = new lively_serial(&str[i], 2000000, 1);
        auto serial = std::make_shared<lively_serial>(m_serial_id, m_seial_baudrate, 1);
        m_serials.push_back(serial);
        m_receive_threads.push_back(std::thread(&lively_serial::recv, serial));
    }
}
