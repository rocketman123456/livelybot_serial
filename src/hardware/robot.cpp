#include "hardware/robot.h"

robot::robot()
{
    if (!m_node.getParam("robot/Seial_Baudrate", m_seial_baudrate))
    {
        ROS_ERROR("Faile to get params seial_baudrate");
    }
    if (!m_node.getParam("robot/Robot_Name", m_robot_name))
    {
        ROS_ERROR("Faile to get params m_robot_name");
    }
    if (!m_node.getParam("robot/CANboard_Num", CANboard_num))
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

    init_ser();
    if (m_serial_allocate == "1for2")
    {
        for (size_t i = 1; i <= m_canboard_num; i++) // 一个CANboard使用两个串口
        {
            m_canboards.push_back(canboard(i, &ser));
        }
    }

    for (canboard& cb : m_canboards)
    {
        cb.push_CANport(&m_canports);
    }
    for (canport* cp : m_canboards)
    {
        // std::thread(&canport::send, &cp);
        cp->puch_motor(&m_motors);
    }

    ROS_INFO("\033[1;32mThe robot has %ld motors\033[0m", m_motors.size());
}

void robot::motor_send()
{
    for (canboard& cb : m_canboards)
    {
        cb.motor_send();
    }
}

void robot::init_ser()
{
    for (size_t i = 0; i < 4; i++)
    {
        m_serial_ids.push_back("/dev/ttyUSB" + std::to_string(i));
    }

    for (size_t i = 0; i < m_serial_ids.size(); i++)
    {
        // lively_serial *s = new lively_serial(&str[i], 2000000, 1);
        lively_serial* s = new lively_serial(&m_serial_ids[i], m_seial_baudrate, 1);
        m_serials.push_back(s);
        m_receive_threads.push_back(std::thread(&lively_serial::recv, s));
    }
}

void robot::test_ser_motor()
{
    for (lively_serial* s : m_serials)
    {
        s->test_ser_motor();
    }
}