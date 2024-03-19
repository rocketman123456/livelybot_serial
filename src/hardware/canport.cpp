#include "hardware/canport.h"

CANPort::CANPort(int _CANport_num, int _CANboard_num, lively_serial* _ser) : ser(_ser)
{
    canboard_id = _CANboard_num;
    canport_id  = _CANport_num;
    if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" +
                       std::to_string(_CANport_num) + "/motor_num",
                   motor_num))
    {
        // ROS_INFO("Got params motor_num: %d",motor_num);
    }
    else
    {
        ROS_ERROR("Faile to get params motor_num");
    }
    for (size_t i = 1; i <= motor_num; i++)
    {
        Motors.push_back(new Motor(i, _CANport_num, _CANboard_num));
    }
    for (Motor* m : Motors)
    {
        Map_Motors_p.insert(std::pair<int, Motor*>(m->get_motor_id(), m));
    }
    ser->init_map_motor(&Map_Motors_p);
    // ser->test_ser_motor();
    sendEnabled = false;
    // std::thread(&canport::send, this);
}

CANPort::~CANPort()
{
    // for (motor_back_t* m:Motor_data)
    // {
    //     delete m;
    // }
}

void CANPort::puch_motor(std::vector<Motor*>* _Motors)
{
    for (Motor* m : Motors)
    {
        _Motors->push_back(m);
    }
}

void CANPort::push_motor_data()
{
    for (motor_back_t* r : Motor_data)
    {
    }
}

void CANPort::motor_send()
{
    for (Motor* m : Motors)
    {
        ser->send(m->return_cmd_p());
        // ROS_INFO("CRC: 0x%x", m->cmd.crc16);
    }
}