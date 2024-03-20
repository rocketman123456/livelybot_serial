#ifndef _CANPORT_H_
#define _CANPORT_H_

#include "hardware/motor.h"
#include "lively_serial.h"

#include <condition_variable>
#include <memory>
#include <ros/ros.h>
#include <thread>

class CANPort
{
private:
    ros::NodeHandle                       m_node;
    std::vector<std::shared_ptr<Motor>>   m_motors;
    std::map<int, std::shared_ptr<Motor>> m_motor_map;

    int  m_canboard_id;
    int  m_canport_id;
    int  m_canport_num;
    int  m_motor_num;
    bool m_send_enabled;

    std::shared_ptr<lively_serial> m_serial;

public:
    CANPort(int canport_num, int canboard_num, std::shared_ptr<lively_serial> serial);
    ~CANPort();

    void puch_motor(std::vector<std::shared_ptr<Motor>>& motors);
    // void push_motor_data();
    void motor_send();

    int get_motor_num() { return m_motor_num; }
    int get_canboard_id() { return m_canboard_id; }
    int get_canport_id() { return m_canport_id; }
};
#endif