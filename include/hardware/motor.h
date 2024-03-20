#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "livelybot_msg/MotorCmd.h"
#include "livelybot_msg/MotorState.h"
#include "serial_struct.h"

#include <ros/ros.h>
#include <stdint.h>
#include <string>

class Motor
{
private:
    std::string m_motor_name;

    int m_type;
    int m_id;
    int m_num;
    int m_canport_num;
    int m_canboard_num;

    ros::NodeHandle m_node;
    ros::Publisher  m_motor_state;
    ros::Publisher  m_motor_cmd;

    livelybot_msg::MotorState m_state_msg;
    livelybot_msg::MotorCmd   m_cmd_msg;

    cdc_acm_rx_message_t m_cmd;
    motor_back_t         m_data;

public:
    Motor(int motor_num, int canport_num, int canboard_num);
    ~Motor() = default;

    template<typename T>
    inline T     float2int(float in_data, uint8_t type);
    inline float int2float(int32_t in_data, uint8_t type);
    float        clamp(float value, float min, float max);

    void fresh_cmd(float position, float velocity, float torque, float Kp, float Kd);
    void fresh_data(int32_t position, int32_t velocity, int32_t torque);

    int get_motor_id() { return m_id; }
    int get_motor_type() { return m_type; }
    int get_motor_num() { return m_num; }
    int get_motor_belong_canport() { return m_canport_num; }
    int get_motor_belong_canboard() { return m_canboard_num; }

    cdc_acm_rx_message_t* return_cmd_p() { return &m_cmd; }
    motor_back_t*         get_current_motor_state() { return &m_data; }
};
#endif