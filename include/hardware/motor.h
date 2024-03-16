#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "livelybot_msg/MotorCmd.h"
#include "livelybot_msg/MotorState.h"
#include "serial_struct.h"

#include <ros/ros.h>
#include <stdint.h>

class motor
{
private:
    int type;
    int id;
    int num;
    int CANport_num;
    int CANboard_num;

    ros::NodeHandle           n;
    motor_back_t              data;
    ros::Publisher            _motor_state;
    ros::Publisher            _motor_cmd;
    livelybot_msg::MotorState p_msg;
    livelybot_msg::MotorCmd   cmd_msg;
    std::string               motor_name;

public:
    cdc_acm_rx_message_t cmd;

    motor(int _motor_num, int _CANport_num, int _CANboard_num);
    ~motor() = default;

    template<typename T>
    inline T     float2int(float in_data, uint8_t type);
    inline float int2float(int32_t in_data, uint8_t type);
    float        clamp(float value, float min, float max);

    void fresh_cmd(float position, float velocity, float torque, float Kp, float Kd);
    void fresh_data(int32_t position, int32_t velocity, int32_t torque);
    int  get_motor_id() { return id; }
    int  get_motor_type() { return type; }
    int  get_motor_num() { return num; }
    int  get_motor_belong_canport() { return CANport_num; }
    int  get_motor_belong_canboard() { return CANboard_num; }

    cdc_acm_rx_message_t* return_cmd_p() { return &cmd; }
    motor_back_t*         get_current_motor_state() { return &data; }
};
#endif