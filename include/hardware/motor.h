#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "serial_struct.h"
#include "livelybot_msg/MotorCmd.h"
#include "livelybot_msg/MotorState.h"

#include <ros/ros.h>
#include <stdint.h>

#define my_2pi (6.28318530717f)
#define my_pi (3.14159265358f)
class motor
{
private:
    int             type, id, num, CANport_num, CANboard_num;
    ros::NodeHandle n;
    // lively_serial *ser;
    motor_back_t              data;
    ros::Publisher            _motor_state, _motor_cmd;
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