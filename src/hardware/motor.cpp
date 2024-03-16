#include "hardware/motor.h"

#define my_2pi (6.28318530717f)
#define my_pi (3.14159265358f)

motor::motor(int _motor_num, int _CANport_num, int _CANboard_num) :
    CANport_num(_CANport_num), CANboard_num(_CANboard_num)
{
    if (!n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" +
                        std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/name",
                    motor_name))
    {
        ROS_ERROR("Faile to get params name");
    }

    if (!n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" +
                        std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/id",
                    id))
    {
        ROS_ERROR("Faile to get params id");
    }

    if (!n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" +
                        std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/type",
                    type))
    {
        ROS_ERROR("Faile to get params type");
    }

    if (!n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" +
                        std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/num",
                    num))
    {
        ROS_ERROR("Faile to get params num");
    }

    _motor_state =
        n.advertise<livelybot_msg::MotorState>("/livelybot_real_real/" + motor_name + "_controller/state", 1);
    _motor_cmd = n.advertise<livelybot_msg::MotorCmd>("/livelybot_real_real/" + motor_name + "_controller/cmd", 1);

    memset(&cmd, 0, sizeof(cmd));
    memset(&data, 0, sizeof(data));
    cmd.motor_cmd.ID = id;
    cmd.head[0]      = 0xFE;
    cmd.head[1]      = 0xFD;
    data.ID          = id;
}

template<typename T>
inline T motor::float2int(float in_data, uint8_t type)
{
    switch (type)
    {
        case 0: // radian float pos/vel to int32
            return (int32_t)(in_data / my_2pi * 100000.0);
        case 1: // angle float pos/vel to int32
            return (int32_t)(in_data / 360.0 * 100000.0);
        case 2: // float torque to int32
            return (int32_t)(in_data * 100000.0);
        case 3: // float kp/kd to int32
            return (int16_t)(in_data * 0x7FF);
        default:
            return T();
    }
}

inline float motor::int2float(int32_t in_data, uint8_t type)
{
    switch (type)
    {
        case 0: // radian float pos/vel to int32
            return (float)(in_data * my_2pi / 100000.0);
        case 1: // angle float pos/vel to int32
            return (float)(in_data * 360.0 / 100000.0);
        case 2: // float torque to int32
            return (float)(in_data / 100000.0);
        default:
            return float();
    }
}

float motor::clamp(float value, float min, float max) { return std::max(min, std::min(max, value)); }

/*
    velocity:
    torque: -4.0~4.0
*/
void motor::fresh_cmd(float position, float velocity, float torque, float Kp, float Kd)
{
    cmd.motor_cmd.position = float2int<int32_t>(position, 1);
    cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 1);
    cmd.motor_cmd.torque   = float2int<int32_t>(clamp(torque, -4.0, 4.0), 2);
    cmd.motor_cmd.Kp       = float2int<int16_t>(Kp, 3);
    cmd.motor_cmd.Kd       = float2int<int16_t>(Kp, 3);
    cmd.crc16              = crc_ccitt(0x0000, (const uint8_t*)&cmd, sizeof(cdc_acm_rx_message_t) - 2);

    cmd_msg.q   = position;
    cmd_msg.dq  = velocity;
    cmd_msg.tau = torque;
    cmd_msg.Kp  = Kp;
    cmd_msg.Kd  = Kd;
    _motor_cmd.publish(cmd_msg);
}

void motor::fresh_data(int32_t position, int32_t velocity, int32_t torque)
{
    p_msg.pos = data.position = int2float(position, 1);
    p_msg.vel = data.velocity = int2float(velocity, 1);
    p_msg.tau = data.torque = int2float(torque, 2);
    _motor_state.publish(p_msg);
}
