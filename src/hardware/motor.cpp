#include "hardware/motor.h"
#include "crc/crc16.h"

// #define my_2pi (6.28318530717f)
// #define my_pi (3.14159265358f)

Motor::Motor(int motor_num, int canport_num, int canboard_num)
    : m_canport_num(canport_num)
    , m_canboard_num(canboard_num)
{
    if (!m_node.getParam(
            "robot/CANboard/No_" + std::to_string(canboard_num) + "_CANboard/CANport/CANport_" +
                std::to_string(canport_num) + "/motor/motor" + std::to_string(motor_num) + "/name",
            m_motor_name
        ))
    {
        ROS_ERROR("Faile to get params name");
    }

    if (!m_node.getParam(
            "robot/CANboard/No_" + std::to_string(canboard_num) + "_CANboard/CANport/CANport_" +
                std::to_string(canport_num) + "/motor/motor" + std::to_string(motor_num) + "/id",
            m_id
        ))
    {
        ROS_ERROR("Faile to get params id");
    }

    if (!m_node.getParam(
            "robot/CANboard/No_" + std::to_string(canboard_num) + "_CANboard/CANport/CANport_" +
                std::to_string(canport_num) + "/motor/motor" + std::to_string(motor_num) + "/type",
            m_type
        ))
    {
        ROS_ERROR("Faile to get params type");
    }

    if (!m_node.getParam(
            "robot/CANboard/No_" + std::to_string(canboard_num) + "_CANboard/CANport/CANport_" +
                std::to_string(canport_num) + "/motor/motor" + std::to_string(motor_num) + "/num",
            m_num
        ))
    {
        ROS_ERROR("Faile to get params num");
    }

    std::cout << "motor_name" << m_motor_name << std::endl;

    m_motor_state =
        m_node.advertise<livelybot_msg::MotorState>("/livelybot_real_real/" + m_motor_name + "_controller/state", 1);
    m_motor_cmd =
        m_node.advertise<livelybot_msg::MotorCmd>("/livelybot_real_real/" + m_motor_name + "_controller/cmd", 1);

    memset(&m_cmd, 0, sizeof(m_cmd));
    memset(&m_data, 0, sizeof(m_data));

    m_cmd.motor_cmd.ID = m_id;
    m_cmd.head[0]      = 0xFE;
    m_cmd.head[1]      = 0xFD;
    m_data.ID          = m_id;
}

template<typename T>
inline T Motor::float2int(float in_data, uint8_t type)
{
    switch (type)
    {
        case 0: // radian float pos/vel to int32
            return (int32_t)(in_data / M_PI / 2.0 * 100000.0);
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

inline float Motor::int2float(int32_t in_data, uint8_t type)
{
    switch (type)
    {
        case 0: // radian float pos/vel to int32
            return (float)(in_data * M_PI / 2.0 / 100000.0);
        case 1: // angle float pos/vel to int32
            return (float)(in_data * 360.0 / 100000.0);
        case 2: // float torque to int32
            return (float)(in_data / 100000.0);
        default:
            return float();
    }
}

float Motor::clamp(float value, float min, float max) { return std::max(min, std::min(max, value)); }

/*
    velocity:
    torque: -4.0~4.0
*/
void Motor::fresh_cmd(float position, float velocity, float torque, float Kp, float Kd)
{
    m_cmd.motor_cmd.position = float2int<int32_t>(position, 1);
    m_cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 1);
    m_cmd.motor_cmd.torque   = float2int<int32_t>(clamp(torque, -4.0, 4.0), 2);
    m_cmd.motor_cmd.Kp       = float2int<int16_t>(Kp, 3);
    m_cmd.motor_cmd.Kd       = float2int<int16_t>(Kp, 3);
    m_cmd.crc16              = crc_ccitt(0x0000, (const uint8_t*)&m_cmd, sizeof(cdc_acm_rx_message_t) - 2);

    m_cmd_msg.q   = position;
    m_cmd_msg.dq  = velocity;
    m_cmd_msg.tau = torque;
    m_cmd_msg.Kp  = Kp;
    m_cmd_msg.Kd  = Kd;
    m_motor_cmd.publish(m_cmd_msg);
}

void Motor::fresh_data(int32_t position, int32_t velocity, int32_t torque)
{
    m_state_msg.pos = m_data.position = int2float(position, 1);
    m_state_msg.vel = m_data.velocity = int2float(velocity, 1);
    m_state_msg.tau = m_data.torque = int2float(torque, 2);
    m_motor_state.publish(m_state_msg);
}
