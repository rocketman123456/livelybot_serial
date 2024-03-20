#ifndef _LIVELYm_serialIAL_H_
#define _LIVELYm_serialIAL_H_

#include "hardware/motor.h"
#include "serial_struct.h"

#include <ros/ros.h>
#include <serial/serial.h>
#include <memory>

class lively_serial
{
private:
    serial::Serial m_serial;
    std::string    m_port;
    std::string    m_result;
    uint32_t       m_baudrate;
    uint8_t        m_debug_level;

    bool m_init_flag = false;
    bool m_send_flag = false;
    bool m_recv_flag = false;

    cdc_acm_tx_message_t m_tx_message; // 串口发送的信息，来自于Motor，反馈给PC
    cdc_acm_rx_message_t m_rx_message; // 串口收到的信息，来自于PC，用于控制Motor

    std::map<int, std::shared_ptr<Motor>> m_motor_map;

    ros::Rate* m_rate;

public:
    lively_serial(const std::string& port, uint32_t baudrate, uint8_t debug_level);
    ~lively_serial();

    lively_serial(const lively_serial&)            = delete;
    lively_serial& operator=(const lively_serial&) = delete;

    void send(uint8_t ID, int32_t position, int32_t velocity, int32_t torque, int16_t Kp, int16_t Kd);
    void send(cdc_acm_rx_message_t* rx_message);

    void recv();

    void init_map_motor(const std::map<int, std::shared_ptr<Motor>>& motor_map);
};

#endif