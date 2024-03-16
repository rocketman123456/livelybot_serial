#ifndef _LIVELYm_serialIAL_H_
#define _LIVELYm_serialIAL_H_

#include "hardware/motor.h"
#include "serial_struct.h"

#include <ros/ros.h>
#include <serial/serial.h>

class lively_serial
{
private:
    serial::Serial m_serial;
    std::string*   m_port;
    std::string    m_result;
    uint32_t       m_baudrate;
    uint8_t        m_debug_level;

    bool init_flag;
    bool send_flag;
    bool recv_flag;

    cdc_acm_tx_message_t cdc_acm_tx_message; // 串口发送的信息，来自于Motor，反馈给PC
    cdc_acm_rx_message_t cdc_acm_rx_message; // 串口收到的信息，来自于PC，用于控制Motor

    std::vector<motor_back_t*> Motor_data;
    std::vector<motor*>        Motors;
    std::map<int, motor*>      Map_Motors_p;

    ros::Rate* rate;
    int*       id;
    uint16_t   crc_head;

public:
    lively_serial(std::string* port, uint32_t baudrate, uint8_t debug_level);
    ~lively_serial() = default;

    lively_serial(const lively_serial&)            = delete;
    lively_serial& operator=(const lively_serial&) = delete;

    void send(uint8_t ID, int32_t position, int32_t velocity, int32_t torque, int16_t Kp, int16_t Kd);
    void send(cdc_acm_rx_message_t* _cdc_acm_rx_message);

    void recv();

    void init_map_motor(std::map<int, motor*>* _Map_Motors_p);
    void test_ser_motor();
};

#endif