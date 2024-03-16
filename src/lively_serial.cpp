#include "lively_serial.h"

lively_serial::lively_serial(std::string* port, uint32_t baudrate, uint8_t debug_level) :
    m_port(port), m_baudrate(baudrate), m_debug_level(debug_level)
{
    init_flag = false;
    send_flag = false;
    recv_flag = false;
    m_serial.setPort(*m_port); // 设置打开的串口名称
    m_serial.setBaudrate(m_baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 创建timeout
    m_serial.setTimeout(to);                                   // 设置串口的timeout
    // 打开串口
    try
    {
        m_serial.open(); // 打开串口
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Unable to open port "); // 打开串口失败，打印信息
    }
    if (m_serial.isOpen())
    {
        ROS_INFO_STREAM("IMU Serial Port initialized."); // 成功打开串口，打印信息
    }
    else
    {
        ROS_ERROR_STREAM("IMU Serial Port failed to initialize."); // 成功打开串口，打印信息
    }
    cdc_acm_tx_message.head[1] = cdc_acm_rx_message.head[0] = 0xFE;
    cdc_acm_tx_message.head[0] = cdc_acm_rx_message.head[1] = 0xFD;
    init_flag                                               = true;
    // start receive thread
}

#define read_by_Byte 1

void lively_serial::recv()
{
    // TODO : this method is read one bytes
    ROS_INFO_STREAM("start thread");
    while (ros::ok() && init_flag)
    {
#ifdef read_by_Byte
        m_result = m_serial.read(2);
        if (*(uint8_t*)&m_result[0] == 0xFD && *(uint8_t*)&m_result[1] == 0xFE)
        {
            m_result = m_serial.read(sizeof(cdc_acm_tx_message_t) - 2);
            memcpy(&cdc_acm_tx_message.motor_back_raw, (const void*)&m_result[0], sizeof(cdc_acm_tx_message_t) - 2);

            if (cdc_acm_tx_message.crc16 ==
                crc_ccitt(0x0000, (const uint8_t*)&cdc_acm_tx_message, sizeof(cdc_acm_tx_message_t) - 2))
            {
                auto it = Map_Motors_p.find(cdc_acm_tx_message.motor_back_raw.ID);
                if (it != Map_Motors_p.end())
                {
                    it->second->fresh_data(cdc_acm_tx_message.motor_back_raw.position,
                                           cdc_acm_tx_message.motor_back_raw.velocity,
                                           cdc_acm_tx_message.motor_back_raw.torque);
                    ROS_INFO("END");
                }
                else
                {
                    ROS_ERROR("OUT RANGE");
                }
            }
            else
            {
                memset(&cdc_acm_tx_message.motor_back_raw, 0, sizeof(cdc_acm_tx_message_t) - 2);
                ROS_ERROR("CRC ERROR");
            }
        }
        else
        {
            ROS_ERROR("FRAME HEAD ERROR");
        }
#else
        // read avalible
        // this method is read the avaliable from the buffer
        m_result = m_serial.read(m_serial.available());
        if (m_result.length() > 0)
        {
            if (*(uint8_t*)&m_result[0] == 0xFD && *(uint8_t*)&m_result[1] == 0xFE)
            {
                if (*(uint16_t*)&m_result[15] ==
                    crc_ccitt(0x0000, (const uint8_t*)&m_result[0], sizeof(cdc_acm_tx_message_t) - 2))
                {
                    auto it = Map_Motors_p.find(*(uint8_t*)&m_result[2]);
                    if (it != Map_Motors_p.end())
                    {
                        it->second->fresh_data(
                            *(int32_t*)&m_result[3], *(int32_t*)&m_result[7], *(int32_t*)&m_result[11]);
                    }
                    else
                    {
                        // ROS_ERROR("OUT RANGE");
                    }
                }
                else
                {
                    // ROS_ERROR("CRC ERROR");
                }
            }
            else
            {
                // ROS_INFO_STREAM("else 3");
            }
        }
        else
        {
            // ROS_INFO_STREAM("else 2");
        }
        // r->sleep();
#endif
    }
}

void lively_serial::send(uint8_t ID, int32_t position, int32_t velocity, int32_t torque, int16_t Kp, int16_t Kd)
{
    // ROS_INFO_STREAM("START");
    cdc_acm_rx_message.motor_cmd.position = position;
    cdc_acm_rx_message.motor_cmd.velocity = velocity;
    cdc_acm_rx_message.motor_cmd.torque   = torque;
    cdc_acm_rx_message.motor_cmd.Kp       = Kp;
    cdc_acm_rx_message.motor_cmd.Kd       = Kd;
    cdc_acm_rx_message.motor_cmd.ID       = ID;
    cdc_acm_rx_message.crc16 = crc_ccitt(0x0000, (const uint8_t*)&cdc_acm_rx_message, sizeof(cdc_acm_rx_message_t) - 2);
    uint8_t* byte_ptr        = (uint8_t*)&cdc_acm_rx_message;
    // ROS_INFO_STREAM("STEP1");
    // for (size_t i = 0; i < sizeof(cdc_acm_rx_message_t); i++)
    // {
    //     printf("0x%02X ", byte_ptr[i]);
    // }
    // std::cout << std::endl;
    // ROS_INFO_STREAM("STEP2");
    m_serial.write((const uint8_t*)&cdc_acm_rx_message, sizeof(cdc_acm_rx_message));
    // ROS_INFO_STREAM("END"); // STEP2 -> END 1.7ms  START -> END 1.71
}

void lively_serial::send(cdc_acm_rx_message_t* _cdc_acm_rx_message)
{
    uint8_t* byte_ptr = (uint8_t*)_cdc_acm_rx_message;
    // ROS_INFO("STEP1 %x",_cdc_acm_rx_message);
    // for (size_t i = 0; i < sizeof(cdc_acm_rx_message_t); i++)
    // {
    //     printf("0x%02X ", byte_ptr[i]);
    // }
    // std::cout << std::endl;
    m_serial.write((const uint8_t*)_cdc_acm_rx_message, sizeof(cdc_acm_rx_message_t));
}

void lively_serial::init_map_motor(std::map<int, motor*>* _Map_Motors_p)
{
    Map_Motors_p = *_Map_Motors_p;
    rate         = new ros::Rate(Map_Motors_p.size() * 1100);
}

void lively_serial::test_ser_motor()
{
    for (const auto& pair : Map_Motors_p)
    {
        std::cout << "Key: " << pair.first << ", Value: " << pair.second->get_motor_id() << std::endl;
    }
    int  keyToFind = 4;
    auto it        = Map_Motors_p.find(keyToFind);

    if (it != Map_Motors_p.end())
    {
        // 找到了键为 keyToFind 的元素
        std::cout << "Found: " << it->first << " -> " << it->second->get_motor_id() << std::endl;
    }
    else
    {
        // 未找到键为 keyToFind 的元素
        std::cout << "Key " << keyToFind << " not found in the map." << std::endl;
    }
}
