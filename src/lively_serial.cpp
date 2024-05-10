#include "lively_serial.h"
#include "crc/crc16.h"

lively_serial::lively_serial(const std::string& port, uint32_t baudrate, uint8_t debug_level)
    : m_port(port)
    , m_baudrate(baudrate)
    , m_debug_level(debug_level)
{
    m_serial.setPort(m_port); // 设置打开的串口名称
    m_serial.setBaudrate(m_baudrate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); // 创建timeout
    m_serial.setTimeout(timeout);                                   // 设置串口的timeout
    // 打开串口
    try
    {
        m_serial.open();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
    }

    if (m_serial.isOpen())
    {
        ROS_INFO_STREAM("IMU Serial Port initialized."); // 成功打开串口，打印信息
    }
    else
    {
        ROS_ERROR_STREAM("IMU Serial Port failed to initialize."); // 成功打开串口，打印信息
    }
    m_tx_message.head[1] = 0xFE;
    m_rx_message.head[0] = 0xFE;
    m_tx_message.head[0] = 0xFD;
    m_rx_message.head[1] = 0xFD;
    m_init_flag          = true;
}

lively_serial::~lively_serial()
{
    if (m_serial.isOpen())
    {
        m_serial.close();
    }
}

void lively_serial::recv()
{
    // TODO : this method is read one bytes
    ROS_INFO_STREAM("start thread");
    while (ros::ok() && m_init_flag)
    {
        m_result = m_serial.read(2);
        if (*(uint8_t*)&m_result[0] == 0xFD && *(uint8_t*)&m_result[1] == 0xFE)
        {
            m_result = m_serial.read(sizeof(cdc_acm_tx_message_t) - 2);
            memcpy(&m_tx_message.motor_back_raw, (const void*)&m_result[0], sizeof(cdc_acm_tx_message_t) - 2);

            uint16_t crc_result = crc_ccitt(0x0000, (const uint8_t*)&m_tx_message, sizeof(cdc_acm_tx_message_t) - 2);
            if (m_tx_message.crc16 == crc_result)
            {
                auto it = m_motor_map.find(m_tx_message.motor_back_raw.ID);
                if (it != m_motor_map.end())
                {
                    it->second->fresh_data(
                        m_tx_message.motor_back_raw.position,
                        m_tx_message.motor_back_raw.velocity,
                        m_tx_message.motor_back_raw.torque
                    );
                }
                else
                {
                    ROS_ERROR("OUT RANGE");
                }
            }
            else
            {
                memset(&m_tx_message.motor_back_raw, 0, sizeof(cdc_acm_tx_message_t) - 2);
                ROS_ERROR("CRC ERROR");
            }
        }
        else
        {
            ROS_ERROR("FRAME HEAD ERROR");
        }
    }
}

void lively_serial::send(uint8_t ID, int32_t position, int32_t velocity, int32_t torque, int16_t Kp, int16_t Kd)
{
    m_rx_message.motor_cmd.position = position;
    m_rx_message.motor_cmd.velocity = velocity;
    m_rx_message.motor_cmd.torque   = torque;
    m_rx_message.motor_cmd.Kp       = Kp;
    m_rx_message.motor_cmd.Kd       = Kd;
    m_rx_message.motor_cmd.ID       = ID;
    m_rx_message.crc16 = crc_ccitt(0x0000, (const uint8_t*)&m_rx_message, sizeof(cdc_acm_rx_message_t) - 2);
    m_serial.write((const uint8_t*)&m_rx_message, sizeof(m_rx_message));
}

void lively_serial::send(cdc_acm_rx_message_t* rx_message)
{
    m_serial.write((const uint8_t*)rx_message, sizeof(cdc_acm_rx_message_t));
}

void lively_serial::init_map_motor(const std::map<int, std::shared_ptr<Motor>>& motor_map)
{
    m_motor_map = motor_map;
    // m_rate      = new ros::Rate(m_motor_map.size() * 1100);
}

// void lively_serial::test_ser_motor()
// {
//     for (const auto& pair : Map_m_motors_p)
//     {
//         std::cout << "Key: " << pair.first << ", Value: " << pair.second->get_motor_id() << std::endl;
//     }

//     int  keyToFind = 4;
//     auto it        = Map_m_motors_p.find(keyToFind);

//     if (it != Map_m_motors_p.end())
//     {
//         // 找到了键为 keyToFind 的元素
//         std::cout << "Found: " << it->first << " -> " << it->second->get_motor_id() << std::endl;
//     }
//     else
//     {
//         // 未找到键为 keyToFind 的元素
//         std::cout << "Key " << keyToFind << " not found in the map." << std::endl;
//     }
// }
