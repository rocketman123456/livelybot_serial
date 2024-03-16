#ifndef _SERIAL_STRUCT_H_
#define _SERIAL_STRUCT_H_

#include "crc/crc16.h"
#include "motor_struct.h"

#include <stdint.h>

/* struct */
#pragma pack(1)

using cdc_acm_tx_message_t = struct cdc_acm_tx_message_struct
{
    uint8_t          head[2]; // 0xFD 0xFE
    motor_back_raw_t motor_back_raw;
    uint16_t         crc16;
};

using cdc_acm_rx_message_t = struct cdc_acm_rx_message_struct
{
    uint8_t     head[2]; // 0xFE 0xFD
    motor_cmd_t motor_cmd;
    uint16_t    crc16;
};

#pragma pack()
/* class*/

#endif