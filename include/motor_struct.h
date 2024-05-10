#pragma once

#include <stdint.h>

#pragma pack(1)

using motor_cmd_t = struct motor_cmd_struct
{
    uint8_t ID;
    int32_t position;
    int32_t velocity;
    int32_t torque;
    int16_t Kp;
    int16_t Kd;
};

using motor_back_t = struct motor_back_struct
{
    uint8_t ID;
    float   position;
    float   velocity;
    float   torque;
};

using motor_back_raw_t = struct motor_back_raw_struct
{
    uint8_t ID;
    int32_t position;
    int32_t velocity;
    int32_t torque;
};

#pragma pack()
