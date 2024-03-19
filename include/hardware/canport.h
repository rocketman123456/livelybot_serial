#ifndef _CANPORT_H_
#define _CANPORT_H_

#include "hardware/motor.h"
#include "lively_serial.h"

#include <condition_variable>
#include <ros/ros.h>
#include <thread>

class CANPort
{
private:
    int                        motor_num;
    int                        CANport_num;
    ros::NodeHandle            n;
    std::vector<Motor*>        Motors;
    std::map<int, Motor*>      Map_Motors_p;
    bool                       sendEnabled;
    int                        canboard_id, canport_id;
    std::vector<motor_back_t*> Motor_data;
    lively_serial*             ser;

public:
    CANPort(int canport_num, int canboard_num, lively_serial* _ser);
    ~CANPort();

    void puch_motor(std::vector<Motor*>* _Motors);
    void push_motor_data();
    void motor_send();

    int get_motor_num() { return motor_num; }
    int get_canboard_id() { return canboard_id; }
    int get_canport_id() { return canport_id; }
};
#endif