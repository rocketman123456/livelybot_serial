#include "ros/ros.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_serial_port");
    ros::NodeHandle n;
    
    robot rb;
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== read now ==============================
    int cont = 0;
    int motor_num = 0;
    float now_motor_pos [10] = {0.0};
    float derta_motor_pos [10];
    ros::Rate r_r(10);
    while (ros::ok() && cont < 10)
    {
        motor_num = 0;
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            now_motor_pos[m->get_motor_num()] = motor.position;
            motor_num++;
            // ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.torque);
        }
        cont++;
        r_r.sleep();
    }
    for (size_t i = 0; i < 10; i++)
    {
        derta_motor_pos[i] = now_motor_pos[i]/500;
    }
    // ========================== slow zero =============================
    ros::Rate s_r(500);
    cont = 0;
    float Kp = 0.0;
    while (ros::ok() && cont < 500)
    {
        for (motor *m : rb.Motors)
        {  
            now_motor_pos[m->get_motor_num()] -= now_motor_pos[m->get_motor_num()];
            Kp+=0.04;
            m->fresh_cmd(now_motor_pos[m->get_motor_num()], 0.0, 0.0, Kp, 0.0);
        }
        rb.motor_send();
        cont++;
        s_r.sleep();
    }
    // ========================== singlethread send =====================
    ros::Rate rc_rate(500);
    float target_height = 0.0; // mm
    float derta_hight = 0.001; // mm
    cont = 0;
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        // ROS_INFO_STREAM("START");
        /////////////////////////calculate
        target_height += derta_hight;
        cont++;
        if (cont == 2000)
        {
            cont = 0;
            derta_hight *= -1.0;
        }
        //////calculate out the motor torque and target pos???????????
        
        /////////////////////////send  // send controll data
        for (motor *m : rb.Motors)
        {
            m->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.0);
        }
        rb.motor_send();
        ////////////////////////recv //not important
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            // ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.velocity);
        }
        // ROS_INFO_STREAM("END"); //
        rc_rate.sleep();
    }
    for (auto &thread : rb.ser_recv_threads)
    {
        thread.join();
    }
    ros::spin();
    return 0;
}
