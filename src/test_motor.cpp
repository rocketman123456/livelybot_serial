#include "hardware/robot.h"
#include "serial_struct.h"

#include <condition_variable>
#include <iostream>
#include <ros/ros.h>
#include <thread>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle n;
    ros::Rate       rate(400);
    robot           rb;
    ROS_INFO("\033[1;32mSTART\033[0m");

    // ========================== singlethread send =====================
    int       temp_count = 0;
    ros::Rate temp_rate(100);
    while (ros::ok() && temp_count < 100)
    {
        for (size_t i = 0; i < 20; i++)
        {
            rb.Motors[i]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.01);
        }
        rb.motor_send();
        temp_count++;
        temp_rate.sleep();
    }

    // ========================== singlethread send =====================
    float dt    = 0.001;
    float time  = 0.0;
    float angle = 0.314;
    float pos   = 0.0;
    int   index = 0;

    motor* m = rb.m_motors[index];

    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        pos = angle * sin(time);
        m->fresh_cmd(pos, 0.0, 0.0, 0.1, 0.01);
        time += dt;

        rb.motor_send();

        ROS_INFO("\033[1;32m Motor Position %f. \033[0m", pos);

        for (motor* m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            // ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.torque);
        }

        rate.sleep();
    }

    for (auto& thread : rb.m_receive_threads)
    {
        thread.join();
    }

    ros::spin();
    return 0;
}
