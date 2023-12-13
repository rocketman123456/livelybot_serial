#include "ros/ros.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle n;
    ros::Rate r(400);
    robot rb;
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    float derta = 0.01;//角度
    int cont = 0;
    float angle = 0.0;
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        // ROS_INFO_STREAM("START");
        /////////////////////////send
        for (motor *m : rb.Motors)
        {
            m->fresh_cmd(angle, 0.0, 0.0, 1.0, 0.0);
        }
        angle+=derta;
        cont++;
        if(cont==1000)
        {
            cont = 0;
            derta*=-1;
        }
        rb.motor_send();
        ////////////////////////recv
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.torque);
        }
        // ROS_INFO_STREAM("END"); //
        r.sleep();
    }

    for (auto &thread : rb.ser_recv_threads)
    {
        thread.join();
    }

    ros::spin();
    return 0;
}
