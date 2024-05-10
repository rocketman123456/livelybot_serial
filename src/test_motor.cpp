#include "hardware/robot.h"
#include "ros/init.h"
#include "serial_struct.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle n;
    ros::Rate       rate(500);
    RobotDriver     robot;
    ROS_INFO("\033[1;32mSTART\033[0m");

    // ========================== singlethread send =====================
    for (size_t i = 0; i < 20; i++)
    {
        robot.m_motors[i]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.01);
    }

    // ========================== singlethread send =====================
    float dt    = 0.01;
    float time  = 0.0;
    float angle = 10.0;
    float pos   = 0.0;
    int   index = 19;

    auto m = robot.m_motors[index];

    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        pos = angle * sin(time);
        // m->fresh_cmd(pos, 0.0, 0.0, 10.0, 0.01);

        for (auto& m : robot.m_motors)
        {
            // motor_back_t* motor;
            // motor = m->get_current_motor_state();
            m->fresh_cmd(0.0, 0.0, 0.0, 0.8, 0.01);
            // ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.torque);
        }

        time += dt;

        robot.motor_send();

        // ROS_INFO("\033[1;32m Motor Position %f. \033[0m", pos);

        for (auto& m : robot.m_motors)
        {
            motor_back_t* motor;
            motor = m->get_current_motor_state();
            // ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.torque);
        }

        rate.sleep();
    }

    for (auto& thread : robot.m_receive_threads)
    {
        thread.join();
    }

    // finalize
    for (size_t i = 0; i < 20; i++)
    {
        robot.m_motors[i]->fresh_cmd(0.0, 0.0, 0.0, 0.0, 0.01);
    }
    robot.motor_send();

    ros::spin();

    ros::shutdown();
    return 0;
}
