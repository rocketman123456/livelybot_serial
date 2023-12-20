#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <string>
#include <stdlib.h>
#include "ros/ros.h"
#include "jacobia.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
// 一个简单的demo，实现机器人站起来/蹲下的操作
bool running = true;
#define PI 3.1415926
void ShutDown(int sig)
{
    std::cout << "stop" << std::endl;
    running = false;
    system("stty sane"); // Terminal back to normal
    exit(0);
}
void set_motor(ros::NodeHandle nh)
{
    std::vector<std::string> _motor_names, side;
    _motor_names.push_back("_hip");
    _motor_names.push_back("_hip2");
    _motor_names.push_back("_thigh");
    _motor_names.push_back("_calf");
    _motor_names.push_back("_toe");
    side.push_back("L");
    side.push_back("R");
    int ID[5] = {5, 4, 3, 2, 1};
    int CAN[2] = {0x10, 0x20};
    int num[5] = {0, 1, 2, 3, 4};
    int count_side = 0;
    int count_motor = 0;
    for (std::string _side : side)
    {
        for (std::string _motor_name : _motor_names)
        {
            std::cout << _side + _motor_name << std::endl;
            nh.setParam(_side + _motor_name + "_ID", CAN[count_side] | ID[count_motor]);
            nh.setParam(_side + _motor_name + "_num", num[count_motor] + count_side * 5);
            count_motor++;
        }
        count_motor = 0;
        count_side++;
    }
}
int main(int argc, char **argv)
{
    // IOInterface *ioInter;
    ros::init(argc, argv, "pai_control", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    set_motor(nh);
    ros::Rate rate(1000);
    double dt = 0.001;
    std::string robot_name = "livelybot";

    std::cout << "init ok" << std::endl;
    signal(SIGINT, ShutDown);
    robot rb;
    ///////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////jocbia////////////////////////////////////////////////
    // jacobia left_j("left_leg_force");
    jacobia j;
    Eigen::MatrixXd left_jacobia;
    Eigen::MatrixXd right_jacobia;
    std::vector<double> joint_value_l, joint_value_r;
    std::vector<std::vector<double>> joint_value;
    for (size_t i = 0; i < 5; i++)
    {
        joint_value_l.push_back(0.0);
    }
    for (size_t i = 0; i < 5; i++)
    {
        joint_value_r.push_back(0.0);
    }
    joint_value.push_back(joint_value_l);
    joint_value.push_back(joint_value_r);

    ///////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////simple control////////////////////////////////////////
    double Fz, hd, h, old_h, Kz, d_hd, d_h, d_Kz;
    old_h = hd = h = 0.3825;
    Kz = 900;
    d_Kz = 40;
    double derta = 0.00002; // 2cm/s
    double derta_t = 0.001;
    d_hd = -derta / derta_t;

    int cont = 0;
    int cont_2 = 0;
    int leg = 0;
    int num = 0;
    while (ros::ok())
    {
        /////////////////////////////fresh target hight///////////////////
        /////////////////////////////calculate F/T ///////////////////////
        //  因为机old_h心在脚底板中心正上方,Fx=0,Fy=0,Tx=0,Ty=0,Tz=0
        //  J(^T)*[Fx Fy Fz Tx Ty Tz](^T) = torque
        printf("=====================================\n");
        // hd -= derta;
        cont++;
        if (cont == 1000)
            derta *= -1;
        cont = 0;

        for (motor *m : rb.Motors)
        {
            joint_value[m->get_motor_num()/5][m->get_motor_num()%5-1] = m->get_current_motor_state()->position;
        }
        j.getJacobian(joint_value);

        h = sqrtf32(0.048*cosf32(joint_value[0][3])-0.0481);
        d_h = (h - old_h) / derta_t;
        Fz = Kz * (h - hd) - 6; //- d_Kz * (d_h - d_hd) ;
        old_h = h;
        std::cout << Kz * (h - hd) << " " << -d_Kz * (d_h - d_hd) << std::endl;
        std::cout << "now pos:" << h << " now vel:" << d_h << "\n"
                  << "target pos:" << hd << " target vel:" << d_hd << "\n";
        std::cout << Fz << "\n";
        Eigen::VectorXd colVector(6);
        colVector << 0.0, 0.0, Fz, 0.0, 0.0, 0.0;

        Eigen::VectorXd right_motor_torque = j.getRight_torque(colVector).cwiseMax(8).cwiseMin(-8);
        Eigen::VectorXd left_motor_torque = j.getLeft_torque(colVector).cwiseMax(8).cwiseMin(-8);
        std::cout << "right torque:\n"
                  << right_motor_torque << std::endl;
        std::cout << "left torque:\n"
                  << left_motor_torque << std::endl;
        if (cont_2 < 5)
        {
            cont_2++;
        }
        else
        {
            for (size_t i = 0; i < 5; i++)
            {
                rb.Motors[i]->fresh_cmd(0.0,0.0,left_motor_torque[i],0.0,0.0);
                rb.Motors[i+5]->fresh_cmd(0.0,0.0,right_motor_torque[i],0.0,0.0);
            }
            // cmd->motorCmd[2].tau*=1.2;
            // cmd->motorCmd[7].tau*=1.2;
        }
        rb.motor_send();
        rate.sleep();
    }
    return 0;
}
