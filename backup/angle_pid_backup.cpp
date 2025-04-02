/*
单角度环pid算法验证的main.cpp内容备份,直接复制到main.cpp覆盖即可
这个代码的模型对应小车摆杆顺时针为正,向右为正的模型
*/
#include <iostream>
#include <math.h>
#include <fstream>
#include "pendulum.h"
#include "pid.h"
#include <bits/algorithmfwd.h>

int main() {
    // 初始化系统
    Invertedpendulum pendulum(1.0, 0.3, 0.5, -5.0);          // 1kg小车，0.3kg摆杆 控制力作用下,摩擦系数必须为负才有效
    PID_Controller angle_pid(150.0, 0.0, 50.0, 30);          // 倒立摆角度内环

    // 数据记录
    std::ofstream data_file("../output/simulation.csv");
    data_file << "time,x,theta,control_input\n";

    // 模拟循环
    const double dt = 0.01;  // 10ms时间步长
    double time = 0.0;

    // 初始条件：偏离平衡位置20度
    pendulum.setState(0.0, 0.0, 20.0*M_PI/180.0, 0);

    // 模拟循环
    while (time < 50.0) {  // 模拟50秒
        // 获取当前状态
        const double* state = pendulum.getState();

        // 角度控制:内环
        double control_pid = angle_pid.update(0.0*M_PI/180.0, state[2], dt);  //将外环计算的角度控制量传入内环

        // 更新系统状态
        pendulum.update(-control_pid, dt);  //注意,这里是-control_pid才有用

        // 输出数据（可保存为CSV文件进行可视化）
        std::cout << time << ","
                  << state[0] << ","  // 小车位置
                  << state[2]*180/M_PI // 摆杆角度（度）
                  << std::endl;

         data_file<< time << ","
                  << state[0] << ","  // 小车位置
                  << state[2]*180/M_PI // 摆杆角度（度）
                  << std::endl;
        time += dt;
    }
    data_file.close();
    std::cout << "Simulation completed. Data saved to simulation.csv" << std::endl;
    return 0;
}
