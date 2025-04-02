/*
纯lqr算法验证的main.cpp内容备份,直接复制到main.cpp覆盖即可
这个代码的模型对应小车摆杆逆时针为正,向右为正的模型
*/
#include <iostream>
#include <math.h>
#include <fstream>
#include "pendulum.h"
#include "lqr.h"
#include <bits/algorithmfwd.h>

int main() {
    // 初始化系统
    Invertedpendulum pendulum(1.0, 0.3, 0.5, 0.0);         // 1kg小车，0.3kg摆杆 
    
    LQR_Controller lqr;

    Eigen::MatrixXd A(4, 4); //定义系统矩阵
    Eigen::VectorXd B(4); //定义控制矩阵(向量)

    //获取模型参数
    double m = pendulum.getPoleMass();
    double M = pendulum.getCarMass();
    double J = pendulum.getInertia();
    double l = pendulum.getPoleLength();
    double g = pendulum.getGravity();
    double b = pendulum.getFriction();

    //分母
    double D = (M+m)*(J+m*pow(l,2))-pow(m*l,2);  

    //定义系统矩阵,连续形式即可,因为lqr里面已经有离散化计算步骤了
    A << 0,       1,            0,         0,
         0, -(J+m*l*l)*b/D, m*m*l*l*g/D,   0,
         0,       0,            0,         1,
         0,   -m*l*b/D,    (M+m)*m*g*l/D,  0;

    B << 0, (J+m*l*l)/D, 0, m*l/D;

    //导入系统矩阵A和控制矩阵B到lqr控制器中
    lqr.setSystemMatrix(A, B);

    //定义QR矩阵
    Eigen::MatrixXd Q = Eigen::Matrix4d::Identity(4, 4);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);
    Eigen::VectorXd x(4);

    Q.diagonal() << 10, 1, 100, 10;  // 加强角度和位置权重
    R << 0.1;  // 降低控制量权重

    //导入矩阵Q和矩阵R到lqr控制器中
    lqr.setWeightMatrix(Q, R);

    // 数据记录
    std::ofstream data_file("../output/simulation.csv");
    data_file << "time,x,theta,control_input\n";

    // 模拟循环
    const double dt = 0.01;  // 10ms时间步长
    double time = 0.0;
    
    //计算增益矩阵
    bool flag = lqr.computeGainMatrix(1000, 1e-6, dt);

    // 初始条件：偏离平衡位置5度
    pendulum.setState(0.0, 0, 20.0*M_PI/180.0, 0);

    // 模拟循环
    while (time < 50.0) {  // 模拟50秒
        // 获取当前状态
        const double* state = pendulum.getState();

        double x_ref = 5.0;   //设定稳定点位置
        x << state[0] - x_ref, state[1], state[2], state[3];  // 输入状态向量

        Eigen::VectorXd control_lqr; // 定义lqr控制输入量
        if(flag) {
            control_lqr = lqr.computeControl(x);
        } else {
            std::cout<<"lqr uncoveraged!"<<std::endl;
            break;
        }

        // 更新系统状态
        pendulum.update(control_lqr(0), dt);
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
