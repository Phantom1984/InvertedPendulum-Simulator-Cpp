#include "pendulum.h"
#include "control.h"
#include <cmath>

Invertedpendulum::Invertedpendulum(double car_mass,double pole_mass,
                                   double length,double friction)
    :m_car(car_mass),m_pole(pole_mass),l_pole(length),B(friction),g_acc(9.8),J((0.333)*m_pole*pow(l_pole, 2)){}

void Invertedpendulum::setState(double x,double x_dot,double theta,double theta_dot)
{
    state[0]=x;
    state[1]=x_dot;
    state[2]=theta;
    state[3]=theta_dot;
}
void Invertedpendulum::computeDerivatives(const double* state, double* dsdt, double control_input)
{
    //只要是用于计算而不应被修改的变量(只读),最好都加const
    const double theta = state[2];
    const double theta_dot = state[3];
    const double total_mass = m_car + m_pole;//总质量
    const double sin_theta = sin(theta);
    const double cos_theta = cos(theta);
    
    /*
    状态变量微分方程的更新采取工程上常用的四阶龙格-库塔法,
    需要对状态取微分,那么会有四个微分变量;
    此时结合已有的数学模型编写即可.在编写过程中,先写数学模型公式,后面缺什么变量加上就好
    */
    /*倒立摆数学模型,不同正方向的模型*/
    //分母
    double fenmu = (m_car + m_pole) * (J + m_pole * pow(l_pole, 2)) - pow(m_pole*l_pole*cos_theta,2);

    // 角度顺时针为正,力方向向左为正的模型表达
    // 加速度计算
    // double acc = (
    //     + (J + m_pole * pow(l_pole, 2)) * B * state[1] // 摩擦力项（B 应为正数）
    //     + m_pole * l_pole * (J + m_pole * pow(l_pole, 2)) * sin_theta * pow(theta_dot, 2)
    //     - (J + m_pole * pow(l_pole, 2)) * control_input // 控制力直接使用（向右为正）
    //     - pow(m_pole * l_pole, 2) * g_acc * cos_theta * sin_theta
    // ) / fenmu;
    // // 角加速度计算
    // double theta_acc = (
    //     - m_pole * l_pole * cos_theta * B * state[1] 
    //     + pow(m_pole * l_pole, 2) * sin_theta * cos_theta * pow(theta_dot, 2)
    //     + m_pole * l_pole * cos_theta * control_input // 控制力矩方向正确
    //     + total_mass * m_pole * g_acc * l_pole * sin_theta
    // ) / fenmu;

    // 角度顺时针为正,力方向向右为正的模型表达
    // 加速度计算
    // double acc = (
    //     - (J + m_pole * pow(l_pole, 2)) * B * state[1] // 摩擦力项（B 应为正数）
    //     + m_pole * l_pole * (J + m_pole * pow(l_pole, 2)) * sin_theta * pow(theta_dot, 2)
    //     + (J + m_pole * pow(l_pole, 2)) * control_input // 控制力直接使用（向右为正）
    //     - pow(m_pole * l_pole, 2) * g_acc * cos_theta * sin_theta
    // ) / fenmu;
    // // 角加速度计算
    // double theta_acc = (
    //     + m_pole * l_pole * cos_theta * B * state[1] 
    //     + pow(m_pole * l_pole, 2) * sin_theta * cos_theta * pow(theta_dot, 2)
    //     - m_pole * l_pole * cos_theta * control_input // 控制力矩方向正确
    //     + total_mass * m_pole * g_acc * l_pole * sin_theta
    // ) / fenmu;

    // 角度逆时针为正,力方向向右为正的模型表达
    // 加速度计算
    double acc = (
        - (J + m_pole * pow(l_pole, 2)) * B * state[1] // 摩擦力项（B 应为正数）
        - m_pole * l_pole * (J + m_pole * pow(l_pole, 2)) * sin_theta * pow(theta_dot, 2)
        + (J + m_pole * pow(l_pole, 2)) * control_input // 控制力直接使用（向右为正）
        + pow(m_pole * l_pole, 2) * g_acc * cos_theta * sin_theta
    ) / fenmu;
    // 角加速度计算
    double theta_acc = (
        - m_pole * l_pole * cos_theta * B * state[1] //必须去掉这里的摩擦项才可以使得摩擦为正时稳定,否则必须是负的才可稳定
        - pow(m_pole * l_pole, 2) * sin_theta * cos_theta * pow(theta_dot, 2)
        + m_pole * l_pole * cos_theta * control_input // 控制力矩方向正确
        + total_mass * m_pole * g_acc * l_pole * sin_theta
    ) / fenmu;

    dsdt[0] = state[1];  //速度v
    dsdt[1] = acc;       //加速度v_dot
    dsdt[2] = theta_dot; //角速度
    dsdt[3] = theta_acc; //角加速度
}
void Invertedpendulum::update(double control_input,double dt)
{
    /*使用四阶龙格库塔法更新微分方程,直接照着公式写代码即可*/
    double k1[4],k2[4],k3[4],k4[4],temp[4];
    
    //计算k1,由于k1在函数外已声明,因此计算结果存储在k1中
    computeDerivatives(state,k1,control_input);

    //计算k2,首先要计算结合k1的导数,由于state是私有变量,设一个temp代替
    for(int i=0;i<4;i++)
        temp[i] = state[i] + 0.5*dt*k1[i];
    computeDerivatives(temp,k2,control_input);

    //计算k3,道理类似
    for(int i=0;i<4;i++)
        temp[i] = state[i] + 0.5*dt*k2[i];
    computeDerivatives(temp,k3,control_input);

    //计算k4
    for(int i=0;i<4;i++)
        temp[i] = state[i] + dt*k3[i];
    computeDerivatives(temp,k4,control_input);

    //状态更新
    for(int i=0;i<4;i++)
        state[i] = state[i] + (dt/6.0)*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
}

double Invertedpendulum::normalizeAngle(double angle)
{
        // // 计算与360的浮点余数
        angle = std::fmod(angle + M_PI, 2*M_PI);
        return (angle < 0) ? angle + 2*M_PI : angle - M_PI;
}
