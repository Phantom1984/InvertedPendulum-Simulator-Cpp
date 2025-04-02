//倒立摆模型类
#ifndef PENDULUM_H
#define PENDULUM_H

class Invertedpendulum
{
public:
    // 参数导入:小车质量、摆杆质量、摆长、摩擦系数
    // 使用构造函数导入参数
    Invertedpendulum(double car_mass,
                     double pole_mass,
                     double length,double friction);

    // 更新系统状态（使用四阶龙格库塔法）
    void update(double control_input,double dt);

    // 设置当前状态
    void setState(double x, double x_dot, double theta, double theta_dot);
  
    // 获取当前状态 [x, x_dot, theta, theta_dot]
    const double* getState() const{return state;}

    //角度限幅至[-pi, pi]
    double normalizeAngle(double angle);
    
    //获取当前模型参数
    double getCarMass() const { return m_car; }
    double getPoleMass() const { return m_pole; }
    double getPoleLength() const { return l_pole; }
    double getFriction() const { return B; }
    double getInertia() const { return J; }
    double getGravity() const { return g_acc; }
    
private:
    /*
    命名规则(暂时):
    前缀:m--质量,l--长度
    后缀:对象名
    */
    double m_car;    //小车质量M
    double m_pole;   //摆杆质量m
    double l_pole;   //摆杆长度l
    double B;        //摩擦系数B
    double state[4]; //系统状态变量
    double g_acc;    //重力加速度g
    double J;        //摆杆转动惯量J
    /*
    将微分计算设为私有,在类内访问,也就是给update使用,免得让外部的东西干扰计算
    如果将 computeDerivatives 设为私有，以后你可以轻松更改其内部逻辑，而不会影响外部调用。
    比如：
    如果你要改用其他数值方法（如欧拉法、隐式积分等）来更新状态，
    你只需要修改 computeDerivatives 和 update 函数，而不需要修改外部代码。
    如果未来需要扩展状态变量，比如考虑多摆系统，你只需要改内部逻辑，不用担心对用户的接口造成影响。
    由于这个方法只在使用倒立摆类时使用,所以,也可以使用lambda表达式.
    */
    void computeDerivatives(const double* state, double* dsdt, double control_input);
};

#endif