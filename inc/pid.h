#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID_Controller{
public:
    PID_Controller(double pid_kp,double pid_ki,double pid_kd,double pid_max_out);
    double update(double setval,double fd,double dt);//override关键字表示这是重写了父类功能
    void reset();

private:
    double kp;
    double ki;
    double kd;
    double max_out;

    double Integral;//积分值
    double prev_error;//前一时刻的误差，用于微分项计算
};

#endif