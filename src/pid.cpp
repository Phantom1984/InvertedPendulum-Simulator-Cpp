#include "pid.h"
#include <algorithm>
#include <cmath>

PID_Controller::PID_Controller(double pid_kp,double pid_ki,double pid_kd,double pid_max_out)
:kp(pid_kp),ki(pid_ki),kd(pid_kd),max_out(pid_max_out)
,Integral(0),prev_error(0){}

void PID_Controller::reset()
{
    Integral = 0.0;
    prev_error = 0.0;
}

double PID_Controller::update(double setval,double fd,double dt)
{
    double error = setval - fd;

    Integral += error*dt;
    double Derivatives = (error-prev_error)/dt;
    double output = kp*error+ki*Integral+kd*Derivatives;
    prev_error = error;                          
    return std::clamp(output,-max_out,max_out);            //输出限幅
}