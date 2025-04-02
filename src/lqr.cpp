#include "lqr.h"
#include <iostream>

LQR_Controller::LQR_Controller()
{
    //初始化矩阵为空
    A_ = Eigen::MatrixXd();//动态矩阵
    B_ = Eigen::MatrixXd();
    Q_ = Eigen::MatrixXd();
    R_ = Eigen::MatrixXd();
    K_ = Eigen::MatrixXd();
}

void LQR_Controller::setSystemMatrix(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
    A_ = A;
    B_ = B;
}

void LQR_Controller::setWeightMatrix(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
{
    Q_ = Q;
    R_ = R;
}

bool LQR_Controller::computeGainMatrix(int max_iter_, double tolerance_, double dt)
{
    //计算增益矩阵K,求解离散黎卡提方程
    Eigen::MatrixXd P_k = Q_;
    Eigen::MatrixXd P_knext;
    int max_iter = max_iter_;    //设置最大迭代次数
    double tolerance = tolerance_;
    bool converged = false;     // 添加收敛标志
    //计算离散化矩阵
    Eigen::MatrixXd Ad = A_ * dt + Eigen::MatrixXd::Identity(A_.rows(),A_.cols());
    Eigen::MatrixXd Bd = B_ * dt;

    for(int i=0;i<max_iter;i++)
    {
        P_knext = Ad.transpose()*P_k*Ad - Ad.transpose()*P_k*Bd*(R_+Bd.transpose()*P_k*Bd).inverse()*Bd.transpose()*P_k*Ad+Q_;
        if((P_knext - P_k).norm() < tolerance)
        {
            converged = true;
            break;
        }
        P_k = P_knext;
    }
    if (!converged)
    { // 未收敛处理
        std::cerr << "LQR: Riccati iteration did not converge!" << std::endl;
        return false;
    }
    //计算增益矩阵K,采用离散形式计算
    K_ = (R_ + Bd.transpose()*P_knext*Bd).inverse()*Bd.transpose()*P_knext*Ad;
    return true;
}

Eigen::VectorXd LQR_Controller::computeControl(const Eigen::VectorXd& x)const
{
    return -K_* x;
}