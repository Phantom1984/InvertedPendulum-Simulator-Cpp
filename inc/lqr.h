#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <Eigen/Dense>

class LQR_Controller {
public:
    LQR_Controller();

    /**
     * @brief 设置系统状态空间矩阵
     * @param A 系统状态矩阵（n x n）
     * @param B 控制输入矩阵（n x m）
     */
    void setSystemMatrix(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

    /**
     * @brief 设置权重矩阵
     * @param Q 状态权重矩阵（n x n，需对称半正定）
     * @param R 控制输入权重矩阵（m x m，需对称正定）
     */
    void setWeightMatrix(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

    /**
     * @brief 计算 LQR 增益矩阵 K
     * @param max_iter_ 最大迭代次数,可取1000
     * @param tolerance_ 迭代容差,可取1e-6
     * @param dt 采样时间
     * @return 计算成功返回 true，失败返回 false
     */
    bool computeGainMatrix(int max_iter_, double tolerance_, double dt);

    /**
     * @brief 计算控制量 u = -Kx
     * @param x 当前状态向量（n x 1）
     * @return 控制输入向量（m x 1）
     */
    Eigen::VectorXd computeControl(const Eigen::VectorXd& x) const;

private:
    Eigen::MatrixXd A_;           // 状态矩阵
    Eigen::MatrixXd B_;           // 控制输入矩阵
    Eigen::MatrixXd Q_;           // 状态权重矩阵
    Eigen::MatrixXd R_;           // 控制输入权重矩阵
    Eigen::MatrixXd K_;           // LQR 增益矩阵
    bool is_initialized_ = false; // 增益矩阵是否已计算
};

#endif // LQR_CONTROLLER_H