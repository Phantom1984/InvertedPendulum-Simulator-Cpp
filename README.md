### Brief

这是一个用C++模拟一级小车倒立摆的控制程序。目前已实现：

- [x] 单级角度环PID
- [x] LQR
- [ ] 位置环PID+角度环PID
- [ ] 速度环PID+角度环PID
- [ ] 滑模控制
- [ ] ......

该项目旨在练习C++编程实践与控制算法模拟，后续将进行可视化的研究。

推荐在VSCode里进行编程，并使用CMake构建项目。项目采用GBK编码格式，如果是其它编码打开可能导致乱码

### 项目结构

backup：里面存放各类算法的main.cpp函数内容，选择想要的算法，直接复制粘贴进src/main.cpp即可

- angle_pid_backup.cpp：单级角度环PID控制，采用摆杆顺时针为正、小车向右为正的模型
- angle2_pid_backup.cpp：单级角度环PID控制，采用摆杆逆时针为正、小车向右为正的模型
- lqr_pid_backup.cpp：LQR控制，采用摆杆顺时针为正、小车向右为正的模型
- lqr2_pid_backup.cpp：LQR控制，采用摆杆逆时针为正、小车向右为正的模型

inc：头文件存放目录

output：模拟结果输出目录

- plot_simulation.py：结果可视化脚本
- simulation.csv：模拟结果数据文件

src：源文件目录

一阶倒立摆.md：模型文档

CMakeLists.txt

REAMDE.md

### Requirement

Eigen 3.4.0

Python 3.8 + matplotlib