# 一阶倒立摆控制算法模拟器

[![CMake](https://img.shields.io/badge/build-CMake-brightgreen)](https://cmake.org/)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## 项目概述

本项目为基于C++的一阶小车倒立摆控制系统仿真平台，旨在提供多种控制算法的实现与对比分析。通过物理建模与数值积分，模拟倒立摆的动态响应过程。

## 🚀 已实现功能

### 核心特性
- ​**多控制算法集成**
  - ✅ 单角度环PID控制（两种模型方向）
  - ✅ LQR线性二次调节器
  - 🚧 级联PID（位置环/速度环 + 角度环）
  - 🚧 滑模控制

- ​**可扩展架构**
  - 📁 模块化代码结构
  - 📊 数据记录与可视化支持
  - 🔄 模型参数快速配置

### 算法实现
| 算法名称                  | 文件路径                          | 模型方向                     |
|--------------------------|-----------------------------------|----------------------------|
| 单角度环PID (顺时针模型) | `backup/angle_pid_backup.cpp`     | 摆杆顺时针为正，小车向右为正 |
| 单角度环PID (逆时针模型)  | `backup/angle2_pid_backup.cpp`    | 摆杆逆时针为正，小车向右为正 |
| LQR控制 (顺时针模型)      | `backup/lqr_pid_backup.cpp`      | 摆杆顺时针为正，小车向右为正 |
| LQR控制 (逆时针模型)      | `backup/lqr2_pid_backup.cpp`     | 摆杆逆时针为正，小车向右为正 |

## 🛠️ 快速开始

### 环境要求
- ​**编译环境**
  - C++17 兼容编译器
  - CMake ≥ 3.12
  - Eigen 3.4.0+

- ​**可视化工具**
  - Python 3.8+
  - Matplotlib

### 构建与运行
```bash
# 克隆项目
git clone https://github.com/yourusername/inverted-pendulum-sim.git
cd inverted-pendulum-sim

# 构建项目
mkdir build && cd build
cmake -G "MinGW Makefiles" ..
make 

# 运行仿真
.\InvertedPendulumSim.exe

# 可视化结果
cd ../output
python plot_simulation.py
