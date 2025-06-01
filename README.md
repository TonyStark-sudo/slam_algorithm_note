# 📌 经典 SLAM 算法注释合集

> 💡 一个专为学习、研究和开发设计的经典 SLAM 算法源码注释工程，覆盖视觉-惯性 SLAM（VI-SLAM）与激光-惯性 SLAM（LI-SLAM）代表性算法，如 VINS-Fusion、Fast-LIO、Fast-LIVO 等。

---

## 🌟 项目简介

本项目旨在提供**结构清晰、注释详细**的经典 SLAM 算法源码，帮助广大 SLAM 学习者、研究者以及工程开发人员更深入地理解其内部机制。

涵盖两大方向：

- 📷 **视觉-惯性 SLAM（VI-SLAM）**
- 🔦 **激光-惯性 SLAM（LI-SLAM）**

通过对公开项目的源码进行逐行注释、模块划分和流程图补充，降低学习门槛，加速理解过程。

---

## 📁 已注释算法列表

### 📷 视觉-惯性 SLAM（VI-SLAM）

| 算法名称 | 论文 | 特点 | 注释状态 |
|---------|------|------|-----------|
| [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | *TRO 2019* Qin 等 | 单目/双目+IMU，支持回环和地图 | ✅ 已完成 |
| [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) | *IROS 2018* Qin 等 | 单目+IMU，紧耦合后端 | ⏳ 注释中 |
| [OpenVINS](https://github.com/rpng/open_vins) | *ICRA 2021* Geneva 等 | 滤波器架构，模块化设计 | ⏳ 注释中 |

### 🔦 激光-惯性 SLAM（LI-SLAM）

| 算法名称 | 论文 | 特点 | 注释状态 |
|---------|------|------|-----------|
| [Fast-LIO](https://github.com/hku-mars/FAST_LIO) | *TRO 2021* Xu 等 | ESKF+iKF，纯激光惯性 | ⏳ 注释中 |
| [Fast-LIVO](https://github.com/hku-mars/FAST_LIVO) | *TRO 2022* Xu 等 | 激光+视觉+IMU 多传感器融合 | ⏳ 注释中 |
| [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) | *IROS 2020* Shan 等 | 基于因子图的平滑优化 | ⏳ 注释中 |

---

## 📂 项目结构

```bash
slam_algorithm_note/
├── VINS-Fusion/     # VINS-Fusion 注释版
├── VINS-Mono/       # VINS-Mono 注释版
├── fast-livo/       # Fast-LIVO 注释版
├── lio-sam/         # LIO-SAM 注释版
├── docs/            # 论文、流程图、模块说明等
└── README.md        # 当前文件

```
## 🧭 使用方法

1. 克隆本项目：

   ```bash
   git clone https://github.com/TonyStark-sudo/slam_algorithm_note.git
   cd slam_algorithm_note
   ```
2. 进入某个算法子目录：
   ```bash
   cd VINS-Fusion
   ```

## 📚 参考文献

- Qin et al., _VINS-Fusion: A Robust Multi-Sensor Visual-Inertial State Estimator_, **TRO 2019**  
- Xu et al., _Fast-LIO: A Fast, Tightly-coupled LiDAR-Inertial Odometry_, **TRO 2021**  
- Xu et al., _Fast-LIVO: Tightly-coupled LiDAR-Inertial-Visual Odometry_, **T-RO 2022**  
- Shan et al., _LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping_, **IROS 2020**  
- Geneva et al., _OpenVINS: A Research Platform for Visual-Inertial Estimation_, **ICRA 2021**
