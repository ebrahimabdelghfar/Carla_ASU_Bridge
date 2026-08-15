<p align="center">
  <h1 align="center">Formula AI simulator</h1>
  <p align="center">
    <em>Formula Ai simulator devloped using Carla and Unrealengine 4.26</em>
  </p>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Jazzy-blue?logo=ros&logoColor=white" alt="ROS 2 Humble"/>
  <img src="https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 22.04"/>
  <img src="https://img.shields.io/badge/License-GPLv3-blue.svg" alt="License"/>
  <img src="https://img.shields.io/badge/Build-Colcon-informational" alt="Build"/>
</p>

<p align="center">
    <img src="images/ARL.png" alt="ARL Logo"/>
    <img src="images/asurt.png" alt="asurt">
</p>

# Steps to Operate
## Download The Simulator

```bash
make download_carla_assets
```

## Install The CARLA API

```bash
pip3 thirdparty_lib/carla/carla-0.9.16-cp310-cp310-linux_x86_64.whl
```

## Build The Workspace

```bash
make setup_ros2_workspace 
```

# Run the Simulator

```bash
make launch_carla_sim && \
ros2 lifecycle set /ASU_RT_Carla_Telemetry_Node configure && \
ros2 lifecycle set /ASU_RT_Carla_Telemetry_Node activate
```

# Video


https://github.com/user-attachments/assets/8a7885ac-42ba-4e7f-9a16-554fe6fa74c3

# How to Cite

If you use this simulator in your research or project, please cite it:

```bibtex
@software{carla_asu_bridge,
  author  = {Abdelghfar, Ebrahim},
  title   = {Formula AI Simulator: A CARLA-based Simulation Bridge for
             Autonomous Formula Student Vehicles},
  year    = {2026},
  url     = {https://github.com/ebrahimabdelghfar/Carla_ASU_Bridge},
  license = {GPL-3.0}
}
```

Plain text:

> E. Abdelghfar, "Formula AI Simulator: A CARLA-based Simulation Bridge for
> Autonomous Formula Student Vehicles," 2026. [Online]. Available:
> https://github.com/ebrahimabdelghfar/Carla_ASU_Bridge

