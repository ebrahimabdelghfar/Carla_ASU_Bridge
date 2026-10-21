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
pip3 install thirdparty_lib/carla/carla-0.9.16-cp310-cp310-linux_x86_64.whl
```

## Build The Workspace

```bash
make setup_ros2_workspace 
```

# Run the Simulator

```bash
make launch_carla_sim && \
ros2 lifecycle set /ASU_RT_Carla_Telemetry_Node 1 && \
ros2 lifecycle set /ASU_RT_Carla_Telemetry_Node 3
```

# Run With Docker

The image is based on ROS 2 Humble (`osrf/ros:humble-desktop`) and ships the
CARLA Python API, CycloneDDS and all build dependencies. The simulator binaries,
sources and config are bind-mounted from the host, so edits on the host are
visible inside the container without rebuilding the image.

## Prerequisites

- Docker with the Compose plugin (`docker compose`) or `docker-compose`.
- An NVIDIA GPU with the
  [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  installed (the container uses `runtime: nvidia`).
- The simulator assets on the host:

  ```bash
  make download_carla_assets
  ```

## Build and Start the Container

```bash
make setup_docker
```

This builds the image, starts the `asurt_carla_simulator` container in the
background and waits until its first-run initialization has finished. The
container uses host networking, so ROS 2 topics are visible on the host and on
the LAN (CycloneDDS config: [docker/cyclonedds.xml](docker/cyclonedds.xml)).

To use a specific GPU instead of all of them:

```bash
GPUS=0 make setup_docker
```

## Open a Shell in the Container

```bash
make exec_docker_compose
```

This also runs `xhost +local:root` so the container can open windows on the
host display. Every command below runs inside that shell, from `/asurt`.

## Build the Workspace (inside the container)

```bash
make setup_ros2_workspace
source install/ros_apps/setup.bash
```

## Run the Simulator (inside the container)

```bash
make launch_carla_sim
```

Then, from a second container shell (`make exec_docker_compose`), activate the
telemetry node:

```bash
ros2 lifecycle set /ASU_RT_Carla_Telemetry_Node 1 && \
ros2 lifecycle set /ASU_RT_Carla_Telemetry_Node 3
```

Use `make launch_carla_sim_low` for lower rendering quality on weaker GPUs.

## Stop the Container

```bash
make down_docker_compose
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

> Ebrahim Abdelghfar, "Formula AI Simulator: A CARLA-based Simulation Bridge for
> Autonomous Formula Student Vehicles," 2026. [Online]. Available:
> https://github.com/ebrahimabdelghfar/Carla_ASU_Bridge

