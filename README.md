# PX4-ROS2 bridge

[![GitHub license](https://img.shields.io/github/license/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/blob/master/LICENSE) [![GitHub (pre-)release](https://img.shields.io/github/release-pre/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/releases/tag/beta) [![DOI](https://zenodo.org/badge/142936318.svg)](https://zenodo.org/badge/latestdoi/142936318) [![Build and Test package](https://github.com/PX4/px4_ros_com/workflows/Build%20and%20Test%20package/badge.svg?branch=master)](https://github.com/PX4/px4_ros_com/actions)

[![Discord Shield](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

This package provides example nodes for exchanging data and commands between ROS2 and PX4.
It also provides a [library](./include/px4_ros_com/frame_transforms.h) to ease the conversion between ROS2 and PX4 frame conventions.
It has a straight dependency on the [`px4_msgs`](https://github.com/PX4/px4_msgs) package.

## Install, build and usage

Check the [uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html) and the [ROS2 Interface](https://docs.px4.io/main/en/ros/ros2_comm.html) sections on the PX4 Devguide for details on how to install the required dependencies, build the package and use it.

## Bug tracking and feature requests

Use the [Issues](https://github.com/PX4/px4_ros_com/issues) section to create a new issue. Report your issue or feature request [here](https://github.com/PX4/px4_ros_com/issues/new).

## Questions and troubleshooting

Reach the PX4 development team on the [PX4 Discord Server](https://discord.gg/dronecode).

## Demo of Random Exploration

This project demonstrates random exploration of a drone using ROS2 Humble, Gazebo Sim (Garden), and PX4-Autopilot v1.15.0.

![Random Exploration Demo](./demo/demo_random_explore.gif)

---

### Prerequisites

1. **PX4-Autopilot v1.15.0**  
   Follow the [official PX4 documentation](https://docs.px4.io/v1.15/en/ros2/user_guide.html) for installation.  
   Note: This version does not include our customized drone model with a mono camera and 2D LiDAR.  
   Download the customized model from [here](#).

2. **Workspace Setup**  
   Clone and build the necessary repositories:

   ```bash
   mkdir -p ws_offboard_control/src
   cd ws_offboard_control/src
   git clone https://github.com/PX4/px4_msgs
   git clone https://github.com/Xiaoang0211/px4_ros_com_customized
   cd ..
   colcon build
   ```

### Running the Demo

1. **Initialize the Drone in Baylands (with CUDA support)**

    ```bash
    cd ~/PX4-Autopilot/
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia make px4_sitl gz_x500_cam_2dlidar_baylands
    ```

2. **Publish Camera and LiDAR Topics**  
   Open a new terminal and run:

    ```bash
    cd ws_offboard_control
    source install/local_setup.bash
    ros2 run px4_ros_com_customized drone_advertiser
    ```

3. **Run Random Exploration with Offboard Control**  
   Open another terminal and execute:

    ```bash
    cd ws_offboard_control
    source install/local_setup.bash
    ros2 run px4_ros_com_customized OffboardControl
    ```

---

### Notes

- Ensure you are using PX4-Autopilot v1.15.0 for compatibility.  
- Replace `__NV_PRIME_RENDER_OFFLOAD=1` and `__GLX_VENDOR_LIBRARY_NAME=nvidia` with your GPU-specific commands if not using NVIDIA.

---

### TODOs

1. Testing the collision prevention module.
2. Using RViz to visualize trajectory.
3. Testing [okvis2](https://github.com/smartroboticslab/okvis2) and visualize point set in RViz
