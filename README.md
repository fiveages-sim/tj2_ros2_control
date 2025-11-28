# Marvin ROS2 Control

## 1. Interfaces

Required hardware interfaces:

* command:
    * joint position
* state:
    * joint effort
    * joint position
    * joint velocity

## 2. Build

* Submodule
```bash
git submodule update --init
```

* Build

```bash
cd ~/ros2_ws
colcon build --packages-up-to marvin_ros2_control --symlink-install
```
