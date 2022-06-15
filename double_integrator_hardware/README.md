# phi_fake_hardware #
This package contains the necessary drivers for interacting with a fake system emulating the phi system using the [`ros2_control`](https://github.com/ros-controls/ros2_control) framework.

## Requirements ##
The `ros2_control` framework is released for `ROS2 Foxy`. To use it, you have to install `ros-foxy-ros2-control` and `ros-foxy-ros2-controllers` packages. Other dependencies are installed automatically.

## Usage ##
The fake system hardware interface and the default controller can be launched by running
```shell
$ ros2 launch phi_fake_hardware phi_fake_hi.launch.py`
```
