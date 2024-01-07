# PX4 Offboard Control
> Ubuntu 22.04 | ROS2 Humble | PX4
## Dependency
- [px4_msgs](https://github.com/PX4/px4_msgs)
- [NavPy](https://pypi.org/project/NavPy/)

## RUN
**MultiCopter**
```
ros2 run px4_offboard mc_control --ros-args -p Mode:=<args>
```
**Fixed-Wing**
```
ros2 run px4_offboard fw_control --ros-args -p Mode:=<args>
```
Mode
- `pos` : position setpoints
- `att` : attitude setpoints
- `act` : direct control the motor and/or servo<br/>
&emsp;&emsp;&emsp;(Hardware check is required.)

## Reference
- https://github.com/PX4/px4_ros_com/tree/main/src/examples/offboard_py
- https://github.com/Jaeyoung-Lim/px4-offboard/tree/master