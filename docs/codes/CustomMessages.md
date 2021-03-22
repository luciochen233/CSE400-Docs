# Custom Messages

::: tip
http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
:::

```cpp
# Command message
float32 left_vel
float32 right_vel
float32 shoulder_vel
float32 elbow_vel


# Motor message
float32 left_pos
float32 left_vel
float32 right_pos
float32 right_vel
float32 shoulder_pos
float32 shoulder_vel
float32 elbow_pos
float32 elbow_vel


# IMU message
float32 quat_w
float32 quat_x
float32 quat_y
float32 quat_z
float32 rot_x
float32 rot_y
float32 rot_z
float32 accel_x
float32 accel_y
float32 accel_z


# LIDAR message
int16 left
int16 center
int16 right



```