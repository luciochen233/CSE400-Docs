# The Teensy simulation node
The Python script shown below can simulate the motors on the CSE400 robot.
Run this script instead of the rosserial Python script.  This script takes the place of the real robot.
It will allow us to simulate the robot in RVIZ rather than Gazebo.
It uses the custom messages for subscribing to the motor velocities and for publishing the motor position and velocity.

::: tip
[Ros Tutorials](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
:::

```Python
#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from robota.msg import Control_msgs
from robota.msg import Motor_msgs

def messageCb(cmd):
    global wheel_left_set, wheel_right_set, arm_shoulder_set, arm_elbow_set
    wheel_left_set = cmd.left_vel;
    wheel_right_set = cmd.right_vel;
    arm_shoulder_set = cmd.shoulder_vel;
    arm_elbow_set = cmd.elbow_vel;
    
rospy.init_node('Teensy_sim', anonymous=True)

rospy.Subscriber("teensy/Commands", Control_msgs, messageCb)
pub = rospy.Publisher("teensy/Motors", Motor_msgs, queue_size=10)

dt = 0.09
rate = rospy.Rate(1/dt) # 11.1Hz

wheel_left_set = 0.0
wheel_right_set = 0.0
arm_shoulder_set = 0.0
arm_elbow_set = 0.0
motor_msg = Motor_msgs()
motor_msg.left_pos = 0.0
motor_msg.left_vel = 0.0
motor_msg.right_pos = 0.0
motor_msg.right_vel = 0.0
motor_msg.shoulder_pos = 0.0
motor_msg.shoulder_vel = 0.0
motor_msg.elbow_pos = 0.0
motor_msg.elbow_vel = 0.0

while not rospy.is_shutdown():
    motor_msg.left_pos = motor_msg.left_pos + wheel_left_set * dt
    motor_msg.left_vel = wheel_left_set
    motor_msg.right_pos = motor_msg.right_pos + wheel_right_set * dt
    motor_msg.right_vel = wheel_right_set
    motor_msg.shoulder_pos = motor_msg.shoulder_pos + arm_shoulder_set * dt
    motor_msg.shoulder_vel = arm_shoulder_set
    motor_msg.elbow_pos = motor_msg.elbow_pos + arm_elbow_set * dt
    motor_msg.elbow_vel = arm_elbow_set
    pub.publish(motor_msg)
    rate.sleep()

```