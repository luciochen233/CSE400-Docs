# The Hardware interface node
::: tip
There are multiple helpful links for this part.
:::
http://wiki.ros.org/hardware_interface
http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface
https://medium.com/@slaterobotics/how-to-implement-ros-control-on-a-custom-robot-748b52751f2e
https://fjp.at/posts/ros/ros-control/
https://sir.upc.edu/projects/rostutorials/10-gazebo_control_tutorial/index.html
https://fjp.at/projects/diffbot/ros-packages/base/

## Building a c++ node
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

## header file
```cpp
#ifndef ROBOT_HW_INTERFACE_H_
#define ROBOT_HW_INTERFACE_H_

#include <ros/console.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <robota/Motor_msgs.h>
#include <robota/Control_msgs.h>

class Robot : public hardware_interface::RobotHW
{
private:
    ros::NodeHandle nh;
    
    hardware_interface::JointStateInterface jnt_state;
    hardware_interface::VelocityJointInterface jnt_cmd;
    
    joint_limits_interface::JointLimits jnt_limits;
    joint_limits_interface::VelocityJointSaturationInterface jnt_vel_sat;

    double pos[4];
    double vel[4];
    double eff[4];
    double cmd[4];
    
    ros::Subscriber sub;
    ros::Publisher pub;
    
    robota::Motor_msgs motor_msg;
    robota::Control_msgs command_msg;
    
    void teensyCallback(const robota::Motor_msgs& msg);

public:
    Robot(ros::NodeHandle& nodehandle);
    
    void read();
    void write(ros::Duration elapsed_time);
    
};

#endif // ROBOT_HW_INTERFACE_H_
```

## cpp file

``` cpp
#include "robota/robot_hw_interface.h"

Robot::Robot(ros::NodeHandle& nodehandle)
{
    // store the node handle passed to the hardware interface
    // in the private member variable
    nh = nodehandle;

    // Initialize the other private member variables
    pos[0] = 0.0; pos[1] = 0.0; pos[2] = 0.0; pos[3] = 0.0;
    vel[0] = 0.0; vel[1] = 0.0; vel[2] = 0.0; vel[3] = 0.0;
    eff[0] = 0.0; eff[1] = 0.0; eff[2] = 0.0; eff[3] = 0.0;
    cmd[0] = 0.0; cmd[1] = 0.0; cmd[2] = 0.0; cmd[3] = 0.0;
    
    // create joint state handles and register them
    hardware_interface::JointStateHandle state_left("motor_left_shaft", &pos[0], &vel[0], &eff[0]);
    hardware_interface::JointStateHandle state_right("motor_right_shaft", &pos[1], &vel[1], &eff[1]);
    hardware_interface::JointStateHandle state_shoulder("motor_shoulder_shaft", &pos[2], &vel[2], &eff[2]);
    hardware_interface::JointStateHandle state_elbow("motor_elbow_shaft", &pos[3], &vel[3], &eff[3]);
    jnt_state.registerHandle(state_left);
    jnt_state.registerHandle(state_right);
    jnt_state.registerHandle(state_shoulder);
    jnt_state.registerHandle(state_elbow);
    registerInterface(&jnt_state);

    // create joint command handles and register them
    hardware_interface::JointHandle cmd_left (state_left, &cmd[0]);
    hardware_interface::JointHandle cmd_right(state_right, &cmd[1]);
    hardware_interface::JointHandle cmd_shoulder(state_shoulder, &cmd[2]);
    hardware_interface::JointHandle cmd_elbow(state_elbow, &cmd[3]);
    jnt_cmd.registerHandle(cmd_left);
    jnt_cmd.registerHandle(cmd_right);
    jnt_cmd.registerHandle(cmd_shoulder);
    jnt_cmd.registerHandle(cmd_elbow);
    registerInterface(&jnt_cmd);
    
    // create joint limits handles and register them
    joint_limits_interface::getJointLimits("motor_left_shaft", nh, jnt_limits);
    joint_limits_interface::VelocityJointSaturationHandle limit_left(cmd_left, jnt_limits);
    joint_limits_interface::getJointLimits("motor_right_shaft", nh, jnt_limits);
    joint_limits_interface::VelocityJointSaturationHandle limit_right(cmd_right, jnt_limits);
    joint_limits_interface::getJointLimits("motor_shoulder_shaft", nh, jnt_limits);
    joint_limits_interface::VelocityJointSaturationHandle limit_shoulder(cmd_shoulder, jnt_limits);
    joint_limits_interface::getJointLimits("motor_elbow_shaft", nh, jnt_limits);
    joint_limits_interface::VelocityJointSaturationHandle limit_elbow(cmd_elbow, jnt_limits);
    jnt_vel_sat.registerHandle(limit_left);
    jnt_vel_sat.registerHandle(limit_right);
    jnt_vel_sat.registerHandle(limit_shoulder);
    jnt_vel_sat.registerHandle(limit_elbow);
    registerInterface(&jnt_vel_sat);
    
    // create a subscriber to get data from the teensy
    sub = nh.subscribe("teensy/Motors", 1000, &Robot::teensyCallback, this);

    // create a publisher to send data to the teensy
    pub = nh.advertise<robota::Control_msgs>("teensy/Commands", 1000);
}

void Robot::read()
{
    pos[0] = motor_msg.left_pos;
    pos[1] = motor_msg.right_pos;
    pos[2] = motor_msg.shoulder_pos;
    pos[3] = motor_msg.elbow_pos;
    
    vel[0] = motor_msg.left_vel;
    vel[1] = motor_msg.right_vel;
    vel[2] = motor_msg.shoulder_vel;
    vel[3] = motor_msg.elbow_vel;
    
    //ROS_INFO("In read Motor_vel: %.2f", vel[1]);

}

void Robot::write(ros::Duration elapsed_time)
{
    jnt_vel_sat.enforceLimits(elapsed_time);
    command_msg.left_vel = cmd[0];
    command_msg.right_vel = cmd[1];
    command_msg.shoulder_vel = cmd[2];
    command_msg.elbow_vel = cmd[3];
    
    pub.publish(command_msg);
    
    //ROS_INFO("In write Commands: %.2f", cmd[1]);
}

void Robot::teensyCallback(const robota::Motor_msgs& msg)
{
    motor_msg = msg;
    
    //ROS_INFO("In callback");

}

int main(int argc, char** argv)
{

    ROS_INFO("Start of main");

    // initialize the hardware interface ros node 
    ros::init(argc, argv, "robot_node");
    
    // and create a ros node handle/mobile_base_controller/left_wheel
    ros::NodeHandle nh;
    
    // create a hardware interface object and connect it to the controller manager
    Robot robot(nh);
    controller_manager::ControllerManager cm(&robot, nh);
    
    // set the period and update rate for 10Hz 
    ros::Duration period(0.1);
    ros::Rate rate(1.0/period.toSec());
 
    // create a new spinner thread for the callback method and start it
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    //ROS_INFO("Before main_loop");
    
    // create an infinite loop  for running the hardware interface
    while(ros::ok())
    {
        //ROS_INFO("In main_loop");
        robot.read();
        cm.update(ros::Time::now(), period);
        robot.write(period);
        rate.sleep();
    }
    spinner.stop();
    return 0;
}
```