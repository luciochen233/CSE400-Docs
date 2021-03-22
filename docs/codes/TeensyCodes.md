# rosserial Teensy Arduino Sketch

The Teensy must run a loop that always updates the stepper motor AccelStepper objects.
It also must always run the ROS spin method so it can subscribe to Topics.
The sketch below uses the millisecond timer to create a scheduler.
The important thing is that the scheduler always allows the main loop to run as fast as possible.
The main loop must keep running so the motors can move and the subscriber can subscribe.
I wrote the scheduler with a Case statement.  
You could use If statements if you want.
I was pretty conservative about timing.
Ultimately the robot is "tuned" for optimum performance by adjusting the publishing frequencies. 
The scheduler publishes the custom messages over special Topics back to the RPi, so ROS knows the state of the Robot.  

```Arduino
/*
 * rosserial Teensy Arduino Sketch for the CSE400 robot
 * publishes custom messages to RPi,
 * and subscribes to custom messages from RPi
 * the subscriber toggles the led
 */

#include <ros.h>
#include <robota/Motor_msgs.h>
#include <robota/IMU_msgs.h>
#include <robota/LIDAR_msgs.h>
#include <robota/GPS_msgs.h>
#include <robota/Control_msgs.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

float scale_wheel_angle = 100.0/3.14159;
float scale_wheel_steps = 3.14159/100.0;
float scale_arm_angle = 1019.0/3.14159;
float scale_arm_steps = 3.14159/1019.0;
float wheel_left_set = 0.0;
float wheel_right_set = 0.0;
float arm_shoulder_set = 0.0;
float arm_elbow_set = 0.0;
unsigned long check_tick_time = millis();
unsigned int number_ticks = 0;
AccelStepper wheel_left(AccelStepper::FULL4WIRE, 20, 21, 22, 23);
AccelStepper wheel_right(AccelStepper::FULL4WIRE, 14, 15, 16, 17);
AccelStepper arm_shoulder(AccelStepper::FULL4WIRE, 2, 4, 3, 5);
AccelStepper arm_elbow(AccelStepper::FULL4WIRE, 9, 11, 10, 12);

ros::NodeHandle  nh;

void messageCb(const robota::Control_msgs& cmd) {
  if (cmd.left_vel != wheel_left_set) {
    wheel_left_set = cmd.left_vel;
    wheel_left.setSpeed(scale_wheel_angle*wheel_left_set);
  }
  if (cmd.right_vel != wheel_right_set) {
    wheel_right_set = cmd.right_vel;
    wheel_right.setSpeed(scale_wheel_angle*wheel_right_set);
  }
  if (cmd.shoulder_vel != arm_shoulder_set) {
    arm_shoulder_set = cmd.shoulder_vel;
    arm_shoulder.setSpeed(scale_arm_angle*arm_shoulder_set);
  }
  if (cmd.elbow_vel != arm_elbow_set) {
    arm_elbow_set = cmd.elbow_vel;
    arm_elbow.setSpeed(scale_arm_angle*arm_elbow_set);
  }
  digitalWrite(13, HIGH-digitalRead(13));   // toggle the led
}

ros::Subscriber<robota::Control_msgs> cmd_sub("teensy/Commands", messageCb);

robota::Motor_msgs motor_msg;
robota::IMU_msgs IMU_msg;
robota::LIDAR_msgs LIDAR_msg;
robota::GPS_msgs GPS_msg;

ros::Publisher motor_publisher("teensy/Motors", &motor_msg);
ros::Publisher IMU_publisher("teensy/IMU", &IMU_msg);
ros::Publisher LIDAR_publisher("teensy/LIDAR", &LIDAR_msg);
ros::Publisher GPS_publisher("teensy/GPS", &GPS_msg);

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
imu::Vector<3> accel;
imu::Vector<3> gyro;
imu::Quaternion quat;

void setup()
{
  wheel_left.disableOutputs();
  wheel_right.disableOutputs();
  arm_shoulder.disableOutputs();
  arm_elbow.disableOutputs();
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(motor_publisher);
  nh.advertise(IMU_publisher);
  nh.advertise(LIDAR_publisher);
  nh.advertise(GPS_publisher);
  wheel_left.setMaxSpeed(50);
  wheel_left.setAcceleration(10);
  wheel_right.setMaxSpeed(50);
  wheel_right.setAcceleration(10);
  arm_shoulder.setMaxSpeed(50);
  arm_shoulder.setAcceleration(10);
  arm_elbow.setMaxSpeed(50);
  arm_elbow.setAcceleration(10);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
}

void loop()
{
  if ((millis() - check_tick_time) >= 15) {
    check_tick_time = check_tick_time + 15;
    number_ticks++;
    number_ticks = number_ticks % 30;
    switch (number_ticks) {
      // IMU read and published every 45ms; 22.2Hz
      case 0: case 3: case 6: case 9: case 12: case 15: case 18: case 21: case 24: case 27:
        quat = bno.getQuat();
        gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        IMU_msg.quat_w = quat.w();
        IMU_msg.quat_x = quat.x();
        IMU_msg.quat_y = quat.y();
        IMU_msg.quat_z = quat.z();
        IMU_msg.rot_x = gyro.x();
        IMU_msg.rot_y = gyro.y();
        IMU_msg.rot_z = gyro.z();
        IMU_msg.accel_x = accel.x();
        IMU_msg.accel_y = accel.y();
        IMU_msg.accel_z = accel.z();
        IMU_publisher.publish(&IMU_msg);
      break;
      // Laser Scanner read and published every 45ms; 22.2Hz
      case 1: case 4: case 7: case 10: case 13: case 16: case 19: case 22: case 25: case 28:
        LIDAR_msg.left = 0;
        LIDAR_msg.center = 0;
        LIDAR_msg.right = 0;
        LIDAR_publisher.publish(&LIDAR_msg);
      break;
      // Motor velocities published every 90ms; 11.1Hz
      case 2: case 8: case 14: case 20: case 26:
        motor_msg.left_pos = scale_wheel_steps*wheel_left.currentPosition();
        motor_msg.left_vel = scale_wheel_steps*wheel_left.speed();
        motor_msg.right_pos = scale_wheel_steps*wheel_right.currentPosition();
        motor_msg.right_vel = scale_wheel_steps*wheel_right.speed();
        motor_msg.shoulder_pos = scale_arm_steps*arm_shoulder.currentPosition();
        motor_msg.shoulder_vel = scale_arm_steps*arm_shoulder.speed();
        motor_msg.elbow_pos = scale_arm_steps*arm_elbow.currentPosition();
        motor_msg.elbow_vel = scale_arm_steps*arm_elbow.speed();
        motor_publisher.publish(&motor_msg);
      break;
      // GPS published every 450ms; 2.22Hz
      case 5:
        GPS_msg.latitude = 0.0;
        GPS_msg.longitude = 0.0;
        GPS_msg.altitude = 0.0;
        GPS_publisher.publish(&GPS_msg);
      break;
    }
  }
  
  wheel_left.runSpeed();
  wheel_right.runSpeed();
  arm_shoulder.runSpeed();
  arm_elbow.runSpeed();
  
  nh.spinOnce();
}
```