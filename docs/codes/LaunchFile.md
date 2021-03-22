# Launch files

::: tip
[http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch)
[http://wiki.ros.org/roslaunch](http://wiki.ros.org/roslaunch)
[http://wiki.ros.org/roslaunch/XML](http://wiki.ros.org/roslaunch/XML)
:::

## Launch file for publishing the joint states over the serial port
```xml
<?xml version="1.0"?>
<launch>

	<!-- Load the urdf on the parameter server -->
	<param name="robot_description" textfile="$(find robota)/urdf/robota.urdf"/>

	<!-- Create robot state publisher for fixed joints to /tf tranforms -->
	<node name="robot_state_publisher" type="robot_state_publisher" pkg="robot_state_publisher" />

	<!-- Create the joint state publisher for movable joints -->
	<node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"/>

	<!-- Create python communication node -->
	<node name="serial_node" pkg="rosserial_python" type="serial_node.py">
		<param name="port" value="/dev/ttyACM0"/>
	</node>

	<!-- Create an rviz node, use file robota.rviz in config folder -->
  	<node name="rviz" pkg="rviz" type="rviz" args="-d $(find robota)/config/robota.rviz"/>

</launch>

```

## Launch file for the PID controllers for real robot

```xml
<?xml version="1.0"?>

<launch>

    <param name="robot_description" textfile="$(find robota)/urdf/robota.urdf"/>
	
	<rosparam file="$(find robota)/config/controllers.yaml" command="load"/>
	<rosparam file="$(find robota)/config/joint_limits.yaml" command="load"/>    

	<node name="serial_node" pkg="rosserial_python" type="serial_node.py">
		<param name="port" value="/dev/ttyACM0"/>
	</node>
	
	<node name="robot_hw_node" pkg="robota" type="robot_hw" output="screen"/>
	
	<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"
		args="/joint_control/joints_state 
		/joint_control/left_position_controller
		/joint_control/right_position_controller
		/joint_control/shoulder_position_controller
		/joint_control/elbow_position_controller"
	/>

    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
    <node name="rviz" pkg="rviz" type="rviz"/>
	
</launch>

```

## Launch file for the PID controllers for simulated robot

```xml
<?xml version="1.0"?>

<launch>

    <param name="robot_description" textfile="$(find robota)/urdf/robot.urdf"/>
	
	<rosparam file="$(find robota)/config/controllers.yaml" command="load"/>
	<rosparam file="$(find robota)/config/joint_limits.yaml" command="load"/>    

	<node name="Teensy_sim" pkg="robota" type="Teensy_sim.py"/>
	
	<node name="robot_hw_node" pkg="robota" type="robot_hw" output="screen"/>
	
	<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"
		args="/joint_control/joints_state 
		/joint_control/left_position_controller
		/joint_control/right_position_controller
		/joint_control/shoulder_position_controller
		/joint_control/elbow_position_controller"
	/>

    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
    <node name="rviz" pkg="rviz" type="rviz"/>
	
</launch>

```
