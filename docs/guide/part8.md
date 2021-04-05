# Create our custom message
There are files in the [code](/codes/) section, you might need to download those file onto your robot when needed.

## Create our own custom msg to save bandwidth

::: tip Source
[http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
:::

before we create our msg file, we need to know the format and where to store it. According to the ROS document, the **msg** file should be stored in the **msg** directory of a package, and **srv** files are stored in the **srv** directory.

> msgs are just simple text files with a field type and field name per line. The field types you can use are:
>
> - int8, int16, int32, int64 (plus uint*)
> - float32, float64
> - string
> - time, duration
> - other msg files
> - variable-length array[] and fixed-length array[C]

Additionally, there is a special type in ROS: Header. The Header contains a timestamp and coordinate frame information that are commonly used in ROS.

In our case, the header file is already given; since we are all using the same robot. 

[--- Link to the header file ---](/codes/CustomMessages.html)

We are going to create different header files that should be in the msg directory of your project. (Probably named beginner_tutorials)

You can choose the name of those header file. However, one example will be:

``` bash
touch Control_msgs.msg
```

After that, you can copy and past the content into the file. One example will be:

``` c
# Command message
float32 left_vel
float32 right_vel
float32 shoulder_vel
float32 elbow_vel
```

Do the same thing for all four header files.

### edit the package.xml

Open `package.xml`, and make sure these two lines are in it and uncommented:

``` xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```



### edit the CMakeLists.txt

```cmake
# Do not just add this to your CMakeLists.txt, 
# modify the existing text to 
# add message_generation before the closing parenthesis
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```

After that, you need to make sure you export the message runtime dependency.

``` cmake
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

Finally, we need to find the following block of code and add our message into it.

``` cmake
add_message_files(
  FILES
  Control_msgs.msg
  Imu_msgs.msg
  ...
)
```

In the end, un-comment the following part:

``` cmake
generate_messages(
   DEPENDENCIES
   std_msgs 
)
```

You don't need to add your previous message into this block.

### test if our custom message works

go back to your catkin workspace folder, do a 

``` bash
catkin_make
```

::: tip 
remember to source your bash/zsh file again (might not needed, but I had to do it)

``` bash
source devel/setup.bash
# or source devel/setup.zsh 
# if you are using zsh
```

:::

now we can run a simple rosmsg command to see if our custom message is working or not.

``` bash
rosmsg show beginner_tutorials/Control_msgs
```

and the output should be:

```cpp
float32 left_vel
float32 right_vel
float32 shoulder_vel
float32 elbow_vel
```

We are done with our custom messages



