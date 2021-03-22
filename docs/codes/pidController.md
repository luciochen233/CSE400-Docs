# Config Files

## Joint PID controller YAML file for our 4 stepper motors

```YAML
joint_control:
    # Publish all joint states -----------------------------------
    joints_state:
      type: joint_state_controller/JointStateController
      publish_rate: 10
    
    # Position Controllers ---------------------------------------
    left_position_controller:
      type: velocity_controllers/JointPositionController
      joint: motor_left_shaft
      pid: {p: 0.2, i: 0.0, d: 0.0}
      
    right_position_controller:
      type: velocity_controllers/JointPositionController
      joint: motor_right_shaft
      pid: {p: 0.2, i: 0.0, d: 0.0}
      
    shoulder_position_controller:
      type: velocity_controllers/JointPositionController
      joint: motor_shoulder_shaft
      pid: {p: 0.2, i: 0.0, d: 0.0}
      
    elbow_position_controller:
      type: velocity_controllers/JointPositionController
      joint: motor_elbow_shaft
      pid: {p: 0.2, i: 0.0, d: 0.0}

```

## Joint limits YAML file for our 4 stepper motors

```YAML
joint_limits:
    motor_left_shaft:
      has_position_limits: false
      has_velocity_limits: true
      max_velocity: 1.57
      has_acceleration_limits: true
      max_acceleration: 0.5
      has_jerk_limits: false
      has_effort_limits: false

    motor_right_shaft:
      has_position_limits: false
      has_velocity_limits: true
      max_velocity: 1.57
      has_acceleration_limits: true
      max_acceleration: 0.5
      has_jerk_limits: false
      has_effort_limits: false

    motor_shoulder_shaft:
      has_position_limits: false
      has_velocity_limits: true
      max_velocity: 0.15
      has_acceleration_limits: true
      max_acceleration: 0.15
      has_jerk_limits: false
      has_effort_limits: false

    motor_elbow_shaft:
      has_position_limits: false
      has_velocity_limits: true
      max_velocity: 0.15
      has_acceleration_limits: true
      max_acceleration: 0.15
      has_jerk_limits: false
      has_effort_limits: false

```