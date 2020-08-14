# ROS Commands

Brief description of ROS commands for controlling the arm using the terminal. 

### Get Current Arm State

```bash
rostopic echo /machine_state
```

**Remarks**
- 0: Not initialized (Needs to be initialized)
- 1: Uncalibrated (Needs to be calibrated)
- 2: Calibrated (Ready to receive Move command)
- 6: Locked (Needs to be disengaged)

### Get Current Algorithm State

```bash
rostopic echo /algorithm_state
```

**Remarks**
- `data: 1` : Finished moving
- `data: 2` : Moving

### Get Arm Position

```bash
rostopic echo /machine_algo_jnt_state
```

**Remarks**: 
- Data type is `edo_core_msgs/JointStateArray`. 
- It gives position, velocity and current. 

### Initialize

```bash
rostopic pub -1 /bridge_init edo_core_msgs/JointInit \
"mode: 0
joints_mask: 127
reduction_factor: 0.0"
```


### Disengage

Unlock breaks

```bash
rostopic pub -1 /bridge_jnt_reset edo_core_msgs/JointReset \
"joints_mask: 127
disengage_steps: 2000
disengage_offset: 3.5"
```

**Remarks**: Unlock breaks, should be used when the current state is 1, and may be useful when robot is on State 6. 


### Calibrate

```bash
rostopic pub -1 /bridge_jnt_calib edo_core_msgs/JointCalibration \
"joints_mask: 127"
```

**Remarks**: Tells the robot that the *current* position represents the zero point. 


### Move Joint

```bash
rostopic pub -1 /machine_move edo_core_msgs/MovementCommand \
"move_command: 77
move_type: 74
ovr: 70
delay: 0
remote_tool: 0
cartesian_linear_speed: 0.0
target:
  data_type: 74
  cartesian_data: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, e: 0.0, r: 0.0, config_flags: ''}
  joints_mask: 127
  joints_data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
via:
  data_type: 0
  cartesian_data: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, e: 0.0, r: 0.0, config_flags: ''}
  joints_mask: 0
  joints_data: [0]
tool: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, e: 0.0, r: 0.0}
frame: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, e: 0.0, r: 0.0}"
```

**Remarks**: 
- Sets the angles for the robot to move to. The angles are in degrees, represented in the array `MovementCommand.target.joints_data`. 
- Sending multiple commands will put the commands in a queue. 


### Cancel Move

```bash
rostopic pub -1 /machine_move edo_core_msgs/MovementCommand \
"move_command: 67
move_type: 74
ovr: 0
delay: 0
remote_tool: 0
cartesian_linear_speed: 0.0
target:
  data_type: 0
  cartesian_data: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, e: 0.0, r: 0.0, config_flags: ''}
  joints_mask: 0
  joints_data: [0.0]
via:
  data_type: 0
  cartesian_data: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, e: 0.0, r: 0.0, config_flags: ''}
  joints_mask: 0
  joints_data: [0.0]
tool: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, e: 0.0, r: 0.0}
frame: {x: 0.0, y: 0.0, z: 0.0, a: 0.0, e: 0.0, r: 0.0}"
```

**Remarks**: Will not cancel current move, but simply clear the move queue. 