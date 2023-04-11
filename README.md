
# Control framework for Sawyer HRI

This package provides a control framework for the Rethink robotics robot "Sawyer". It consists of a main control loop, which calculates all necessary kinematics and dynamics and forward them as an input into a state machine which contains instances of different controllers. The control nodes inputs are the URDF, and actual robots kinematics/ dynamics values. The output are torque commands to the robot joints.


## Authors

- [Nils Sichert](https://www.github.com/nils-sichert)

## Table of Content

1. [Installation](#Installation)
2. [Configuration](#Configuration)
3. [Framework](#Framework)

# Installation
Install Ubuntu 20.04 LTS and follow installation guide of Rethink robotics for setting up [ROS, Sawyer's SDK](https://support.rethinkrobotics.com/support/solutions/articles/80000980134-workstation-setup) and [Simulation](https://support.rethinkrobotics.com/support/solutions/articles/80000980381-gazebo-tutorial).

Install necessary packages:
```bash
  sudo apt-get update
  sudo apt-get install python-is-python3
  sudo apt-get install python3-pykdl
  sudo apt-get install ros-noetic-kdl-parser-py
  pip install scipy
  sudo apt-get install python3-matplotlib
  sudo apt-get ros-noetic-tf-transformations
```
    
## Documentation


## ROS commands

### Launch controller:
```bash
  roslaunch sawyer_ctrl_stability HRI_controller.launch
```

### Start/ Stop controller - default: controller stoped:
if stop robot will return to default position controller\
Start:
```bash
  rosparam set /control_node/control_is_on True
```
Stop:
```bash
  rosparam set /control_node/control_is_on False
```
### Set neutral pose of robot:
```bash
  rosparam set named_poses/right/poses/neutral [-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981]
```

### Set desired joint angle (rad) and joint velocity (rad/s):
Angle:
```bash
  rosparam set /control_node/joint_angle_desi [-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981]
```
Velocity:
```bash
  rosparam set /control_node/joint_velocity_desi [0, 0, 0, 0, 0, 0, 0]
```

### Set control algorithm - default: 3 
1: DLR Alin Albu-Schäffer Cartesian impedance control - not working!\
2: PD impedance control cartesian space - not working!\
3: Simple Spring-Damper impedance control - working (default)\
4: PD impedance control jointspace - working

```bash
  rosparam set /control_node/controllerstate 3
```

### Set joint stifness or joint damping, creates diagonal matrix from value - default: [20] 
has to be >=0\
Either put in one value which will be taken as values for all diagonal values or put in a list of 7 (7 joints) values.\
Order of entries: [Base -> Endeffector]
```bash
  rosparam set /control_node/Kd [100]
  rosparam set /control_node/Kd [100,80,60,40,20,10,10]
  
  rosparam set /control_node/Dd [1]
  rosparam set /control_node/Dd [1,1,1,1,1,1,1]
```


# Configuration

A .yaml file is used to set the inital configuration of the ros parameter server. A sample configuration file is shown at the end of this section.  

Explanation of varibles:  
**control_is_on:** Boolean to turn on (true) and off (false) main control while loop, which returns the setpoints for the motor torques. If while loop is turned off, Sawyer will return to a position hold control and hold the last position of the joints.  

## Control node
**joint_angle_desired:** Desired joint configuration.  
**cartesian_pose_desired:** Desired cartesian configuration. (Not in use by default, only for cartesian space controller).
**joint_velocity_desired:** Desired joint velocity.  
**selected_filter_number:** Number of the statemachine state. Each state of the statemachine contains a filter law.  
**Kd_upper_limit:** Upper limit of joint/ cartesian EE stiffness. If cartesian controller used, the last entry will not be used but still needs to be added. Valid input either list (7x1) or list (1x1). If list contains only one number, number will be used for every joint.  
**Kd_lower_limit:** Lower limit of joint/ cartesian EE stiffness. Format see upper limit stiffness.  
**Dd_upper_limit:** Upper limit of joint/ cartesian EE damping. Format see upper limit stiffness.  
**Dd_lower_limit:** Lower limit of joint/ cartesian EE damping. Format see upper limit stiffness.  
**Lowpass_coeff:** Coefficient of lowpass filter which is weighting between new value and old value. Lowpass filter is out of use at the moment.  
**move2neutral:** Flag to trigger a move to the neutral position of the robot (true=triggers the move and after move is finished will be reset to false).  
**suppress_self_collision_detection:** Flag to suppress the self collision detection (true = suppressed).  
**suppress_contact_collision_detection:** Flag to suppress the contact collision detection (true = suppressed).  

## Oscillation guard
**window_length:** Length ob observer window (number of recorded values) for the computation of a fast fourier transformation.  
**corner_frequency:** For all frequencies which are equal or higher than the corner frequency the magnitude is observed to detect a magnitude higher than the allowed limits. If violation is detected control loop will shut down and controller will switch to position hold with the default positon controller by setting the *control_is_on* flag to *false*.  
**magnitude_limit:** Limit of allowed magnitude. If limit is exceeded controller will shut down, as described in *corner_frequency*.  

## Nullspace
**is_locked:** Flag to lock (true) the nullspace to desired pose.  
**jointspace_pose:** Desired nullspace joint configuration.  

## Named pose (Sawyer SDK configuration)
**neutral:** Set neutral pose. *Move2neutral* is subscribing this pose as neutral pose.  

```sh
control_node: {
  control_is_on: false,
  joint_angle_desired: [-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40],
  cartesian_pose_desired: [0,0,0,0,0,0],
  joint_velocity_desired: [0,0,0,0,0,0,0],
  selected_filter_number: 3,
  Kd_upper_limit: [100,80,60,20,10,10,10],
  Kd_lower_limit: [10,10,10,10,10,10,10,],
  Dd_upper_limit: [1],
  Dd_lower_limit: [1],
  Lowpass_coeff: 0.6,
  move2neutral: false,
  suppress_self_collision_detection: false,
  suppress_contact_collision_detection: false,
}

oscillation_guard: {
  window_length: 30,
  corner_frequency: 10,
  magnitude_limit: 10,
}

nullspace: {
  is_locked: false,
  jointspace_pose: [-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40],
}
named_poses: {
  right: {
    poses: {
      neutral: [-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40],
    }
  }
}
```

# Framework
```
sawyer_ctrl_stability
├── config
│   └── inital_config.yaml
├── launch
│   ├── debug_Plots.launch
│   ├── HRI_controller.launch
│   ├── HRI_controller_sim.launch
│   ├── HRI_controller_sim_wPlots.launch
│   └── HRI_controller_wPlots.launch
├── scripts
    ├── cartesianspace_controller.py
    ├── configuration_server.py
    ├── control_node.py
    ├── head_light_manager.py
    ├── jointspace_controller.py
    ├── oscillation_monitor.py
    ├── robot_dyn_kin_server.py
    ├── safety_regulator.py
    ├── stiffness_manager.py
    └── trajectory_executer.py
```

## Config
Contains the config file(s). Structure of files see [configuration](Configuration).

## Launch
Contains .launch files. The launch files with *_sim* are for SIL and the others for HIL experiments. To addition *_wPlots* launchs rqt and the FFT oscillation plot additional to the control-, trajectory-, stiffness/ damping manager node.

## Scripts
Contains all python scripts.

### control node
Main node, managing the control and containing the main while loop. Also, in charge of calculation errors and running the statemachine which as activating the control laws.

### Cartesianspace controller
Contains different control law classes of cartesian space controllers.

### Jointspace controller
Contains different control law classes of joint space controllers.

### configuration server
Gets and sets ros parameter from the ros parameter server.

### Head light manager
Turn on/ off the RGB LED of the headlight depending on the saturation level of the joint torques.

### Oscillation monitor
Creates a plot based on the published frequency and magnitude of each joint.

### Robot dynamic and kinematic server
Conects to the robot, interface to get all kinematic and dynamic values, to set robot parameters and to send setpoints.

### Safety regulator
Regulates the joint angle/torque limit and guards the oscillation.

### Trajectory executer
Executes either a trajectory from a file or publishes a desired pose from the ros param server.

## Record RosBag of Thesis parameters
```sh
rosbag record /control_node/Kd_Dd /control_node/joint_states_desi /control_node/cartesian_pose_desi /robot/joint_states /control_node_debug/joint_velocity /control_node_debug/setpoints_motor_torque /control_node_debug/cartesian_EE_state /control_node_debug/runtime /control_node_debug/color /control_node_debug/oscillation_joint/0 /control_node_debug/oscillation_joint/1 /control_node_debug/oscillation_joint/2 /control_node_debug/oscillation_joint/3 /control_node_debug/oscillation_joint/4 /control_node_debug/oscillation_joint/5 /control_node_debug/oscillation_joint/6 /mediapipe/angle /mediapipe/runtime
```
