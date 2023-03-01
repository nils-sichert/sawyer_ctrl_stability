
# Control framework for Sawyer HRI

This package provides a control framework for the Rethink robotics robot "Sawyer". It consists of a main control loop, which calculates all necessary kinematics and dynamics and forward them as an input into a state machine which contains instances of different controllers. The control nodes inputs are the URDF, and actual robots kinematics/ dynamics values. The output are torque commands to the robot joints.


## Authors

- [Nils Sichert](https://www.github.com/nils-sichert)


## Installation

Install Ubuntu 20.04 LTS and follow installation guide of Rethink robotics for setting up [ROS, Sawyer's SDK](https://support.rethinkrobotics.com/support/solutions/articles/80000980134-workstation-setup) and [Simulation](https://support.rethinkrobotics.com/support/solutions/articles/80000980381-gazebo-tutorial).

Install necessary packages:
```bash
  sudo apt-get update
  sudo apt-get install python-is-python3
  sudo apt-get install python3-pykdl
  sudo apt-get install ros-noetic-kdl-parser-py
  pip install scipy
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
  rosparam set control_node/control_flag True
```
Stop:
```bash
  rosparam set control_node/control_flag False
```
### Set neutral pose of robot:
```bash
  rosparam set named_poses/right/poses/neutral [-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981]
```

### Set desired joint angle (rad) and joint velocity (rad/s):
Angle:
```bash
  rosparam set control_node/joint_angle_desi [-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981]
```
Velocity:
```bash
  rosparam set control_node/joint_velocity_desi [0, 0, 0, 0, 0, 0, 0]
```

### Set control algorithm - default: 2 
1: DLR Alin Albu-Schäffer Cartesian impedance control\
2: Simple Spring-Damper impedance control\
3: PD impedance control

```bash
  rosparam set control_node/controllerstate 2
```

### Set joint stifness, creates diagonal matrix from value - default: 100 
has to be >=0

```bash
  rosparam set control_node/Kd 100
```
