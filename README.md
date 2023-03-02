
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
  rosparam set /control_node/control_flag True
```
Stop:
```bash
  rosparam set /control_node/control_flag False
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

