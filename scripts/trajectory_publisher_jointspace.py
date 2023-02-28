#!/usr/bin/env python3

import rospy
import numpy as np

class trajectoy_executer():
    def __init__(self) -> None:
        rospy.init_node("trajectory_executer")
        setpoint1 = np.array([-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981])
        setpoint2 = np.array([-1.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981])
        setpoint3 = np.array([-1.3588, -0.5833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981])
        setpoint4 = np.array([-2.3588, -0.5833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981])
        setpoint5 = np.array([-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981])
        self.setpoints = [setpoint1, setpoint2, setpoint3, setpoint4, setpoint5]
        self.radspeed = 0.0002

    def publish(self):
        for i in range(len(self.setpoints)-1):
            diff = self.setpoints[i+1]-self.setpoints[i]
            distance = np.max(np.abs(diff))
            print(self.setpoints[i+1],self.setpoints[i])
            steps = int(distance/self.radspeed)
            dealta_step = diff/steps
            r = rospy.Rate(1000)
            for j in range(steps):
                desired_pose = np.array(self.setpoints[i]+j*dealta_step).tolist()
                rospy.set_param("control_node/joint_angle_desi", desired_pose)
                r.sleep()
    
if __name__ =="__main__":
    trajectory_ex = trajectoy_executer()
    trajectory_ex.publish()

    
    
    
        
