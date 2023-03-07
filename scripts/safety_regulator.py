#!/usr/bin/env python3

import numpy as np


class Safety_regulator():
    def __init__(self, joint_angle_limits_upper, joint_angle_limits_lower, joint_efforts_limits_upper, joint_efforts_limits_lower):
        safety_margin = 0.05 # %-safety margin

        self.joint_angle_limit_upper = joint_angle_limits_upper[0]
        self.joint_angle_limit_lower = joint_angle_limits_lower[0]
        self.joint_angle_safety_upper = self.joint_angle_limit_upper*(1-safety_margin)
        self.joint_angle_safety_lower = self.joint_angle_limit_lower*(1-safety_margin)
        self.joint_efforts_limit_upper = joint_efforts_limits_upper[0]
        self.joint_efforts_limit_lower = joint_efforts_limits_lower[0]
        

    ############ Watchdogs ############ 
    
    def watchdog_joint_limits_torque_control(self, jointangles, gravitycompensation, motor_torques):
        for value in self.joints_in_safe_limits(jointangles)[0]:
            if value == False:
                tmp = False
                break
            else:
                tmp = True
        
        if tmp == True:
            return motor_torques
        else:
            motor_torques = self.clip_joint_effort_approach_jointlimit(motor_torques, jointangles, gravitycompensation)
            return motor_torques
    
    def watchdog_joint_limits_jointangle_control(self, jointangles):
        joint_angles = self.clip_joints_angle_safe(jointangles)
        return joint_angles
    
    def watchdog_torque_limits(self, motor_torques):
        motor_torques = self.clip_joints_effort_safe(motor_torques)
        return motor_torques
    
    def watchdog_oscillation(self, motor_torques):
        # TODO implement Oscillation detection
        if True:
            control_StartStop_Flag = True
        else:    
            control_StartStop_Flag = False
        return control_StartStop_Flag

    ############ Manipulators #############
        
    def joints_in_limits(self, q):
        """
        Control if joint angle are within limits.
        Parameters: joint angle (7x1)
        Return: limited joint anlges (7x1) or 0 if not in limit
        """
        lower_lim = self.joint_angle_limits_lower
        upper_lim = self.joint_angle_limits_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    def joints_in_safe_limits(self, q):
        """
        Control if joint angle are within safety area.
        Parameters: joint angle (7x1)
        Return: safety joint anlges (7x1) or 0 if not in limit
        """
        lower_lim = self.joint_angle_safety_lower
        upper_lim = self.joint_angle_safety_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)
    
    def clip_joints_angle_safe(self, joint_angles):
        """
        Clip joint angles to safe angles.
        Parameters: joint anlges (7x1)
        Retrun: clipt joint angles (7x1)
        """
        lower_lim = self.joint_angle_safety_lower
        upper_lim = self.joint_angle_safety_upper
        return np.clip(joint_angles, lower_lim, upper_lim)

    def clip_joints_effort_safe(self, effort):
        """
        Clip joint angles to safe angles.
        Parameters: joint efforts (7x1)
        Retrun: clipt joint efforts (7x1)
        """
        lower_lim = self.joint_efforts_limit_lower
        upper_lim = self.joint_efforts_limit_upper
        return np.clip(effort, lower_lim, upper_lim)
    
    def clip_joint_effort_approach_jointlimit(self, motor_torques, joint_angles, gravity_compensation, torque_reduction_factor = 0.1):
        # TODO debug effort decreasing when approaching joint limit 
        # for i in range(len(motor_torques)):
        #    if joint_angles[i] <= self.joint_angle_safety_lower[i] or joint_angles[i] >= self.joint_angle_safety_upper[i]:
        #        new_torque = (motor_torques[i]-gravity_compensation[i])*torque_reduction_factor+gravity_compensation[i]
        #        motor_torques[i] = new_torque
        return motor_torques

def main():
    watchdog = Safety_regulator(np.array([1,1,1,1,1,1,1]),np.array([-1,-1,-1,-1,-1,-1,-1]))
    joint_angles = np.array([0.5, 0.5, 0.9, 2, -2, 0.5, 0.5])
    gravity_compensation = np.array([1,2,3,4,5,6,7])
    motor_torques = np.array([11,12,13,14,15,16,17])
    print(watchdog.watchdog_joint_limits_torque_control(joint_angles, gravity_compensation, motor_torques))
    print(watchdog.watchdog_joint_limits_jointangle_control(joint_angles))
    print(watchdog.watchdog_torque_limits(motor_torques))

if __name__ == "__main__":
    main()