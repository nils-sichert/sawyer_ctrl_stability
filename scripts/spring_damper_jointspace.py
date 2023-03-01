#!/usr/bin/env python
import numpy as np

class Spring_damper_jointspace():
    def __init__(self) -> None:
        pass

    def calc_torque(self, joint_angle_desi, cur_joint_angle, cur_joint_velo, Kd, Dd, gravity):
        """
        Calculate torques based on displacement from target position and current velocity.
        Parameters: joint angle desired (numpy array: 7x1), current joint angle (numpy array: 7x1), current joint velocity (numpy array: 7x1), 
            joint stiffness matrix Kd (numpy array: 7x7), joint damping matrix Dd (numpy array: 7x7), gravity vector (numpy array: 7x1)
        Return: joint torques (list: 7x1)
        """
        torque_list = [0]*len(joint_angle_desi)
        
        for joint in range(len(joint_angle_desi)):
            torque_list[joint] = Kd[joint][joint] * (joint_angle_desi[joint] - cur_joint_angle[joint]) - Dd[joint][joint] * cur_joint_velo[joint] + gravity[joint]

        return self.array_to_list(torque_list)
    
    def array_to_list(self, array):
        """
        Converte numpy array into list format.
        Parameters: Array (numpy arra: nx1)
        Return: list (dict: nx1)
        """
    
        list = [0]*len(array)
        for i in range(len(array)):
            list[i] = array[i][0]
        return list

if __name__ == "__main__":
    ctrl = Spring_damper_jointspace()
    print("Spring damper Ctrl started")