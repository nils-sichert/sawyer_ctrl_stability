#!/usr/bin/env python

import numpy as np
import math
from scipy.linalg import sqrtm
import rospy

class pd_impedance_jointspace():
    def __init__(self):
        pass

    def calc_joint_torque(self, gravity_compensation, Kd, Dd, coriolis_compensation, error_joint_angles, error_joint_velocity):
        '''
        Calculate torques based on displacement from target position, current velocity and includes gravity and coriolis compensation.
        Parameter: Jacobian dim:6x7 (J), gravity torque dim:7x1 (g), stiffness matrix dim:6x6 (Kd), joint_angle dim:7x1 (q),
                joint_velocity dim: 7x1 (dq), cartesian pose error dim:6x1 (err_cart), cartesian velocity error dim:(6x1) (derr_cart),
                inertia matrix dim: 7x7 (inetria), coriolis matrix which is already coriolis times joint velocity dim: 7x1 (coriolis)
        Return: motor torque for each joint motor dim: list (motor_torque)
        '''

        # Desired Torque
        torque_list = np.zeros((len(error_joint_angles),1))


        for joint in range(len(error_joint_angles)):
            torque_list[joint] =  -Kd[joint][joint] * (error_joint_angles[joint]) - Dd[joint][joint] * error_joint_velocity[joint]+ gravity_compensation[joint] + coriolis_compensation[joint]
        return self.vec2list(torque_list)
    
    def vec2list(self,vec):
        """
        Convert vector into list.
        Parameters: vector (numpy array: nx1)
        Return: list (list: nx1)
        """
        len_vec = len(vec)
        list = [0]*len_vec
        for i in range(len_vec):
            list[i] = vec[i][0]
        return list

if __name__ == "__main__":
    ctrl = pd_impedance_jointspace()
    print("PD Impedance started")