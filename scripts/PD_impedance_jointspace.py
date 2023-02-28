#!/usr/bin/env python

import numpy as np
import math
from scipy.linalg import sqrtm
import rospy

class PD_Impedance_ctrl_woutMass():
    def __init__(self):
        pass

    def calc_joint_torque(self, Jacobian, gravity, Kd, Dd, err_cart, derr_cart, coriolis, joint_angle_desi, cur_joint_angle, cur_joint_velo,joint_angle_error, joint_velocity_error):
        '''
        Input: Jacobian dim:6x7 (J), gravity torque dim:7x1 (g), stiffness matrix dim:6x6 (Kd), joint_angle dim:7x1 (q),
                joint_velocity dim: 7x1 (dq), cartesian pose error dim:6x1 (err_cart), cartesian velocity error dim:(6x1) (derr_cart),
                inertia matrix dim: 7x7 (inetria), coriolis matrix which is already coriolis times joint velocity dim: 7x1 (coriolis)
        
        Ouput: motor torque for each joint motor dim: list (motor_torque)
        '''

        tmp = rospy.get_param("/Dd")
        Dd = np.diag([tmp, tmp, tmp, tmp, tmp, tmp, tmp])

        # Desired Torque
        torque_list = np.zeros((len(joint_angle_error),1))


        for joint in range(len(joint_angle_desi)):
            torque_list[joint] =  -Kd[joint][joint] * joint_angle_error[joint] - Dd[joint][joint] * joint_velocity_error[joint]+ gravity[joint] + coriolis[joint]

        return self.vec2list(torque_list)
    
    def vec2list(self,vec):
        len_vec = len(vec)
        list = [0]*len_vec
        for i in range(len_vec):
            list[i] = vec[i][0]
        return list

if __name__ == "__main__":
    ctrl = PD_Impedance_ctrl_woutMass()
    print("PD Impedance started")