#!/usr/bin/env python3

import numpy as np

class spring_damper_jointspace():
    def __init__(self) -> None:
        pass

    def calc_joint_torque(self, error_joint_angles, error_joint_velocity, Kd, Dd, gravity_compensation):
        """
        Calculate torques based on displacement from target position and current velocity.
        Parameters: error between desired and actual joint angel (numpy arra: 7x1), current joint velocity (numpy array: 7x1), 
            joint stiffness matrix Kd (numpy array: 7x7), joint damping matrix Dd (numpy array: 7x7), gravity compensation vector (numpy array: 7x1)
        Return: joint torques (list: 7x1)
        """
        torque_list = [0]*len(error_joint_angles)
        
        # Control law
        for joint in range(len(error_joint_angles)):
            torque_list[joint] = -Kd[joint][joint] * (error_joint_angles[joint]) - Dd[joint][joint] * error_joint_velocity[joint] + gravity_compensation[joint]
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

        # Control law
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
    ctrl = spring_damper_jointspace()
    # ctrl = pd_impedance_jointspace()
    print("Spring damper Ctrl started")