#!/usr/bin/env python3

import numpy as np


class pd_impedance_cartesian():
    def __init__(self):
        pass

    def calc_joint_torque(self, Kd, Dd, Kn, Dn, qn, cartesian_pose_error, cartesian_velocity_error, coriolis, jacobian, gravity, dq, q):
        tau_task = np.transpose(jacobian) @ (-Kd @ cartesian_pose_error-Dd@ (jacobian @ dq))
        tau_nullspace = (np.identity(7) - np.transpose(jacobian) @ np.linalg.pinv(np.transpose(jacobian))) @ (Kn @ (qn-q)-(2.0*np.sqrt(Kn)) @ dq)
        tau_d = tau_task + tau_nullspace + coriolis + gravity
        return self.vec2list(tau_d)
    
    def vec2list(self,vec):
        """
        Convert vector into list.
        Parameters: vector (numpy array: nx1)
        Return: list (list: nx1)
        """
        len_vec = len(vec)
        list = [0]*len_vec
        for i in range(len_vec):
            list[i] = vec[i][0,0]
        return list
        
class dlr_impedance_cartesian():
    def __init__(self) -> None:
        self._Kn = np.diag([1,1,1,1,1,1,1]) # positiv definite stiffness matrix - 7x7 matrix
        self._Dn = np.diag([1,1,1,1,1,1,1]) # positiv definite damping matrix - 7x7 matrix
        self._qn = np.atleast_2d(np.array([-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40])).T # desired joint configuration of arm at nullspace - 7x1 vector
        self._D_eta = np.diag([0.7,0.7,0.7,0.7,0.7,0.7])
        self._flag_nullspace = False # True: free Nullspace movement allowed
        self._lambda_prev = np.zeros((6,6))


    def calc_joint_torque(self,  Kd, cartesian_acceleration, cur_joint_angle, cur_joint_velocity, cartesian_pose_error, cartesian_velocity_error,  mass, coriolis, jacobian, djacobian, gravity, periodTime, external_load = 0):
        '''
        Input: Jacobian dim:6x7 (J), gravity torque dim:7x1 (g), stiffness matrix dim:6x6 (Kd), joint_angle dim:7x1 (q),
                joint_velocity dim: 7x1 (dq), cartesian pose error dim:6x1 (err_cart), cartesian velocity error dim:(6x1) (derr_cart),
                mass matrix dim: 7x7 (inetria), coriolis matrix which is already coriolis times joint velocity dim: 7x1 (coriolis)
        
        Ouput: motor torque for each joint motor dim: list (motor_torque)
        '''
        
        Kn = self._Kn * 16
        Dn = self._Dn * np.sqrt(Kn[0][0])
        mass_inv = np.linalg.inv(mass)
        print(mass_inv)
        Lambda = np.linalg.inv(jacobian @ mass_inv @ jacobian.T)

        #TODO clean algorithm
        # Find best damping matrix with factorization damping design
        A = np.sqrt(Lambda) 
        
        Kd1 = np.sqrt(Kd)
        Dd = A @ self._D_eta @ Kd1 + Kd1 @ self._D_eta @ A

        C_hat = 0.5*(Lambda-self._lambda_prev)/ periodTime
        # Law
        F_tau = Lambda @ cartesian_acceleration - Dd @ cartesian_velocity_error - Kd @ cartesian_pose_error - C_hat* cartesian_velocity_error - Lambda @ djacobian @ cur_joint_velocity #- external_load
        # Dd eliminated
        # F_tau = Lambda @ cartesian_acceleration - Kd @ cartesian_pose_error - Lambda @ djacobian @ cur_joint_velocity #- external_load
        # Velocity = 0
        # F_tau = Lambda @ cartesian_acceleration - Kd @ cartesian_pose_error #- external_load
        
        tau_task = jacobian.T @ F_tau

        # Nullspace control
        N = np.identity(7) - jacobian.T @ Lambda @ jacobian @ mass_inv
        # Law
        tau_nullspace = N @ (-Kn @ (cur_joint_angle-self._qn) - Dn @ cur_joint_velocity)
        # Velocity = 0
        # tau_nullspace = N @ (-Kn @ (cur_joint_angle-self._qn))

        # Desired torque
        tau_d = tau_task + tau_nullspace + coriolis + gravity

        self._qn = cur_joint_angle

        return self.vec2list(tau_d)

    def vec2list(self, vec):
        len_vec = len(vec)
        list = [0]*len_vec
        for i in range(len_vec):

            list[i] = vec[i][0,0]
        return list


if __name__ == "__main__":
    # ctrl = pd_impedance_cartesian()
    ctrl = dlr_impedance_cartesian()
    print("DLR Impedance started")
