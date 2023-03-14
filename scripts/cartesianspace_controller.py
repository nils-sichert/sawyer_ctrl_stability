#!/usr/bin/env python3

import numpy as np
import math


class pd_impedance_cartesian():
    def __init__(self):
        self._q_n = np.atleast_2d([-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40]).T

    def calc_joint_torque(self, K_d, D_d, K_n, q_n, error, coriolis, jacobian, gravity, dq, q, nullspace_is_locked):
        """
        PD cartesian controller (source: Franka Emika/ Springer - Impedance control). 
        Parameters: desired stiffness (K_d: 6x6), desired damping (D_d: 6x6), stiffness of nullspace (K_n: 7x7), desired joint pose nullspace (q_n: 7x1), 
                    error of cartesian pose (error: 6x1), coriolis compensation (coriolis: 7x1), jacobian matrix of robot (jacobian: 6x7), gravity compensation (gravity: 7x1),
                    joint velocity (dq: 7x1), joint angle (q: 7x1)
        Return: list of motor torque setpoints (tau_d: 7x1)
        """
        # Set nullspace configuration
        if nullspace_is_locked == True:
            q_n_tmp = q_n
        else: 
            q_n_tmp = self._q_n

        # Compute PD control torques
        tau_task = np.transpose(jacobian) @ (-K_d @ error - D_d @ (jacobian @ dq))

        # Compute nullspace torques
        tau_nullspace = (np.identity(7) - np.transpose(jacobian) @ np.linalg.pinv(np.transpose(jacobian))) @ (K_n @ (q_n_tmp-q)-(2.0*np.sqrt(K_n)) @ dq)

        # Compute final torque with coriolis and gravity compensation
        tau_d = tau_task + tau_nullspace + coriolis + gravity

        # Reset nullspace configuration setpoint
        self._q_n = q

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
        self._D_eta = np.diag([0.7,0.7,0.7,0.7,0.7,0.7])
        self._lambda_prev = np.zeros((6,6))
        self._q_n = np.atleast_2d([-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40]).T


    def calc_joint_torque(self, K_d, K_n, D_n, q_n, mass, jacobian, djacobian, coriolis, gravity, error, derror, ddx, q, dq, periodTime, nullspace_is_locked):
        '''
        Cartesian impedance controller based on "Cartesian Impedance Control of redundand Robots" (DLR: Alin Albu-Schäfer, et. al.)
        Parameters: desired stiffness (K_d: 6x6), stiffness of nullspace (K_n: 7x7), damping of nullspace (D_n: 7x7), desired joint pose nullspace (q_n: 7x1), 
                    mass matrix (mass: 7x7), jacobian matrix of robot (jacobian: 6x7), first derivate of jacobian (djacobian: 6x7), coriolis compensation (coriolis: 7x1), 
                    gravity compensation (gravity: 7x1), error of cartesian pose (error: 6x1), error of cartesian velocity (derror: 6x1), cartesian acceleration (ddx: 6x1),
                    joint angle (q: 7x1), joint velocity (dq: 7x1), time between two controllops (periodTime: float), flag if nullspace is locked (=True) or free (=False)
        Ouput: motor torque for each joint motor dim: list (motor_torque)
        '''
        
        # Set nullspace configuration
        if nullspace_is_locked == True:
            q_n_tmp = q_n
        else: 
            q_n_tmp = self._q_n

        # Compute variables which are used more than once to safe computational power
        mass_inv = np.linalg.inv(mass)
        arg_lambda_complex = np.array(jacobian @ mass_inv @ jacobian.T, dtype = np.complex_)
        Lambda = np.linalg.inv(arg_lambda_complex)

        # Design damping matrix: Factorization damping design
        A = np.sqrt(Lambda) 
        K_d1 = np.sqrt(K_d)
        D_d = A @ self._D_eta @ K_d1 + K_d1 @ self._D_eta @ A

        # Compute C_hat
        C_hat = 0.5*(Lambda - self._lambda_prev)/ periodTime

        # Control Law without nullspace control
        F_tau = Lambda @ ddx - D_d @ derror - K_d @ error - C_hat @ derror - Lambda @ djacobian @ dq #- external_load
        
        tau_task = jacobian.T @ F_tau

        # Nullspace control
        N = np.identity(7) - jacobian.T @ Lambda @ jacobian @ mass_inv
        tau_nullspace = N @ (-K_n @ (q-q_n_tmp) - D_n @ dq)

        # Compute desired torque
        tau_d = tau_task + tau_nullspace + coriolis + gravity

        # Reset nullspace configuration setpoint
        self._q_n = q

        return self.vec2list(tau_d)

    def vec2list(self, vec):
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


if __name__ == "__main__":
    # ctrl = pd_impedance_cartesian()
    ctrl = dlr_impedance_cartesian()
    print("DLR Impedance started")
