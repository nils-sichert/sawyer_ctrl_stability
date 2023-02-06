import numpy as np
import math
from scipy.linalg import sqrtm

class PD_Impedance_ctrl_woutMass():
    def __init__(self) -> None:
        # TODO change into rosparams
        self._Kn = np.diag([1,1,1,1,1,1,1]) # positiv definite stiffness matrix - 7x7 matrix
        self._Dn = np.diag([1,1,1,1,1,1,1]) # positiv definite damping matrix - 7x7 matrix
        self._qn = np.atleast_2d(np.array([1,1,1,1,1,1,1])).T # desired joint configuration of arm at nullspace - 7x1 vector
        self._D_eta = np.diag([0.7,0.7,0.7,0.7,0.7,0.7])

    def calc_joint_torque(self, Jacobian, gravity, Kd, err_cart, derr_cart, coriolis, joint_angle_desi, cur_joint_angle, cur_joint_velo):
        '''
        Input: Jacobian dim:6x7 (J), gravity torque dim:7x1 (g), stiffness matrix dim:6x6 (Kd), joint_angle dim:7x1 (q),
                joint_velocity dim: 7x1 (dq), cartesian pose error dim:6x1 (err_cart), cartesian velocity error dim:(6x1) (derr_cart),
                inertia matrix dim: 7x7 (inetria), coriolis matrix which is already coriolis times joint velocity dim: 7x1 (coriolis)
        
        Ouput: motor torque for each joint motor dim: list (motor_torque)
        '''

        # Compute constant values for faster algorithm 
        Jt = np.transpose(Jacobian)     # transpose matrix of the jacobian


        Dd = np.diag([1,1,1,1,1,1,1])

        # Desired Torque
        torque_list = [0]*len(joint_angle_desi)
        
        for joint in range(len(joint_angle_desi)):
            # spring portion
            torque_list[joint] = Kd[joint][joint] * (joint_angle_desi[joint] - cur_joint_angle[joint]) - Dd[joint][joint] * cur_joint_velo[joint] + gravity[joint] + coriolis[joint]

        
        return self.vec2list(torque_list)
    
    def vec2list(self,vec):
        len_vec = len(vec)
        list = [0]*len_vec
        for i in range(len_vec):
            list[i] = vec[i][0]
        return list

    def srqt_mat(self, matrix):
        numRows = len(matrix)
        numColums = len(matrix[0])
        sqrt_matrix = np.zeros((numRows, numColums))
        for i in range(numRows):
            for j in range(numColums):
                sqrt_matrix[i][j] = math.sqrt(matrix[i][j])
        return sqrt_matrix
    
def main():
    Jacobian = np.random.rand(6,7) # numpy 6x1
    gravity = np.random.rand(7,1)# numpy 7x1
    Kd = np.diag([1,4,9,16,25,36])# numpy 6x6
    q =  np.random.rand(7,1)# numpy 7x1
    dq =  np.random.rand(7,1)# numpy 7x1
    err_cart = np.random.rand(6,1)# numpy 6x1
    derr_cart = np.random.rand(6,1)# numpy 6x1
    mass = np.random.rand(7,7)# numpy 7x7
    coriolis = np.random.rand(7,1)# numpy 7x1
   
    """  print('Jacobian: ', Jacobian)
    print('gravity: ', gravity)
    print('Kd', Kd)
    print('q', q)
    print('dq', dq)
    print('err_cart', err_cart)
    print('derr_cart', derr_cart)
    print('inertia', inertia)
    print('coriolis',coriolis) """

    ctrl = PD_Impedance_ctrl()
    print(ctrl.calc_joint_torque(Jacobian, gravity, Kd, err_cart, derr_cart, mass, coriolis))

if __name__ == '__main__':
    main()