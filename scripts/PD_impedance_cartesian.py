import numpy as np
from scipy.linalg import sqrtm

class PD_Impedance_ctrl():
    def __init__(self) -> None:
        # TODO change into rosparams
        self._Kn = np.diag([1,1,1,1,1,1,1]) # positiv definite stiffness matrix - 7x7 matrix
        self._Dn = np.diag([1,1,1,1,1,1,1]) # positiv definite damping matrix - 7x7 matrix
        self._qn = np.atleast_2d(np.array([1,1,1,1,1,1,1])).T # desired joint configuration of arm at nullspace - 7x1 vector
        self._D_eta = np.diag([0.7,0.7,0.7,0.7,0.7,0.7])
        self._flag_nullspace = True # True: free Nullspace movement allowed

    def calc_joint_torque(self, Jacobian, gravity, Kd, q, dq, err_cart, derr_cart, mass, coriolis):
        '''
        Input: Jacobian dim:6x7 (J), gravity torque dim:7x1 (g), stiffness matrix dim:6x6 (Kd), joint_angle dim:7x1 (q),
                joint_velocity dim: 7x1 (dq), cartesian pose error dim:6x1 (err_cart), cartesian velocity error dim:(6x1) (derr_cart),
                inertia matrix dim: 7x7 (inetria), coriolis matrix which is already coriolis times joint velocity dim: 7x1 (coriolis)
        
        Ouput: motor torque for each joint motor dim: list (motor_torque)
        '''

        # Compute constant values for faster algorithm 
        Jt = np.transpose(Jacobian)     # transpose matrix of the jacobian
        Mass_inv = np.linalg.inv(mass)    # pseudo inverse of the inertia matrix
        F_load = self.get_F_load()      # compute the load compensation dim: 6x1
        
        Lambda = np.linalg.inv(Jacobian @ Mass_inv @ Jt) #V=(J*M^⁻1*J.T)^-1

        # Find best damping matrix with factorization damping design
        A = sqrtm(Lambda)                    # TODO has to be sqrtm --> Erro in calculation, therefore complex 
        Kd1 = sqrtm(Kd)
        Dd = A @ self._D_eta @ Kd1 + Kd1 @ self._D_eta @ A 

        # Nullspace control
        N = np.identity(7)-Jt @ Lambda @ Jacobian @ Mass_inv
        tau_nullspace = N @ (-self._Kn @ (q-self._qn)-self._Dn @ dq)

        if self._flag_nullspace == True:
            self._qn = q

        # Desired Torque
        motor_torque = Jt @ (-(Kd @ err_cart+Dd @ derr_cart)+F_load)+coriolis+gravity+tau_nullspace
        return self.vec2list(motor_torque)
    
    def get_F_load(self):
        F_load = np.zeros((6,1))
        return F_load
    
    def vec2list(self,vec):
        len_vec = len(vec)
        list = [0]*len_vec
        for i in range(len_vec):
            list[i] = vec[i][0,0]
        return list
    
def main():
    Jacobian = np.random.rand(6,7) # numpy 6x1
    gravity = np.random.rand(7,1)# numpy 7x1
    Kd = np.random.rand(6,6)# numpy 6x6
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
    print(ctrl.calc_joint_torque(Jacobian, gravity, Kd, q, dq, err_cart, derr_cart, mass, coriolis))

if __name__ == '__main__':
    main()