import numpy as np
import math 
import PyKDL as kdl
import time

class imedance_ctrl:
    def __init__(self, robot_chain):
        self._J  = self._set_J() # 6x7
        self._J_1 = self._set_J() # 6x7
        self._A = self._set_A() # 6x6
        self._A_1 = self._set_A() # 6x6
        self._Kd = self._set_Kd() # 6x6
        self._Dd = self._set_Dd() # 6x6
        self._Kn = self._set_Kn() # 6x6
        self._Dn = self._set_Dn() # 6x6
        self._qn = self._set_qn() # 7x1
        self.pose_1 = self._set_dpose() # 6x1
        self.dpose_1 = self._set_dpose() # 6x1


        self._err = np.zeros(6) # 6x1
        self._err_1 = np.zeros(6) # 6x1
        self._derr = np.zeros(6) # 6x1

        self.grav_vector = kdl.Vector(0, 0, -9.81)
        self.dyn_kdl = kdl.ChainDynParam(robot_chain, self.grav_vector)
        self.jac_kdl = kdl.ChainJntToJacSolver(robot_chain)
        self._robotic_chain = robot_chain
        

    def run_impedance_controll(self, q, dq, pose, pose_desi, sampling_Time, Dd=np.identity(6), Kd=np.identity(6)):
        self._update_Dd(Dd)
        self._update_Kd(Kd)
        self._update_J(q)
        M = self._update_M(q)
        C = self._update_C(q, dq)
        g = self._update_g(q)

        # Calculate J/dt (derivate of Jacobian)
        dJ = self._calc_dJacobain(sampling_Time)
        
        # Calculate A & A/dt
        self._calc_A(M)
        dA = self._calc_dA(sampling_Time)
        
        # Calculate Corriolis and centrifugal torque
        C_hat = self._calc_C_hat(dA)

        # Calculate N and nullspace torque
        N = self._calc_N(q, M)
        tau_nullspace = self._calc_nullspace(q, dq, N)
        
        # Calculate Position and Orientation Error und the first derivation of the Error
        err = self._calc_err(pose, pose_desi)
        dErr = self._calc_derr(sampling_Time)
        ddx = self._calc_ddx(pose)

        # Calculate Output torque for robot motors
        self.calc_torque(C_hat, dJ, C, tau_nullspace, g, ddx, err, dErr, dq)

        # Set new A,J,err = old A,J,err
        self._A_1 = self._A
        self._J_1 = self._J
        self._err_1 = self._err
        self._dpose_1 = self.dpose
        self.pose_1 = pose



    def _calc_dJacobain(self, dt):
        dJ = (1/dt)*(self._get_J()-self._get_J_1())
        return dJ # return 6x7
    def _calc_A(self, M):
        self._A = np.matmul(np.matmul(self._get_J(),np.linalg.inv(M)),np.transpose(self._get_J()))
        return # return 6x6
    def _calc_dA(self, dt):
        dA = (1/dt)*(self._get_A()-self._get_A_1())
        return dA # return 6x6
    def _calc_C_hat(self, dA):
        C_hat = 0.5*dA
        return C_hat # return 7x7
    def _calc_N(self, q, M):
        N = np.identity(len(q))-np.matmul(np.matmul(np.matmul(np.transpose(self._J),self._A),self._J),np.linalg.inv(M))
        return N # return 7x7
    def _calc_err(self, pose, pose_desi):
        a = self._calc_err_scalar(pose[:3], pose_desi[:3])
        b = self._calc_err_quat(pose[3:], pose_desi[3:])
        tmp = np.concatenate((a,b))
        return tmp # return 6x1
    def _calc_err_scalar(self, pose, pose_desi):
        err_tmp=[]
        for i in range(len(pose)):
            err_tmp.append(pose[i]-pose_desi[i])
        err = np.array(err_tmp)
        return err
    def _calc_derr(self,dt):
        derr = (1/dt)*(self._get_err()-self._get_err_1())
        return derr # return 6x1
    
    def _calc_err_quat(self, pose, pose_desi):
        # TODO Quaterionen Error Implementieren
        err_tmp=[]
        for i in range(len(pose)):
            err_tmp.append(0)
        err = np.array(err_tmp)
        return err #return 3x1
    
    def _calc_nullspace(self, q, dq, N):
        tau_nullspace = np.matmul(N,(np.matmul(-self._Kn,self._calc_err_scalar(q, self._qn))-np.matmul(self._Dn,dq)))
        return tau_nullspace # return 6x1
    def _calc_ddx(self, pose, dt):
        self.dpose = (1/dt)*(self.pose_1()-pose()) # dx #TODO implement either direct kinematic -> ddx or velocity to acceleration, is there a measured acceleration in Gazebo
        ddx = (1/dt)*(self.dpose_1()-self.dpose()) # ddx
        return ddx # return 6x1

    def calc_torque(self, C_hat, dJ, C, tau_nullspace, g, ddx, err, dErr, dq):
        tau_motor = np.transpose(self._J)@(self._A@ddx-(self._Kd@err+self._Dd@dErr)-C_hat@dErr-self._A@dJ@dq)+C@dq+tau_nullspace+g
        return tau_motor # return 7x1
    def convert_KDL2Numpy(self, kdl, nrRows, nrColums):
        numpy = np.zeros(nrRows, nrColums)
        for i in range(nrRows):
            for j in range (nrColums): #TODO evtl. umdrehen, Überprüfen!
                numpy[i][j] = kdl[i, j]
        return np.array(numpy)
    
    """
    Setting-methods to set variables
    """
    def _set_J(self):
        J = np.ones((6,7)) # Dimension is 6x7 matrix
        return J
    def _set_A(self):
        A = np.ones((6,6)) # Dimension is 7x7
        return A
    def _set_Kd(self):
        Kd = np.identity(6)
        return Kd
    def _set_Dd(self):
        Dd = np.identity(6)
        return Dd
    def _set_Kn(self):
        Kn = np.identity(7)
        return Kn
    def _set_Dn(self):
        Dn = np.identity(7)
        return Dn
    def _set_qn(self):
        qn = np.zeros(7)
        return qn
    def _set_dpose(self):
        dpose = np.zeros(6)
        return dpose
    """
    Getter-methods to get Variables
    """

    def _get_J(self):
        return self._J
    def _get_J_1(self):
        return self._J_1
    def _get_A(self):
        return self._A
    def _get_A_1(self):
        return self._A_1
    def _get_Kd(self):
        return self._Kd
    def _get_Dd(self):
        return self._Dn
    def _get_Kn(self):
        return self._Kn
    def _get_Dn(self):
        return self._Dn
    def _get_qn(self):
        return self._qn
    def _get_err(self):
        return self._err
    def _get_err_1(self):
        return self._err_1
    """
    Update-methods to update variables values
    """

    def _update_J(self, q):
        j_kdl = kdl.Jacobian(len(q))
        J = self.jac_kdl.JntToJac(q, j_kdl)
        self._J = self.convert_KDL2Numpy(J,6,7)
        return
    def _update_M(self, q):
        M_kdl = kdl.JntSpaceInertiaMatrix(len(q))
        trq_intertia = self.dyn_kdl.JntToMass(q, M_kdl)
        M = self.convert_KDL2Numpy(trq_intertia,7,7)
        return M
    def _update_C(self, q, dq):
        coriolis_torques = kdl.JntArray(len(q))
        trq_coriolis = self.dyn_kdl.JntToCoriolis(q, dq, coriolis_torques)	# C(q, q_dot) * q_dot
        C = self.convert_KDL2Numpy(trq_coriolis,7,7)
        return C   
    def _update_g(self, q):
        grav_torques = kdl.JntArray(len(q))
        trq_grav = self.dyn_kdl.JntToGravity(q, grav_torques)
        g = self.convert_KDL2Numpy(trq_grav,7,1)
        return g
    def _update_Kd(self, Kd):
        if Kd is not self._Kd:
            self._Kd = Kd
        return
    def _update_Dd(self, Dd):
        if Dd is not self._Dd:
            self._Dd = Dd
        return
    def _update_Kn(self):
        self._Kn = self._Kn
        return
    def _update_Dn(self):
        self._Dn = self._Dn
        return
    def _update_qn(self):
    
        # Predefined value -> no free movement
        self._qn = self._qn
        # Freely movement qn = q_1
        return

def main():

    q = np.ones((7,1))
    dq = np.ones((7,1))
    pose = np.array([1,2,3,4,5,6])#np.ones((6,1))
    pose_desi = np.ones((6,1))*0.5
    sampling_Time = 0.01
    controller = imedance_ctrl()
    start = time.time()
    controller.run_impedance_controll(q, dq, pose, pose_desi, sampling_Time) #looptime with fixed input = 0.0006s
    end = time.time()


if __name__ == '__main__':
    main()