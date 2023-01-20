import numpy as np
import math 
import PyKDL as kdl

class imedance_ctrl:
    def __init__(self):
        self._J  = self._set_J()
        self._J_1 = self._set_J()
        self._A = self._set_A()
        self._A_1 = self._set_A()
        self._Kd = self._set_Kd()
        self._Dd = self._set_Dd()
        self._Kn = self._set_Kn()
        self._Dn = self._set_Dn()
        self._qn = self._set_qn()

        self._err = np.zeros(6)
        self._err_1 = np.zeros(6)
        self._derr = np.zeros(6)

    def run_impedance_controll(self, q, dq, pose, pose_desi, sampling_Time, Dd=None, Kd=None,): #TODO change default values
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
        ddx = self._calc_ddx()

        # Calculate Output torque for robot motors
        self.calc_torque(C_hat, dJ, C, tau_nullspace, g, ddx, err, dErr, dq)

        # Set new A,J,err = old A,J,err
        self._A_1 = self._A
        self._J_1 = self._J
        self._err_1 = self._err

    def _calc_dJacobain(self, dt):
        dJ = (1/dt)*(self._get_J()-self._get_J_1())
        return dJ
    def _calc_A(self, M):
        self._A = np.matmul(np.matmul(np.transpose(self._get_J()),np.linalg.inv(M)),self._get_J())
        return
    def _calc_dA(self, dt):
        dA = (1/dt)*(self._get_A()-self._get_A_1())
        return dA
    def _calc_C_hat(self, dA):
        C_hat = 0.5*dA
        return C_hat
    def _calc_N(self, q, M):
        N = np.identity(len(q))-np.transpose(self._J)*self._A*self._J*np.linalg.inv(M)
        return N
    def _calc_err(self, pose, pose_desi):
        a = self._calc_err_scalar(pose, pose_desi)
        b = self._calc_err_quat(pose, pose_desi)
        tmp = [] #TODO implement more elegant solution
        for i in range(len(a)):
            tmp.append(a[i])
        for j in range(len(b)):
            tmp.append(b[j])
        return tmp
    def _calc_err_scalar(self, pose, pose_desi):
        err_tmp=[]
        for i in range(3):
            err_tmp.append(pose[i]-pose_desi[i])
        self._err = err_tmp
        return
    def _calc_derr(self,dt):
        derr = (1/dt)*(self._get_err()-self._get_err_1())
        return derr
    def _calc_err_quat(self, pose, pose_desi):
        err_tmp=[]
        for i in range(3):
            #TODO add quaterionen error calculation
            err_tmp = 0
        return err_tmp
    def _calc_nullspace(self, q, dq, N):
        tau_nullspace = N*(-self._get_Kn*self._calc_err(q, self._get_qn)-self._get_Dn*dq)
        return tau_nullspace
    def _calc_ddx(self):
        pass # TODO Implement x/ddt reread in Paper
    def calc_torque(self, C_hat, dJ, C, tau_nullspace, g, ddx, err, dErr, dq):
        tau_motor = np.transpose(self._J)*(self._A*ddx-(self._Kd*err+self._Dd* dErr)-C_hat*dErr-self._A*dJ*dq)+C+tau_nullspace+g
    def convert_KDL2Numpy(self, kdl):
        num_colum = 0
        num_rows = 0 #TODO unabhängig machen Breite und Länge der Matrix KDL ermitteln erstellen (0 ersetzten)
        numpy = np.zeros(num_rows, num_colum)
        for i in range(num_rows):
            for j in range (num_colum): #TODO evtl. umdrehen, Überprüfen!
                numpy[i][j] = kdl[i, j]
        return numpy
    
    """
    Setting-methods to set variables
    """
    #TODO implement settings
    def _set_J(self):
        J = np.ones((7,6)) #TODO implement right dimension and control them
        return J
    def _set_A(self):
        A = np.ones((7,6)) #TODO implement right dimension and control them
        return A
    def _set_Kd(self):
        Kd = np.identity(6)
        return Kd
    def _set_Dd(self):
        Dd = np.identity(6)
        return Dd
    def _set_Kn(self):
        Kn = np.identity(6)
        return Kn
    def _set_Dn(self):
        Dn = np.identity(6)
        return Dn
    def _set_qn(self):
        qn = np.zeros(7)
        return qn

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
        # TODO Import Library
        self._J = np.ones((7,6))
        return
    def _update_M(self, q):
        # TODO Import Library
        M = 0
        return M
    def _update_C(self, q, dq):
        # TODO Import Library
        C = 0
        return C   
    def _update_g(self, q):
        # TODO Import Library
        g = 0
        return g
    def _update_Kd(self, Kd):
        if Kd is not None:
            self._Kd = Kd
        return
    def _update_Dd(self, Dd):
        if Dd is not None:
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
    q = np.array([1,1,1,1,1,1,1])
    dq = np.array([1,1,1,1,1,1,1])
    pose = np.array([1,1,1,0.5,0.5,0.5])
    pose_desi = np.array([1.1,1.1,1.1,0.4,0.4,0.4])
    sampling_Time = 0.01
    controller = imedance_ctrl()
    controller.run_impedance_controll(q, dq, pose, pose_desi, sampling_Time)


if __name__ == '__main__':
    main()