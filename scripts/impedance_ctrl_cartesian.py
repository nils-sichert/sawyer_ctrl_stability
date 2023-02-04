import numpy as np
import math 
import time

class impedance_ctrl:
    def __init__(self):
        self._A = self._set_A() # 6x6
        self._A_1 = self._set_A() # 6x6
        self._Kn = self._set_Kn() # 7x7
        self._Dn = self._set_Dn() # 7x7
        self._qn = self._set_qn() # 7x1
        
        self._err = np.zeros([6,1]) # 6x1
        self._err_1 = np.zeros([6,1]) # 6x1
        self._derr = np.zeros([6,1]) # 6x1
        
        
        # joint_angles, joint_velocities, pose, pose_desi, self._rate, j_numpy, grav_numpy, mass_numpy, coriolis_numpy
    def run_impedance_controll(self, Kd, Dd, joint_angle, joint_velocity, rate, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, ddx):
        sampling_Time = 1/rate
        # Calculate J/dt (derivate of Jacobian)
        dJ = self._calc_derivate(jacobian, jacobian_1, sampling_Time)
        
        # Calculate A & A/dt
        self._calc_A(inertia, jacobian)
        dA = self._calc_derivate(self._A, self._A_1, sampling_Time)
        
        # Calculate Corriolis and centrifugal torque
        C_hat = self._calc_C_hat(dA)

        # Calculate N and nullspace torque
        N = self._calc_N(joint_angle, inertia, jacobian)
        tau_nullspace = self._calc_nullspace(joint_angle, joint_velocity, N)
        
        # Calculate Position and Orientation Error und the first derivation of the Error
        err = self._calc_err(pose, pose_desi)
        dErr = self._calc_derivate(err, self._err_1, sampling_Time)

        
        # Calculate Output torque for robot motors
        motor_torque = self.calc_torque(jacobian, Kd, Dd, C_hat, dJ, coriolis, tau_nullspace, gravity, ddx, err, dErr, joint_velocity)

        # print('Kd= ', Kd)
        # print('Dd= ', Dd)
        # print('joint angle= ', joint_angle)
        # print('joint velocity= ', joint_velocity)
        # print('rate= ', rate)
        # print('pose= ', pose)
        # print('pose_desi= ', pose_desi)
        # print('coriolis= ', coriolis)
        # print('intertia= ', inertia)
        # print('gravity= ', gravity)
        # print('jacobian= ', jacobian)
        # print('ddx= ', ddx)
        # print('dJ= ', dJ)
        # print('A= ', self._A)
        # print('C_hat= ',C_hat)
        # print('N= ', N)
        # print('tau_nullspace= ',tau_nullspace)
        # print('Motor torque= ', motor_torque)

        # Set new A,J,err = old A,J,err
        self._A_1 = self._A
        self._err_1 = self._err
        return motor_torque

    def _calc_derivate(self, new, old, timestep):
        dT = (1/timestep)*(new-old)
        return dT

    def _calc_A(self, M, J):
        self._A = np.linalg.pinv(J@np.linalg.pinv(M)@np.transpose(J))
        return # return 6x6
    
    def _calc_C_hat(self, dA):
        C_hat = 0.5*dA
        return C_hat # return 7x7
    
    def _calc_N(self, q, M, J):
        N = np.identity(len(q))-np.matmul(np.matmul(np.matmul(np.transpose(J),self._A),J),np.linalg.inv(M))
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
    
    def _calc_err_quat(self, pose, pose_desi):
        # TODO Quaterionen Error Implementieren
        err_tmp=[]
        for i in range(len(pose)):
            err_tmp.append(pose[i]-pose_desi[i])
        err = np.array(err_tmp)
        return err #return 3x1
    
    def _calc_nullspace(self, q, dq, N):
        Kn_q = np.matmul(-self._Kn,self._calc_err_scalar(q, self._qn))

        tau_nullspace = np.matmul(N,Kn_q)-np.matmul(self._Dn,dq)
        return tau_nullspace # return 6x1

    def calc_torque(self, J, Kd, Dd, C_hat, dJ, C, tau_nullspace, g, ddx, err, dErr, dq):
        print('np.matmul(self._A,ddx): ', np.matmul(self._A,ddx))
        print('np.matmul(Dd, dErr): ',np.matmul(Dd, dErr))
        print('np.matmul(Kd, err):' , np.matmul(Kd, err))
        print('np.matmul(C_hat,dErr): ', np.matmul(C_hat,dErr))
        print('self._A@dJ@dq: ',self._A@dJ@dq)
        
        F_t = np.matmul(self._A,ddx)-np.matmul(Dd, dErr)-np.matmul(Kd, err)-np.matmul(C_hat,dErr)-self._A@dJ@dq #6x1
        tau_motor = np.transpose(J)@F_t+C+g+tau_nullspace #c = C*q (computed within KDL)
        return tau_motor # return 7x1
    
    """
    Setting-methods to set variables
    """
  
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
    """
    Getter-methods to get Variables
    """

    def _get_A(self):
        return self._A
    def _get_A_1(self):
        return self._A_1
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
   
    # TODO add update for qn

def main():
    Kd = np.identity(6)
    Dd = np.identity(6)
    joint_angle = np.ones((7,1))
    joint_velocity = np.ones((7,1))
    rate = 100
    coriolis = np.random.rand(7,7)
    inertia = np.random.rand(7,7)
    gravity = np.random.rand(7,1)
    jacobian = np.random.rand(6,7)
    jacobian_1 = np.random.rand(6,7)
    jacobian_2 = np.random.rand(6,7)
    ddx = np.random.rand(6,1)
    pose = np.ones((6,1))
    pose_desi = np.ones((6,1))*0.5
    controller = impedance_ctrl()
    controller.run_impedance_controll(Kd, Dd, joint_angle, joint_velocity, rate, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, ddx) #looptime with fixed input = 0.0006s


if __name__ == '__main__':
    main()