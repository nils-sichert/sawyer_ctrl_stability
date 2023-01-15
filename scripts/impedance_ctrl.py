import numpy as np
import PyKDL as KDL

'''
Input: gravity vector, joint angles, joint velocities, joint efforts, jacobian, 
    first derivativ of jacobian, 

Output: Controll torques
'''

class impedance_ctrl:
    def __init__(self, robot_chain, nrOfJoints):
        # load kinematics and dynamics models
        self.grav_vector = KDL.Vector(0, 0, -9.81)
        self.dyn_kdl = KDL.ChainDynParam(robot_chain, self.grav_vector)
        self.joint_angles = KDL.JntArray(nrOfJoints)
        self.joint_velocities = KDL.JntArray(nrOfJoints)
        self.joint_efforts = KDL.JntArray(nrOfJoints)
        self.grav_torques = KDL.JntArray(nrOfJoints)
        self.coriolis_torques = KDL.JntArray(nrOfJoints)
        self.B_kdl = KDL.JntSpaceInertiaMatrix(nrOfJoints)
        self.j_kdl = KDL.Jacobian(nrOfJoints)
        self.jdot_kdl = KDL.Jacobian(nrOfJoints)
        self.nrOfJoints = nrOfJoints

    def run(self, joint_angles, joint_velocities, joint_efforts, rate, jac_kdl, jacdot_kdl, current_ee_wrench, current_ee_pose, joint_torques):
        # Define impedance controller parameters
        # TODO fix misstakes and write pseudo code algorithm
        M = np.identity(3) # mass matrix in the mass-spring-damper system TODO update while running
        D = 20 * np.identity(3)
        K = 10 * np.identity(3)
        
        self._dataconversion()
        trq_grav, trq_coriolis, trq_intertia= self._compute_torques(joint_angles, joint_velocities)
        jdot_qdot = self._compute_jacobians(joint_angles, joint_velocities, rate, jac_kdl, jacdot_kdl)
        self._update_desi_pose()
        self._update_ee_pose()
   
        # create our command dict
        cmd = dict()

        inertia_torques = [0] * self.nrOfJoints
        interaction_torques = [0]* self.nrOfJoints
        
        #################### Compute u, the control input ####################
        # reformatting j_kdl to numpy format
        j_numpy = [[0 for _ in range(7)] for _ in range(6)]
        for i in range(0, 6):
            for j in range (0, 7):
                j_numpy[i][j] = self.j_kdl[i, j]

        # computing pseudo-invers of the jacobian transpose
        j_kdl_inv = np.linalg.pinv(j_numpy)

        # Define desired end effector pose
        self.xd = [self.init_ee_pose.position.x + 0.5, self.init_ee_pose.position.y, self.init_ee_pose.position.z]	# xd
        xd_dot = [0, 0, 0]	# xd_dot
        xd_dot_dot = [0, 0, 0]	# xd_dot
        
        x_dot = [0] * 3
        for i in range(0, 3):
            for j in range (0, self.nrOfJoints):
                x_dot[i] += self.j_kdl[i, j] * joint_velocities[j]
        
        # Caclulate Error
        x_tilde = self._calculate_error(self.xd, current_ee_pose)
        x_tilde_dot = self._calculate_error(xd_dot, x_dot)
        
        # TODO understand and correct
        M_jdot_qdot = [0] * 3
        for i in range(0, 3):
            for j in range (0, 3):
                M_jdot_qdot[i] += M[i][j] * jdot_qdot[j]	# M * J_dot * q_dot

        JM_inv = [[0 for _ in range(3)] for _ in range(7)]
        for i in range(0, 7):
            for j in range (0, 3):
                for k in range (0, 3):
                     JM_inv[i][j] += j_kdl_inv[i][k] * M[k, j]	# J_inv * M_inv
        
        u = np.matmul(JM_inv, np.matmul(M, xd_dot_dot) + np.matmul(D, x_tilde_dot) + np.matmul(K, x_tilde) - M_jdot_qdot)#- Fa[0:3])

        for i in range(0, self.nrOfJoints):
            for j in range (0, self.nrOfJoints):
                inertia_torques[i] += self.B_kdl[i, j] * u[j]	# B * u

        # assign to torques commands
        i = 0
        for joint in range(self.nrOfJoints):
            cmd[joint] = inertia_torques[i] + trq_coriolis[i] + trq_grav[i] #+ interaction_torques[i]

        return cmd

    def _compute_torques(self, joint_angles, joint_velocities):
        # compute torques
        trq_grav = self.dyn_kdl.JntToGravity(joint_angles, self.grav_torques)	# g(q)
        trq_coriolis = self.dyn_kdl.JntToCoriolis(joint_angles, joint_velocities, self.coriolis_torques)	# C(q, q_dot) * q_dot
        trq_intertia = self.dyn_kdl.JntToMass(joint_angles, self.B_kdl)
        return trq_grav, trq_coriolis, trq_intertia

    def _compute_jacobians(self, joint_angles, joint_velocities, rate, jac_kdl, jacdot_kdl):
        q_dqdt = KDL.JntArray(joint_angles)
        for i in range(joint_angles.rows()):
            q_dqdt[i] +=  rate * joint_velocities[i]

        jac_kdl.JntToJac(joint_angles, self.j_kdl)	# Jacobian
        jacdot_kdl.JntToJacDot(KDL.JntArrayVel(q_dqdt, joint_velocities), self.jdot_kdl)	# J_dot
        
        # taken from KDL example
        jdot_qdot = KDL.Twist()
        ret = KDL.MultiplyJacobian(self.jdot_kdl, joint_velocities, jdot_qdot)	# J_dot * q_dot
        return ret

    def _calculate_error(self,first, second): #first and second need to be same dimension
        tmp = [0]*len(first)
        for i in range(len(first)):
            tmp[i]=first[i]-second[i]