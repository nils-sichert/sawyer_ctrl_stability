#!/usr/bin/env python3
        
import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from intera_core_msgs.msg import JointLimits, SEAJointState
import threading
import PyKDL as kdl
from kdl_parser_py import urdf
from sys import argv
from os.path import dirname, join, abspath
from impedance_ctrl_cartesian import impedance_ctrl

import intera_interface
from intera_interface import CHECK_VERSION


class controller():
    def __init__(self, limb = "right"):
        rospy.init_node('Passiv_Activ_Controller', anonymous=True)
        self.Kd = rospy.get_param("Kd", [1,1,1,1,1,1])
        self.Dd = rospy.get_param("Dd", [1,1,1,1,1,1])
        self._init_joint_angles = rospy.get_param("Init_angle", [-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981])

        # control parameters
        self.rate = 100 # Controlrate - 100Hz
        self._missed_cmd = 20 # Missed cycles before triggering timeout
        
        # Instance Impedance controller
        self.impedance_ctrl = impedance_ctrl()

        # create limb instance
        self._limb = intera_interface.Limb(limb)
        # Instance Optimal Controller
        # Instance Robotic Chain
        urdf_filepath = "src/sawyer_robot/sawyer_description/urdf/sawyer_base.urdf.xacro"
        (ok, robot) = urdf.treeFromFile(urdf_filepath)
        self._robot_chain = robot.getChain('right_arm_base_link', 'right_l6')
        self._nrOfJoints = self._robot_chain.getNrOfJoints()
        self._jac_kdl = kdl.ChainJntToJacSolver(self._robot_chain)
        self.grav_vector = kdl.Vector(0, 0, -9.81)
        self.FKSolver = kdl.ChainFkSolverPos_recursive(self._robot_chain)
        self.dyn_kdl = kdl.ChainDynParam(self._robot_chain, self.grav_vector)
        self._joint_names = self._limb.joint_names()
      
        ### publisher ### (tau_motor: , gravity_compensation_turn_off)

        self.motor_torque_pub = rospy.Publisher('computed_gc_torques_sender', JointState, queue_size=10)
        
        # create publisher to disable default gravity compensation
        gc_ns = 'robot/limb/' + limb + '/suppress_gravity_compensation'
        self._pub_gc_disable = rospy.Publisher(gc_ns, Empty, queue_size=1)
        
        # create publisher to disable cuff
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)


        ### subscriber ###
        self.lock = threading.RLock()
        self.default_gc_torques = np.empty([7,1])
        self.default_gc_model_torques = np.empty([7,1])
        self.default_joint_torques = np.empty([7,1])   

        self.joint_angle = np.empty([7,1])
        self.joint_vel = np.empty([7,1])
        self.joint_torque = np.empty([7,1])
        # robot joint state subscriber
        rospy.Subscriber('robot/joint_states', JointState, self.callback_joint_state)
        self.robot_sea_state_subscriber = rospy.Subscriber('robot/limb/' + limb + '/gravity_compensation_torques', SEAJointState, self.callback_default_gc)
    

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")


        # Instance state varibles (joint_angle, joint_velocity, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2)
        self.pose_desi = self.calc_pose()
        self.pose_1 = self.pose_desi
        self.current_angle = self.joint_angle
        self.jacobian = self.calc_jacobian()
        self.jacobian_1, self.jacobian_2 = self.jacobian, self.jacobian
        
    def move_to_neutral(self):
        #self._limb.move_to_neutral()
        new_limb_pose = {}
        i = 0
        for joint in self._limb.joint_names():
            new_limb_pose[joint] = self._init_joint_angles[i]
            i += 1
        self._limb.move_to_joint_positions(new_limb_pose)
        rospy.sleep(self._waitingtime)
        print (self._limb.joint_names())
        print ("######## Ready for next action. ########")

    def callback_joint_state(self, data: JointState):
        with self.lock:
            if len(data.position) >= 7:
                for i in range(0, 7):
                    self.joint_angle[i] = data.position[i]
                    self.joint_vel[i] = data.velocity[i]
                    self.joint_torque[i] = data.effort[i]

    def callback_default_gc(self, data: SEAJointState):
        with self.lock:
            for i in range(0, 7):
                self.default_gc_torques[i] = data.gravity_only[i]	#	This is the torque required to hold the arm against gravity returned by KDL - SDK
                self.default_gc_model_torques[i] = data.gravity_model_effort[i]	#	This includes the inertial feed forward torques when applicable. - SDK
                self.default_joint_torques[i] = data.actual_effort[i]	# Joint torques feedback 
            
    def publish_torques(self, torques, publisher: rospy.Publisher):
        msg = JointState()
        msg.effort = torques
        publisher.publish(msg)

    def update_parameters(self):
        self.update_Kd()
        self.update_Dd()
        
    def update_Kd(self):
        self.Kd = np.diag(rospy.get_param("Kd", [1,1,1,1,1,1]))

    def update_Dd(self):
        self.Dd = np.diag(rospy.get_param("Dd", [1,1,1,1,1,1]))

    def run_statemachine(self, statecondition, Kd, Dd, joint_angle, joint_velocity, rate, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, force_ee, ddPose_cart):
        if statecondition == 1:
        # State 1: Impedance Controller
            #   Input: Kd, Dd, pose, pose_desi, joint_angles, joint_velocity, rosrate, coriolis, inertia, gravity, jacoabian, jacobian_1, jacobian_2)
            #   Output: tau_motor
            torque_motor = self.impedance_ctrl.run_impedance_controll(Kd, Dd, joint_angle, joint_velocity, rate, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, ddPose_cart) #TODO correct input
        
        elif statecondition == 2:
        # State 2: optimal Controller
            #   TODO implement optimal Controller
            pass
        
        return torque_motor

    def calc_pose(self, joint_values = None, link_number = 7):
        endeffec_frame = kdl.Frame()
        kinematics_status = self.FKSolver.JntToCart(self.joints_to_kdl('positions',joint_values),
                                                   endeffec_frame,
                                                   link_number)
        if kinematics_status >= 0:
            p = endeffec_frame.p

            M = endeffec_frame.M
            return np.array([p.x(), p.y(), p.z(),0,0,0])
            # return np.mat([[M[0,0], M[0,1], M[0,2], p.x()], 
                        #    [M[1,0], M[1,1], M[1,2], p.y()], 
                        #    [M[2,0], M[2,1], M[2,2], p.z()],
                        #    [     0,      0,      0,     1]])
        else:
            return None
    
    def calc_coriolis(self, joint_angles=None, joint_velocities=None):
        coriolis_torques = kdl.JntArray(self._nrOfJoints)
        self.dyn_kdl.JntToCoriolis(self.joints_to_kdl('positions',joint_angles), self.joints_to_kdl('velocities',joint_velocities), coriolis_torques)	# C(q, q_dot)
        return self.array_kdl_to_list(coriolis_torques)

    def calc_inertia(self, joint_values=None):
        inertia = kdl.JntSpaceInertiaMatrix(self._nrOfJoints)
        self.dyn_kdl.JntToMass(self.joints_to_kdl('positions',joint_values), inertia)
        return self.kdl_to_mat(inertia)

    def calc_gravity(self, joint_values=None):
        grav_torques = kdl.JntArray(self._nrOfJoints)
        self.dyn_kdl.JntToGravity(self.joints_to_kdl('positions',joint_values), grav_torques)
        return self.array_kdl_to_list(grav_torques)

    def calc_jacobian(self,joint_values=None):
        jacobian = kdl.Jacobian(self._nrOfJoints)
        self._jac_kdl.JntToJac(self.joints_to_kdl('positions',joint_values), jacobian)
        return self.kdl_to_mat(jacobian)

    def get_Kd(self):
        return self.Kd

    def get_Dd(self):
        return self.Dd

    def get_rate(self):
        # TODO to detect if rosrate not fullfiled and return actual rate after last run
        return self.rate

    def get_pose_desi(self):
        # TODO implement callback for pose
        return self.pose_desi

    def get_force_ee(self):
        # TODO implement get_force_ee
        pass

    def get_jacobian_1(self):
        return self.jacobian_1

    def get_jacobian_2(self):
        return self.jacobian_2

    def get_statecondition(self, force):
        statecondition = 1
        return statecondition

    def reset_gravity_compensation(self):
        # TODO implement gravity_compensation publisher
        return

    def kdl2numpy(self):
        pass

    def numpy2kdl(self):
        pass
    
    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()

    ##
    # Tests to see if the given joint angles are in the joint limits.
    # @param List of joint angles.
    # @return True if joint angles in joint limits.
    def joints_in_limits(self, q):
        lower_lim = self.joint_limits_lower
        upper_lim = self.joint_limits_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    ##
    # Tests to see if the given joint angles are in the joint safety limits.
    # @param List of joint angles.
    # @return True if joint angles in joint safety limits.
    def joints_in_safe_limits(self, q):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    ##
    # Clips joint angles to the safety limits.
    # @param List of joint angles.
    # @return np.array list of clipped joint angles.
    def clip_joints_safe(self, q):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.clip(q, lower_lim, upper_lim)

    def kdl_to_mat(self, m):
        mat =  np.mat(np.zeros((m.rows(), m.columns())))
        for i in range(m.rows()):
            for j in range(m.columns()):
                mat[i,j] = m[i,j]
        return mat

    def joints_to_kdl(self, type, values=None):
        kdl_array = kdl.JntArray(self._nrOfJoints)

        if values is None:
            if type == 'positions':
                cur_type_values = self._limb.joint_angles()
            elif type == 'velocities':
                cur_type_values = self._limb.joint_velocities()
            elif type == 'torques':
                cur_type_values = self._limb.joint_efforts()
        else:
            cur_type_values = values
        
        for idx, name in enumerate(self._joint_names):
            kdl_array[idx] = cur_type_values[name]
        #if type == 'velocities':
        #    kdl_array = kdl.JntArrayVel(kdl_array)
        return kdl_array
    
    def array_kdl_to_list(self, q):
        if q == None:
            return None
        return [q[i] for i in range(q.rows())]

    def joint_list_to_kdl(self, q):
        if q is None:
            return None
        if type(q) == np.matrix and q.shape[1] == 0:
            q = q.T.tolist()[0]
        q_kdl = kdl.JntArray(len(q))
        for i, q_i in enumerate(q):
            q_kdl[i] = q_i
        return q_kdl

    def dictionary2list(self, dic):  
        list_tmp = []
        for key, value in dic.items():
            list_tmp.append(value)
        return list_tmp
    
    def matrix_to_list(self, matrix):
        # input numpy matrix vector (nx1)
        list = []
        for i in range(len(matrix)):
            list.append(matrix[i].item(0))
        return list
 
    def run_controller(self):
        self.move_to_neutral
        self._limb.set_command_timeout((1.0 / self.rate) * self._missed_cmd)
        
        controller_node = True # TODO implement a switch condition to turn it on and off
        
        while not rospy.is_shutdown():
            if controller_node == True:
                self.update_parameters() # updates Kd, Dd
                Kd = self.get_Kd() # numpy 6x6
                Dd = self.get_Dd() # numpy 6x6
                rate = self.get_rate() # int
                jacobian_1 = self.get_jacobian_1() # numpy 6x7
                jacobian_2 = self.get_jacobian_2() # numpy 6x7
                pose_desi = self.get_pose_desi() # numpy 6x1
                ddPose_cart = np.atleast_2d(np.array([0,0,0,0,0,0])).T # TODO add ddPose cartesian # numpy 6x1
                force_ee = self.get_force_ee() # numpy 7x1
                statecondition = self.get_statecondition(force_ee) # int

                cur_joint_angle = np.atleast_2d(self.dictionary2list(self._limb.joint_angles())).T#self.joint_angle # numpy 7x1
                cur_joint_velocity = np.atleast_2d(self.dictionary2list(self._limb.joint_velocities())).T#self.joint_vel # numpy 7x1
                cur_joint_efforts = np.atleast_2d(self.dictionary2list(self._limb.joint_efforts())).T

                pose = np.atleast_2d(self.calc_pose()).T # numpy 6x1
                inertia = np.atleast_2d(self.calc_inertia()) # numpy 7x7 # TODO Error: KDL Object
                gravity = np.atleast_2d(self.calc_gravity()).T # numpy 7x1 #TODO Error 1x1
                jacobian = np.atleast_2d(self.calc_jacobian()) # numpy 6x7 #TODO Error KDL Object
                coriolis = np.atleast_2d(self.calc_coriolis()).T # numpy 7x7 #TODO Error 1x1
                
                # return numpy 7x1 martix of torques 
                torque_motor = self.run_statemachine(statecondition, Kd, Dd, cur_joint_angle, cur_joint_velocity, rate, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, force_ee, ddPose_cart)
                torque_motor_list = self.matrix_to_list(torque_motor)
                self.publish_torques(torque_motor_list, self.motor_torque_pub)

                self.jacobian_1 = jacobian
                self.jacobian_2 = jacobian_1
                self.pose_1 = pose
            
            self.reset_gravity_compensation()
            rospy.Rate(self.rate).sleep()
        self.clean_shutdown()

def main():
    control = controller()
    control.run_controller()

if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass


