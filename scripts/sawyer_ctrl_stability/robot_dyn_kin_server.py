#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from intera_core_msgs.msg import JointLimits, SEAJointState
import PyKDL as kdl
from kdl_parser_py import urdf
import sys, os

from scipy.spatial.transform import Rotation as R
import tf.transformations as tft


import intera_interface
from intera_interface import CHECK_VERSION

class Robot_dynamic_kinematic_server():

    def __init__(self, limb, rate, missed_cmd=2):
        rospy.loginfo("[Robot Kin_Dyn]: Initializing Robot dynamic and kinematic server")

        # create limb instance
        self._limb = intera_interface.Limb(limb)
        self._limits = intera_interface.JointLimits()
        self._head = intera_interface.Head()
    
        ########## Robot initialisation ##########
        # Instance Robotic Chain
        # TODO automatic path
        #urdf_filepath = os.path.abspath(os.path.join(os.path.abspath(__file__), os.pardir, os.pardir, os.pardir, 'sawyer_robot/sawyer_description/urdf/sawyer_base.urdf.xacro'))
        tmp = os.path.dirname(__file__)
        urdf_filepath = os.path.join(tmp,'urdf/sawyer.urdf')
        (ok, robot) = urdf.treeFromFile(urdf_filepath)
        self._robot_chain = robot.getChain('right_arm_base_link', 'right_l6')
        self._nrOfJoints = self._robot_chain.getNrOfJoints()
        self._jac_kdl = kdl.ChainJntToJacSolver(self._robot_chain)
        self.grav_vector = kdl.Vector(0, 0, -9.81)
        self.FKSolver = kdl.ChainFkSolverPos_recursive(self._robot_chain)
        self.dyn_kdl = kdl.ChainDynParam(self._robot_chain, self.grav_vector)
        self._joint_names = self._limb.joint_names()
        

        # verify robot is enabled
        rospy.loginfo("[Robot Kin_Dyn]: Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        rospy.loginfo("[Robot Kin_Dyn]: Enabling robot... ")
        self._rs.enable()
        rospy.loginfo("[Robot Kin_Dyn]: Running. Ctrl-c to quit")


        # Robot limit and safety
        self.joint_angle_limit_upper = np.atleast_2d(np.array(self._limits.get_joint_upper_limits(self._joint_names)))
        self.joint_anlge_limit_lower = np.atleast_2d(np.array(self._limits.get_joint_lower_limits(self._joint_names)))
        self.joint_velocity_limits_upper =   np.atleast_2d(np.array(self._limits.get_joint_velocity_limits(self._joint_names)))
        self.joint_velocity_limits_lower =  -np.atleast_2d(np.array(self._limits.get_joint_velocity_limits(self._joint_names)))
        self.joint_acceleration_limits = np.atleast_2d(np.array(self._limits.get_joint_acceleration_limits(self._joint_names)))
        self.joint_efforts_limit_upper = np.atleast_2d(np.array(self._limits.get_joint_effort_limits(self._joint_names)))
        self.joint_efforts_limit_lower = - np.atleast_2d(np.array(self._limits.get_joint_effort_limits(self._joint_names)))
        

        # Instance state varibles
        self.jacobian = np.zeros((6,7))
        self.jacobian_prev = np.zeros((6,7))
        self.djacobian = np.zeros((6,7))
        self.periodTime = 0.01

        # set control timeout
        # self.set_limb_timeout(rate,missed_cmd)
        

        
    def clean_shutdown(self):
        """
        Clean exit of controller node
        """
        rospy.loginfo("[Robot Kin_Dyn]: Exiting Control node...")
        self._limb.exit_control_mode()

    def calc_pose_cartesianSpace(self, type, joint_values = None, link_number = 7):
        """
        Calculation of position or velocities (depending on the parameter type) with forward kinematis. 
        It can calculate the FK for given joint_values. If no are given, the methods takes the actual values 
        of the Robot. The link number can be choosen to calculate the FK for the given link.
        Parameters: type (str), joint_values (7x1), link_number (int)
        Return: transformation matrix (4x4)
        """
        endeffec_frame = kdl.Frame()
        if type == 'positions':
            kinematics_status = self.FKSolver.JntToCart(self.joints_to_kdl('positions',joint_values),
                                                   endeffec_frame,
                                                   link_number)
        elif type == 'velocities':
            kinematics_status = self.FKSolver.JntToCart(self.joints_to_kdl('velocities',joint_values),
                                                   endeffec_frame,
                                                   link_number)
        else:
            print('Err: [Calc_pose] Wrong type choosen.')

        if kinematics_status >= 0:
            p = endeffec_frame.p
            M = endeffec_frame.M
            transformation_mat = np.array([[M[0,0], M[0,1], M[0,2], p.x()],
                            [M[1,0], M[1,1], M[1,2], p.y()], 
                            [M[2,0], M[2,1], M[2,2], p.z()],
                            [0,0,0,1]])

            return transformation_mat
                            
        else:
            return None
    
    def calc_coriolis(self, joint_angles=None, joint_velocities=None):
        """
        Calculation of coriolis compensation torques based on the joint angles and velocities.
        Parameters: joint angles (7x1), joint velocites (7x1)
        Return: coriolis torques - C*q_dot (7x1)
        """
        coriolis_torques = kdl.JntArray(self._nrOfJoints)
        self.dyn_kdl.JntToCoriolis(self.joints_to_kdl('positions',joint_angles), self.joints_to_kdl('velocities',joint_velocities), coriolis_torques)	# C(q, q_dot)
        return np.atleast_2d(self.array_kdl_to_list(coriolis_torques))

    def calc_mass(self, joint_values=None):
        """
        Calculation of intertia matrix based on the joint_angle.
        Parameters: joint angles (7x1)
        Return: inertia matrix (7x7)
        """
        mass = kdl.JntSpaceInertiaMatrix(self._nrOfJoints)
        self.dyn_kdl.JntToMass(self.joints_to_kdl('positions',joint_values), mass)
        return np.atleast_2d(self.kdl_to_mat(mass))

    def calc_gravity(self, joint_values=None):
        """
        Calculation of gravity compensation torques based on the joint values.
        Parameters: joint angles (7x1)
        Return: gravity torques (7x1)
        """
        grav_torques = kdl.JntArray(self._nrOfJoints)
        self.dyn_kdl.JntToGravity(self.joints_to_kdl('positions',joint_values), grav_torques)
        return np.atleast_2d(self.array_kdl_to_list(grav_torques))

    def calc_jacobian(self,joint_values=None):
        """
        Calculation of jacobian matrix mased on joint angles.
        Parameters: joint angles (7x1)
        Return: jacobian (6x7)
        """
        self.jacobian_prev = self.jacobian
        jacobian = kdl.Jacobian(self._nrOfJoints)
        self._jac_kdl.JntToJac(self.joints_to_kdl('positions',joint_values), jacobian)
        self.jacobian = self.kdl_to_mat(jacobian)
        return np.atleast_2d(self.jacobian)

    def calc_djacobian(self):
        """
        Calculates first derivative of jacobian.
        Parameters: joacobian (6x7), previous jacobian (6x7), period Time (sec)
        Return: djacobian (6x7)
        """
        periodTime = self.get_periodTime()
        if periodTime > 0:
            self.djacobian = np.atleast_2d((self.get_jacobian()-self.get_jacobian_prev())/self.get_periodTime())
        else:
            self.djacobian = np.zeros((6,7))
        return 


   ############ Format converter (KDL/Numpy, List/Dict) ############ 
    
    def kdl_to_mat(self, m):
        """
        Transform KDL matrix into numpy matrix format.
        Parameters: KDL matrix (nxm)
        Return: numpy array (nxm)
        """
        mat =  np.mat(np.zeros((m.rows(), m.columns())))
        for i in range(m.rows()):
            for j in range(m.columns()):
                mat[i,j] = m[i,j]
        return mat

    def joints_to_kdl(self, type, values=None):
        """
        Transform array into KDL array. If no value given type of joint value is taken into account.
        Parameters: array (nx1)
        Return: KDL array (nx1)
        TODO Errorhandling
        """

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
        return kdl_array
    
    def array_kdl_to_list(self, q):
        """
        Transform KDL array into list.
        Parameters: array (nx1)
        Return: list (nx1)
        """
        if q == None:
            return None
        return [q[i] for i in range(q.rows())]

    def dictionary2list(self, dic):  
        """
        Transform dictionary into list.
        Parameters: dictionary
        Return: list
        """

        list_tmp = list(dic.values())
        return list_tmp
    
    def list2dictionary(self, list):
        """
        Transform list into dictionary.
        Parameters: list
        Return: dictionary
        """
        new_limb_torque = {}
        i = 0
        for joint in self._limb.joint_names():
            new_limb_torque[joint] = list[i]
            i += 1
        return new_limb_torque

    def move2position(self, position):
        """
        Send command to robot SDK to move to joint pose
        Parameters: desired position (list: 7x1)
        Return: None
        """
        self._limb.move_to_joint_positions(position, timeout = 4)
        return

    def move2neutral(self, timeout, speed):
        """
        Send command to robot SDK to move to neutral pose
        Parameters: time to wait untill movement is done (float), speed of movement - % [0,1] (float)
        Return: None
        """
        self._limb.move_to_neutral(timeout, speed)
        return

    
    ############ Getter methods ############ 

    def get_jacobian_prev(self):
        """
        Gets previous jacobian matrix.
        Parameters: None
        Return: jacobian matrix (6x7)
        """
        return self.jacobian_prev   #copy.deepcopy(self.jacobian_prev)

    def get_djacobian(self):
        """
        Gets derivate of jacobian matrix.
        Parameters: None
        Return: derivate jacobian matrix (6x7)
        """
        return self.djacobian       
    
    def get_jacobian(self):
        """
        Gets jacobian matrix.
        Parameters: None
        Return: jacobian matrix (6x7)
        """
        return self.jacobian        

    def get_coriolis(self):
        """
        Gets coriolis compensation vector.
        Parameters: None
        Return: coriolis vector (7x1)
        """
        return self.calc_coriolis().T

    def get_mass(self):
        """
        Gets mass matrix.
        Parameters: None
        Return: mass matrix (7x7)
        """
        return self.calc_mass()

    def get_gravity_compensation(self):
        """
        Gets gravity compensation vector.
        Parameters: None
        Return: gravity compensation vector (7x1)
        """
        return self.calc_gravity().T

    def get_current_positions_cartesianSpace(self, type='positions'):
        return self.calc_pose_cartesianSpace(type)
    
    def get_current_velocities_cartesianSpace(self, type='velocities'):
        return self.calc_pose_cartesianSpace(type)

    def get_current_joint_angles(self):
        return np.atleast_2d(self.dictionary2list(self._limb.joint_angles())).T   
    
    def get_current_joint_angles_list(self):
        return self.dictionary2list(self._limb.joint_angles())
    
    def get_current_joint_velocities(self):
        return np.atleast_2d(self.dictionary2list(self._limb.joint_velocities())).T  
    
    def get_current_cartesian_tf(self):
        pose = self.calc_pose_cartesianSpace('positions')
        return pose
    
    def get_current_cartesian_velocity(self):
        velocity = self.calc_pose_cartesianSpace('velocity')
        return velocity

    def get_periodTime(self):
        return self.periodTime 
    
    def get_current_DisplayAngle(self):
        return self._head.pan()
    
    def get_joint_limits(self):
        upper_limit = self.joint_angle_limit_upper 
        lower_limit = self.joint_anlge_limit_lower 
        return upper_limit, lower_limit

    def get_torque_limits(self):
        upper_limit = self.joint_efforts_limit_upper 
        lower_limit = self.joint_efforts_limit_lower 
        return upper_limit, lower_limit
    
    def get_velocity_limits(self):
        upper_limit = self.joint_velocity_limits_upper 
        lower_limit = self.joint_velocity_limits_lower 
        return upper_limit, lower_limit
    
    def get_joint_names(self):
        return self._joint_names 
    
    def get_cartesian_acceleration_EE(self):
        # TODO improve
        return np.zeros((6,1))
    
    def get_current_cartesian_pose(self):
        """
        TODO redundance
        """
        tf_mat = self.calc_pose_cartesianSpace('positions')
        position_cart = np.atleast_2d([tf_mat[0,3], tf_mat[1,3], tf_mat[2,3]]).T
        quat_cart = np.atleast_2d(tft.quaternion_from_matrix(tf_mat)).T
        pose = np.concatenate((position_cart, quat_cart), axis = 0)
        return pose
    
    def get_current_cartesian_velocity(self):
        """
        TODO redundance
        """
        tf_mat = self.calc_pose_cartesianSpace('velocities')
        position_cart = np.atleast_2d([tf_mat[0,3], tf_mat[1,3], tf_mat[2,3]]).T
        quat_cart = np.atleast_2d(tft.quaternion_from_matrix(tf_mat)).T
        pose = np.concatenate((position_cart, quat_cart), axis = 0)
        return pose
    
    ############ Update methods ############

    def update_periodTime(self, periodTime):
        """
        Update time of since last update
        Parameters: time since last period (sec - float)
        Return: None
        """
        self.periodTime = periodTime

    def update_robot_states(self, periodTime):
        """
        Updates jacobian and period time.
        """
        self.update_periodTime(periodTime)
        self.calc_jacobian()
        self.calc_djacobian()

    ############ Setter methods ############

    def set_joint_torques(self, torques):
        """
        Get the current pan angle of the head.

        @rtype: float
        @return: current angle in radians
        """
        self._limb.set_joint_torques(torques)
    
    def set_DispalyAngle(self, jointangle):
        """
        Pan at the given speed to the desired angle.

        @type angle: float
        @param angle: Desired pan angle in radians.
        @type speed: int
        @param speed: Desired speed to pan at, range is 0-1.0 [1.0]
        @type timeout: float
        @param timeout: Seconds to wait for the head to pan to the
                        specified angle. If 0, just command once and
                        return. [10]
        @param active_cancellation: Specifies if the head should aim at
                        a location in the base frame. If this is set to True,
                        the "angle" param is measured with respect to
                        the "/base" frame, rather than the actual head joint
                        value. Valid range is [-pi, pi) radians.
        @type active_cancellation: bool
        """
        self._head.set_pan(jointangle, speed = 1.0, timeout = 10.0, active_cancellation = False)

    def set_limb_timeout(self, rate, missed_cmd):

        # Set limb controller timeout to return to Sawyer position controller
        self._limb.set_command_timeout((1.0 / rate) * missed_cmd)

  
  
def main():
    robot_dyn_kin_server = Robot_dynamic_kinematic_server()
    robot_dyn_kin_server.update_robot_states()
    print(robot_dyn_kin_server.get_djacobian())
    print(robot_dyn_kin_server.get_jacobian())
    print(robot_dyn_kin_server.get_jacobian_prev())
    print(robot_dyn_kin_server.get_coriolis())
    print(robot_dyn_kin_server.get_mass())
    print(robot_dyn_kin_server.get_gravity_compensation())
    print(robot_dyn_kin_server.get_current_joint_angles())
    print(robot_dyn_kin_server.get_current_joint_velocities())
    print(robot_dyn_kin_server.get_current_positions_cartesianSpace())
    print(robot_dyn_kin_server.get_current_velocities_cartesianSpace())
    
if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass


    