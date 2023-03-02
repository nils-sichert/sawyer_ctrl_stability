#!/usr/bin/env python3
        
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from intera_core_msgs.msg import JointLimits, SEAJointState
import threading
import PyKDL as kdl
import copy
import sys
from kdl_parser_py import urdf
import sys, os


from os.path import dirname, join, abspath
from Spring_damper_jointspace import spring_damper_jointspace
from DLR_impedance_cartesian import dlr_impedance_cartesian
from PD_impedance_jointspace import pd_impedance_jointspace
from PD_impedance_cartesian import pd_impedance_cartesian
from scipy.spatial.transform import Rotation as R

import intera_interface
from intera_interface import CHECK_VERSION

"""
Please add Bugs and Todos here:
TODO easier handle of controller positions
TODO each controller hold position while changing controller (same setpoints)

"""

class controller():
    def __init__(self, limb = "right"):
        print("Initializing node...")
        rospy.init_node('Passiv_Activ_Controller', anonymous=True)
        rospy.set_param("control_node/control_flag", False)
        rospy.set_param("control_node/joint_angle_desi", [-0.1558798828125, 0.1269013671875, -1.63815625, 1.5093447265625, -1.41862890625, 1.5380302734375, -1.40465625])
        rospy.set_param("control_node/joint_velocity_desi", [0, 0, 0, 0, 0, 0, 0])
        rospy.set_param("control_node/controllerstate", 3)
        self.Kd = rospy.set_param("control_node/Kd", [20])
        self.Dd = rospy.set_param("control_node/Dd", [1])
        self.lowpass_coeff = rospy.get_param("control_node/Lowpass_weight", 0.7)
        self.lock = threading.RLock()

        # set neutral pose of sawyer
        rospy.set_param("named_poses/right/poses/neutral", [-0.1558798828125, 0.1269013671875, -1.63815625, 1.5093447265625, -1.41862890625, 1.5380302734375, -1.40465625])

        # control parameters
        self.rate = 100 # Controlrate - 100Hz
        self._missed_cmd = 20 # Missed cycles before triggering timeout
        
        # Instance Controller
        self.DLR_Impedance_Cartesian = dlr_impedance_cartesian()
        self.PD_impedance_cartesian = pd_impedance_cartesian()

        self.impedance_ctrl_simple = spring_damper_jointspace()
        self.PD_impedance_jointspace  = pd_impedance_jointspace()
        

        # create limb instance
        self._limb = intera_interface.Limb(limb)

        ### Publisher ###
        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # create publisher to disable default gravity compensation
        gc_ns = 'robot/limb/' + limb + '/suppress_gravity_compensation'
        self._pub_gc_disable = rospy.Publisher(gc_ns, Empty, queue_size=1)
        
        ### Debug Publisher ###
        self._pub_error = rospy.Publisher('/control_node_debug/EE_Pose_Error', JointState, queue_size=1)
        self._pub_joint_velocity = rospy.Publisher('/control_node_debug/joint_velocity', JointState, queue_size=1)

        ########## Robot initialisation ##########
        # Instance Robotic Chain
        # TODO add relativ path
        urdf_filepath = os.path.join(os.getcwd(), 'src/sawyer_robot/sawyer_description/urdf/sawyer_base.urdf.xacro')
        (ok, robot) = urdf.treeFromFile(urdf_filepath)
        self._robot_chain = robot.getChain('right_arm_base_link', 'right_l6')
        self._nrOfJoints = self._robot_chain.getNrOfJoints()
        self._jac_kdl = kdl.ChainJntToJacSolver(self._robot_chain)
        self.grav_vector = kdl.Vector(0, 0, -9.81)
        self.FKSolver = kdl.ChainFkSolverPos_recursive(self._robot_chain)
        self.dyn_kdl = kdl.ChainDynParam(self._robot_chain, self.grav_vector)
        self._joint_names = self._limb.joint_names()
      
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

        # Robot limit and safety
        limit = 100
        self.joint_efforts_safety_lower = [-limit,-limit,-limit,-limit,-limit,-limit,-limit]
        self.joint_efforts_safety_upper = [limit,limit,limit,limit,limit,limit,limit]

        # Instance state varibles
        self.jacobian = self.calc_jacobian()
        self.jacobian_prev = self.jacobian
        self.torque_motor_t_1 = [0,0,0,0,0,0,0]
        self.timestamp_t_1 = 0


    ############ Publisher ############ 

    def publish_error(self, error_pos, error_vel, publisher: rospy.Publisher):
        """
        Publishs position and velocity error of endeffector.
        Parameters: position error (6x1, 'error_pos), velocity error (6x1, 'error_vel)
        Returns: publisher.publish()
        """
        msg = JointState()
        msg.position = error_pos
        msg.velocity = error_vel
        publisher.publish(msg)


    ############ Update States/ Variables ############ 

    def update_parameters(self):
        """
        Updates stiffness matrix, time of period and gets the jacobian at time t-1, desired pose and velocity.
        Parameters: None
        Return: Kd (7x7), samlpingTime (time of last period) (float), jacobian_prev (6x7), desired pose (6x1), desired velocity (6x1)
        """
        Kd = self.update_Kd()
        Dd = self.update_Dd()
        samlpingTime = self.get_samlpingTime() 
        jacobian_prev = self.get_jacobian_prev() 
        pose_desi = self.get_pose_desi() 
        velocities_desi = self.get_velocities_desi()

        return Kd, Dd, samlpingTime, jacobian_prev, pose_desi, velocities_desi
        
    def update_Kd(self):
        """
        Updates the stiffness matrix Kd (7x7).
        Paramteres: None
        Return: Kd (7x7)
        """
        old_Kd = self.Kd
        tmp = rospy.get_param("control_node/Kd")
        if len(tmp)==7:
            self.Kd = np.diag(tmp)
        else:
            tmp = tmp[0]
            self.Kd = np.diag([tmp,tmp,tmp,tmp,tmp,tmp,tmp])
        if not np.array_equal(old_Kd, self.Kd):
            print('New Stiffness was choosen: ', self.Kd)
        return self.Kd
    
    def update_Dd(self):
        """
        Updates the stiffness matrix Kd (7x7).
        Paramteres: None
        Return: Kd (7x7)
        """
        old_Dd = self.Dd
        tmp = rospy.get_param("control_node/Dd")
        if len(tmp) == 7:
            self.Dd = np.diag(tmp)
        else:
            tmp = tmp[0]
            self.Dd = np.diag([tmp,tmp,tmp,tmp,tmp,tmp,tmp])
        if not np.array_equal(old_Dd, self.Dd):
            print('New Damping was choosen: ', self.Dd)
        return self.Dd
   
    def calc_pose(self, type, joint_values = None, link_number = 7):
        """
        Calculation of position or velocities (depending on the parameter type) with forward kinematis. 
        It can calculate the FK for given joint_values. If no are given, the methods takes the actual values 
        of the Robot. The link number can be choosen to calculate the FK for the given link.
        Parameters: type (str), joint_values (7x1), link_number (int)
        Return: pos_vec (3x1), rot_mat (3x3), pose (6x1)
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
            pos_vec = [p.x(),p.y(),p.z()]
            M = endeffec_frame.M
            rot_mat = np.mat([[M[0,0], M[0,1], M[0,2]],
                            [M[1,0], M[1,1], M[1,2]], 
                            [M[2,0], M[2,1], M[2,2]]])
            r = R.from_matrix(rot_mat)
            rot_vec = r.as_rotvec()
            pose = [0]*6
            for i in range(3):
                pose[i]= pos_vec[i]
                pose[i+3] = rot_vec[i]

            return pos_vec, rot_mat, pose
                            
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
        return self.array_kdl_to_list(coriolis_torques)

    def calc_inertia(self, joint_values=None):
        """
        Calculation of intertia matrix based on the joint_angle.
        Parameters: joint angles (7x1)
        Return: inertia matrix (7x7)
        """
        inertia = kdl.JntSpaceInertiaMatrix(self._nrOfJoints)
        self.dyn_kdl.JntToMass(self.joints_to_kdl('positions',joint_values), inertia)
        return self.kdl_to_mat(inertia)

    def calc_gravity(self, joint_values=None):
        """
        Calculation of gravity compensation torques based on the joint values.
        Parameters: joint angles (7x1)
        Return: gravity torques (7x1)
        """
        grav_torques = kdl.JntArray(self._nrOfJoints)
        self.dyn_kdl.JntToGravity(self.joints_to_kdl('positions',joint_values), grav_torques)
        return self.array_kdl_to_list(grav_torques)

    def calc_jacobian(self,joint_values=None):
        """
        Calculation of jacobian matrix mased on joint angles.
        Parameters: joint angles (7x1)
        Return: jacobian (6x7)
        """
        jacobian = kdl.Jacobian(self._nrOfJoints)
        self._jac_kdl.JntToJac(self.joints_to_kdl('positions',joint_values), jacobian)
        return self.kdl_to_mat(jacobian)

    def calc_error_cart(self, pos_vec, rot_mat, curr_velocity, pose_desi):
        """
        Calculation of position-, orientation-, velociteserrors and acceleration
        Parameters: position vector (3x1), rotation matrix (3x3), current velocity (6x1), desired pose (6x1)
        Return: error pose (6x1), error velocities (6x1), acceleration (6x1)
        TODO debug Orientation error calculation
        """
        # Position Error
        error_pos = np.array([pos_vec[0]-pose_desi[0], pos_vec[1]-pose_desi[1], pos_vec[2]-pose_desi[2]])

        # Orientation Error
        r = R.from_matrix([rot_mat])
        quat_c = np.atleast_2d(np.array(r.as_quat()))
        pose_to_quat = R.from_euler('xyz', [pose_desi[3], pose_desi[4], pose_desi[5]], degrees=True)
        quat_d = np.atleast_2d(np.array(pose_to_quat.as_quat()))
        
        #error_orientation = -(quat_c[0][0] * quat_d[0][1:] - quat_d[0][0] * quat_c[0][1:] - np.cross(quat_d[0][1:], quat_c[0][1:]))
        error_orientation = np.zeros((3))
        error_pose = np.atleast_2d(np.concatenate((error_pos, error_orientation))).T
        
        # Velocity error
        velocity_d = np.zeros((6,1))
        curr_velocity_tmp = np.zeros((6,1))
        for i in range(6):
            curr_velocity_tmp[i] = curr_velocity[i][0,0]
        error_velocity = curr_velocity_tmp - curr_velocity_tmp # TODO change to not zero and actual error
        
        # Acceleration Vector
        ddx = np.zeros((6,1))

        return error_pose, error_velocity, ddx

    def calc_error_joint(self, current, desired):
        """
        Calculatation of joint angle and joint velocities errors.
        Parameters: current state (7x1), desired state (7x1)
        Return: error (7x1)
        """
        error = current - desired
        return error
    
    def lowpass_filter(self, y_t, y_t_1):
        """
        Lowpassfilter to weight between acutal and previous step. Lowpass coefficient can be set as ROSPARAM.
        Parameters: actual value (nxm), previous value (nxm)
        TODO clear method, make general useable
        """
        if len(y_t) is not len(y_t_1):
            print("Err Controller_node.py: Length of input lowpass filter is not equal.")
        
        else:
            y = [0]*len(y_t)
            for i in range(len(y_t)):
                y[i] = (1-self.lowpass_coeff)*y_t_1[i]+self.lowpass_coeff*y_t[i]
        return y

    def clean_shutdown(self):
        """
        Clean exit of controller node
        """
        print("\nExiting Control node...")
        self._limb.exit_control_mode()

    ############ Robot safety methods ############ 

    def joints_in_limits(self, q):
        """
        Control if joint angle are within limits.
        Parameters: joint angle (7x1)
        Return: limited joint anlges (7x1) or 0 if not in limit
        """
        lower_lim = self.joint_limits_lower
        upper_lim = self.joint_limits_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    def joints_in_safe_limits(self, q):
        """
        Control if joint angle are within safety area.
        Parameters: joint angle (7x1)
        Return: safety joint anlges (7x1) or 0 if not in limit
        """
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    def clip_joints_safe(self, q):
        """
        Clip joint angles to safe angles.
        Parameters: joint anlges (7x1)
        Retrun: clipt joint angles (7x1)
        """
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.clip(q, lower_lim, upper_lim)

    def clip_joints_effort_safe(self, q):
        """
        Clip joint angles to safe angles.
        Parameters: joint efforts (7x1)
        Retrun: clipt joint efforts (7x1)
        """
        lower_lim = self.joint_efforts_safety_lower
        upper_lim = self.joint_efforts_safety_upper
        return np.clip(q, lower_lim, upper_lim)
    

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
        list_tmp = []
        for key, value in dic.items():
            list_tmp.append(value)
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

    
    ############ Getter methods ############ 

    def get_samlpingTime(self):
        """
        Compute exact time of last period.
        Parameters: None
        Return: period (sec)
        """
        time_now = rospy.get_time()
        if self.timestamp_t_1 != 0:
            period = time_now -self.timestamp_t_1
        else:
            period = 1/self.rate # period time in sec
        self.timestamp_t_1 = time_now    
        return period
    
    def get_cur_pose(self, pos_vec, rot_mat):
        """
        Get current pose from position and rotation matrix.
        Parameters: position (3x1), roation matrix (3x3)
        Return: pose (6x1)
        """
        r = R.from_matrix(rot_mat)
        rot_vec = r.as_rotvec()
        pose = [0]*6
        for i in range(3):
            pose[i]=pos_vec[i]
            pose[i+3] = rot_vec[i]
        return pose
 
    def get_pose_desi(self):
        """
        Gets the desired pose from subscribed topic.
        Parameters: None
        Return: desired pose (6x1)

        TODO implement callback/ subscriber for pose
        """
        return self.pose_desi
    
    def get_velocities_desi(self):
        """
        Gets desired velocites from subscribed topic.
        Parameters: None
        Return: desired velocity (6x1)
        """
        vel_desi = np.array([0,0,0,0,0,0]) # 6x1 TODO implement subscriber
        return vel_desi

    def get_jacobian_prev(self):
        """
        Gets previous jacobian matrix.
        Parameters: None
        Return: jacobian matrix (6x7)
        """
        return copy.deepcopy(self.jacobian_prev)

    def get_djacobian(self, jacobian, jacobian_prev, periodTime):
        """
        Calculates first derivative of jacobian.
        Parameters: joacobian (6x7), previous jacobian (6x7), period Time (sec)
        Return: djacobian (6x7)
        """
        djacobian = (jacobian-jacobian_prev)/periodTime
        return djacobian
    
    def get_statecondition(self):
        """
        Gets statecondition from ROSPARAM server. This method can be used to implemend a decision
        algorithm.
        Parameters: None
        Return: statecondition (int)
        """
        statecondition = rospy.get_param("control_node/controllerstate")
        return statecondition
    
    def set_initalstate(self, current_joint_angle):
        rospy.set_param("control_node/joint_angle_desi", current_joint_angle)
        rospy.set_param("named_poses/right/poses/neutral", current_joint_angle)

    ############ Control loop ############ 

    def run_statemachine(self, statecondition, cur_joint_angle, cur_joint_velocity, err_pose_cart, err_vel_cart, joint_angle_error, joint_velocity_error, ddx, Kd, Dd, sampling_time, coriolis, inertia, gravity, jacobian, djacobian):
        """
        Statemachine is shifting between different control algorithms. The statecondition is a Ros parameter and can therefore be changed 
            while running the loop. Also, the method get_statecondition can be used to implement a switch condition or a switch observer.
        Parameters: statecondition (int),  current joint angle (7x1), current joint velocity (7x1),  current pose error (6x1), 
            current velocity error (6x1), joint angle error (7x1), joint velocity error (7x1), acceleration ddx (6x1), 
            stiffness matrix Kd (7x7), sampling time (float), coriolis vecotr (7x1), inertia matrix (7x7), gravity vector (7x1),
            jacobian matrix (6x7), derivative of jacobian - djacobian (6x7)
        Return: List of control torques for the motor (7x1)
        TODO clear names of controller
        """
        
        if statecondition == 1: # State 1: DLR impedance controller - cartesian space (Source DLR: Alin Albu-Schäffer )
            Kd = Kd[:6,:6]
            motor_torque = self.DLR_Impedance_Cartesian.calc_joint_torque(Kd, ddx, cur_joint_angle, cur_joint_velocity, err_pose_cart, err_vel_cart, inertia, coriolis, jacobian, djacobian, gravity, sampling_time)
            return motor_torque
        
        elif statecondition == 2: # State 2: PD impedance controller - cartesian space 
            Kd = Kd[:6,:6]
            motor_torque = self.PD_impedance_cartesian.calc_joint_torque()
            return motor_torque

        elif statecondition == 3: # State 3: Spring/Damper impedance controller - jointspace
            motor_torque = self.impedance_ctrl_simple.calc_joint_torque(joint_angle_error, joint_velocity_error, Kd, Dd, gravity)
            return motor_torque
        
        elif statecondition == 4: # State 4: PD impedance Controller - jointspace
            motor_torque = self.PD_impedance_jointspace.calc_joint_torque(gravity, Kd, Dd, coriolis, joint_angle_error, joint_velocity_error)
            return motor_torque
                                                                            
        else:
            print('State does not exists. Exiting...')
            pass
     
    def run_controller(self):
        """
        Controlloop to calculate the joint torques based on a controller which is aktivated within a statemachine.
        Parameters: None
        Return: motor torques (dict: 7x1)
        """
        # timeout = 4 # @type timeout: float / @param timeout: seconds to wait for move to finish [15]
        # speed = 0.2 # @type speed: float / @param speed: ratio of maximum joint speed for execution default= 0.3; range= [0.0-1.0]
        # self._limb.move_to_neutral(timeout, speed)
        pos_vec, rot_mat, pose = self.calc_pose(type='positions')
        
        # Set limb controller timeout to return to Sawyer position controller
        self._limb.set_command_timeout((1.0 / self.rate) * self._missed_cmd)
        
        while not rospy.is_shutdown():
            controller_flag = rospy.get_param("control_node/control_flag")
            joint_angle_desi = np.atleast_2d(np.array(rospy.get_param("control_node/joint_angle_desi"))).T
            joint_velocity_desi = np.atleast_2d(np.array(rospy.get_param("control_node/joint_velocity_desi"))).T

            if controller_flag == False:
                current_joint_angle = self.dictionary2list(self._limb.joint_angles())
                self.set_initalstate(current_joint_angle)

            if controller_flag == True:
                # Init hold actual position
                self.pose_desi = self.get_cur_pose(pos_vec, rot_mat)
                self.pose_1 = self.pose_desi
                Kd, Dd, periodTime, jacobian_prev, pose_desi, velocities_desi = self.update_parameters() # updates Kd, Dd
                statecondition = self.get_statecondition() # int

                inertia = np.atleast_2d(self.calc_inertia())    # numpy 7x7 
                gravity = np.atleast_2d(self.calc_gravity()).T  # numpy 7x1 
                jacobian = np.atleast_2d(self.calc_jacobian())  # numpy 6x7 
                coriolis = np.atleast_2d(self.calc_coriolis()).T # numpy 7x1

                ### get current Joint-angle, -velocity and -effort
                cur_joint_angle = np.atleast_2d(self.dictionary2list(self._limb.joint_angles())).T          # numpy 7x1
                cur_joint_velocity = np.atleast_2d(self.dictionary2list(self._limb.joint_velocities())).T   # numpy 7x1

                joint_angle_error = self.calc_error_joint(cur_joint_angle, joint_angle_desi)
                joint_velocity_error = self.calc_error_joint(cur_joint_velocity, joint_velocity_desi)

                ### Update position/ velocity errors, inertia, gravity, jacobian, coriolis
                pos_vec, rot_mat, pose = self.calc_pose(type='positions') 
                err_pose_cart, err_vel_cart, ddx = self.calc_error_cart(pos_vec, rot_mat,jacobian@cur_joint_angle,pose_desi)      # numpy 6x1
                djacobian = self.get_djacobian(jacobian, jacobian_prev, periodTime)
                ### Calculate motor torque for each joint motor
                torque_motor = self.run_statemachine(statecondition, cur_joint_angle, cur_joint_velocity, err_pose_cart, err_vel_cart, joint_angle_error, joint_velocity_error, ddx, Kd, Dd, periodTime, coriolis, inertia, gravity, jacobian, djacobian)
                
                ### Disable cuff and gravitation compensation and publish debug values
                self._pub_cuff_disable.publish()
                self._pub_gc_disable.publish()
                self.publish_error(err_pose_cart, err_vel_cart, self._pub_error)
                self.publish_error(joint_velocity_error, cur_joint_velocity,self._pub_joint_velocity)

                ### Transfer controller output in msg format dictionary
                
                torque_motor_dict = self.list2dictionary(self.clip_joints_effort_safe((torque_motor)))
                
                ### Publish Joint torques
                self._limb.set_joint_torques(torque_motor_dict)

                ### Set values for t-1 calcualtions
                self.jacobian_prev = jacobian
            
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


####### Gazebo apply force of 100N for Time of 5 sec on endeffector ########
'''
rosservice call /gazebo/apply_body_wrench "body_name: 'sawyer::right_l6' 1T 
reference_frame: 'sawyer::base'
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
  force: {x: 100.0, y: 0.0, z: 0.0}
  torque: {x: 0.0, y: 0.0, z: 0.0}
start_time: {secs: 0, nsecs: 0}
duration: {secs: 5, nsecs: 0}" 
'''
