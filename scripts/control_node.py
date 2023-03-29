#!/usr/bin/env python3

        
import rospy
from pyquaternion import Quaternion
import numpy as np
from sensor_msgs.msg import JointState, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Empty, Header, String
from intera_core_msgs.msg import JointLimits, SEAJointState
import tf.transformations as tft


from sawyer_ctrl_stability.cartesianspace_controller import pd_impedance_cartesian, dlr_impedance_cartesian
from sawyer_ctrl_stability.jointspace_controller import pd_impedance_jointspace, spring_damper_jointspace
from sawyer_ctrl_stability.configuration_server import Configuration_server
from sawyer_ctrl_stability.robot_dyn_kin_server import Robot_dynamic_kinematic_server
from sawyer_ctrl_stability.safety_regulator import Safety_regulator # by using sawyer_ctrl_stability folder in package avoid bug of ros noetic
from scipy.spatial.transform import Rotation as R

from matplotlib import pyplot as plt
from matplotlib import animation

import cProfile


"""
Please add Bugs and Todos here:
Necessary ToDos:
TODO add switch between cartesian pose and joint space pose
TODO improve cartesian pose getter and setter

Nice to have ToDos:
TODO Detect URDF path automatically
TODO Safety regulator: implement behavior when getting close to joint angle limits releasing torque.
"""

class controller():
    def __init__(self, limb = "right"):
        """
        Control class, handling the controlloop and a statemachine which is switching the control laws. 
        Parameter: limb configuration (Str)
        Output: joint torque setpoints for internal PID controller (dict: 7x1)
        """
        
        # Initalizing Rosnode
        rospy.init_node('Control_manager', anonymous=True)
        rospy.loginfo("[Control node]: Initializing node...")

        # Control rate
        self.rate = 100 

        ##### Instances ######
        ## Settings
        self.settings = Configuration_server()

        ## Robot
        self.robot_dyn_kin = Robot_dynamic_kinematic_server(limb, self.rate)
        
        ## Controller
        # cartesian space
        self.DLR_Impedance_Cartesian = dlr_impedance_cartesian()
        self.PD_impedance_cartesian = pd_impedance_cartesian()

        # joint space
        self.impedance_ctrl_simple = spring_damper_jointspace()
        self.PD_impedance_jointspace  = pd_impedance_jointspace()

        
        # Safety and Watchguard
        joint_angle_limit_upper, joint_angle_limit_lower = self.robot_dyn_kin.get_joint_limits()
        joint_effort_limit_upper, joint_effort_limit_lower = self.robot_dyn_kin.get_torque_limits()
        self.safety_regulator = Safety_regulator(joint_angle_limit_upper, joint_angle_limit_lower, joint_effort_limit_upper, joint_effort_limit_lower, self.settings.get_oscillation_window_len(), self.settings.get_oscillation_corner_freq(),self.settings.get_oscillation_power_limit() )

        ###### Publisher ######
        # create publisher to suppress cuff 
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # create publisher to suppress default gravity compensation
        gc_ns = 'robot/limb/' + limb + '/suppress_gravity_compensation'
        self._pub_gc_disable = rospy.Publisher(gc_ns, Empty, queue_size=1)

        # create publisher to suppress self collision detection
        sc_ns = 'robot/limb/' + limb + '/suppress_contact_safety'
        self._pub_self_collision_disable = rospy.Publisher(sc_ns, Empty, queue_size=1)

        # create publisher to suppress contac collision detection
        cc_ns = 'robot/limb/' + limb + '/suppress_collision_avoidance'
        self._pub_contact_collision_disable = rospy.Publisher(cc_ns, Empty, queue_size=1)

        
        ###### Debug ######
        self.pr = cProfile.Profile()

        ### Publisher ###
        # Publisher error of Endeffector between desired and current pose
        self._pub_error = rospy.Publisher('/control_node_debug/EE_Pose_Error', JointState, queue_size=1)
        
        # Publisher error of joint velocitys and joint angles
        self._pub_joint_velocity = rospy.Publisher('/control_node_debug/joint_velocity', JointState, queue_size=1)
        
        # Publisher of motor torque setpoints (Outpu of controller)
        self._pub_joint_torques = rospy.Publisher('control_node_debug/setpoints_motor_torque', JointState, queue_size=1)
        
        # Publisher of colors for headlight (setpoint torque: green < 75%, yellow >= 75% <95%, red >=95% of torque limit)        
        self._pub_lightcolor = rospy.Publisher('control_node_debug/color', String, queue_size=1)

        # Publisher: FFT of joint velocity, publish frequency and magnitude
        self._pub_oscillation = [None]*7 
        for index, value in enumerate(joint_effort_limit_upper.T):
            topic = '/control_node_debug/oscillation_joint/' + str(index)
            self._pub_oscillation[index] = rospy.Publisher(topic, JointState, queue_size=1)

        ### Subscriber ###
        # Stiffness and damping matrix
        self._sub_Kd_Dd = rospy.Subscriber('/control_node/Kd_Dd', JointState, self.callback_Kd_Dd)

        # desired joint state
        self._sub_joint_states_desired = rospy.Subscriber('/control_node/joint_states_desi', JointState, self.callback_jointstate)

        
        # Instance state varibles
        self.torque_motor_t_1 = [0,0,0,0,0,0,0]
        self.pose_cart = None
        self.timestamp_t_1 = 0
        self.K_d = [1]
        self.K_d_prev = [1]
        self.D_d = [1]
        self.D_d_prev = [1]
        self._joint_angle_desired = [0,0,0,0,0,0,0]
        self._joint_velocity_desired = [0,0,0,0,0,0,0]

        self.set_initalstate(self.robot_dyn_kin.get_current_joint_angles_list())
        self.set_cartesian_pose(self.robot_dyn_kin.get_current_cartesian_pose())

    ############ Publisher & Callbacks (Debug purpose) ############ 
    def publish_jointstate(self, position, velocity, publisher: rospy.Publisher):
        msg = JointState()
        msg.position = position
        msg.velocity = velocity
        publisher.publish(msg)

    def publish_effortstate(self, effort, publisher: rospy.Publisher):
        msg = JointState()
        msg.effort = effort
        publisher.publish(msg)

    def publish_head_light_color(self, string, publisher: rospy.Publisher):
        msg = String()
        msg.data = string
        publisher.publish(msg)

    def callback_Kd_Dd(self, data):
        self.K_d = data.position
        self.D_d = data.velocity
    
    def callback_jointstate(self, data):
        self._joint_angle_desired = data.position
        self._joint_velocity_desired = data.velocity

    ############ Update States/ Variables ############ 
    def update_parameters(self):
        """
        Updates stiffness matrix, time of period and gets the jacobian at time t-1, desired pose and velocity.
        Parameters: None
        Return: K_d (7x7), samlpingTime (time of last period) (float), jacobian_prev (6x7), desired pose (6x1), desired velocity (6x1)
        """
        K_d = self.update_K_d()
        D_d = self.update_D_d()
        joint_angle_desi = np.atleast_2d(self.safety_regulator.watchdog_joint_limits_jointangle_control(self._joint_angle_desired)).T 
        joint_velocity_desi = np.atleast_2d(np.array(self._joint_velocity_desired)).T 

        return K_d, D_d, joint_angle_desi, joint_velocity_desi
        
    def update_K_d(self):
        """
        Updates the stiffness matrix K_d (7x7).
        Paramteres: None
        Return: K_d (7x7)
        """
        tmp = list(self.K_d)
        if type(tmp) is list:
            if len(tmp)==7:
                tmp_list = [0,1,2,3,4,5,6]
                for i in range(len(tmp)):
                    tmp_list[i] = tmp[i]
                mat = np.diag(tmp)
            elif len(tmp)==1:
                tmp = tmp[0]
                mat = np.diag([tmp,tmp,tmp,tmp,tmp,tmp,tmp])
            else:
                rospy.loginfo('[Control node / Update KD]: Wrong input format. Update of K_d is not executed')
        else:
                rospy.loginfo('[Control node / Update KD]: Wrong input format. Update of K_d is not executed')

        # Log if new stiffness is choosen
        # if not np.array_equal(self.K_d_prev, mat):
        #     rospy.loginfo("[Control node / Update KD]: New Stiffness was choosen: \n{0}\n".format(mat))

        self.K_d_prev = mat
        return mat
    
    def update_D_d(self):
        """
        Updates the stiffness matrix K_d (7x7).
        Paramteres: None
        Return: K_d (7x7)
        """
        
        tmp = list(self.D_d)
        if type(tmp) is list:
            if len(tmp)==7:
                tmp_list = [0,1,2,3,4,5,6]
                for i in range(len(tmp)):
                    tmp_list[i] = tmp[i]
                mat = np.diag(tmp)
            elif len(tmp)==1:
                tmp = tmp[0]
                mat = np.diag([tmp,tmp,tmp,tmp,tmp,tmp,tmp])
            else:
                rospy.loginfo_once('[Control node / Update D_d]: Wrong input format. Update of D_d is not executed')
        else:
                rospy.loginfo_once('[Control node / Update D_d]: Wrong input format. Update of D_d is not executed')

        # Log if new stiffness is choosen
        # if not np.array_equal(self.D_d_prev, mat):
        #     rospy.loginfo("[Control node / Update D_d]: New damping was choosen: \n{0}\n".format(mat))
        
        self.D_d_prev = mat
        return mat
   
    def calc_error_cart(self, pose_desi, jacobian, current_joint_velocity, velocity_desired = [0,0,0,0,0,0],):
        """
        Calculation of position-, orientation-, velociteserrors and acceleration
        Parameters:  desired pose (pose_desi: 6x1), jacobian matrix (jacobian: 6x7), 
                    current joint velocity (current_joint_velocity: 6x1), desired joint velocity (velocity_desired: 6x1)
        Return: error pose (6x1), error velocities (6x1), acceleration (6x1)
        """
        # input current pose
        tf_mat_cur = self.robot_dyn_kin.get_current_cartesian_tf()
        
        # Position Error 
        pos_x, pos_y, pos_z = tf_mat_cur[0,3], tf_mat_cur[1,3], tf_mat_cur[2,3]

        error_position = np.atleast_2d([pos_x-pose_desi[0], pos_y-pose_desi[1], pos_z-pose_desi[2]])

        # Orientation Error
        quat_c = np.atleast_2d(tft.quaternion_from_matrix(tf_mat_cur)).T

        # calculate quaternions desired position / euler sequence xyz
        quat_d = np.atleast_2d(tft.quaternion_from_euler(pose_desi[3], pose_desi[4], pose_desi[5])).T # roll. pitch, yaw
        
        quat_C = Quaternion(-quat_c).conjugate
        quat_D = Quaternion(quat_d)

        err = quat_C*quat_D
        rot_err = np.atleast_2d(err.elements[1:]).T
        error_pose = np.atleast_2d(np.concatenate((error_position, rot_err)))
        
        # Velocity error
        velocity_desired = np.atleast_2d(velocity_desired).T
        error_velocity = jacobian * current_joint_velocity - velocity_desired

        return error_pose, error_velocity

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
        """
        if len(y_t) is not len(y_t_1):
            rospy.loginfo("[Control node]: Length of input lowpass filter is not equal.")
        
        else:
            y = [0]*len(y_t)
            for i in range(len(y_t)):
                y[i] = (1-self.settings.get_lowpass_coeff())*y_t_1[i]+self.settings.get_lowpass_coeff()*y_t[i]
        return y

    ############ Format converter (KDL/Numpy, List/Dict) ############ 
    def list2dictionary(self, list, joint_names):
        """
        Transform list into dictionary.
        Parameters: list
        Return: dictionary
        """
        new_limb_torque = {}
        i = 0
        for joint in joint_names:
            new_limb_torque[joint] = list[i]
            i += 1
        return new_limb_torque

    ############ Getter methods ############ 
    def get_periodTime(self):
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
        #rospy.loginfo("Period Time: \n{0}\n".format(period))
        return period
       
    def get_statecondition(self):
        """
        Gets statecondition from ROSPARAM server. This method can be used to implemend a decision
        algorithm and to de/activate control laws.
        Parameters: None
        Return: statecondition (int)
        """
        statecondition = self.settings.get_Statemachine_condition()
        if statecondition != 3 and statecondition != 4: # and statecondition !=2 and statecondition !=1:
            statecondition = 3
            self.settings.set_Statemachine_condition(3)
        return statecondition
    
    def get_cartesian_pose_desired(self):
        """
        Get current desired cartesian pose from rosparam. TODO publisher/ subscriber of desired pose
        Parameters: None
        Return: current desired cartesian pose (6x1)
        """
        return np.atleast_2d(self.settings.get_cartesian_pose_desired())
    
    ############ Setter methods ############
    def set_initalstate(self, joint_angle_desired):
        """
        Set desired joint angles.
        Parameters: desired joint angle (list: 6x1)
        Retrun: None
        """
        rospy.set_param("control_node/joint_angle_desired", joint_angle_desired)
        
    def set_cartesian_pose(self, pose):
        """
        Set cartesian pose.
        Parameters: pose (6x1)
        Return: None
        """
        list_tmp = [0,0,0,0,0,0]
        pose_list = np.ndarray.tolist(pose)
        for index, value in enumerate(pose_list):
            list_tmp[index] = pose_list[index][0]
        self.settings.set_cartesian_pose_desired(list_tmp)

    def set_headlight_color(self, saturation):
        """
        Set headlight colors.
        Parameters: Saturation of torque limit (saturation: float)
        Return: None
        """
        
        if saturation >= 0.95:
            #light_name='head_red_light'
            #self.light.set_light_state(light_name)
            msg = "red"
            self.publish_head_light_color(msg, self._pub_lightcolor)
        elif 0.75 <= saturation <0.95:
            #light_name='head_blue_light'
            #self.light.set_light_state(light_name)
            msg = "yellow"
            self.publish_head_light_color(msg, self._pub_lightcolor)
        else:
            #light_name='head_green_light'
            #self.light.set_light_state(light_name)
            msg = "green"
            self.publish_head_light_color(msg, self._pub_lightcolor)

    ############ Control loop ############ 
    def clean_shutdown(self):
        """
        Clean exit of controller node
        """
        rospy.loginfo("\nExiting Control node...")
        self.robot_dyn_kin.clean_shutdown()

    def run_statemachine(self, statecondition, cur_joint_angle, cur_joint_velocity, joint_angle_error, joint_velocity_error, K_d, D_d, periodTime, coriolis, mass, gravity, jacobian, djacobian, cartesian_pose_error, cartesian_velocity_error, cartesian_acceleration):
        """
        Statemachine is shifting between different control algorithms. The statecondition is a Ros parameter and can therefore be changed 
            while running the loop. Also, the method get_statecondition can be used to implement a switch condition or a switch observer.
        Parameters: statecondition (int),  current joint angle (7x1), current joint velocity (7x1),  current pose error (6x1), 
            current velocity error (6x1), joint angle error (7x1), joint velocity error (7x1), acceleration ddx (6x1), 
            stiffness matrix K_d (7x7), sampling time (float), coriolis vecotr (7x1), mass matrix (7x7), gravity vector (7x1),
            jacobian matrix (6x7), derivative of jacobian - djacobian (6x7)
        Return: List of control torques for the motor (7x1)
        """
        
        if statecondition == 1:     # State 1: DLR impedance controller - cartesian space (Source DLR: Alin Albu-Schäffer )           
            K_d = K_d[:6,:6]          # Reduce 7x7 to 6x6
            D_d = D_d[:6,:6]          # Reduce 7x7 to 6x6
            K_n = np.identity(7)*1 # positiv definite stiffness matrix - 7x7 matrix #TODO add to config file
            D_n = np.identity(7)*1   # positiv definite damping matrix - 7x7 matrix #TODO add to config file
            q_n = np.atleast_2d(np.array(self.settings.get_nullspace_pose())).T  # get Nullspace configuration

            if periodTime == 0:
                periodTime = 1/self.rate
            
            motor_torque = self.DLR_Impedance_Cartesian.calc_joint_torque( K_d, K_n, D_n, q_n, mass, jacobian, djacobian, coriolis, gravity, cartesian_pose_error, 
                                                                          cartesian_velocity_error, cartesian_acceleration, cur_joint_angle, cur_joint_velocity, periodTime, 
                                                                          self.settings.get_nullspace_is_locked())
            return motor_torque
        
        elif statecondition == 2:   # State 2: PD impedance controller - cartesian space 
            K_d = K_d[:6,:6]          # Reduce 7x7 to 6x6
            D_d = D_d[:6,:6]          # Reduce 7x7 to 6x6
            K_n = np.identity(7)*30  # positiv definite stiffness matrix - 7x7 matrix #TODO add to config file
            D_n = np.identity(7)*1   # positiv definite damping matrix - 7x7 matrix   #TODO add to config file
            q_n = np.atleast_2d(np.array(self.settings.get_nullspace_pose())).T  # get Nullspace configuration
            
            motor_torque = self.PD_impedance_cartesian.calc_joint_torque(K_d, D_d, K_n, q_n, cartesian_pose_error, coriolis, jacobian, gravity, cur_joint_velocity, 
                                                                         cur_joint_angle, self.settings.get_nullspace_is_locked())
            return motor_torque

        elif statecondition == 3: # State 3: Spring/Damper impedance controller - jointspace
            motor_torque = self.impedance_ctrl_simple.calc_joint_torque(joint_angle_error, joint_velocity_error, K_d, D_d, gravity)
            return motor_torque
        
        elif statecondition == 4: # State 4: PD impedance Controller - jointspace
            motor_torque = self.PD_impedance_jointspace.calc_joint_torque(gravity, K_d, D_d, coriolis, joint_angle_error, joint_velocity_error)
            return motor_torque

        else:
            rospy.loginfo('State does not exists. Exiting...')
            pass
     
    def run_controller(self):
        """
        Controlloop to calculate the joint torques based on a controller which is aktivated within a statemachine.
        Parameters: None
        Return: motor torques (dict: 7x1)
        """
        rospy.loginfo("[Control_node]: Own controlloop is running: {0}\n".format(self.settings.get_control_flag()))
        rospy.loginfo("[Control_node]: If false, too turn on control set rosparam to true.")
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            #self.pr.enable()


            ### UPDATE current robot state ###
            periodTime = self.get_periodTime()
            self.robot_dyn_kin.update_robot_states(periodTime)
            controller_flag = self.settings.get_control_flag()
            move2neutral = self.settings.get_move2neutral()

            if move2neutral == True: 
                controller_flag = False
                timeout = 5
                speed = 0.2
                self.robot_dyn_kin.move2neutral(timeout, speed)
                rospy.loginfo("[Control_node]: Moved to inital pose.")
                self.settings.set_move2neutral(False)
        
            if controller_flag == False:              
                self.set_initalstate(self.robot_dyn_kin.get_current_joint_angles_list())
                self.set_cartesian_pose(self.robot_dyn_kin.get_current_cartesian_pose())
                self.robot_dyn_kin.set_limb_timeout(self.rate, 0)
                self.safety_regulator.reset_watchdig_oscillation()

            if controller_flag == True:

                ### GET current robot state #
                self.robot_dyn_kin.set_limb_timeout(self.rate, 50)
                cur_joint_angle = self.robot_dyn_kin.get_current_joint_angles()         # numpy 7x1
                cur_joint_velocity = self.robot_dyn_kin.get_current_joint_velocities()  # numpy 7x1
                mass = self.robot_dyn_kin.get_mass()    # numpy 7x7 
                gravity = self.robot_dyn_kin.get_gravity_compensation()  # numpy 7x1 
                jacobian = self.robot_dyn_kin.get_jacobian()  # numpy 6x7 
                djacobian = self.robot_dyn_kin.get_djacobian()
                coriolis = self.robot_dyn_kin.get_coriolis() # numpy 7x1
                
                ### UPDATE current parameter and desired position and velocity ###
                K_d, D_d, joint_angle_desi, joint_velocity_desi = self.update_parameters() # updates K_d, D_d
                statecondition = self.get_statecondition() # int

                ### CALCULATE error
                joint_angle_error = self.calc_error_joint(cur_joint_angle, joint_angle_desi)
                joint_velocity_error = self.calc_error_joint(cur_joint_velocity, joint_velocity_desi)
                pose_desired = self.get_cartesian_pose_desired().T
                cartesian_pose_error, cartesian_velocity_error = self.calc_error_cart(pose_desired, self.robot_dyn_kin.get_jacobian(), self.robot_dyn_kin.get_current_joint_velocities())
                cartesian_acceleration = self.robot_dyn_kin.get_cartesian_acceleration_EE()
                
                torque_motor = self.run_statemachine(statecondition, cur_joint_angle, cur_joint_velocity, joint_angle_error, joint_velocity_error, K_d, D_d, periodTime, 
                                                     coriolis, mass, gravity, jacobian, djacobian, cartesian_pose_error, cartesian_velocity_error, cartesian_acceleration)
                
                ### Safety Regulator
                torque_motor = self.safety_regulator.watchdog_joint_limits_torque_control(cur_joint_angle, gravity, torque_motor)
                torque_motor, saturation = self.safety_regulator.watchdog_torque_limits(torque_motor)
                
                flag, power, frequency = self.safety_regulator.watchdog_oscillation(cur_joint_velocity, self.rate, self.settings.get_oscillation_window_len(), self.settings.get_oscillation_corner_freq() , self.settings.get_oscillation_power_limit())
                if flag == False:
                    self.settings.set_control_flag(flag)
                for i in range(len(torque_motor)):
                    power_tmp = power[i]
                    frequency_tmp = frequency[i]
                    self.publish_jointstate(frequency_tmp, power_tmp, self._pub_oscillation[i])

                ### DISABLE cuff and gravitation compensation and PUBLISH debug values
                self._pub_cuff_disable.publish()  
                self._pub_gc_disable.publish()
                if self.settings.get_self_collision_is_disabled() == True:
                    self._pub_self_collision_disable.publish()
                if self.settings.get_contact_collision_disabled() == True:
                    self._pub_contact_collision_disable.publish()

                self.publish_jointstate(joint_velocity_error, cur_joint_velocity, self._pub_joint_velocity)
                self.publish_effortstate(torque_motor, self._pub_joint_torques)
                self.set_headlight_color(saturation)
                ### CONVERT controller output in msg format dictionary

                torque_motor_dict = self.list2dictionary(torque_motor, self.robot_dyn_kin.get_joint_names())

                ### PUBLISH Joint  torques
                self.robot_dyn_kin.set_joint_torques(torque_motor_dict)

                #self.pr.disable()
                #self.pr.print_stats(sort='time')  
            
            r.sleep()
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
duration: {secs: 5, nsecs: 0}
'''
