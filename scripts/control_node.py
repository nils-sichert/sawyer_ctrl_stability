#!/usr/bin/env python3

        
import rospy
from pyquaternion import Quaternion
import numpy as np
from sensor_msgs.msg import JointState, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Empty, Header, String
from intera_core_msgs.msg import JointLimits, SEAJointState
import sys, os
import tf.transformations as tft

from cartesianspace_controller import pd_impedance_cartesian, dlr_impedance_cartesian
from jointspace_controller import pd_impedance_jointspace, spring_damper_jointspace
from configuration_server import Configuration_server
from robot_dyn_kin_server import Robot_dynamic_kinematic_server
from safety_regulator import Safety_regulator
from scipy.spatial.transform import Rotation as R

from matplotlib import pyplot as plt
from matplotlib import animation

import cProfile


"""
Please add Bugs and Todos here:
TODO https://support.rethinkrobotics.com/support/solutions/articles/80000980380-motion-interface-tutorial
TODO add switch between cartesian pose and joint space pose
TODO improve cartesian pose getter and setter
"""

class controller():
    def __init__(self, limb = "right"):
        
        print("Initializing node...")

        rospy.init_node('Control_manager', anonymous=True)
        
        ##### Instances ######
        # Settings
        self.settings = Configuration_server()

        # Robot
        self.robot_dyn_kin = Robot_dynamic_kinematic_server(limb)
        
        # Controller
        self.DLR_Impedance_Cartesian = dlr_impedance_cartesian()
        self.PD_impedance_cartesian = pd_impedance_cartesian()

        self.impedance_ctrl_simple = spring_damper_jointspace()
        self.PD_impedance_jointspace  = pd_impedance_jointspace()

        self.pr = cProfile.Profile()
        # Safety and Watchguard
        joint_angle_limit_upper, joint_angle_limit_lower = self.robot_dyn_kin.get_joint_limits()
        joint_effort_limit_upper, joint_effort_limit_lower = self.robot_dyn_kin.get_torque_limits()
        self.safety_regulator = Safety_regulator(joint_angle_limit_upper, joint_angle_limit_lower, joint_effort_limit_upper, joint_effort_limit_lower, self.settings.get_oscillation_window_len(), self.settings.get_oscillation_corner_freq(),self.settings.get_oscillation_power_limit() )

        ### Publisher ###
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

        
        ### Debug Publisher ###
        self._pub_error = rospy.Publisher('/control_node_debug/EE_Pose_Error', JointState, queue_size=1)
        self._pub_joint_velocity = rospy.Publisher('/control_node_debug/joint_velocity', JointState, queue_size=1)
        self._pub_joint_torques = rospy.Publisher('control_node_debug/setpoints_motor_torque', JointState, queue_size=1)
        
        self._pub_oscillation = [None]*7 #TODO delete hardcode
        for index, value in enumerate(joint_effort_limit_upper.T):
            topic = '/control_node_debug/oscillation_joint/' + str(index)
            self._pub_oscillation[index] = rospy.Publisher(topic, JointState, queue_size=1)
        
        self._pub_lightcolor = rospy.Publisher('control_node_debug/color', String, queue_size=1)

        self.rate = 100 # Control rate

        # Instance state varibles
        self.torque_motor_t_1 = [0,0,0,0,0,0,0]
        self.pose_cart = None
        self.timestamp_t_1 = 0
        self.Kd = None
        self.Dd = None

        self.set_initalstate(self.robot_dyn_kin.get_current_joint_angles_list())
        self.set_cartesian_inital_pose(self.robot_dyn_kin.get_current_cartesian_pose())


    ############ Publisher (Debugpurpose) ############ 

    def publish_jointstate(self, position, velocity, publisher: rospy.Publisher):
        """
        Publishs position and velocity error of endeffector.
        Parameters: position error (6x1, 'error_pos), velocity error (6x1, 'error_vel)
        Returns: publisher.publish()
        """
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

    ############ Update States/ Variables ############ 

    def update_parameters(self):
        """
        Updates stiffness matrix, time of period and gets the jacobian at time t-1, desired pose and velocity.
        Parameters: None
        Return: Kd (7x7), samlpingTime (time of last period) (float), jacobian_prev (6x7), desired pose (6x1), desired velocity (6x1)
        """
        Kd = self.update_Kd()
        Dd = self.update_Dd()
        periodTime = self.get_periodTime() 
        joint_angle_desi = np.atleast_2d(self.safety_regulator.watchdog_joint_limits_jointangle_control(self.settings.get_joint_angle_desired())).T
        joint_velocity_desi = np.atleast_2d(np.array(self.settings.get_joint_velocity_desired())).T

        return Kd, Dd, periodTime, joint_angle_desi, joint_velocity_desi
        
    def update_Kd(self):
        """
        Updates the stiffness matrix Kd (7x7).
        Paramteres: None
        Return: Kd (7x7)
        """
        old_Kd = self.Kd
        tmp = self.settings.get_stiffness()
        if type(tmp) is list:
            if len(tmp)==7:
                tmp_list = [0,1,2,3,4,5,6]
                for i in range(len(tmp)):
                    tmp_list[i] = tmp[i]
                self.Kd = np.diag(tmp)
            elif len(tmp)==1:
                tmp = tmp[0]
                self.Kd = np.diag([tmp,tmp,tmp,tmp,tmp,tmp,tmp])
            else:
                print('[Control node / Update KD]: Wrong input format. Update of Kd is not executed')
        else:
                print('[Control node / Update KD]: Wrong input format. Update of Kd is not executed')

        if not np.array_equal(old_Kd, self.Kd):
            print('[Control node / Update KD]: New Stiffness was choosen: \n', self.Kd)
        return self.Kd
    
    def update_Dd(self):
        """
        Updates the stiffness matrix Kd (7x7).
        Paramteres: None
        Return: Kd (7x7)
        """
        old_Dd = self.Dd
        tmp = self.settings.get_damping()
        if type(tmp) is list:
            if len(tmp)==7:
                tmp_list = [0,1,2,3,4,5,6]
                for i in range(len(tmp)):
                    tmp_list[i] = tmp[i]
                self.Dd = np.diag(tmp)
            elif len(tmp)==1:
                tmp = tmp[0]
                self.Dd = np.diag([tmp,tmp,tmp,tmp,tmp,tmp,tmp])
            else:
                print('[Control node / Update Dd]: Wrong input format. Update of Dd is not executed')
        else:
                print('[Control node / Update Dd]: Wrong input format. Update of Dd is not executed')

        if not np.array_equal(old_Dd, self.Dd):
            print('New Damping was choosen: \n', self.Dd)
        return self.Dd
   
    def calc_error_cart(self, pose_desi, jacobian, current_joint_velocity, velocity_desired = [0,0,0,0,0,0],):
        """
        Calculation of position-, orientation-, velociteserrors and acceleration
        Parameters: position vector (3x1), rotation matrix (3x3), current velocity (6x1), desired pose (6x1)
        Return: error pose (6x1), error velocities (6x1), acceleration (6x1)
        TODO debug Orientation error calculation
        """
        # input current pose
        tf_mat_cur = self.robot_dyn_kin.get_current_cartesian_tf()
        
        # Position Error # TODO check sign
        pos_x, pos_y, pos_z = tf_mat_cur[0,3], tf_mat_cur[1,3], tf_mat_cur[2,3]

        error_position = np.atleast_2d([pos_x-pose_desi[0], pos_y-pose_desi[1], pos_z-pose_desi[2]])

        # Orientation Error
        # calculate quaternions current position
        # R = tft.rotation_matrix(0.123, (1, 2, 3))

        quat_c = np.atleast_2d(tft.quaternion_from_matrix(tf_mat_cur)).T

        # calculate quaternions desired position / euler sequence xyz
        roll = pose_desi[3]
        pitch = pose_desi[4]
        yaw = pose_desi[5]
        quat_d = np.atleast_2d(tft.quaternion_from_euler(roll, pitch, yaw)).T
        
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
        TODO clear method, make general useable
        """
        if len(y_t) is not len(y_t_1):
            print("Err Controller_node.py: Length of input lowpass filter is not equal.")
        
        else:
            y = [0]*len(y_t)
            for i in range(len(y_t)):
                y[i] = (1-self.settings.get_lowpass_coeff())*y_t_1[i]+self.settings.get_lowpass_coeff()*y_t[i]
        return y

    ############ Format converter (KDL/Numpy, List/Dict) ############ 
    
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
        return period
       
    def get_statecondition(self):
        """
        Gets statecondition from ROSPARAM server. This method can be used to implemend a decision
        algorithm.
        Parameters: None
        Return: statecondition (int)
        """
        statecondition = self.settings.get_Statemachine_condition()
        if statecondition != 3 and statecondition != 4 and statecondition !=2: # and statecondition !=2:
            statecondition = 3
            self.settings.set_Statemachine_condition(3)
        return statecondition
    
    def get_cartesian_pose_desired(self):
        return np.atleast_2d(self.settings.get_cartesian_pose_desired())
        
    def set_initalstate(self, current_joint_angle):
        rospy.set_param("control_node/joint_angle_desired", current_joint_angle)
        
    def set_cartesian_inital_pose(self, pose):
        list_tmp = [0,0,0,0,0,0]
        pose_list = np.ndarray.tolist(pose)
        for i in range(len(pose)):
            list_tmp[i] = pose_list[i][0]
        self.settings.set_cartesian_pose_desired(list_tmp)

    def set_headlight_color(self, saturation):
        
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
        print("\nExiting Control node...")
        self.robot_dyn_kin.clean_shutdown()

    def run_statemachine(self, statecondition, cur_joint_angle, cur_joint_velocity, joint_angle_error, joint_velocity_error, Kd, Dd, periodTime, coriolis, mass, gravity, jacobian, djacobian, cartesian_pose_error, cartesian_velocity_error, cartesian_acceleration):
        """
        Statemachine is shifting between different control algorithms. The statecondition is a Ros parameter and can therefore be changed 
            while running the loop. Also, the method get_statecondition can be used to implement a switch condition or a switch observer.
        Parameters: statecondition (int),  current joint angle (7x1), current joint velocity (7x1),  current pose error (6x1), 
            current velocity error (6x1), joint angle error (7x1), joint velocity error (7x1), acceleration ddx (6x1), 
            stiffness matrix Kd (7x7), sampling time (float), coriolis vecotr (7x1), mass matrix (7x7), gravity vector (7x1),
            jacobian matrix (6x7), derivative of jacobian - djacobian (6x7)
        Return: List of control torques for the motor (7x1)
        TODO clear names of controller
        """
        
        if statecondition == 1: # State 1: DLR impedance controller - cartesian space (Source DLR: Alin Albu-Schäffer )
            Kd = Kd[:6,:6]
            motor_torque = self.DLR_Impedance_Cartesian.calc_joint_torque(Kd, cartesian_acceleration, cur_joint_angle, cur_joint_velocity, cartesian_pose_error, cartesian_velocity_error,  mass, coriolis, jacobian, djacobian, gravity, periodTime)
            return motor_torque
        
        elif statecondition == 2: # State 2: PD impedance controller - cartesian space 
            Kd = Kd[:6,:6]
            Dd = Dd[:6,:6]
            Kn = np.identity(7)*30 # positiv definite stiffness matrix - 7x7 matrix
            Dn = np.identity(7)*1 # positiv definite damping matrix - 7x7 matrix
            qn = np.atleast_2d(np.array([-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40])).T # desired joint configuration of arm at nullspace - 7x1 vector
            if self.settings.get_nullspace_is_free() == True: # free Nullspace
                qn = cur_joint_angle
            else:
                #qn = self.settings.get_nullspace_pose()
                qn = np.atleast_2d(np.array([-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40])).T
            motor_torque = self.PD_impedance_cartesian.calc_joint_torque(Kd, Dd, Kn, Dn, qn, cartesian_pose_error, cartesian_velocity_error, coriolis, jacobian, gravity, cur_joint_velocity, cur_joint_angle)
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
        print("[Control_node]: Own controlloop is working: ", self.settings.get_control_flag())
        print("[Control_node]: If false, too turn on set rosparam to true.")
        while not rospy.is_shutdown():
            # self.pr.enable()


            ### UPDATE current robot state ###
            self.robot_dyn_kin.update_robot_states(self.get_periodTime())
            controller_flag = self.settings.get_control_flag()
            move2neutral = self.settings.get_move2neutral()

            if move2neutral == True:
                # TODO debug
                controller_flag = False
                timeout = 5
                speed = 0.2
                self.robot_dyn_kin.move2neutral(timeout, speed)
                print("[Control_node]: Moved to inital pose.")
                self.settings.set_move2neutral(False)
        
            if controller_flag == False:              
                self.set_initalstate(self.robot_dyn_kin.get_current_joint_angles_list())
                self.set_cartesian_inital_pose(self.robot_dyn_kin.get_current_cartesian_pose())
                self.safety_regulator.reset_watchdig_oscillation()

            if controller_flag == True:

                ### GET current robot state ###
                cur_joint_angle = self.robot_dyn_kin.get_current_joint_angles()         # numpy 7x1
                cur_joint_velocity = self.robot_dyn_kin.get_current_joint_velocities()  # numpy 7x1
                mass = self.robot_dyn_kin.get_mass()    # numpy 7x7 
                gravity = self.robot_dyn_kin.get_gravity_compensation()  # numpy 7x1 
                jacobian = self.robot_dyn_kin.get_jacobian()  # numpy 6x7 
                djacobian = self.robot_dyn_kin.get_djacobian()
                coriolis = self.robot_dyn_kin.get_coriolis() # numpy 7x1
                
                ### UPDATE current parameter and desired position and velocity ###
                Kd, Dd, periodTime, joint_angle_desi, joint_velocity_desi = self.update_parameters() # updates Kd, Dd
                statecondition = self.get_statecondition() # int

                ### CALCULATE error
                joint_angle_error = self.calc_error_joint(cur_joint_angle, joint_angle_desi)
                joint_velocity_error = self.calc_error_joint(cur_joint_velocity, joint_velocity_desi)
                pose_desired = self.get_cartesian_pose_desired().T
                cartesian_pose_error, cartesian_velocity_error = self.calc_error_cart(pose_desired, self.robot_dyn_kin.get_jacobian(), self.robot_dyn_kin.get_current_joint_velocities())
                cartesian_acceleration = self.robot_dyn_kin.get_cartesian_acceleration_EE()
                
                torque_motor = self.run_statemachine(statecondition, cur_joint_angle, cur_joint_velocity, joint_angle_error, joint_velocity_error, Kd, Dd, periodTime, 
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

                # self.pr.disable()
                # self.pr.print_stats(sort='time')  
            
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
duration: {secs: 5, nsecs: 0}
'''
