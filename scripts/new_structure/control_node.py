#!/usr/bin/env python3

#!/usr/bin/env python3
        
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from intera_core_msgs.msg import JointLimits, SEAJointState
import sys, os

from cartesianspace_controller import pd_impedance_cartesian, dlr_impedance_cartesian
from jointspace_controller import pd_impedance_jointspace, spring_damper_jointspace
from setting_server import Setting_server
from robot_dyn_kin_server import Robot_dynamic_kinematic_server
from safety_regulator import Safety_regulator
from scipy.spatial.transform import Rotation as R


"""
Please add Bugs and Todos here:
TODO easier handle of controller positions
TODO each controller hold position while changing controller (same setpoints)
TODO add method to move to neutral
TODO suppres collision avoidance
TODO joint angles 8x1 inlc. head
ctrl.move2neutral(neutral pose)
move2neutral(self, neutral_pose=rospy.get_param...)

TODO https://support.rethinkrobotics.com/support/solutions/articles/80000980380-motion-interface-tutorial
"""

class controller():
    def __init__(self, limb = "right", ControlStartStop = False, joint_angle_desired = [-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40],
                 joint_velocity_desired = [0, 0, 0, 0, 0, 0, 0], controlState = 3, joint_stiffness = [20], joint_damping = [1], neutral_pose = [-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40]):
        print("Initializing node...")
        #TODO ROS_SET: passed as default option when init class
        rospy.init_node('Control_node', anonymous=True)
        
        ##### Instances ######
        # Settings
        self.settings = Setting_server(ControlStartStop, joint_angle_desired, joint_velocity_desired, controlState,joint_stiffness, joint_damping, neutral_pose)

        # Robot
        self.robot_dyn_kin = Robot_dynamic_kinematic_server(limb)

        # Controller
        self.DLR_Impedance_Cartesian = dlr_impedance_cartesian()
        self.PD_impedance_cartesian = pd_impedance_cartesian()

        self.impedance_ctrl_simple = spring_damper_jointspace()
        self.PD_impedance_jointspace  = pd_impedance_jointspace()

        # Safety and Watchguard
        self.safety_regulator = Safety_regulator(self.robot_dyn_kin.get_joint_limits(), self.robot_dyn_kin.get_torque_limits())

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

        self.rate = 100

        # Robot limit and safety
        # TODO limit subscriber /robot/joint_limits
        limit = 100
        self.joint_efforts_safety_lower = [-limit,-limit,-limit,-limit,-limit,-limit,-limit]
        self.joint_efforts_safety_upper = [limit,limit,limit,limit,limit,limit,limit]

        # Instance state varibles
        self.jacobian = self.calc_jacobian()
        self.jacobian_prev = self.jacobian
        self.torque_motor_t_1 = [0,0,0,0,0,0,0]
        self.timestamp_t_1 = 0


    ############ Publisher (Debugpurpose) ############ 

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
        samlpingTime = self.get_periodTime() 
        joint_angle_desi = self.safety_regulator.watchdog_joint_limits_jointangle_control(np.atleast_2d(np.array(self.settings.get_joint_angle_desired())).T)
        joint_velocity_desi = np.atleast_2d(np.array(self.settings.get_joint_velocity_desired())).T

        return Kd, Dd, samlpingTime, joint_angle_desi, joint_velocity_desi
        
    def update_Kd(self):
        """
        Updates the stiffness matrix Kd (7x7).
        Paramteres: None
        Return: Kd (7x7)
        """
        old_Kd = self.Kd
        tmp = rospy.get_param("control_node/Kd")
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
                print('[Update KD]: Wrong input format. Update of Kd is not executed')
        else:
                print('[Update KD]: Wrong input format. Update of Kd is not executed')

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
                print('[Update Dd]: Wrong input format. Update of Dd is not executed')
        else:
                print('[Update Dd]: Wrong input format. Update of Dd is not executed')

        if not np.array_equal(old_Dd, self.Dd):
            print('New Damping was choosen: ', self.Dd)
        return self.Dd
   
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
    
    def get_statecondition(self):
        """
        Gets statecondition from ROSPARAM server. This method can be used to implemend a decision
        algorithm.
        Parameters: None
        Return: statecondition (int)
        """
        statecondition = self.settings.get_Statemachine_condition()
        if statecondition != 3 and statecondition != 4:
            statecondition = 3
            rospy.set_param("control_node/controllerstate", 3)
        return statecondition
    
    def set_initalstate(self, current_joint_angle):
        rospy.set_param("control_node/joint_angle_desi", current_joint_angle)
        rospy.set_param("named_poses/right/poses/neutral", current_joint_angle)

    ############ Control loop ############ 

    def clean_shutdown(self):
        """
        Clean exit of controller node
        """
        print("\nExiting Control node...")
        self.robot_dyn_kin.clean_shutdown()

    def run_statemachine(self, statecondition, cur_joint_angle, cur_joint_velocity, joint_angle_error, joint_velocity_error, Kd, Dd, periodTime, coriolis, inertia, gravity, jacobian, djacobian):
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
            ddx = None
            err_pose_cart = None
            err_vel_cart = None
            sampling_time = 0
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
        
        while not rospy.is_shutdown():

            ### UPDATE current robot state ###
            self.robot_dyn_kin.update_robot_states(self.get_periodTime())
            controller_flag = self.settings.get_control_flag()
        
            if controller_flag == False:
                current_joint_angle = self.dictionary2list(self.robot_dyn_kin.get_current_joint_angles())
                self.set_initalstate(current_joint_angle)

            if controller_flag == True:

                ### GET current robot state ###
                cur_joint_angle = self.robot_dyn_kin.get_current_joint_angles().T          # numpy 7x1
                cur_joint_velocity = self.robot_dyn_kin.get_current_joint_velocities().T   # numpy 7x1
                inertia = self.robot_dyn_kin.get_inertia()    # numpy 7x7 
                gravity = self.robot_dyn_kin.get_gravity_compensation().T  # numpy 7x1 
                jacobian = self.robot_dyn_kin.get_jacobian()  # numpy 6x7 
                djacobian = self.robot_dyn_kin.get_djacobian()
                coriolis = self.robot_dyn_kin.get_coriolis().T # numpy 7x1
                
                ### UPDATE current parameter and desired position and velocity ###
                Kd, Dd, periodTime, jacobian_prev, joint_angle_desi, joint_velocity_desi = self.update_parameters() # updates Kd, Dd
                statecondition = self.settings.get_Statemachine_condition() # int

                ### CALCULATE error
                joint_angle_error = self.calc_error_joint(cur_joint_angle, joint_angle_desi)
                joint_velocity_error = self.calc_error_joint(cur_joint_velocity, joint_velocity_desi)
                
                torque_motor = self.run_statemachine(statecondition, cur_joint_angle, cur_joint_velocity, joint_angle_error, joint_velocity_error, Kd, Dd, periodTime, coriolis, inertia, gravity, jacobian, djacobian)
                
                ### Safety Regulator
                torque_motor = self.safety_regulator.watchdog_joint_limits_torque_control(cur_joint_angle, gravity, torque_motor)
                torque_motor = self.safety_regulator.watchdog_torque_limits(torque_motor)
                
                self.settings.set_control_flag(self.safety_regulator.watchdog_oscillation(torque_motor))

                ### DISABLE cuff and gravitation compensation and PUBLISH debug values
                self._pub_cuff_disable.publish()  # TODO cuff disable/ enable by default / maybe FLAG script
                self._pub_gc_disable.publish()

                self.publish_error(joint_velocity_error, cur_joint_velocity,self._pub_joint_velocity)

                ### CONVERT controller output in msg format dictionary
                
                torque_motor_dict = self.list2dictionary(self.clip_joints_effort_safe((torque_motor)))
                
                ### PUBLISH Joint torques
                self.robot_dyn_kin.set_joint_torques(torque_motor_dict)

                ### SET values for t-1 calcualtions
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
