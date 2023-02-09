import numpy as np
import math
from scipy.linalg import sqrtm

class PD_Impedance_ctrl_woutMass():
    def __init__(self) -> None:
        # TODO change into rosparams
        self._Kn = np.diag([1,1,1,1,1,1,1]) # positiv definite stiffness matrix - 7x7 matrix
        self._Dn = np.diag([1,1,1,1,1,1,1]) # positiv definite damping matrix - 7x7 matrix
        self._qn = np.atleast_2d(np.array([1,1,1,1,1,1,1])).T # desired joint configuration of arm at nullspace - 7x1 vector
        self._D_eta = np.diag([0.7,0.7,0.7,0.7,0.7,0.7])

    def calc_joint_torque(self, joint_angle_error, joint_velocity_error, gravity, Kd, coriolis, inertia):
        '''
        Input: Jacobian dim:6x7 (J), gravity torque dim:7x1 (g), stiffness matrix dim:6x6 (Kd), joint_angle dim:7x1 (q),
                joint_velocity dim: 7x1 (dq), cartesian pose error dim:6x1 (err_cart), cartesian velocity error dim:(6x1) (derr_cart),
                inertia matrix dim: 7x7 (inetria), coriolis matrix which is already coriolis times joint velocity dim: 7x1 (coriolis)
        
        Ouput: motor torque for each joint motor dim: list (motor_torque)
        '''


        Dd = np.diag([1,1,1,1,1,1,1])# 2* np.sqrt(Kd*np.abs(inertia)) # critical damping of system

        # Desired Torque
        torque_list = [0]*len(joint_angle_error)
        
        for joint in range(len(joint_angle_error)):
            # spring portion
            torque_list[joint] = Kd[joint][joint] * joint_angle_error[joint] - Dd[joint][joint] * joint_velocity_error[joint] + gravity[joint] + coriolis[joint]

        
        return self.vec2list(torque_list)
    
    def vec2list(self,vec):
        len_vec = len(vec)
        list = [0]*len_vec
        for i in range(len_vec):
            list[i] = vec[i][0]
        return list

    def srqt_mat(self, matrix):
        numRows = len(matrix)
        numColums = len(matrix[0])
        sqrt_matrix = np.zeros((numRows, numColums))
        for i in range(numRows):
            for j in range(numColums):
                sqrt_matrix[i][j] = math.sqrt(matrix[i][j])
        return sqrt_matrix
    
def main():
    Jacobian = np.random.rand(6,7) # numpy 6x1
    gravity = np.random.rand(7,1)# numpy 7x1
    Kd = np.diag([1,4,9,16,25,36])# numpy 6x6
    q =  np.random.rand(7,1)# numpy 7x1
    dq =  np.random.rand(7,1)# numpy 7x1
    err_cart = np.random.rand(6,1)# numpy 6x1
    derr_cart = np.random.rand(6,1)# numpy 6x1
    mass = np.random.rand(7,7)# numpy 7x7
    coriolis = np.random.rand(7,1)# numpy 7x1
   
    """  print('Jacobian: ', Jacobian)
    print('gravity: ', gravity)
    print('Kd', Kd)
    print('q', q)
    print('dq', dq)
    print('err_cart', err_cart)
    print('derr_cart', derr_cart)
    print('inertia', inertia)
    print('coriolis',coriolis) """

    ctrl = PD_Impedance_ctrl_woutMass()
    print(ctrl.calc_joint_torque(Jacobian, gravity, Kd, err_cart, derr_cart, mass, coriolis))

if __name__ == '__main__':
    main()



###### Controller Brain ######

#!/usr/bin/env python3
        
import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from intera_core_msgs.msg import JointLimits, SEAJointState
import threading
import PyKDL as kdl
import copy
from kdl_parser_py import urdf
from sys import argv
from os.path import dirname, join, abspath
from DLR_impedance_ctrl_cartesian import impedance_ctrl
from spring_damper_jointspace import impedance_jointspace
from PD_impedance_cartesian import PD_Impedance_ctrl
from PD_impedance_joint_woMass_wGrav_wCoriolis import PD_Impedance_ctrl_woutMass
from scipy.spatial.transform import Rotation as R

import intera_interface
from intera_interface import CHECK_VERSION


class controller():
    def __init__(self, limb = "right"):
        print("Initializing node...")
        rospy.init_node('Passiv_Activ_Controller', anonymous=True)
        rospy.set_param("/Lowpass_weight", 1)
        self.Kd = rospy.set_param("/Kd", [100,100,100,100,100,100,100])
        rospy.set_param("/Control_flag", True)
        rospy.set_param("Controllerstate", 3)
        rospy.set_param("/joint_angle_desi", [0.0, -1.18, 0.0, 2.18, 0.0, 0.57, 3.3161])
        rospy.set_param("/joint_velocity_desi", [0, 0, 0, 0, 0, 0, 0])
        self.lock = threading.RLock()

        # control parameters
        self.rate = 100 # Controlrate - 100Hz
        self._missed_cmd = 20 # Missed cycles before triggering timeout
        
        # Instance Controller
        self.impedance_ctrl = impedance_ctrl()
        self.impedance_ctrl_simple = impedance_jointspace()
        self.PD_impedance_ctrl_cart = PD_Impedance_ctrl()
        self.PD_impedance_ctrl_cart_woutMass  = PD_Impedance_ctrl_woutMass()

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
        self._pub_error = rospy.Publisher('EE_Pose_Error', JointState, queue_size=1)

        ### Robot ###
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
        self.jacobian_1, self.jacobian_2 = self.jacobian, self.jacobian
        self.torque_motor_t_1 = None
        self.timestamp_t_1 = 0

    def publish_error(self, error_pos, error_vel, publisher: rospy.Publisher):
        msg = JointState()
        msg.position = error_pos
        msg.velocity = error_vel
        publisher.publish(msg)

    def update_parameters(self):
        Kd = self.update_Kd()
        rate = self.get_rate() # int
        jacobian_1 = self.get_jacobian_1() # numpy 6x7
        jacobian_2 = self.get_jacobian_2() # numpy 6x7
        pose_desi = self.get_pose_desi() # numpy 6x1
        velocities_desi = self.get_velocities_desi()
        ddPose_cart = np.atleast_2d(np.array([0,0,0,0,0,0])).T # TODO add ddPose cartesian # numpy 6x1
        force_ee = self.get_force_ee() # numpy 7x1

        return Kd, rate, jacobian_1, jacobian_2, pose_desi, velocities_desi, ddPose_cart, force_ee
        
    def update_Kd(self):
        old_Kd = self.Kd
        self.Kd = np.diag(rospy.get_param("Kd"))
        if not np.array_equal(old_Kd, self.Kd):
            print('New Stiffness was choosen: ', self.Kd)
        return self.Kd
    
    def run_statemachine(self, statecondition, Kd, joint_angle, joint_velocity, rate, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, force_ee, ddPose_cart, err_cart, err_vel_cart, joint_angle_error, joint_velocity_error):
        if statecondition == 1: # State 1: Impedance Controller
        
            #   Input: Kd, Dd, pose, pose_desi, joint_angles, joint_velocity, rosrate, coriolis, inertia, gravity, jacoabian, jacobian_1, jacobian_2)
            #   Output: tau_motor
            Kd = np.diag([10,10,10,10,10,10,10])
            joint_angle_desi = [0.0, -1.18, 0.0, 2.18, 0.0, 0.57, 3.3161]
            motor_torque = self.PD_impedance_ctrl_cart_woutMass.calc_joint_torque(jacobian, gravity, Kd, err_cart, err_vel_cart, coriolis, joint_angle_desi, joint_angle, joint_velocity) #TODO correct input
            return motor_torque
        
        elif statecondition == 2: # State 2: Only for debugg purpose !- Imedance Controller simple
        
            """ if len(Kd)<=6:
                print('Please add joint spring, dimension to small.')
                while len(Kd)<=6:
                    self.update_Kd()
                    Kd = self.Kd
                    rospy.sleep(1)
            """

            Kd = np.diag([10,10,10,10,10,10,10])
            Dd = np.diag([1,1,1,1,1,1,1])
            joint_angle_desi = [0.0, -1.18, 0.0, 2.18, 0.0, 0.57, 3.3161]
            motor_torque = self.impedance_ctrl_simple.calc_torque(joint_angle_desi, joint_angle,joint_velocity,Kd,Dd, gravity)

            return motor_torque
        
        elif statecondition == 3: # State 3: PD impedance Controller
            joint_angle_desi = [0.0, -1.18, 0.0, 2.18, 0.0, 0.57, 3.3161]
            motor_torque = self.PD_impedance_ctrl_cart_woutMass.calc_joint_torque(joint_angle_error, joint_velocity_error, gravity, Kd, coriolis, inertia)
            return motor_torque
        
        else:
            print('State does not exists. Exiting...')
            pass
        
    def calc_pose(self, type, joint_values = None, link_number = 7):
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
        coriolis_torques = kdl.JntArray(self._nrOfJoints)
        self.dyn_kdl.JntToCoriolis(self.joints_to_kdl('positions',joint_angles), self.joints_to_kdl('velocities',joint_velocities), coriolis_torques)	# C(q, q_dot)
        return self.kdl_array_to_numpy_array(coriolis_torques)

    def calc_inertia(self, joint_values=None):
        inertia = kdl.JntSpaceInertiaMatrix(self._nrOfJoints)
        self.dyn_kdl.JntToMass(self.joints_to_kdl('positions',joint_values), inertia)
        return self.kdl_to_array(inertia)

    def calc_gravity(self, joint_values=None):
        grav_torques = kdl.JntArray(self._nrOfJoints)
        self.dyn_kdl.JntToGravity(self.joints_to_kdl('positions',joint_values), grav_torques)
        return self.kdl_array_to_numpy_array(grav_torques)

    def calc_jacobian(self,joint_values=None):
        jacobian = kdl.Jacobian(self._nrOfJoints)
        self._jac_kdl.JntToJac(self.joints_to_kdl('positions',joint_values), jacobian)
        return self.kdl_to_array(jacobian)

    def calc_error(self, pos_vec, rot_mat, pose_desi):
        err_pos = np.array([pos_vec[0]-pose_desi[0], pos_vec[1]-pose_desi[1], pos_vec[2]-pose_desi[2]])

        rot_vect_desi = np.array([pose_desi[3],pose_desi[4],pose_desi[5]])
        r = R.from_matrix([rot_mat])
        rot_vec_cur = r.as_rotvec()

        err_rot = rot_vec_cur-rot_vect_desi
        err = [0]*6
        for i in range(3):
            err[i]=err_pos[i]
            err[i+3]=err_rot[0][i]
        
        err = np.atleast_2d(err).T
        return err

    def lowpass_filter(self, y_t, y_t_1):
        lowpass_coeff = rospy.get_param("/Lowpass_weight")
        if len(y_t) is not len(y_t_1):
            print("Err Controller_node.py: Length of input lowpass filter is not equal.")
        
        else:
            y = [0]*len(y_t)
            for i in range(len(y_t)):
                y[i] = (1-lowpass_coeff)*y_t_1[i]+lowpass_coeff*y_t[i]
        return y

    """ Robot safety methods """
    
    def joints_in_limits(self, q):
        lower_lim = self.joint_limits_lower
        upper_lim = self.joint_limits_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    def joints_in_safe_limits(self, q):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.all([q >= lower_lim, q <= upper_lim], 0)

    def clip_joints_safe(self, q):
        lower_lim = self.joint_safety_lower
        upper_lim = self.joint_safety_upper
        return np.clip(q, lower_lim, upper_lim)

    def clip_joints_effort_safe(self, q):
        lower_lim = self.joint_efforts_safety_lower
        upper_lim = self.joint_efforts_safety_upper
        return np.clip(q, lower_lim, upper_lim)
    
    """ Format converter (KDL/Numpy, List/Dict)"""
    def kdl_to_array(self, kdl):
        array = np.zeros(((kdl.rows(), kdl.columns())))
        for i in range(kdl.rows()):
            for j in range(kdl.columns()):
                array[i][j] = kdl[i,j]
        return array
    
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
    
    def kdl_array_to_numpy_array(self, q):
        if q == None:
            return None
        array = np.zeros((q.rows(),1))
        for i in range(q.rows()):
            array[i] = q[i]
        return array

    def dictionary2list(self, dic):  
        list_tmp = []
        for key, value in dic.items():
            list_tmp.append(value)
        return list_tmp
    
    def list2dictionary(self, list):
        new_limb_torque = {}
        i = 0
        for joint in self._limb.joint_names():
            new_limb_torque[joint] = list[i]
            i += 1
        return new_limb_torque


    def get_cur_pose(self, pos_vec, rot_mat):
        r = R.from_matrix(rot_mat)
        rot_vec = r.as_rotvec()
        pose = [0]*6
        for i in range(3):
            pose[i]=pos_vec[i]
            pose[i+3] = rot_vec[i]
        return pose
 
    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()

    """ Getter methods"""

    def get_rate(self):
        time_now = rospy.get_time()
        if self.timestamp_t_1 != 0:
            period = time_now -self.timestamp_t_1
        else:
            period = 1/self.rate # period time in sec
        self.timestamp_t_1 = time_now    
        return period

    def get_pose_desi(self):
        # TODO implement callback/ subscriber for pose
        return self.pose_desi
    
    def get_velocities_desi(self):
        vel_desi = np.array([0,0,0,0,0,0]) # 6x1 TODO implement subscriber
        return vel_desi

    def get_force_ee(self):
        # TODO implement get_force_ee
        pass

    def get_jacobian_1(self):
        return copy.deepcopy(self.jacobian_1)

    def get_jacobian_2(self):
        return copy.deepcopy(self.jacobian_2)

    def get_statecondition(self, force):
        statecondition = rospy.get_param("Controllerstate")
        return statecondition

    def calc_error_joint(self, current, desired):
        error = desired - current
        return error
    
    def run_controller(self):
        self._limb.move_to_neutral()
        pos_vec, rot_mat, pose = self.calc_pose(type='positions')
        
        # Init hold actual position
        self.pose_desi = self.get_cur_pose(pos_vec, rot_mat)
        self.pose_1 = self.pose_desi
        
        # Set limb controller timeout to return to Sawyer position controller
        self._limb.set_command_timeout((1.0 / self.rate) * self._missed_cmd)
        
        
        while not rospy.is_shutdown():
            controller_flag = rospy.get_param("/Control_flag")
            joint_angle_desi = np.atleast_2d(np.array(rospy.get_param("/joint_angle_desi"))).T
            joint_velocity_desi = np.atleast_2d(np.array(rospy.get_param("/joint_velocity_desi"))).T
            if controller_flag == True:
                Kd, rate, jacobian_1, jacobian_2, pose_desi, velocities_desi, ddPose_cart, force_ee = self.update_parameters() # updates Kd, Dd
                statecondition = rospy.get_param("Controllerstate") # int

                ### get current Joint-angle, -velocity and -effort
                cur_joint_angle = np.atleast_2d(self.dictionary2list(self._limb.joint_angles())).T          # numpy 7x1
                cur_joint_velocity = np.atleast_2d(self.dictionary2list(self._limb.joint_velocities())).T   # numpy 7x1
                cur_joint_efforts = np.atleast_2d(self.dictionary2list(self._limb.joint_efforts())).T
                joint_angle_error = self.calc_error_joint(cur_joint_angle, joint_angle_desi)
                joint_velocity_error = self.calc_error_joint(cur_joint_velocity, joint_velocity_desi)

                ### Update position/ velocity errors, inertia, gravity, jacobian, coriolis
                pos_vec, rot_mat, pose = self.calc_pose(type='positions') 
                err_pos_cart = self.calc_error(pos_vec, rot_mat, pose_desi)      # numpy 6x1
                vel_vec, vel_rot_mat, velo_pose = self.calc_pose(type='velocities')
                err_vel_cart = self.calc_error(vel_vec, vel_rot_mat, velocities_desi)

                inertia = self.calc_inertia()   # numpy 7x7 
                gravity = self.calc_gravity()  # numpy 7x1 
                jacobian = self.calc_jacobian()  # numpy 6x7 
                coriolis = self.calc_coriolis()# numpy 7x7 
                
                ### Calculate motor torque for each joint motor
                torque_motor = self.run_statemachine(statecondition, Kd, cur_joint_angle, cur_joint_velocity, rate, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, force_ee, ddPose_cart, err_pos_cart, err_vel_cart, joint_angle_error, joint_velocity_error)
                
                ### Disable cuff and gravitation compensation and publish debug values
                self._pub_cuff_disable.publish()
                self._pub_gc_disable.publish()
                self.publish_error(err_pos_cart, err_vel_cart, self._pub_error)
                
                ### Transfer controller output in msg format dictionary
                if self.torque_motor_t_1 is not None:
                    torque_motor = self.lowpass_filter(torque_motor, self.torque_motor_t_1)
                
                torque_motor_dict = self.list2dictionary(self.clip_joints_effort_safe((torque_motor)))
                
                ### Publish Joint torques
                self._limb.set_joint_torques(torque_motor_dict)

                ### Set values for t-1 calcualtions
                self.torque_motor_t_1 = torque_motor
                self.jacobian_1 = jacobian
                self.jacobian_2 = jacobian_1
                self.pose_1 = pose
                self.d_pose_1 = velo_pose
            
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


'''
apply force in gazebo:
rosservice call /gazebo/apply_body_wrench "body_name: 'sawyer::right_l6'
reference_frame: 'sawyer::base'
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
  force: {x: 100.0, y: 0.0, z: 0.0}
  torque: {x: 0.0, y: 0.0, z: 0.0}
start_time: {secs: 0, nsecs: 0}
duration: {secs: 5, nsecs: 0}" 

'''
