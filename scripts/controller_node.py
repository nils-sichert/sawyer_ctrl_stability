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
        rospy.init_node("activ_passiv_controller") 
        self.Kd = rospy.get_param("Kd", [1,1,1,1,1,1])
        self.Dd = rospy.get_param("Dd", [1,1,1,1,1,1])
        self._init_joint_angles = rospy.get_param("Init_angle", [-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981])

        # control parameters
        self.rate = rospy.Rate(100) # Controlrate - 100Hz
        self._missed_cmd = 20 # Missed cycles before triggering timeout
        
        # Instance Impedance controller
        self.impedance_ctrl = impedance_ctrl()

        # create limb instance
        self._limb = intera_interface.Limb(limb)
        # Instance Optimal Controller
        # Instance Robotic Chain
        urdf_filepath = "sawyer_robot/sawyer_description/urdf/sawyer_base.gazebo.xacro"
        (ok, robot) = urdf.treeFromFile(urdf_filepath)
        self._robot_chain = robot.getChain('right_arm_base_link', 'right_l6')
        self._nrOfJoints = self._robot_chain.getNrOfJoints()
        self._jac_kdl = kdl.ChainJntToJacSolver(self._robot_chain)
        self.grav_vector = kdl.Vector(0, 0, -9.81)
        self.FKSolver = kdl.ChainFkSolverPos_recursive(self._robot_chain)
        self.dyn_kdl = kdl.ChainDynParam(self._robot_chain, self.grav_vector)
        
        # Kd, Dd = rosparam(default)
        rospy.init_node('Passiv_Activ_Controller', anonymous=True)
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
        self.pose_desi = self.calc_pose(self._init_joint_angles)
        self.pose_1 = self.pose_desi
        self.current_angle = self.joint_angle
        self.jacobian = self.calc_jacobian(self.current_angle)
        self.jacobian_1 = self.calc_jacobian(self.current_angle)
        self.jacobian_2 = self.calc_jacobian(self.current_angle)

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
        self.Kd = rospy.get_param("Kd", [1,1,1,1,1,1])

    def update_Dd(self):
        self.Dd = rospy.get_param("Dd", [1,1,1,1,1,1])

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

    def calc_pose(self, joint_angle):
        Frame = kdl.eeFrame
        self.FKSolver.JntToCart(joint_angle, Frame)

    def calc_coriolis(self, joint_angles, joint_velocities):
        coriolis_torques = kdl.JntArray(self._nrOfJoints)
        self.dyn_kdl.JntToCoriolis(joint_angles, joint_velocities, coriolis_torques)	# C(q, q_dot)
        return coriolis_torques

    def calc_inertia(self, joint_angles):
        B_kdl = kdl.JntSpaceInertiaMatrix(self._nrOfJoints)
        self.dyn_kdl.JntToMass(joint_angles, B_kdl)
        return B_kdl

    def calc_gravity(self, joint_angles):
        grav_torques = kdl.JntArray(self._nrOfJoints)
        self.dyn_kdl.JntToGravity(joint_angles, grav_torques)
        return grav_torques

    def calc_jacobian(self, joint_angles):
        j_kdl = kdl.Jacobian(self._nrOfJoints)
        self._jac_kdl.JntToJac(joint_angles, j_kdl)
        return j_kdl

    def get_Kd(self):
        #TODO load param
        Kd = np.identity(7)
        return Kd

    def get_Dd(self):
        #TODO load param
        Dd = np.identity(7)
        return Dd

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

    def get_statecondition(self):
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
                ddPose_cart = 0 # TODO add ddPose cartesian # numpy 6x1
                force_ee = self.get_force_ee() # numpy 7x1
                statecondition = self.get_statecondition(force_ee) # int

                cur_joint_angle = self._limb.joint_angles()#self.joint_angle # numpy 7x1
                cur_joint_velocity = self._limb.joint_velocities()#self.joint_vel # numpy 7x1
                cur_joint_efforts = self._limb.joint_efforts()
                joint_angles = kdl.JntArray(self._nrOfJoints)
                joint_velocities = kdl.JntArray(self._nrOfJoints)
                joint_efforts = kdl.JntArray(self._nrOfJoints)
                
                # change data to KDL format
                i = 0
                for joint in self._limb.joint_names():
                    joint_angles[i] = cur_joint_angle[joint]
                    joint_velocities[i] = cur_joint_velocity[joint]
                    joint_efforts[i] = cur_joint_efforts[joint]
                    i += 1

                pose = self.calc_pose(joint_angles) # numpy 6x1
                coriolis = self.calc_coriolis(joint_angles, joint_velocities) # numpy 7x7
                inertia = self.calc_inertia(joint_angles) # numpy 7x7
                gravity = self.calc_gravity(joint_angles) # numpy 7x1
                jacobian = self.calc_jacobian(joint_angles) # numpy 6x7
                
                # return numpy 7x1 vector of torques 
                torque_motor = self.run_statemachine(statecondition, Kd, Dd, cur_joint_angle, cur_joint_velocity, rate, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, force_ee, ddPose_cart)
                
                self.publish_torques(torque_motor, self.motor_torque_pub)

                self.jacobian_1 = jacobian
                self.jacobian_2 = jacobian_1
                self.pose_1 = pose
            
            self.reset_gravity_compensation()
            self.rate.sleep()
        self.clean_shutdown()

def main():
    control = controller()
    control.run_controller()

if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass


