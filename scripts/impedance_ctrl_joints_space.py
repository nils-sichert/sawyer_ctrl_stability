import rospy
import numpy as np
import PyKDL as KDL
from kdl_parser_py import urdf
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Pose, Twist, Wrench
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointLimits, EndpointState, SEAJointState
import argparse
import os
import intera_interface
from intera_interface import CHECK_VERSION
from impedance_ctrl import impedance_ctrl

class ImpedanceCtrl:
    def __init__(self, limb ='right', rate = 100):#TODO überarbeiten und kürzen
        parser = argparse.ArgumentParser()
        parser.add_argument('--mode', help = 'Control mode: [gc, pos]')
        parser.add_argument('--cuff', help = 'Enable or disable cuff with [on, off], when on, default GC is enabled when squeezing the cuff.')	#cuffs off enable gc - required is: when off  default gc enabled
        parser.add_argument('--defgc', help = 'Disable default gravity compensation [on, off]')
        parser.add_argument('--cuffswitch', help = 'Use button on the cuff to switch control mode.')	#The control mode is the impedance mode - cuffswitch is ON for impedance control to be activated
        parser.add_argument('--collision', help = 'Switch off the automatic collision detection.')
        parser.add_argument('--damping', help = 'Damping constant.')
        parser.add_argument('--rate', help = 'Control rate, default is 100 Hz.')
        parser.add_argument('--resetneutral', help = 'At startup reset to neutral pose [on, off]')
        parser.add_argument('--waiting', help = 'Waiting time before switching to torque control.')
        parser.add_argument('--verbose', help = ('Print outputs'))

        # create Instances
        self._limb = intera_interface.Limb(limb)
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled

        self._joint_names = self._limb.joint_names()

        input_args = parser.parse_args() # TODO create config file and change loading regarding
        
        # TODO add controll params (rate/ missed cycles)

        self._ee_pose = [Pose()]
        self._ee_twist = [Twist()]
        self._ee_wrench = [Wrench()]

        self._joint_pos = [0] * 7
        self._joint_vel = [0] * 7
        self._joint_torque = [0] * 7

        self._default_gc_torques = [0] * 7
        self._default_gc_model_torques = [0] * 7
        self._torque_limits = [0] * 7

        self._default_joint_torques = [0] * 7

        self.prev_x_dot = [0] * 3
        self.xd = []
        
        self.joint_angles_data = []
        self.joint_efforts_data = []
        self.ee_pose_data = []
        self.Fa_data = []
        
        self._rate = rate
        self._missed_cmd = 20
        self._waitingtime = 2
        
        # Init publisher
        self._pub_robot_motion = rospy.Publisher('robot_motion_sender', String, queue_size=10)
        self._pub_gc_torques = rospy.Publisher('computed_gc_torques_sender', JointState, queue_size=10)
        self._gc_torques_msg = JointState()
        self._gc_torques_msg.effort = [0] * 7
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)
        
         # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)
        if input_args.cuff == "off":
            self._cuffon = False
            print ("-- Cuffs OFF.")
        else:
            self._cuffon = True
            print ("-- Cuffs ON.")

        if input_args.cuffswitch == "off":
            self._cuffswitch = False
            print ("-- Cuff switch OFF.")
        else:
            if self._cuffon:
                self._cuffswitch = False
                print ("-- Cuff switch OFF. (When cuff is set to on, cannot use it as switch.)")
            else:
                self._cuffswitch = True
                self._cuff = intera_interface.Cuff()
                print ("-- Cuff switch ON.")

        if input_args.mode == "pos":
            self._activategc = False
            print ("-- Position control only.")
        else:
            self._activategc = True
            if self._cuffon:
                print ("-- WARNING: cuff is on. When NOT using cuffs, torque control with custom gravity compensation. With cuffs, default gravity compensation.")
            else:
                print ("-- Custom gravity compensation.")

        if input_args.verbose:
            self._verbose = True
            print ("-- Verbose ON.")
        else:
            self._verbose = False
            print ("-- Verbose OFF.")
        

        # Init Subscriber
        rospy.Subscriber('robot/limb/' + limb + '/endpoint_state', EndpointState, self.callback_ee_state)
        rospy.Subscriber('robot/joint_states', JointState, self.callback_joint_state)        
        rospy.Subscriber('robot/limb/' + limb + '/gravity_compensation_torques', SEAJointState, self.callback_default_gc)
        rospy.Subscriber('robot/joint_limits', JointLimits, self.callback_limits)

        self.set_cuff(limb)
        self._get_robot_params()
        self.cntrl = impedance_ctrl(self._robot_chain, self._nrOfJoints)

# callbacks
    def callback_ee_state(self, data):
        self._ee_pose[0] = data.pose
        self._ee_twist[0] = data.twist
        self._ee_wrench[0] = data.wrench

    def callback_joint_state(self, data):
        if len(data.position) >= 7:
            for i in range(0, 7):
                self._joint_pos[i] = data.position[i]
                self._joint_vel[i] = data.velocity[i]
                self._joint_torque[i] = data.effort[i]

    def callback_default_gc(self, data):
        for i in range(0, 7):
            self._default_gc_torques[i] = data.gravity_only[i]	#	This is the torque required to hold the arm against gravity returned by KDL - SDK
            self._default_gc_model_torques[i] = data.gravity_model_effort[i]	#	This includes the inertial feed forward torques when applicable. - SDK
            
            self._default_joint_torques[i] = data.actual_effort[i]	# Joint torques feedback 
            
    def callback_limits(self, data):
        for i in range(0, 7):
            self._torque_limits[i] = data.effort[i]

    def set_cuff(self, limb): #TODO input args bearbeiten
        pass
   
    def disable_automatic_collision_detection(self, limb = 'right'): # TODO error beheben
        gc_ns = 'robot/limb/' + limb + '/suppress_gravity_compensation'
        self._pub_gc_disable = rospy.Publisher(gc_ns, Empty, queue_size=1)
        self._gcoff = True
        if self.input_args.defgc == "on" or not self._activategc:
            self._gcoff = False
            print ("-- Default GC ON.")
        else:
            print ("-- Default GC OFF.")

        # create publisher to switch off automatic collision detection
        coll_ns = 'robot/limb/' + limb + '/suppress_contact_safety'
        self._pub_coll_disable = rospy.Publisher(coll_ns, Empty, queue_size=1)
        self._colloff = True
        if input_args.collision == "on":
            self._colloff = False
            print ("-- Default collision avoidance ON.")
        else:
            print ("-- Default collision avoidance OFF.")

    def check_robot_state(self):
        # verify robot is enabled
        print(("Getting robot state... "))
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print(("Enabling robot... "))
        self._rs.enable()

    def check_robot_limits(self, cmd):
        for joint in self._limb.joint_names():
            # check limits
            if cmd[joint] > self._torque_limits[joint]:
                print ("Upper limit reached")
            elif cmd[joint] < -self._torque_limits[joint]:
                print ("Lower limit reached")
            self._gc_torques_msg.effort[joint] = cmd[joint]	# assign them to robot
    
    def _get_robot_params(self):
        (ok, robot) = urdf.treeFromFile('/home/nilssichert/ros_ws/src/sawyer_robot/sawyer_description/urdf/sawyer_base.urdf.xacro') # implement relative path
        self._robot_chain = robot.getChain('right_arm_base_link', 'right_l6')
        self._nrOfJoints = self._robot_chain.getNrOfJoints()
        self._jac_kdl = KDL.ChainJntToJacSolver(self._robot_chain)
        #self._jacdot_kdl = KDL.ChainJntToJacDotSolver(self._robot_chain) #TODO Lösung finden

        self.init_ee_pose = self._get_current_ee_pose()
    
    def _get_current_ee_pose(self):
        return self._ee_pose[0]

    def _get_current_ee_twist(self):
        return self._ee_twist[0]

    def _get_current_ee_wrench(self):
        return self._ee_wrench[0]

    def _get_current_joint_torques(self):
        return self._default_joint_torques

    def _update_forces(self):
        # disable cuff interaction
        if not self._cuffon:
            self._pub_cuff_disable.publish()

        ee_wrench = self._get_current_ee_wrench()
        curr_ee_pose = self._get_current_ee_pose()
        joint_torques = self._get_current_joint_torques()
        _jacdot_kdl = None
        cmd = self.cntrl.run(self._limb.joint_angles(), self._limb.joint_velocities(), self._limb.joint_efforts(), self._rate, self._jac_kdl, _jacdot_kdl, ee_wrench, curr_ee_pose, joint_torques)

        # publish the computed gc torques
        self._pub_gc_torques.publish(self._gc_torques_msg)

        
        # command new joint torques
        # if self._activategc:
        #     self._pub_gc_disable.publish()
        #     # if using cuff as switch to torque mode, when pressing the cuff, torques will be sent, otherwise, it is in position control
        #     if self._cuffswitch:
        #         if self._cuff.cuff_button():
        #             print("************HERE*****************")
        #             if self._verbose:
        #                 print ("Cuff button pressed.")
        #             self._pub_gc_disable.publish()
        #             if self._colloff:
        #                 self._pub_coll_disable.publish()
        #             self._limb.set_joint_torques(cmd)
        #         #else:
        #             #self._limb.exit_control_mode()
        #     # if not using the cuff as switch, send directly the torques
        #     else:
        # if self._colloff:
        #     self._pub_coll_disable.publish()
        self._limb.set_joint_torques(cmd)
    
    def _update_ee_state(self):
        new_ee_pose = self._get_current_ee_pose()
        wrist_angle = self._limb.joint_angle(self._joint_names[6])
        message_array = 'x: ' + str(new_ee_pose.position.x) + ' y: ' + str(new_ee_pose.position.y) + ' z: ' + str(new_ee_pose.position.z) + ' r: ' + str(wrist_angle)
        self._pub_robot_motion.publish(message_array)
               
    # def move_to_neutral(self):
    #         """
    #         Moves the limb to neutral location.
    #         """
    #         #self._limb.move_to_neutral()
    #         sawyer_neutral_pose = [-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981]
    #         new_limb_pose = {}
    #         i = 0
    #         for joint in self._limb.joint_names():
    #             new_limb_pose[joint] = sawyer_neutral_pose[i]
    #             i += 1
    #         self._limb.move_to_joint_positions(new_limb_pose)
    #         rospy.sleep(self._waitingtime)
    #         print (self._limb.joint_names())
    #         print ("######## Ready for next action. ########")

    def activate_controller(self):
            """
            Switches to joint torque mode and attached joint springs to current
            joint positions.
            """
            # record initial joint angles
            self._start_angles = self._limb.joint_angles()

            # set control rate
            control_rate = rospy.Rate(self._rate)

            # for safety purposes, set the control rate command timeout.
            # if the specified number of command cycles are missed, the robot
            # will timeout and return to Position Control Mode
            self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmd)

            # loop at specified rate commanding new joint torques
            while not rospy.is_shutdown():
                if not self._rs.state().enabled:
                    rospy.logerr("Joint torque failed to meet: specified control rate timeout.")
                    break
                if self._cuffswitch:
                    if self._cuff.lower_button():
                        break
                self._update_forces()
                self._update_ee_state()
                control_rate.sleep()

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()
    
def main():
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    rospy.init_node('sdk_impedance_ctrl_{}'.format(valid_limbs[0]))
    impedance_Ctrl = ImpedanceCtrl(limb=valid_limbs[0])
    rospy.on_shutdown(impedance_Ctrl.clean_shutdown)
    impedance_Ctrl.activate_controller()


if __name__ == '__main__':
    main()