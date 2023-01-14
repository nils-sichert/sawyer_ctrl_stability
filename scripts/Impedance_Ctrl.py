#!/usr/bin/env python3

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Impedance Control for Interactio
python3 Impedance_Ctrl.py --mode gc --cuff off --resetneutral on --defgc off --cuffswitch on --collision off 

Impedance Control with no Interaction
python3 Impedance_Ctrl.py --mode gc --cuff off --resetneutral on --defgc off --cuffswitch off --collision off 
"""

import argparse
import importlib
import PyKDL as KDL
from kdl_parser_py import urdf
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime as dt

import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Empty, String, Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointLimits, EndpointState, SEAJointState

import intera_interface
from intera_interface import CHECK_VERSION

import csv


# Input arguments
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

_ee_pose = [Pose()]
_ee_twist = [Twist()]
_ee_wrench = [Wrench()]

_joint_pos = [0] * 7
_joint_vel = [0] * 7
_joint_torque = [0] * 7

_default_gc_torques = [0] * 7
_default_gc_model_torques = [0] * 7
_torque_limits = [0] * 7

_default_joint_torques = [0] * 7

def callback_ee_state(data):
    _ee_pose[0] = data.pose
    _ee_twist[0] = data.twist
    _ee_wrench[0] = data.wrench

def callback_joint_state(data):
    if len(data.position) >= 7:
        for i in range(0, 7):
            _joint_pos[i] = data.position[i]
            _joint_vel[i] = data.velocity[i]
            _joint_torque[i] = data.effort[i]

def callback_default_gc(data):
    for i in range(0, 7):
        _default_gc_torques[i] = data.gravity_only[i]	#	This is the torque required to hold the arm against gravity returned by KDL - SDK
        _default_gc_model_torques[i] = data.gravity_model_effort[i]	#	This includes the inertial feed forward torques when applicable. - SDK
        
        _default_joint_torques[i] = data.actual_effort[i]	# Joint torques feedback 
        
def callback_limits(data):
    for i in range(0, 7):
        _torque_limits[i] = data.effort[i]

class GCController(object):

    """
    @param limb: limb on which to run joint springs example, default is "right"
    """
    def __init__(self, limb = "right"):

        input_args = parser.parse_args()
        print ("Executing with: ")

        # control parameters
        if input_args.rate:
            self._rate = float(input_args.rate)
        else:
            self._rate = 100.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout
        print ("-- Control rate: " + str(self._rate))

        if input_args.waiting:
            self._waitingtime = int(input_args.waiting)
        else:
            self._waitingtime = 2.0  # sec
        print ("-- Waiting time: " + str(self._waitingtime))

        # create our limb instance
        self._limb = intera_interface.Limb(limb)
        self._joint_names = self._limb.joint_names()

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

        # robot motion data publisher
        self._pub_robot_motion = rospy.Publisher('robot_motion_sender', String, queue_size=10)
        
        # robot gc torques publisher
        self._pub_gc_torques = rospy.Publisher('computed_gc_torques_sender', JointState, queue_size=10)
        self._gc_torques_msg = JointState()
        self._gc_torques_msg.effort = [0] * 7
        
        # robot ee state subscriber
        rospy.Subscriber('robot/limb/' + limb + '/endpoint_state', EndpointState, callback_ee_state)
        
        # robot joint state subscriber
        rospy.Subscriber('robot/joint_states', JointState, callback_joint_state)        
        
        # robot sea state subscriber
        rospy.Subscriber('robot/limb/' + limb + '/gravity_compensation_torques', SEAJointState, callback_default_gc)

        # create publisher to disable default gravity compensation
        gc_ns = 'robot/limb/' + limb + '/suppress_gravity_compensation'
        self._pub_gc_disable = rospy.Publisher(gc_ns, Empty, queue_size=1)
        self._gcoff = True
        if input_args.defgc == "on" or not self._activategc:
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


        # verify robot is enabled
        print(("Getting robot state... "))
        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print(("Enabling robot... "))
        self._rs.enable()

        # get the robot limits
        rospy.Subscriber('robot/joint_limits', JointLimits, callback_limits)

        if input_args.resetneutral:
            if input_args.resetneutral == "on":
                print ("-- Reset to neutral ON.")
                self.move_to_neutral()
            else:
                print ("-- Reset to neutral OFF.")
        else:
            print ("-- Reset to neutral ON.")
            self.move_to_neutral()

        (ok, robot) = urdf.treeFromFile('sawyer_robot/sawyer_description/urdf/sawyer.urdf.xacro')
        self._robot_chain = robot.getChain('right_arm_base_link', 'right_l6')
        self._nrOfJoints = self._robot_chain.getNrOfJoints()
        self._jac_kdl = KDL.ChainJntToJacSolver(self._robot_chain)
        self._jacdot_kdl = KDL.ChainJntToJacDotSolver(self._robot_chain)

        self.init_ee_pose = self._get_current_ee_pose()
        self.prev_x_dot = [0] * 3
        self.xd = []
        
        self.joint_angles_data = []
        self.joint_efforts_data = []
        self.ee_pose_data = []
        self.Fa_data = []
                            
        print("Running. Ctrl-c to quit")

    def _get_current_ee_pose(self):
        return _ee_pose[0]

    def _get_current_ee_twist(self):
        return _ee_twist[0]

    def _get_current_ee_wrench(self):
        return _ee_wrench[0]

    def _get_current_joint_torques(self):
        return _default_joint_torques
        
    def _update_forces(self):
        
        """
        Calculates the torques
        """

        # disable cuff interaction
        if not self._cuffon:
            self._pub_cuff_disable.publish()

	# declaring/allocating necessary quantities
        grav_vector = KDL.Vector(0, 0, -9.81)
        dyn_kdl = KDL.ChainDynParam(self._robot_chain, grav_vector)
        joint_angles = KDL.JntArray(self._nrOfJoints)
        joint_velocities = KDL.JntArray(self._nrOfJoints)
        joint_efforts = KDL.JntArray(self._nrOfJoints)
        grav_torques = KDL.JntArray(self._nrOfJoints)
        coriolis_torques = KDL.JntArray(self._nrOfJoints)
        B_kdl = KDL.JntSpaceInertiaMatrix(self._nrOfJoints)
        j_kdl = KDL.Jacobian(self._nrOfJoints)
        jdot_kdl = KDL.Jacobian(self._nrOfJoints)

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()
        cur_eff = self._limb.joint_efforts()

        # change data to KDL format
        i = 0
        for joint in self._limb.joint_names():
            joint_angles[i] = cur_pos[joint]
            joint_velocities[i] = cur_vel[joint]
            joint_efforts[i] = cur_eff[joint]
            i += 1
         
        # compute torques
        dyn_kdl.JntToGravity(joint_angles, grav_torques)	# g(q)
        dyn_kdl.JntToCoriolis(joint_angles, joint_velocities, coriolis_torques)	# C(q, q_dot) * q_dot
        dyn_kdl.JntToMass(joint_angles, B_kdl)

        #compute Jacobian and Jacobian dot
        q_dqdt = KDL.JntArray(joint_angles)
        for i in range(joint_angles.rows()):
            q_dqdt[i] +=  self._rate * joint_velocities[i]

        self._jac_kdl.JntToJac(joint_angles, j_kdl)	# Jacobian
        self._jacdot_kdl.JntToJacDot(KDL.JntArrayVel(q_dqdt, joint_velocities), jdot_kdl)	# J_dot
        
        # taken from KDL example
        jdot_qdot = KDL.Twist()
        KDL.MultiplyJacobian(jdot_kdl, joint_velocities, jdot_qdot)	# J_dot * q_dot

        inertia_torques = [0] * self._nrOfJoints
        interaction_torques = [0]* self._nrOfJoints

        ee_wrench = self._get_current_ee_wrench()
        
        j_kdl_transpose = [[0 for _ in range(6)] for _ in range(7)]
        for i in range(0, 6):
            for j in range (0, self._nrOfJoints):
                j_kdl_transpose[j][i] = j_kdl[i, j]	# J_transpose

        # computing pseudo-invers of the jacobian transpose
        #j_kdl_transpose_inv = np.linalg.pinv(j_kdl_transpose)
        #joint_torques = _get_current_joint_torques()
        
        #for i in range(0, self._nrOfJoints):
            #for j in range (0, 6):
                #interaction_torques[i] += j_kdl_transpose_inv[i][j] * joint_torques[j]        
                
        # EE forces are estimated from Joint torques - NEEDS FILTERING
        #Fa = [0]*6
        #Fa[0] = ee_wrench.force.x
        #Fa[1] = ee_wrench.force.y
        #Fa[2] = ee_wrench.force.z
        #Fa[3] = ee_wrench.torque.x
        #Fa[3] = ee_wrench.torque.y
        #Fa[3] = ee_wrench.torque.z

        #for i in range(0, self._nrOfJoints):
            #for j in range (0, 6):
                #interaction_torques[i] += j_kdl_transpose[i][j] * Fa[j]	# J_transpose * Fa = joint torques (the same as the one measured from the robot)

        #################### Compute u, the control input ####################
        # reformatting j_kdl to numpy format
        j_numpy = [[0 for _ in range(7)] for _ in range(6)]
        for i in range(0, 6):
            for j in range (0, 7):
                j_numpy[i][j] = j_kdl[i, j]

        # computing pseudo-invers of the jacobian transpose
        j_kdl_inv = np.linalg.pinv(j_numpy)
        
        

        # Define impedance controller parameters
        M = np.identity(3) # mass matrix in the mass-spring-damper system
        Kd = 20 * np.identity(3)
        Kp = 10 * np.identity(3)

        # Define desired end effector pose
        self.xd = [self.init_ee_pose.position.x + 0.5, self.init_ee_pose.position.y, self.init_ee_pose.position.z]	# xd
        xd_dot = [0, 0, 0]	# xd_dot
        xd_dot_dot = [0, 0, 0]	# xd_dot
        
        x_dot = [0] * 3
        for i in range(0, 3):
            for j in range (0, self._nrOfJoints):
                x_dot[i] += j_kdl[i, j] * joint_velocities[j]	# x_dot = J * q_dot

        curr_ee_pose = self._get_current_ee_pose()	# x
        
        # Caclulate Error
        x_tilde = x_tilde_dot = [0] * 3
         
        for i in range(0, 3):
            x_tilde_dot[i] = xd_dot[i] - x_dot[i]	# x_tilde_dot
            
        i = 0
        x_tilde[i] = self.xd[i] - curr_ee_pose.position.x
        i += 1
        x_tilde[i] = self.xd[i] - curr_ee_pose.position.y
        i += 1
        x_tilde[i] = self.xd[i] - curr_ee_pose.position.z
        i += 1
							# x_tilde ^^^
        
        # compute x_dot_dot by numerical differentiation
        #x_dot_dot = [0] * 3
        #for i in range(0, 3):
            #x_dot_dot[i] = (x_dot[i] - self.prev_x_dot[i]) /self._rate	# x_dot_dot

        #self.prev_x_dot = x_dot
        
        M_jdot_qdot = [0] * 3
        for i in range(0, 3):
            for j in range (0, 3):
                M_jdot_qdot[i] += M[i][j] * jdot_qdot[j]	# M * J_dot * q_dot

 
        JM_inv = [[0 for _ in range(3)] for _ in range(7)]
        for i in range(0, 7):
            for j in range (0, 3):
                for k in range (0, 3):
                     JM_inv[i][j] += j_kdl_inv[i][k] * M[k, j]	# J_inv * M_inv
        
        u = np.matmul(JM_inv, np.matmul(M, xd_dot_dot) + np.matmul(Kd, x_tilde_dot) + np.matmul(Kp, x_tilde) - M_jdot_qdot)#- Fa[0:3])

        for i in range(0, self._nrOfJoints):
            for j in range (0, self._nrOfJoints):
                inertia_torques[i] += B_kdl[i, j] * u[j]	# B * u

        # assign to torques commands
        i = 0
        for joint in self._limb.joint_names():
            cmd[joint] = inertia_torques[i] + coriolis_torques[i] + grav_torques[i]# + interaction_torques[i]

            # check limits
            if cmd[joint] > _torque_limits[i]:
                print ("Upper limit reached")
            elif cmd[joint] < -_torque_limits[i]:
                print ("Lower limit reached")
            self._gc_torques_msg.effort[i] = cmd[joint]	# assign them to robot
            i += 1
        
        # publish the computed gc torques
        self._pub_gc_torques.publish(self._gc_torques_msg)

        if self._verbose:
            print ("GC torques: ")
            print (cmd)
            print ("Default GC torques: ")
            print (_default_gc_torques)
            print ("Default GC model torques: ")
            print (_default_gc_model_torques)
            print ("Current torques: ")
            print (self._limb.joint_efforts())

        
        # command new joint torques
        if self._activategc:
            self._pub_gc_disable.publish()
            # if using cuff as switch to torque mode, when pressing the cuff, torques will be sent, otherwise, it is in position control
            if self._cuffswitch:
                if self._cuff.cuff_button():
                    print("************HERE*****************")
                    if self._verbose:
                        print ("Cuff button pressed.")
                    self._pub_gc_disable.publish()
                    if self._colloff:
                        self._pub_coll_disable.publish()
                    self._limb.set_joint_torques(cmd)
                #else:
                    #self._limb.exit_control_mode()
            # if not using the cuff as switch, send directly the torques
            else:
                if self._colloff:
                    self._pub_coll_disable.publish()
                #self._limb.set_joint_torques(cmd)
                
    def _update_ee_state(self):
        new_ee_pose = self._get_current_ee_pose()
        wrist_angle = self._limb.joint_angle(self._joint_names[6])
        message_array = 'x: ' + str(new_ee_pose.position.x) + ' y: ' + str(new_ee_pose.position.y) + ' z: ' + str(new_ee_pose.position.z) + ' r: ' + str(wrist_angle)
        self._pub_robot_motion.publish(message_array)

    def _print_robot_state(self):
        print (self._get_current_ee_pose())
        print (self._get_current_ee_wrench())

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        #self._limb.move_to_neutral()
        sawyer_neutral_pose = [-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981]
        new_limb_pose = {}
        i = 0
        for joint in self._limb.joint_names():
            new_limb_pose[joint] = sawyer_neutral_pose[i]
            i += 1
        self._limb.move_to_joint_positions(new_limb_pose)
        rospy.sleep(self._waitingtime)
        print (self._limb.joint_names())
        print ("######## Ready for next action. ########")

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
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint torques
        counter = 0
        while not rospy.is_shutdown():
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break
            if self._cuffswitch:
                if self._cuff.lower_button():
                    break
                     
            # compute torques
            self._update_forces()
            self._update_ee_state()
            self._dump_data()
            print("time: ", (1/self._rate) * counter)
            counter += 1
            control_rate.sleep()

    def _dump_data(self):

        wrench = [0] * 6
        wrench[0] = _ee_wrench[0].force.x
        wrench[1] = _ee_wrench[0].force.y
        wrench[2] = _ee_wrench[0].force.z
        wrench[3] = _ee_wrench[0].torque.x
        wrench[4] = _ee_wrench[0].torque.y
        wrench[5] = _ee_wrench[0].torque.z
        
        pose = [0] * 7
        pose[0] = _ee_pose[0].position.x
        pose[1] = _ee_pose[0].position.y
        pose[2] = _ee_pose[0].position.z
        pose[3] = _ee_pose[0].orientation.x
        pose[4] = _ee_pose[0].orientation.y
        pose[5] = _ee_pose[0].orientation.z
        pose[6] = _ee_pose[0].orientation.w
        
        self._write_data(open('/home/nourhan/ros_ws/src/sawyer_test/src/imFa.csv', "a"), wrench)
        self._write_data(open('/home/nourhan/ros_ws/src/sawyer_test/src/imEEpose.csv', "a"), pose)
        self._write_data(open('/home/nourhan/ros_ws/src/sawyer_test/src/imJointAngles.csv', "a"), _joint_pos)
        self._write_data(open('/home/nourhan/ros_ws/src/sawyer_test/src/imJointVelocities.csv', "a"), _joint_vel)
        self._write_data(open('/home/nourhan/ros_ws/src/sawyer_test/src/imJointTorques.csv', "a"), _joint_torque)
        
    def _write_data(self, fileobj, data):
        for datum in data:
            fileobj.write("%f, " % datum)
        fileobj.write("\n")
        fileobj.close()
                    
    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()


def main():
    # Querying the parameter server to determine Robot model and limb name(s)
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    # Starting node connection to ROS
    print("Initializing node... ")
    rospy.init_node("sdk_joint_torque_springs_{0}".format(valid_limbs[0]))

    gcc = GCController(limb = valid_limbs[0])
    # register shutdown callback
    rospy.on_shutdown(gcc.clean_shutdown)
    #js.move_to_neutral()
    gcc.activate_controller()
    print ("xd = ", gcc.xd)


if __name__ == "__main__":
    main()




