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

import rospy
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointLimits, EndpointState
from impedance_ctrl_cartesian import imedance_ctrl

import intera_interface
from intera_interface import CHECK_VERSION


# Input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--mode', help = 'Control mode: [gc, pos]')
parser.add_argument('--cuff', help = 'Enable or disable cuff with [on, off], when on, default GC is enabled when squeezing the cuff.')	#cuffs off enable gc - required is: when off  default gc enabled
parser.add_argument('--defgc', help = 'Disable default gravity compensation [on, off]')
parser.add_argument('--cuffswitch', help = 'Use button on the cuff to switch control mode.')	#The control mode is the impedance mode - cuffswitch is ON for impedance control to be activated
parser.add_argument('--collision', help = 'Switch off the automatic collision detection.')
parser.add_argument('--waiting', help = 'Waiting time before switching to torque control.')

_ee_pose = [Pose()]

_joint_pos = [0] * 7
_joint_vel = [0] * 7
_joint_torque = [0] * 7

_torque_limits = [0] * 7


def callback_ee_state(data):
    _ee_pose[0] = data.pose

def callback_joint_state(data):
    if len(data.position) >= 7:
        for i in range(0, 7):
            _joint_pos[i] = data.position[i]
            _joint_vel[i] = data.velocity[i]
            _joint_torque[i] = data.effort[i]
        
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
        self._rate = 100.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout
        print ("-- Control rate: " + str(self._rate))

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

        
        print ("-- Reset to neutral ON.")
        self.move_to_neutral()

        (ok, robot) = urdf.treeFromFile('sawyer_robot/sawyer_description/urdf/sawyer_base.urdf.xacro')
        self._robot_chain = robot.getChain('right_arm_base_link', 'right_l6')
        print("Running. Ctrl-c to quit")
        self._nrOfJoints = self._robot_chain.getNrOfJoints()
        self.joint_angles = KDL.JntArray(self._nrOfJoints)
        self.joint_velocities = KDL.JntArray(self._nrOfJoints)
        self.controler = imedance_ctrl(self._robot_chain)
        self.kin = sawyer_kinematics('right')

    def _get_current_ee_pose(self):
        return _ee_pose[0]
        
    def _update_forces(self, joint_desi):
        
        """
        Calculates the torques
        """

        # disable cuff interaction
        if not self._cuffon:
            self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()

        # change data to KDL format
        i = 0
        for joint in self._limb.joint_names():
            self.joint_angles[i] = cur_pos[joint]
            self.joint_velocities[i] = cur_vel[joint]
            i += 1
        KDL.JntArray(self._nrOfJoints)
        pose = self.kin.forward_position_kinematics(self.joint_angles)
        pose_desi = self.kin.forward_position_kinematics(joint_desi)

        cmd = self.controler.run_impedance_controll(self.joint_angles, self.joint_velocities, pose, pose_desi, self._rate)
        # TODO add limiter

        # command new joint torques
        self._pub_coll_disable.publish()
        self._limb.set_joint_torques(cmd)
        

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

        goal_joint_angle = [-2.4, -0.1, -1.6, -2.2, -3, -0.2,  0.11]
        while not rospy.is_shutdown():
            # compute torques
            self._update_forces(goal_joint_angle)
            print("time: ", (1/self._rate) * counter)
            counter += 1
            control_rate.sleep()

                    
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


if __name__ == "__main__":
    main()


