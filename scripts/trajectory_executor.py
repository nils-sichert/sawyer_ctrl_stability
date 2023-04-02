#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
import csv
import os

class trajectoy_executer():
    """
    Trajactory are executed either from file with joint position (resolution 400Hz), 
    or from static pose based on the desired pose of the ros parameter server.
    """
    def __init__(self) -> None:
        rospy.init_node("trajectory_publisher")
        
        self._pub_joint_angle_desi = rospy.Publisher("/control_node/joint_states_desi", JointState, queue_size=1)
        self._pub_cartesian_pose = rospy.Publisher('/control_node/cartesian_pose_desi', JointState, queue_size=1)
        self.rate = 400 #Hz
        self.counter = 0
        self.joint_angle = []
        self.cartesian_coordinates = []
        tmp = os.path.dirname(__file__)
        
        with open(os.path.join(tmp, 'sawyer_ctrl_stability/trajectory/square_traj_joint.csv')) as f:
            reader = csv.reader(f,delimiter=',')
            for row in reader:
                self.joint_angle.append(row)
        
        with open(os.path.join(tmp, 'sawyer_ctrl_stability/trajectory/square_traj_cart.csv')) as f:
            reader = csv.reader(f,delimiter=',')
            for row in reader:
                self.cartesian_coordinates.append(row)
        
        self.numberOfCommands = len(self.joint_angle)

    def publish_jointstates(self, position, velocity, publisher: rospy.Publisher):
        """
        Publisher of desired joint angles and desired joint velocities
        """
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = position
        msg.velocity = velocity
        publisher.publish(msg)

    def get_joint_state_desired(self):
        """
        Getter of desired joint anlges and desired joint velocities
        """  
        position = rospy.get_param("control_node/joint_angle_desired")
        velocity = rospy.get_param("control_node/joint_velocity_desired")
        return position, velocity
            
    def get_traj(self):
        if self.counter >= self.numberOfCommands:
            self.counter = 0
        
        cart = [float(x) for x in (self.cartesian_coordinates[self.counter])]
        joint = [float(x) for x in (self.joint_angle[self.counter])]
        self.counter += 1
        return cart, joint


    def publish(self):
        """
        Main method coordinating the publishing and getting the new desired pose.
        """
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            trajectory_method = rospy.get_param("trajectory_executor/method")
            if trajectory_method == "path":
                #path = rospy.get_param("trajectory_executer/path")
                rospy.logwarn_once("[{}]: Executing path".format(rospy.get_name()))
                cart, joint = self.get_traj()
                velocity = [0,0,0,0,0,0,0]
                self.publish_jointstates(cart, velocity, self._pub_cartesian_pose)
                self.publish_jointstates(joint, velocity, self._pub_joint_angle_desi)

            else:
                position, velocity = self.get_joint_state_desired()
                rospy.logwarn_once("[{}]: Holding position (to execute path change to: path)".format(rospy.get_name()))
                self.publish_jointstates(position, velocity, self._pub_joint_angle_desi)
            r.sleep()
    
if __name__ =="__main__":
    trajectory_ex = trajectoy_executer()
    trajectory_ex.publish()

    
    
    
        
