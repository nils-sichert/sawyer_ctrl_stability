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

    def publish_jointstates(self, position, velocity, publisher: rospy.Publisher):
        """
        Publisher of desired joint angles and desired joint velocities
        """
        msg = JointState()
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
            
    def publish_traj(self):
        r = rospy.Rate(self.rate)
        for j, c in zip(self.joint_angle, self.cartesian_coordinates):
            joint_position = [float(x) for x in j]
            joint_velocity = [0,0,0,0,0,0,0]

            cartesian_pose = [float(x) for x in c]
            cartesian_velocity = [0,0,0,0,0,0,0]

            self.publish_jointstates(joint_position, joint_velocity, self._pub_joint_angle_desi)
            self.publish_jointstates(cartesian_pose, cartesian_velocity, self._pub_cartesian_pose)
            r.sleep()


    def publish(self):
        """
        Main method coordinating the publishing and getting the new desired pose.
        """
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            trajectory_method = rospy.get_param("trajectory_executor/method")
            if trajectory_method == "path":
                #path = rospy.get_param("trajectory_executer/path")
                rospy.logwarn("[Traj. executer]: Executing path")
                self.publish_traj()

            else:
                position, velocity = self.get_joint_state_desired()
                self.publish_jointstates(position, velocity, self._pub_joint_angle_desi)
            r.sleep()
    
if __name__ =="__main__":
    trajectory_ex = trajectoy_executer()
    trajectory_ex.publish()

    
    
    
        
