#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState

class trajectoy_executer():
    """
    Trajactory are executed either from file with joint position (resolution 400Hz), 
    or from static pose based on the desired pose of the ros parameter server.
    """
    def __init__(self) -> None:
        rospy.init_node("trajectory_publisher")
        
        self._pub_joint_angle_desi = rospy.Publisher("/control_node/joint_states_desi", JointState, queue_size=10)
        self.rate = 400 #Hz
        self.counter = 0

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
            
    def publish(self):
        """
        Main method coordinating the publishing and getting the new desired pose.
        """
        r = rospy.Rate(self.rate)
        position, velocity = self.get_joint_state_desired()

        while not rospy.is_shutdown():
             self.publish_jointstates(position, velocity, self._pub_joint_angle_desi)
             self.counter += 1
             if self.counter >= self.rate/2:
                  position, velocity = self.get_joint_state_desired()
             r.sleep()
    
if __name__ =="__main__":
    trajectory_ex = trajectoy_executer()
    trajectory_ex.publish()

    
    
    
        
