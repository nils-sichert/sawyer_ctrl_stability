#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState

class trajectoy_executer():
    def __init__(self) -> None:
        rospy.init_node("trajectory_publisher")
        
        self._pub_joint_angle_desi = rospy.Publisher("/control_node/joint_angle_desi", JointState, queue_size=10)
        self.rate = 100 #Hz
        self.counter = 0

    def publish_jointstates(self, position, velocity, publisher: rospy.Publisher):
        msg = JointState()
        msg.position = position
        msg.velocity = velocity
        publisher.publish(msg)

    def get_joint_state_desired(self):
            position = rospy.get_param("control_node/joint_angle_desired")
            velocity = rospy.get_param("control_node/joint_velocity_desired")
            return position, velocity
            
    def publish(self):
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

    
    
    
        
