#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import math

class Stiffness_mangager():
    def __init__(self):
        rospy.init_node("Stiffness_manager", anonymous=True)
        self.K_d_upper_limit = rospy.get_param("control_node/Kd_upper_limit")
        self.D_d_upper_limit = rospy.get_param("control_node/Dd_upper_limit")
        self.K_d_lower_limit = rospy.get_param("control_node/Kd_lower_limit")
        self.D_d_lower_limit = rospy.get_param("control_node/Dd_lower_limit")

        self._pub_Kd = rospy.Publisher('/control_node/Kd_Dd', JointState, queue_size=1)
        self.rate = 100 # updaterate

    def publish_jointstate(self, position, velocity, publisher: rospy.Publisher):
        msg = JointState()
        msg.position = position # Kd
        msg.velocity = velocity # Dd
        publisher.publish(msg)
    
    def update(self):
        i = 0
        while not rospy.is_shutdown():
            self.update_limits()
            angle = math.sin(i/500)*35 # 35 degree to each side
            K_d, D_d = self.calculate(np.abs(angle))
            self.publish_jointstate(K_d, D_d, self._pub_Kd)
            i += 1
            rospy.sleep(1/self.rate)

    def update_limits(self):
        self.K_d_upper_limit = rospy.get_param("control_node/Kd_upper_limit")
        self.D_d_upper_limit = rospy.get_param("control_node/Dd_upper_limit")
        self.K_d_lower_limit = rospy.get_param("control_node/Kd_lower_limit")
        self.D_d_lower_limit = rospy.get_param("control_node/Dd_lower_limit")
    
    def calculate(self, angle=0):
        offset = 5 # degrees
        critical_angle = 30 #degrees        

        delta_Kd = np.array(self.K_d_upper_limit)-np.array(self.K_d_lower_limit)
        delta_Dd = np.array(self.D_d_upper_limit)-np.array(self.D_d_lower_limit)

        if angle <= offset:
            Kd = self.K_d_lower_limit
            Dd = self.D_d_lower_limit
        
        elif angle > offset and angle <= critical_angle:
            Kd = ((angle-offset)/(critical_angle-offset))*delta_Kd+self.K_d_lower_limit
            Dd = ((angle-offset)/(critical_angle-offset))*delta_Dd+self.D_d_lower_limit
    
        else:
            Kd = self.K_d_upper_limit
            Dd = self.D_d_upper_limit
        return Kd, Dd

if __name__ == "__main__":
    mng = Stiffness_mangager()
    mng.update()