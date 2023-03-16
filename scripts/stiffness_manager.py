#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import numpy as np
import math

class Stiffness_mangager():
    """
    Publish stiffness and damping values within an upper/lower limit boundary based on desicion methods.    
    """
    def __init__(self):
        rospy.init_node("Stiffness_manager", anonymous=True)
        self.K_d_upper_limit = rospy.get_param("control_node/Kd_upper_limit")
        self.D_d_upper_limit = rospy.get_param("control_node/Dd_upper_limit")
        self.K_d_lower_limit = rospy.get_param("control_node/Kd_lower_limit")
        self.D_d_lower_limit = rospy.get_param("control_node/Dd_lower_limit")

        self._pub_Kd = rospy.Publisher('/control_node/Kd_Dd', JointState, queue_size=1)

        self._sub_media_pipe = rospy.Subscriber('/mediapipe/angle', Float32, self.callback_mediapipe_angle)
        self.rate = 100 # updaterate
        self._angle = 0

    def publish_jointstate(self, position, velocity, publisher: rospy.Publisher):
        """
        Publisher of stiffness and damping.
        Parameters: stiffness values (position: 7x1), damping values (velocity: 7x1)
        Return: sensor_msg
        """
        msg = JointState()
        msg.position = position # Kd
        msg.velocity = velocity # Dd
        publisher.publish(msg)

    def callback_mediapipe_angle(self, data):
        self._angle = data.data
    
    def update(self):
        """
        Update method coordinating the computation of new stiffness values.
        Parameters: None
        Return: None
        """
        i = 0
        while not rospy.is_shutdown():
            self.update_limits()
            angle = math.sin(i/500)*35 # 35 degree to each side
            K_d, D_d = self.calculate(np.abs(angle))
            #K_d, D_d = self.calculate(np.abs(self._angle))
            self.publish_jointstate(K_d, D_d, self._pub_Kd)
            i += 1
            rospy.sleep(1/self.rate)

    def update_limits(self):
        """
        Updates limits from rospy parameter server.
        """
        self.K_d_upper_limit = rospy.get_param("control_node/Kd_upper_limit")
        self.D_d_upper_limit = rospy.get_param("control_node/Dd_upper_limit")
        self.K_d_lower_limit = rospy.get_param("control_node/Kd_lower_limit")
        self.D_d_lower_limit = rospy.get_param("control_node/Dd_lower_limit")
    
    def calculate(self, angle=0):
        """
        Calculate the desired stiffness and damping based on linear interpolation between upper/lower limit with the leaning angle (input).
        Parameters: leaning angle of body (float)
        Return: stiffness (7x1), damping (7x1)
        """
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