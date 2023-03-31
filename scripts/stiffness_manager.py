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
        self.critical_velocity = rospy.get_param("stiffness_manager/critical_velocity", default=0.2)
        self.stiffness_calculation_method = rospy.get_param("stiffness_manager/calculation_method", default="pose")

        self._pub_Kd = rospy.Publisher('/control_node/Kd_Dd', JointState, queue_size=20)

        self._sub_media_pipe = rospy.Subscriber('/mediapipe/angle', Float32, self.callback_mediapipe_angle)
        self._sub_joint_velocity = rospy.Subscriber('/robot/joint_states',JointState, self.callback_joint_velocity)
        self.rate = 200 # updaterate
        self._angle = 0
        self._joint_velocity = np.zeros((7,1))
        self.counter = 0 # sec
        self.ratio = 0.7 # 0.7 = 70% of max. Kd and Dd
        self.timeout_stiff_joint = 2.0 # Time joint stays stiff bevor releasing stifness
        self.timeout = [0,0,0,0,0,0,0]
        self.velo_prev = [0,0,0,0,0,0,0]
        

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

    def callback_joint_velocity(self, data):
        self._joint_velocity = np.array(data.velocity[1:8])
    
    def update(self):
        """
        Update method coordinating the computation of new stiffness values.
        Parameters: None
        Return: None
        """
        
        while not rospy.is_shutdown():
            self.update_limits()
            if self.stiffness_calculation_method == "pose":
                self.counter = 0
                K_d, D_d = self.calculate_cv_stiffness()    
            elif self.stiffness_calculation_method == "velocity":
                self.counter = 0
                K_d, D_d = self.calculate_velocity_stiffness()
            elif self.stiffness_calculation_method == "ramp":
                K_d, D_d = self.calculate_increasing_stiffness()

            elif self.stiffness_calculation_method == "combined":
                self.counter = 0
                K_d, D_d = self.combine_stiffness(self.calculate_cv_stiffness(),self.calculate_velocity_stiffness())
            else:
                rospy.logerr_once("[Stiffness manager]: Wrong calculation method choosen (methods: pose/ velocity)")
                self.counter = 0
                K_d = self.K_d_upper_limit
                D_d = self.D_d_upper_limit
            self.publish_jointstate(K_d, D_d, self._pub_Kd)
            rospy.sleep(1/self.rate)

    def update_limits(self):
        """
        Updates limits from rospy parameter server.
        """
        self.K_d_upper_limit = rospy.get_param("control_node/Kd_upper_limit")
        self.D_d_upper_limit = rospy.get_param("control_node/Dd_upper_limit")
        self.K_d_lower_limit = rospy.get_param("control_node/Kd_lower_limit")
        self.D_d_lower_limit = rospy.get_param("control_node/Dd_lower_limit")
        self.critical_velocity = rospy.get_param("stiffness_manager/critical_velocity")
        self.offset_angle = rospy.get_param("stiffness_manager/offset_angle")
        self.critical_angle = rospy.get_param("stiffness_manager/critical_angle")
        self.stiffness_calculation_method = rospy.get_param("stiffness_manager/calculation_method", default="pose")
    
    def calculate_cv_stiffness(self):
        """
        Calculate the desired stiffness and damping based on linear interpolation between upper/lower limit with the leaning angle (input).
        Parameters: leaning angle of body (float)
        Return: stiffness (7x1), damping (7x1)
        """
        angle = self._angle
        offset = self.offset_angle # degrees
        critical_angle = self.critical_angle #degrees        

        delta_Kd = (np.array(self.K_d_upper_limit)-np.array(self.K_d_lower_limit))*self.ratio
        delta_Dd = (np.array(self.D_d_upper_limit)-np.array(self.D_d_lower_limit))*self.ratio

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
    
    def calculate_velocity_stiffness(self):
        velocities = self._joint_velocity
        delta_Kd = (np.array(self.K_d_upper_limit)-np.array(self.K_d_lower_limit))*self.ratio
        delta_Dd = (np.array(self.D_d_upper_limit)-np.array(self.D_d_lower_limit))*self.ratio
        Kd = np.zeros(len(velocities))
        Dd = np.zeros(len(velocities))
        for index, value in enumerate(velocities):
            abs_velo = np.abs(value)

            if self.timeout[index] < 0:
                self.timeout[index] = 0
            else: self.timeout[index] -= 1/self.rate

            if abs_velo <= self.critical_velocity and self.timeout[index]<=0:
                Kd[index] = self.K_d_lower_limit[index]
                Dd[index] = self.D_d_lower_limit[index]
                self.velo_prev[index] = abs_velo

            else:
                if abs_velo<=self.velo_prev[index]:
                    Kd[index] = ((self.velo_prev[index]-self.critical_velocity))*delta_Kd[index]+self.K_d_lower_limit[index]
                    Dd[index] = ((self.velo_prev[index]-self.critical_velocity))*delta_Dd[index]+self.D_d_lower_limit[index]
                
                else:
                    Kd[index] = ((abs_velo-self.critical_velocity))*delta_Kd[index]+self.K_d_lower_limit[index]
                    Dd[index] = ((abs_velo-self.critical_velocity))*delta_Dd[index]+self.D_d_lower_limit[index]   
                    self.timeout[index] = self.timeout_stiff_joint
                    self.velo_prev[index] = abs_velo              

                if abs_velo > self.velo_prev[index]:
                    pass

                if Kd[index] > self.K_d_upper_limit[index]:
                    Kd[index] = self.K_d_upper_limit[index]
                    Dd[index] = self.D_d_upper_limit[index]

        return Kd, Dd

    def calculate_increasing_stiffness(self):
        # Debug and oscillation guard test function. DO NOT use for a real stiffness calculation
        delta_Kd = np.array(self.K_d_upper_limit)-np.array(self.K_d_lower_limit)
        delta_Dd = np.array(self.D_d_upper_limit)-np.array(self.D_d_lower_limit)
        Kd = self.K_d_lower_limit + self.counter * delta_Kd/(self.rate*6)
        Dd = self.D_d_lower_limit + self.counter * delta_Dd/(self.rate*6)
        self.counter += 1
        return Kd, Dd

    def combine_stiffness(self, Kd_cv, Dd_cv, Kd_velocity, Dd_velocity):
        Kd = max(Kd_cv, Kd_velocity)
        Dd = max(Dd_cv, Dd_velocity)

        if Kd > self.K_d_upper_limit:
            Kd = self.K_d_upper_limit
            Dd= self.D_d_upper_limit
        return Kd, Dd

if __name__ == "__main__":
    mng = Stiffness_mangager()
    mng.update()