#!/usr/bin/env python3
import rospy

class Configuration_server():
    def __init__(self):
        """
        Configuration server class gets and sets rosparameter from parameter server.
        """
        pass

    ######## Getter Methods ########

    def get_control_flag(self):
        """
        Flag if controlloop is turned on (true) or off (false). If turned off internal position controller activ
        and holding last position.
        """
        return rospy.get_param("control_node/control_is_on")
        
    def get_joint_angle_desired(self):
        """
        Desired joint angle configuration (7x1)
        """
        return rospy.get_param("control_node/joint_angle_desired")
    
    def get_joint_velocity_desired(self):
        """
        Desired joint velocity configuration (7x1)
        """
        return rospy.get_param("control_node/joint_velocity_desired")
    
    def get_Statemachine_condition(self):
        """
        State number of statemachine (int).
        """
        return rospy.get_param("control_node/selected_filter_number")
    
    def get_stiffness(self):
        """
        Stiffness of joints or EE (depending on choosen controller).
        Allowed inputs [K], [K,K,K,K,K,K,K]
        """
        return rospy.get_param("control_node/Kd")
    
    def get_damping(self):
        """
        Damping of joints or EE (depending on choosen controller).
        Allowed inputs [D], [D,D,D,D,D,D,D]
        """
        return rospy.get_param("control_node/Dd")
    
    def get_neutral_pose(self):
        """
        Define neutral pose as list of joint angles.
        """
        return rospy.get_param("named_poses/right/poses/neutral")
    
    def get_lowpass_coeff(self):
        """
        Weighting coefficient of Lowpassfilter.
        """
        return rospy.get_param("control_node/Lowpass_coeff")
    
    def get_move2neutral(self):
        """
        Flag triggering the robot moving to neutral pose, if true movement initalized.
        """
        return rospy.get_param("/control_node/move2neutral")
    
    def get_oscillation_window_len(self):
        """
        Oscillation obeserver window lenght.
        """
        return rospy.get_param("oscillation_guard/window_length")
    
    def get_oscillation_corner_freq(self):
        """
        Oscillation corner frequency for observing the magnitude higher as the set frequency.
        """
        return rospy.get_param("oscillation_guard/corner_frequency")
        
    def get_oscillation_power_limit(self):   
        """
        Shutdown magnitude to turn of controller.
        """ 
        return rospy.get_param("oscillation_guard/magnitude_limit")
    
    def get_cartesian_pose_desired(self):
        """
        Desired cartesian pose.
        """
        return rospy.get_param("control_node/cartesian_pose_desired")

    def get_nullspace_is_locked(self):
        """
        Flag locking the nullspace (true), or enabling the nullspace (false).
        """
        return rospy.get_param("nullspace/is_locked")
    
    def get_nullspace_pose(self):
        """
        Desired nullspace pose in jointspace.
        """
        return rospy.get_param("nullspace/jointspace_pose")

    def get_self_collision_is_disabled(self):
        """
        Flag to enable (false) or disable (true) self collision avoidance
        """
        return rospy.get_param("control_node/suppress_self_collision_detection")

    def get_contact_collision_disabled(self):
        """
        Flag to enable (false) or disable (true) contact collision avoidance
        """
        return rospy.get_param("control_node/suppress_contact_collision_detection")
    

    ######## Setter Methods ########

    def set_control_flag(self, controlFlag):
        return rospy.set_param("control_node/control_is_on", controlFlag)
        
    def set_joint_angle_desired(self, joint_angle_desired):
        return rospy.set_param("control_node/joint_angle_desired", joint_angle_desired)
    
    def set_joint_velocity_desired(self, joint_velocity_desired):
        return rospy.set_param("control_node/joint_velocity_desired", joint_velocity_desired)
    
    def set_Statemachine_condition(self, statemachine_condition):
        return rospy.set_param("control_node/selected_filter_number", statemachine_condition)
    
    def set_stiffness(self, joint_stiffness):
        return rospy.set_param("control_node/Kd", joint_stiffness)
        
    def set_damping(self, joint_damping):
        return rospy.set_param("control_node/Dd", joint_damping)
    
    def set_neutral_pose(self, neutral_pose):
        return rospy.set_param("named_poses/right/poses/neutral", neutral_pose)
    
    def set_lowpass_coeff(self, coeff):
        return rospy.set_param("control_node/Lowpass_coeff", coeff)

    def set_move2neutral(self, Flag):
        return rospy.set_param("control_node/move2neutral", Flag)
    
    def set_oscillation_window_len(self, length):
        return rospy.set_param("oscillation_guard/window_length", length)
    
    def set_oscillation_corner_freq(self, frequency):
        return rospy.set_param("oscillation_guard/magnitude_limit", frequency)
        
    def set_oscillation_power_limit(self, limit):    
        return rospy.set_param("oscillation_guard/magnitude_limit", limit)
    
    def set_cartesian_pose_desired(self, pose):
        return rospy.set_param("control_node/cartesian_pose_desired", pose)

    
def main():
    settings = Configuration_server()
    print("Finished.")

if __name__ == "__main__":
    main()