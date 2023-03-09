#!/usr/bin/env python3
import rospy

class Setting_server():
    def __init__(self, ControlStartStop, joint_angle_desired, joint_velocity_desired, statemachine_condition,joint_stiffness, joint_damping, neutral_pose, move2neutral, cartesian_pose = [0,0,0,0,0,0], window_length = 50, corner_frequency = 20, power_limit = 40):
        # rospy.init_node('setting_server', anonymous=True)

        # Set inital parameters # 

        rospy.set_param("control_node/control_flag", ControlStartStop)
        rospy.set_param("control_node/joint_angle_desi", joint_angle_desired)
        rospy.set_param("control_node/joint_velocity_desi", joint_velocity_desired)
        rospy.set_param("control_node/statemachine_condition", statemachine_condition)
        rospy.set_param("control_node/Kd", joint_stiffness)
        rospy.set_param("control_node/Dd", joint_damping)
        rospy.set_param("named_poses/right/poses/neutral", neutral_pose)
        rospy.set_param("control_node/Lowpass_coeff", 0.6)
        rospy.set_param("control_node/move2neutral", move2neutral)
        rospy.set_param("control_node/oscillation_window_len", window_length)
        rospy.set_param("control_node/oscillation_corner_freq", corner_frequency)
        rospy.set_param("control_node/oscillation_power_limit", power_limit)
        rospy.set_param("control_node/cartesian_pose_desired", cartesian_pose)

       


    ######## Getter Methods ########

    def get_control_flag(self):
        return rospy.get_param("control_node/control_flag")
        
    def get_joint_angle_desired(self):
        return rospy.get_param("control_node/joint_angle_desi")
    
    def get_joint_velocity_desired(self):
        return rospy.get_param("control_node/joint_velocity_desi")
    
    def get_Statemachine_condition(self):
        return rospy.get_param("control_node/statemachine_condition")
    
    def get_stiffness(self):
        return rospy.get_param("control_node/Kd")
    
    def get_damping(self):
        return rospy.get_param("control_node/Dd")
    
    def get_neutral_pose(self):
        return rospy.get_param("named_poses/right/poses/neutral")
    
    def get_lowpass_coeff(self):
        return rospy.get_param("control_node/Lowpass_coeff")
    
    def get_move2neutral(self):
        return rospy.get_param("control_node/move2neutral")
    
    def get_oscillation_window_len(self):
        return rospy.get_param("control_node/oscillation_window_len")
    
    def get_oscillation_corner_freq(self):
        return rospy.get_param("control_node/oscillation_corner_freq")
        
    def get_oscillation_power_limit(self):    
        return rospy.get_param("control_node/oscillation_power_limit")
    
    def get_cartesian_pose_desired(self):
        return rospy.get_param("control_node/cartesian_pose_desired")


    ######## Setter Methods ########

    def set_control_flag(self, controlFlag):
        return rospy.set_param("control_node/control_flag", controlFlag)
        
    def set_joint_angle_desired(self, joint_angle_desired):
        return rospy.set_param("control_node/joint_angle_desi", joint_angle_desired)
    
    def set_joint_velocity_desired(self, joint_velocity_desired):
        return rospy.set_param("control_node/joint_velocity_desi", joint_velocity_desired)
    
    def set_Statemachine_condition(self, statemachine_condition):
        return rospy.set_param("control_node/Statemachine_condition", statemachine_condition)
    
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
        return rospy.set_param("control_node/oscillation_window_len", length)
    
    def set_oscillation_corner_freq(self, frequency):
        return rospy.set_param("control_node/oscillation_corner_freq", frequency)
        
    def set_oscillation_power_limit(self, limit):    
        return rospy.set_param("control_node/oscillation_power_limit", limit)
    
    def set_cartesian_pose_desired(self, pose):
        return rospy.set_param("control_node/cartesian_pose_desired", pose)

    
def main():
    settings = Setting_server(ControlStartStop = False, joint_angle_desired = [-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40],
                 joint_velocity_desired = [0, 0, 0, 0, 0, 0, 0], statemachine_condition = 3, joint_stiffness = [20], joint_damping = [1], neutral_pose = [-0.155, 0.126, -1.638, 1.509, -1.418, 1.538, -1.40])
    print("Finished.")

if __name__ == "__main__":
    main()