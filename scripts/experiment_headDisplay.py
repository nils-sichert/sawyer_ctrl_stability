#!/usr/bin/env python3

import rospy
import argparse
from intera_interface import HeadDisplay, Lights, Head
from sawyer_ctrl_stability.robot_dyn_kin_server import Robot_dynamic_kinematic_server
from math import floor


"""
Author: Nourhan Abdulazeem
Purpose: Physical Exercise Experiment with Sawyer
Description: Displays different facial expression according to the user's perfromance, i.e, number of pushes performed.
"""


def main():
    #control = controller()
    rospy.init_node("HeadDispaly")
    robot_dyn_kin = Robot_dynamic_kinematic_server('right', 50)
    head_display = HeadDisplay()
    head = Head()

    initial_joint_3_pos = robot_dyn_kin.get_current_joint_angles_list()[3]  # reads current 4th joint position/angle
    joint_3_pos_offset = 0.6 # required offset for a push to be counted in radians
    initial_pos_flag = True # indicates initial arm configuration
    
    push_counts = 0 
    push_counts_flag = True # push_counts is re-set

    session_duration = 2 #minutes
    number_of_pushes_10_sec = int(input("Enter number of pushes perrformed in 10 seconds (baseline): "))  # baseline - 4 is an average number of pushes
    duration_of_push = 10/number_of_pushes_10_sec #seconds
    push_counts_baseline = floor((60 * session_duration) / duration_of_push)

    condition  = int(input("Enter: \n1 for inactive condition, \n2 for unresponsive condition, \n3 for responsive condition: "))

    if condition == 3:
        print("Happy face after ", push_counts_baseline * 0.25," pushes")
        print("Surprised face after ", push_counts_baseline * 0.75," pushes")

    head.set_pan(-0.2)
    
    while not rospy.is_shutdown():
        
        current_joint_3_pos = robot_dyn_kin.get_current_joint_angles_list()[3]
        current_offset = abs(initial_joint_3_pos - current_joint_3_pos)
        
        if current_offset >= joint_3_pos_offset and initial_pos_flag: # offset occured from the initial joint pose
            initial_pos_flag = False
            push_counts += 1
            push_counts_flag = True # indicates if a new face should be published, i.e., if a 25% or 75% of the baseline are exceeded
            print("Pushes = ", push_counts)


        if round(current_offset) == 0:  # returned to initial joint pose
            initial_pos_flag = True
        
        
            if push_counts_flag: # publish facial emotions only if push_counts changed to minimze the time out of the controller
                push_counts_flag = False
                if condition == 3:  # Responseive Condition
                    if push_counts <= (push_counts_baseline * 0.25): # publish purple neutral face for 25% pushes of the baseline
                        print("Neutral face")
                        head_display.display_image("/home/airlab5/ros_ws/src/sawyer_ctrl_stability/StudyFaceFiles/PurpleNeutralBaxterFace.jpg")
                    elif push_counts <= (push_counts_baseline * 0.75): # publish purple happy face for 75% pushes of the baseline
                        print("Happy face")
                        head_display.display_image("/home/airlab5/ros_ws/src/sawyer_ctrl_stability/StudyFaceFiles/PurpleHappyBaxterFace.jpg")
                    else:   # publish purple susprised face for more than 75% pushes of the baseline 
                        print("Surprised face")
                        head_display.display_image("/home/airlab5/ros_ws/src/sawyer_ctrl_stability/StudyFaceFiles/PurpleSurprisedBaxterFace.jpg")
                elif condition == 2:    #   Unresponsive Condition
                    print("Happy face")
                    head_display.display_image("/home/airlab5/ros_ws/src/sawyer_ctrl_stability/StudyFaceFiles/PurpleHappyBaxterFace.jpg")
                else:   # Inactive Condition
                    print("Default Screen")
                    head_display.display_image("/home/airlab5/ros_ws/src/sawyer_ctrl_stability/StudyFaceFiles/defaultScreen.png")
    #control.clean_shutdown()

if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass