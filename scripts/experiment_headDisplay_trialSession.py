#!/usr/bin/env python3

import rospy
import argparse
from intera_interface import HeadDisplay, Lights, Head
from sawyer_ctrl_stability.robot_dyn_kin_server import Robot_dynamic_kinematic_server

def main():
    #control = controller(node_name="Control_manager1")
    rospy.init_node("HeadDisplay_trail")
    robot_dyn_kin = Robot_dynamic_kinematic_server('right', 50)
    head_display = HeadDisplay()
    head = Head()

    initial_joint_3_pos = robot_dyn_kin.get_current_joint_angles_list()[3]
    joint_3_pos_offset = 0.6
    initial_pos_flag = True
    
    push_counts = 0
    push_counts_flag = True # push_counts is re-set

    head.set_pan(-3.14) # for Trial sessions
    
    while not rospy.is_shutdown():
        
        current_joint_3_pos = robot_dyn_kin.get_current_joint_angles_list()[3]
        current_offset = abs(initial_joint_3_pos - current_joint_3_pos)
        
        if current_offset >= joint_3_pos_offset and initial_pos_flag: # offset occured from the initial joint pose
            initial_pos_flag = False
            push_counts += 1
            push_counts_flag = True
            print("Pushes = ", push_counts)


        if round(current_offset) == 0:  # returned to initial joint pose 
            initial_pos_flag = True
    #control.clean_shutdown()

if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass