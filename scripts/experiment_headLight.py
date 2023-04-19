#!/usr/bin/env python3

import rospy
import argparse
from intera_interface import HeadDisplay, Lights, Head
from sawyer_ctrl_stability.robot_dyn_kin_server import Robot_dynamic_kinematic_server



def main():
    #control = controller()
    rospy.init_node("HeadLight")
    robot_dyn_kin = Robot_dynamic_kinematic_server('right', 50)
    head_display = HeadDisplay()
    head = Head()
    l = Lights()
    light_name='head_green_light'

    initial_joint_3_pos = robot_dyn_kin.get_current_joint_angles_list()[3]
    initial_state = l.get_light_state(light_name)
    initial_pos_flag = True
    joint_3_pos_offset = 0.6


    if initial_state: # turn headlight off if on
        l.set_light_state(light_name, False)

    while not rospy.is_shutdown():
        
        current_joint_3_pos = robot_dyn_kin.get_current_joint_angles_list()[3]
        current_offset = abs(initial_joint_3_pos - current_joint_3_pos)
        
        if current_offset >= joint_3_pos_offset and initial_pos_flag: # offset occured from the initial joint pose
            initial_pos_flag = False
            l.set_light_state(light_name, True)
            rospy.sleep(1)
            l.set_light_state(light_name, False)
            
        if round(current_offset) == 0:  # returned to initial joint pose 
            initial_pos_flag = True

if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass