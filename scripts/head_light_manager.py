#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from intera_interface import Lights

class Head_light_manager():
    def __init__(self) -> None:
        # initalizing node
        rospy.init_node("head_light_manager")      

        # Instance of headlight manager and show avaible color settings
        self.light = Lights()
        rospy.loginfo("All available lights on this robot:\n{0}\n".format(
                                               ', '.join(self.light.list_all_lights())))
        # Update Rate of headlight color
        self.rate = 2 # HZ

        # Turn all lights off
        self.light.set_light_state('head_red_light', False)
        self.light.set_light_state('head_green_light', False)
        self.light.set_light_state('head_blue_light', False)

        # Subscribe headlight color setting
        self._sub_headlightColor = rospy.Subscriber('/control_node_debug/color', String, self.callback_color)
        
        # Init variables
        self.color = 'red'
        self.color_prev = 'red'
        self.flag = False

    def callback_color(self, data):
        """
        Callback of desired headlight colors.
        Parameters: desired color (data.data: Str)
        Return: new desired color (Str)
        """
        if data.data != self.color:
            self.flag = True
            self.color_prev = self.color
        self.color = data.data
        
    
    def set_color(self):
        """
        Set headlight color (green, yellow, red).
        Parameters: 
        Return: None
        """
        name = 'head_' + self.color + '_light'
        # if self.flag == True:
        #     self.light.set_light_state('head_red_light', False)
        #     self.light.set_light_state('head_green_light', False)
        #     self.light.set_light_state('head_blue_light', False)
        #     self.flag = False
        if name == 'head_red_light':
            self.light.set_light_state('head_green_light', False)
            self.light.set_light_state('head_blue_light', False)
            self.light.set_light_state(name, True)
        elif name == 'head_green_light':
            self.light.set_light_state('head_red_light', False)
            self.light.set_light_state('head_blue_light', False)
            self.light.set_light_state(name, True)
        elif name == 'head_yellow_light':
            self.light.set_light_state('head_blue_light', False)
            self.light.set_light_state('head_red_light', True)
            self.light.set_light_state('head_green_light', True)
    
    def run(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.set_color()
            r.sleep()

if __name__ == '__main__':
    lm = Head_light_manager()
    lm.run()