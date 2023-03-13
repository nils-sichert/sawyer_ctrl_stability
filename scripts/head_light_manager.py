#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from intera_interface import Lights

class Head_light_manager():
    def __init__(self) -> None:
        rospy.init_node("head_light_manager")
        self._sub_headlightColor = rospy.Subscriber('/control_node_debug/color', String, self.callback_color)
        self.color = 'red'
        self.light = Lights()
        rospy.loginfo("All available lights on this robot:\n{0}\n".format(
                                               ', '.join(self.light.list_all_lights())))
        self.rate = 0.1

    def callback_color(self, data):
        self.color = data
    
    def set_color(self):
        name = 'head_'+self.color+'_light'
        self.light.set_light_state(name, True)
    
    def run(self):
        while not rospy.is_shutdown():
            self.set_color()
            rospy.sleep(self.rate)

if __name__ == '__main__':
    lm = Head_light_manager()
    lm.run()