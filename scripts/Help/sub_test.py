import rospy
import PyKDL as kdl
import numpy as np

from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointLimits, EndpointState, SEAJointState
from geometry_msgs.msg import Pose, Twist, Wrench
from std_msgs.msg import Empty, String, Float64

import intera_interface
from intera_interface import CHECK_VERSION

class test():
    def __init__(self) -> None:
        rospy.init_node("tester") 
        limb = 'right'
        self._ee_pose = [Pose()]
        self._ee_twist = [Twist()]
        self._ee_wrench = [Wrench()]

        self._joint_pos = [0] * 7
        self._joint_vel = [0] * 7
        self._joint_torque = [0] * 7

        self._default_gc_torques = [0] * 7
        self._default_gc_model_torques = [0] * 7
        self._torque_limits = [0] * 7

        self._default_joint_torques = [0] * 7
        self._limb = intera_interface.Limb(limb)
        self.rate = 1/20

        self._waitingtime = 2
        
        # Publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)
        
        # create publisher to disable default gravity compensation
        gc_ns = 'robot/limb/' + limb + '/suppress_gravity_compensation'
        self._pub_gc_disable = rospy.Publisher(gc_ns, Empty, queue_size=1)
        
        # create publisher to switch off automatic collision detection
        coll_ns = 'robot/limb/' + limb + '/suppress_contact_safety'
        self._pub_coll_disable = rospy.Publisher(coll_ns, Empty, queue_size=1)

        # robot ee state subscriber
        rospy.Subscriber('robot/limb/' + limb + '/endpoint_state', EndpointState, self.callback_ee_state)
        
        # robot joint state subscriber
        rospy.Subscriber('robot/joint_states', JointState, self.callback_joint_state)        
        
        # robot sea state subscriber
        rospy.Subscriber('robot/limb/' + limb + '/gravity_compensation_torques', SEAJointState, self.callback_default_gc)
    
    def callback_ee_state(self, data):
        self._ee_pose[0] = data.pose
        self._ee_twist[0] = data.twist
        self._ee_wrench[0] = data.wrench

    def callback_joint_state(self, data):
        if len(data.position) >= 7:
            for i in range(0, 7):
                self._joint_pos[i] = data.position[i]
                self._joint_vel[i] = data.velocity[i]
                self._joint_torque[i] = data.effort[i]

    def callback_default_gc(self, data):
        for i in range(0, 7):
            self._default_gc_torques[i] = data.gravity_only[i]	#	This is the torque required to hold the arm against gravity returned by KDL - SDK
            self._default_gc_model_torques[i] = data.gravity_model_effort[i]	#	This includes the inertial feed forward torques when applicable. - SDK
            
            self._default_joint_torques[i] = data.actual_effort[i]	# Joint torques feedback 
            
    def callback_limits(self, data):
        for i in range(0, 7):
            self._torque_limits[i] = data.effort[i]

    def move_to_neutral(self):
        """
        Moves the limb to neutral location. Position controller
        """
        #self._limb.move_to_neutral()
        sawyer_neutral_pose = [-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981]
        new_limb_pose = {}
        i = 0
        for joint in self._limb.joint_names():
            new_limb_pose[joint] = sawyer_neutral_pose[i]
            i += 1
        self._limb.move_to_joint_positions(new_limb_pose)
        rospy.sleep(self._waitingtime)
        print (self._limb.joint_names())
        print ("######## Ready for next action. ########")

    def limiter(self, value, lower_bound, upper_bound):
        limit_value = np.clip(value, lower_bound, upper_bound)
        return limit_value

    def run(self):
        self.move_to_neutral()
        self._pub_gc_disable
        self._pub_coll_disable
        while not rospy.is_shutdown():
            print('EE_Pose: ', self._ee_pose)
            print('EE Effort: ', self._ee_wrench)
            print('GC Torques: ', self._default_gc_torques)
            print('Joint Pose: ', self._joint_pos)
            print('Joint Torque', self._joint_torque)
            print('Actual Torque: ', self._default_joint_torques)
            print('Posted')
            rospy.sleep(self.rate)

def main():
    tester = test()
    
    print('init test')
    tester.run()
    print('exit test')
    



if __name__=='__main__':
    main()