#!/usr/bin/env python3

import rospy
import numpy as np

class trajectoy_executer():
    def __init__(self) -> None:
        rospy.init_node("trajectory_executer")
        
        setpoint1 = np.array([-0.102431640625, 0.0428837890625, -1.551865234375, 1.5323720703125, -2.9784638671875, 1.376140625, -1.474814453125])
        setpoint2 = np.array([0.68933984375, 0.0551298828125, -1.7256201171875, 0.9122099609375, -2.9595625, -0.1810732421875, -1.1953828125])
        setpoint3 = np.array([0.8638984375, 0.2003271484375, -1.309107421875, 0.7584423828125, -1.97009375, -0.179833984375, -1.1953828125])
        setpoint4 = np.array([0.351072265625, 0.183025390625, -0.9879375, 1.5975751953125, -2.7829541015625, -0.3334609375, -1.1953828125])
        setpoint5 = np.array([-0.102431640625, 0.0428837890625, -1.551865234375, 1.5323720703125, -2.9784638671875, 1.376140625, -1.474814453125])
        self.setpoints = [setpoint1, setpoint2, setpoint3, setpoint4, setpoint5]
        """

        setpoint1 = np.array([-0.1558798828125, 0.1269013671875, -1.63815625, 1.5093447265625, -1.41862890625, 1.5380302734375, -1.40465625])
        setpoint2 = np.array([0.040005859375, 0.2301962890625, -1.546802734375, 1.7408759765625, -1.3434228515625, 1.567390625, -1.36163671875])
        setpoint3 = np.array([-0.56184765625, 0.13050390625, -1.54463671875, 1.2308095703125, -1.37666796875, 1.630421875, -0.9787373046875])
        setpoint4 = np.array([-0.7299560546875, 0.1151640625, -1.5363984375, 0.265998046875, -1.678474609375, 1.687681640625, -1.11640625])
        setpoint4 = np.array([0.4005009765625, 0.118765625, -1.6057705078125, 0.9712509765625, -1.462875, 1.6033603515625, -1.9172080078125])
        setpoint5 = np.array([1.01065234375, 0.1649072265625, -1.57325, 1.5816396484375, -1.3982685546875, 1.48118359375, -1.9172080078125])
        setpoint5 = np.array([-0.1558798828125, 0.1269013671875, -1.63815625, 1.5093447265625, -1.41862890625, 1.5380302734375, -1.40465625])
        self.setpoints = [setpoint1, setpoint2, setpoint3, setpoint4, setpoint5]
        """
        self.radspeed = 0.0015

    def publish(self):
        for i in range(len(self.setpoints)-1):
            diff = self.setpoints[i+1]-self.setpoints[i]
            distance = np.max(np.abs(diff))
            print(self.setpoints[i+1],self.setpoints[i])
            steps = int(distance/self.radspeed)
            dealta_step = diff/steps
            r = rospy.Rate(1000)
            for j in range(steps):
                desired_pose = np.array(self.setpoints[i]+j*dealta_step).tolist()
                rospy.set_param("control_node/joint_angle_desi", desired_pose)
                r.sleep()
    
if __name__ =="__main__":
    trajectory_ex = trajectoy_executer()
    trajectory_ex.publish()

    
    
    
        
