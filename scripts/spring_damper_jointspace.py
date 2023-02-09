import numpy as np

class spring_damper_jointspace():
    def __init__(self) -> None:
        pass

    """
    Good Values for testing:
    D>1: KD: [10,10,10,10,10,10,10]
        Dd: [1,1,1,1,1,1,1]

    D = 0.7: Kd: [100,100,100,100,100,100,100]
        Dd: [1,1,1,1,1,1,1]
    """
    def calc_torque(self, joint_angle_desi, cur_joint_angle, cur_joint_velo, Kd, Dd, gravity):
        torque_list = [0]*len(joint_angle_desi)
        
        for joint in range(len(joint_angle_desi)):
            # spring portion
            torque_list[joint] = Kd[joint][joint] * (joint_angle_desi[joint] - cur_joint_angle[joint]) - Dd[joint][joint] * cur_joint_velo[joint] + gravity[joint]

        return self.array_to_list(torque_list)
    
    def array_to_list(self, array):
        list = [0]*len(array)
        for i in range(len(array)):
            list[i] = array[i][0]
        return list
