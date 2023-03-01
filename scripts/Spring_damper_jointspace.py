#!/usr/bin/env python

class spring_damper_jointspace():
    def __init__(self) -> None:
        pass

    def calc_joint_torque(self, error_joint_angles, error_joint_velocity, Kd, Dd, gravity_compensation):
        """
        Calculate torques based on displacement from target position and current velocity.
        Parameters: error between desired and actual joint angel (numpy arra: 7x1), current joint velocity (numpy array: 7x1), 
            joint stiffness matrix Kd (numpy array: 7x7), joint damping matrix Dd (numpy array: 7x7), gravity compensation vector (numpy array: 7x1)
        Return: joint torques (list: 7x1)
        """
        torque_list = [0]*len(error_joint_angles)
        
        for joint in range(len(error_joint_angles)):
            torque_list[joint] = -Kd[joint][joint] * (error_joint_angles[joint]) - Dd[joint][joint] * error_joint_velocity[joint] + gravity_compensation[joint]
        return self.array_to_list(torque_list)
    
    def array_to_list(self, array):
        """
        Converte numpy array into list format.
        Parameters: Array (numpy arra: nx1)
        Return: list (dict: nx1)
        """
    
        list = [0]*len(array)
        for i in range(len(array)):
            list[i] = array[i][0]
        return list

if __name__ == "__main__":
    ctrl = spring_damper_jointspace()
    print("Spring damper Ctrl started")