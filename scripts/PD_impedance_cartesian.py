#!/usr/bin/env python
import numpy as np

class pd_impedance_cartesian():
    def __init__(self):
        pass

    def calc_joint_torque(self):
        pass
        # return self.vec2list(torque_list)
    
    def vec2list(self,vec):
        """
        Convert vector into list.
        Parameters: vector (numpy array: nx1)
        Return: list (list: nx1)
        """
        len_vec = len(vec)
        list = [0]*len_vec
        for i in range(len_vec):
            list[i] = vec[i][0]
        return list

if __name__ == "__main__":
    ctrl = pd_impedance_cartesian()
    print("PD Impedance started")