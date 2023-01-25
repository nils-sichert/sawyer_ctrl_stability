#!/usr/bin/env python3
        
import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from intera_core_msgs.msg import JointLimits, SEAJointState
import threading
import pinocchio
from sys import argv
from os.path import dirname, join, abspath
from impedance_ctrl_cartesian import impedance_ctrl


class controller():
    def __init__(self, limb = "right"):
        
        # Instance Impedance controller
        self.impedance_ctrl = impedance_ctrl()
        # Instance Optimal Controller
        # Instance Robotic Chain
        urdf_filepath = "sawyer_robot/sawyer_description/urdf/sawyer_base.gazebo.xacro"
        model = pinocchio.buildModelFromUrdf(urdf_filepath)
        print('model name: ' + model.name)
        data = model.createData()
        q = pinocchio.randomConfiguration(model)
        print('q: %s' % q.T)

        pose = pinocchio.forwardKinematics(model, data, q)
        
        # Rosrate
        self.rate = rospy.Rate(100) # 100Hz
        # Kd, Dd = rosparam(default)
        rospy.init_node('Passiv_Activ_Controller', anonymous=True)
        ### publisher ### (tau_motor: , gravity_compensation_turn_off)

        self.motor_torque_pub = rospy.Publisher('computed_gc_torques_sender', JointState, queue_size=10)
        
        # create publisher to disable default gravity compensation
        gc_ns = 'robot/limb/' + limb + '/suppress_gravity_compensation'
        self._pub_gc_disable = rospy.Publisher(gc_ns, Empty, queue_size=1)
        
        ### subscriber ###
        self.lock = threading.RLock()
        self.default_gc_torques = np.empty([7,1])
        self.default_gc_model_torques = np.empty([7,1])
        self.default_joint_torques = np.empty([7,1])   

        self.joint_angle = np.empty([7,1])
        self.joint_vel = np.empty([7,1])
        self.joint_torque = np.empty([7,1])
        # robot joint state subscriber
        rospy.Subscriber('robot/joint_states', JointState, self.callback_joint_state)
        
        # robot sea state subscriberrospy.Subscriber('robot/limb/' + limb + '/gravity_compensation_torques', SEAJointState, self.callback_default_gc)
    
        # Instance state varibles (joint_angle, joint_velocity, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2)
        self.neutral_pose_jnt = [-2.3588, -0.0833594, -1.625, -2.2693, -2.98359, -0.234008,  0.10981]
        self.pose_desi = self.calc_pose(self.neutral_pose_jnt)
        self.pose_1 = self.pose_desi
        self.current_angle = self.joint_angle
        self.jacobian = self.calc_jacobian(self.current_angle)
        self.jacobian_1 = self.calc_jacobian(self.current_angle)
        self.jacobian_2 = self.calc_jacobian(self.current_angle)

    def callback_joint_state(self, data: JointState):
        with self.lock:
            if len(data.position) >= 7:
                for i in range(0, 7):
                    self.joint_angle[i] = data.position[i]
                    self.joint_vel[i] = data.velocity[i]
                    self.joint_torque[i] = data.effort[i]

    def callback_default_gc(self, data: SEAJointState):
        with self.lock:
            for i in range(0, 7):
                self.default_gc_torques[i] = data.gravity_only[i]	#	This is the torque required to hold the arm against gravity returned by KDL - SDK
                self.default_gc_model_torques[i] = data.gravity_model_effort[i]	#	This includes the inertial feed forward torques when applicable. - SDK
                self.default_joint_torques[i] = data.actual_effort[i]	# Joint torques feedback 
            
    def publish_torques(self, torques, publisher: rospy.Publisher):
        msg = JointState()
        msg.effort = torques
        publisher.publish(msg)

    def run_statemachine(self, statecondition, Kd, Dd, joint_angle, joint_velocity, rate, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, force_ee, ddPose_cart):
        if statecondition == 1:
        # State 1: Impedance Controller
            #   Input: Kd, Dd, pose, pose_desi, joint_angles, joint_velocity, rosrate, coriolis, inertia, gravity, jacoabian, jacobian_1, jacobian_2)
            #   Output: tau_motor
            torque_motor = self.impedance_ctrl.run_impedance_controll(Kd, Dd, joint_angle, joint_velocity, rate, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, ddPose_cart) #TODO correct input
        
        elif statecondition == 2:
        # State 2: optimal Controller
            #   TODO implement optimal Controller
            pass
        
        return torque_motor

    def calc_pose(self, joint_angle):
        # TODO implement calc_pose
        pass

    def calc_coriolis(self):
        # TODO implement calc_coriolis
        pass

    def calc_inertia(self):
        # TODO implement calc_inertia
        pass

    def calc_gravity(self):
        # TODO implement calc_gravity
        pass

    def calc_jacobian(self):
        # TODO implement calc_jacobian
        pass

    def get_Kd(self):
        #TODO load param
        Kd = np.identity(7)
        return Kd

    def get_Dd(self):
        #TODO load param
        Dd = np.identity(7)
        return Dd

    def get_rate(self):
        # TODO to detect if rosrate not fullfiled and return actual rate after last run
        return self.rate

    def get_pose_desi(self):
        # TODO implement callback for pose
        return self.pose_desi

    def get_force_ee(self):
        # TODO implement get_force_ee
        pass

    def get_jacobian_1(self):
        return self.jacobian_1

    def get_jacobian_2(self):
        return self.jacobian_2

    def get_statecondition(self):
        statecondition = 1
        return statecondition

    def reset_gravity_compensation(self):
        # TODO implement gravity_compensation publisher
        return

    def run_controller(self):
        controller_node = True # TODO implement a switch condition to turn it on and off
        while not rospy.is_shutdown():
            if controller_node == True:
                Kd = self.get_Kd() # numpy 6x6
                Dd = self.get_Dd() # numpy 6x6
                joint_angle = self.joint_angle # numpy 7x1
                joint_velocity = self.joint_vel # numpy 7x1
                rate = self.get_rate() # int
                jacobian_1 = self.get_jacobian_1() # numpy 6x7
                jacobian_2 = self.get_jacobian_2() # numpy 6x7
                pose = self.calc_pose(joint_angle) # numpy 6x1
                ddPose_cart = 0 # TODO add ddPose cartesian # numpy 6x1
                pose_desi = self.get_pose_desi() # numpy 6x1
                coriolis = self.calc_coriolis() # numpy 7x7
                inertia = self.calc_inertia() # numpy 7x7
                gravity = self.calc_gravity() # numpy 7x1
                jacobian = self.calc_jacobian() # numpy 6x7
                force_ee = self.get_force_ee() # numpy 7x1
                statecondition = self.get_statecondition(force_ee) # int

                # return numpy 7x1 vector of torques 
                torque_motor = self.run_statemachine(statecondition, Kd, Dd, joint_angle, joint_velocity, rate, pose, pose_desi, coriolis, inertia, gravity, jacobian, jacobian_1, jacobian_2, force_ee, ddPose_cart)
                
                self.publish_torques(torque_motor, self.motor_torque_pub)

                self.jacobian_1 = jacobian
                self.jacobian_2 = jacobian_1
                self.pose_1 = pose
            
            self.reset_gravity_compensation()
            self.rate.sleep()



def main():
    control = controller()
    control.run_controller()

if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass


