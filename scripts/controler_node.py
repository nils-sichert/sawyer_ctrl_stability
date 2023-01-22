        cmd = self.controler.run_impedance_controll(self.joint_angles, self.joint_velocities, pose, pose_desi, self._rate)
        # TODO add limiter

        # command new joint torques
        self._pub_coll_disable.publish()
        self._limb.set_joint_torques(cmd)
        

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
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

    def activate_controller(self):
        """
        Switches to joint torque mode and attached joint springs to current
        joint positions.
        """
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and return to Position Control Mode
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        # loop at specified rate commanding new joint torques
        counter = 0

        goal_joint_angle = [-2.4, -0.1, -1.6, -2.2, -3, -0.2,  0.11]
        while not rospy.is_shutdown():
            # compute torques
            self._update_forces(goal_joint_angle)
            print("time: ", (1/self._rate) * counter)
            counter += 1
            control_rate.sleep()

                    
    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()


def main():
    # Querying the parameter server to determine Robot model and limb name(s)
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    # Starting node connection to ROS
    print("Initializing node... ")
    rospy.init_node("sdk_joint_torque_springs_{0}".format(valid_limbs[0]))

    gcc = GCController(limb = valid_limbs[0])
    # register shutdown callback
    rospy.on_shutdown(gcc.clean_shutdown)
    #js.move_to_neutral()
    gcc.activate_controller()


if __name__ == "__main__":
    main()


