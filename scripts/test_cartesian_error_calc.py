import rospy
import argparse
from tf_conversions import posemath
import PyKDL
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def main():
    """
    Move the robot arm to the specified configuration.
    Call using:
    $ rosrun intera_examples go_to_cartesian_pose.py  [arguments: see below]

    -p 0.4 -0.3 0.18 -o 0.0 1.0 0.0 0.0 -t right_hand
    --> Go to position: x=0.4, y=-0.3, z=0.18 meters
    --> with quaternion orientation (0, 1, 0, 0) and tip name right_hand
    --> The current position or orientation will be used if only one is provided.

    -q 0.0 -0.9 0.0 1.8 0.0 -0.9 0.0
    --> Go to joint angles: 0.0 -0.9 0.0 1.8 0.0 -0.9 0.0 using default settings
    --> If a Cartesian pose is not provided, Forward kinematics will be used
    --> If a Cartesian pose is provided, the joint angles will be used to bias the nullspace

    -R 0.01 0.02 0.03 0.1 0.2 0.3 -T
    -> Jog arm with Relative Pose (in tip frame)
    -> x=0.01, y=0.02, z=0.03 meters, roll=0.1, pitch=0.2, yaw=0.3 radians
    -> The fixed position and orientation paramters will be ignored if provided

    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        "-p", "--position", type=float,
        nargs='+',
        help="Desired end position: X, Y, Z")
    parser.add_argument(
        "-o", "--orientation", type=float,
        nargs='+',
        help="Orientation as a quaternion (x, y, z, w)")
    parser.add_argument(
        "-R", "--relative_pose", type=float,
        nargs='+',
        help="Jog pose by a relative amount in the base frame: X, Y, Z, roll, pitch, yaw")
    args = parser.parse_args(rospy.myargv()[1:])

    try:
        rospy.init_node('go_to_cartesian_pose_py')

        if (args.position is None and args.orientation is None
            and args.relative_pose is None):
        else:
            endpoint_state = limb.tip_state(args.tip_name)
            pose = endpoint_state.pose

            if args.relative_pose is not None:
                if len(args.relative_pose) != 6:
                    rospy.logerr('Relative pose needs to have 6 elements (x,y,z,roll,pitch,yaw)')
                    return None
                # create kdl frame from relative pose
                rot = PyKDL.Rotation.RPY(args.relative_pose[3],
                                         args.relative_pose[4],
                                         args.relative_pose[5])
                trans = PyKDL.Vector(args.relative_pose[0],
                                     args.relative_pose[1],
                                     args.relative_pose[2])
                f2 = PyKDL.Frame(rot, trans)
                # and convert the result back to a pose message
                else:
                  # base frame
                  pose = posemath.toMsg(f2 * posemath.fromMsg(pose))
            else:
                if args.position is not None and len(args.position) == 3:
                    pose.position.x = args.position[0]
                    pose.position.y = args.position[1]
                    pose.position.z = args.position[2]
                if args.orientation is not None and len(args.orientation) == 4:
                    pose.orientation.x = args.orientation[0]
                    pose.orientation.y = args.orientation[1]
                    pose.orientation.z = args.orientation[2]
                    pose.orientation.w = args.orientation[3]
            poseStamped = PoseStamped()
            poseStamped.pose = pose


        rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=args.timeout)


if __name__ == '__main__':
    main()
