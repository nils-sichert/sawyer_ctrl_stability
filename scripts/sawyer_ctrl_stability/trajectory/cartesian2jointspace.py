import rospy
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion,)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
import csv
from intera_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest,)
import tqdm
import os

rospy.init_node("IK_translater")
tmp = os.path.dirname(__file__)
def ik_service_client(position, orientation, limb = "right", use_advanced_options = True):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=position[0],
                    y=position[1],
                    z=position[2],
                ),
                orientation=Quaternion(
                    x=orientation[0],
                    y=orientation[1],
                    z=orientation[2],
                    w=orientation[3],
                ),
            ),
        ),
    }
    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(poses[limb])
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')

    if (use_advanced_options):
        # Optional Advanced IK parameters
        rospy.loginfo("Running Advanced IK Service Client example.")
        # The joint seed is where the IK position solver starts its optimization
        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        seed.position = [0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
        ikreq.seed_angles.append(seed)

        # Once the primary IK task is solved, the solver will then try to bias the
        # the joint angles toward the goal joint configuration. The null space is 
        # the extra degrees of freedom the joints can move without affecting the
        # primary IK task.
        ikreq.use_nullspace_goal.append(True)
        # The nullspace goal can either be the full set or subset of joint angles
        goal = JointState()
        goal.name = ['right_j1', 'right_j2', 'right_j3']
        goal.position = [0.1, -0.3, 0.5]
        ikreq.nullspace_goal.append(goal)
        # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
        # If empty, the default gain of 0.4 will be used
        ikreq.nullspace_gain.append(0.4)
    else:
        pass
        #rospy.loginfo("Running Simple IK Service Client example.")

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False
    
        # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp.result_type[0], 'None')
        #rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
        #      (seed_str,))
        # Format solution into Limb API-compatible dictionary
        joint_list = resp.joints[0].position
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        #rospy.loginfo("------------------")
        #rospy.loginfo("Response Message:\n%s", resp)
        return joint_list
    else:
        rospy.loginfo("INVALID POSE - No Valid Joint Solution Found.")

        return False

def convert():
    joint_list = []
    with open(os.path.join(tmp, 'square_traj_cart.csv'), newline='') as f:
        reader = csv.reader(f,delimiter=',')
        for row in reader:
            position = [float(row[0]), float(row[1]), float(row[2])]
            orientation = [float(row[3]), float(row[4]), float(row[5]), float(row[6])]
            list_tmp = ik_service_client(position, orientation)
            joint_list.append(list_tmp)
    
    with open(os.path.join(tmp,'square_traj_joint.csv'), 'w', newline='') as file:
        writer = csv.writer(file, delimiter=',')
        for i in range(len(joint_list)):
            writer.writerow([joint_list[i][0],joint_list[i][1],joint_list[i][2],joint_list[i][3],joint_list[i][4],joint_list[i][5],joint_list[i][6]])
convert()