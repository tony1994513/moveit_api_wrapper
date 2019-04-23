#!/usr/bin/env python

import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import (PoseStamped, Pose, Vector3, Quaternion)
from moveit_api_warpper import _Constant
from staubli_val3_driver.srv import IOCommand,IOCommandRequest,IOCommandResponse
from linemod_pose_estimation.srv import linemod_pose, linemod_poseRequest, linemod_poseResponse
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
import tf,numpy
import pdb

def scale_trajectory_speed(traj, scale):
       new_traj = RobotTrajectory()
       # Initialize the new trajectory to be the same as the input trajectory
       new_traj.joint_trajectory = traj.joint_trajectory
       # Get the number of joints involved
       n_joints = len(traj.joint_trajectory.joint_names)
       # Get the number of points on the trajectory
       n_points = len(traj.joint_trajectory.points)
       # Store the trajectory points
       points = list(traj.joint_trajectory.points)
       # Cycle through all points and joints and scale the time from start,
       # as well as joint speed and acceleration
       for i in range(n_points):
           point = JointTrajectoryPoint()
           # The joint positions are not scaled so pull them out first
           point.positions = traj.joint_trajectory.points[i].positions
           # Next, scale the time_from_start for this point
           point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
           # Get the joint velocities for this point
           point.velocities = list(traj.joint_trajectory.points[i].velocities)
           # Get the joint accelerations for this point
           point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
           # Scale the velocity and acceleration for each joint at this point
           for j in range(n_joints):
               point.velocities[j] = point.velocities[j] * scale
               point.accelerations[j] = point.accelerations[j] * scale * scale
           # Store the scaled trajectory point
           points[i] = point
       # Assign the modified points to the new trajectory
       new_traj.joint_trajectory.points = points
       # Return the new trajecotry
       return new_traj
   
def set_trajectory_speed(traj, speed):
       # Create a new trajectory object
       new_traj = RobotTrajectory()
       
       # Initialize the new trajectory to be the same as the input trajectory
       new_traj.joint_trajectory = traj.joint_trajectory
       
       # Get the number of joints involved
       n_joints = len(traj.joint_trajectory.joint_names)
       
       # Get the number of points on the trajectory
       n_points = len(traj.joint_trajectory.points)
        
       # Store the trajectory points
       points = list(traj.joint_trajectory.points)
       
       # Cycle through all points and joints and scale the time from start,
       # as well as joint speed and acceleration
       for i in range(n_points):
           point = JointTrajectoryPoint()
           
           # The joint positions are not scaled so pull them out first
           point.positions = traj.joint_trajectory.points[i].positions

           # Next, scale the time_from_start for this point
           point.time_from_start = traj.joint_trajectory.points[i].time_from_start
           
           # Initialize the joint velocities for this point
           point.velocities = [speed] * n_joints
           
           # Get the joint accelerations for this point
           point.accelerations = [speed / 4.0] * n_joints
        
           # Store the scaled trajectory point
           points[i] = point

       # Assign the modified points to the new trajectory
       new_traj.joint_trajectory.points = points

       # Return the new trajecotry
       return new_traj
   

def IK_MoveIt(MoveIt_arm, StartPose, EndPose , Accuracy):
    wpose = Pose()
    waypoints = []  
    waypoints.append(group.get_current_pose().pose)
    waypoints.append(copy.deepcopy(StartPose))
    waypoints.append(copy.deepcopy(EndPose))  
    (plan, fraction) = MoveIt_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               Accuracy,        # eef_step
                               0.0)         # jump_threshold
    
    return cartesian_plan


def iK_calculate(MoveIt_arm,pose_target):
    arm_goal_pose = PoseStamped()
    arm_goal_pose.header.frame_id = "/base_link"
    arm_goal_pose.pose = pose_target
    MoveIt_arm.set_pose_target(arm_goal_pose)
    MoveIt_arm.set_start_state_to_current_state()
    IK_plan = MoveIt_arm.plan()
    return IK_plan 


def fK_calculate(MoveIt_arm,JointAngle):
    MoveIt_arm.set_joint_value_target(JointAngle)
    # MoveIt_arm.set_start_state_to_current_state()
    FK_plan = MoveIt_arm.plan()
    return FK_plan
    
def execute_plan(MoveIt_arm,plan):
    if _Constant.HUMAN_CONTROL: 
        raw_input('Press Enter to go') 
        # pdb.set_trace()
        plan.joint_trajectory.points[-1].time_from_start.secs = 1000
        MoveIt_arm.execute(plan,wait=True)
    else:
        MoveIt_arm.execute(plan,wait=True)
        # rospy.sleep(1)


def print_robot_Jointstate(robot):
    joint = robot.get_current_state().joint_state.position
    time = robot.get_current_state().joint_state.header.stamp.secs
    return {'t': time, 'joint': joint}


def object_pose(object_id):
    rospy.wait_for_service('/linemod_object_pose',timeout=3)
    try:
        req = linemod_poseRequest()
        req.object_id = object_id
        client = rospy.ServiceProxy('/linemod_object_pose', linemod_pose)
        res = client(req)
        rospy.loginfo("res is %s", res)
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s"%e)
    return res

def gripper_control(state):
    rospy.wait_for_service('io_command',timeout=3)
    try:
        req = IOCommandRequest()
        if state == "open":
            req.io_command.append(0)
            req.io_command.append(0)
        elif state == "chip_close":
            req.io_command.append(1)
            req.io_command.append(0)
        elif state == "cpu_close":
            req.io_command.append(0)
            req.io_command.append(1)
        client = rospy.ServiceProxy('io_command', IOCommand)
        res = client(req)
        rospy.loginfo("res is %s", res)
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: %s"%e)

def linear_interplotation(pick_pose, offset=0.05, num=10):
    mat = quaternion_matrix((pick_pose.orientation.x, pick_pose.orientation.y, pick_pose.orientation.z, pick_pose.orientation.w))
    y_dir = mat[:,1]
    step = offset/num
    new_traj = []
    pdb.set_trace()

    pick_pose.pose.translation.x = pick_pose.pose.translation.x + 0.3 * y_dir[0]
    pick_pose.pose.translation.y = pick_pose.pose.translation.y + 0.3 * y_dir[1]
    pick_pose.pose.translation.z = pick_pose.pose.translation.z + 0.3 * y_dir[2]

    for idx in range(num):
        temp = Pose()
        temp.position.x = pick_pose.pose.translation.x + step * y_dir[0]*(num-idx)
        temp.position.y = pick_pose.pose.translation.y + step * y_dir[1]*(num-idx)
        temp.position.z = pick_pose.pose.translation.z + step * y_dir[2]*(num-idx)
        temp.orientation = pick_pose.pose.rotation
        new_traj.append(temp)
    return new_traj


def objectPoseToPickpose(object_pose):
    pos = object_pose.pose.translation
    ori = object_pose.pose.rotation 
    object_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z,ori.w)))
    trans = translation_from_matrix(object_mat)
    quat = quaternion_from_matrix(object_mat)

    # x 90
    rot_1 = numpy.array((
        [[1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1, 0.0],
        [0.0, 1, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]]
        ), dtype=numpy.float64),

    aa = 2**0.5/2
    rot_2 = numpy.array((
        [[-aa, -aa, 0, 0],
        [aa, -aa, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]
        ), dtype=numpy.float64),


        
    new_mat = numpy.dot(numpy.dot(object_mat,rot_1[0]), rot_2[0])
    
    new_trans = translation_from_matrix(new_mat)
    new_quat = quaternion_from_matrix(new_mat)

    new_pose = Pose()
    new_pose.position.x = new_trans[0]
    new_pose.position.y = new_trans[1]
    new_pose.position.z = new_trans[2]
    new_pose.orientation.x = new_quat[0]
    new_pose.orientation.y = new_quat[1]
    new_pose.orientation.z = new_quat[2]
    new_pose.orientation.w = new_quat[3]

    SHOW_TF = True
    if SHOW_TF:
        broadcaster = tf.TransformBroadcaster()

        broadcaster.sendTransform(
            trans,
            quat,
            rospy.Time.now(),
            'object_pose',
            'base_link', 
            )

        broadcaster.sendTransform(
        new_trans,
        new_quat,
        rospy.Time.now(),
        'picking_pose',
        'base_link', 
    )
    return new_pose
