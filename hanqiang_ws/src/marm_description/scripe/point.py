#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from tf.transformations import quaternion_from_euler
from math import pi
import copy
import geometry_msgs.msg
from copy import deepcopy
import tf2_ros
import tf2_geometry_msgs

def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_go_to_pose', anonymous=True)

        self._planning_group = "ur5_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

       

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def set_joint_angles(self, arg_list_joint_angles):
        
            list_joint_values = self._group.get_current_joint_values()
            rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
            rospy.loginfo(list_joint_values)

            self._group.set_joint_value_target(arg_list_joint_angles)
            self._group.plan()
            flag_plan = self._group.go(wait=True)

            list_joint_values = self._group.get_current_joint_values()
            rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
            rospy.loginfo(list_joint_values)

            pose_values = self._group.get_current_pose().pose
            rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
            #rospy.loginfo(pose_values)
            rospy.loginfo("(%f , %f, %f, %f, %f, %f, %f)" % (pose_values.position.x,
                                                            pose_values.position.y,
                                                            pose_values.position.z,
                                                            pose_values.orientation.x,
                                                            pose_values.orientation.y,
                                                            pose_values.orientation.z,
                                                            pose_values.orientation.w))

            if (flag_plan == True):
                rospy.loginfo(
                    '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
            else:
                rospy.logerr(
                    '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

            return flag_plan



    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

if __name__ == '__main__':
    ur5 = Ur5Moveit()
    
    pose1 =[-1.606309, -1.606156, -0.956727, 5.551023, 1.559013, 3.377792]
    ur5.set_joint_angles(pose1)
    rospy.sleep(10)
    end_effector_link = ur5._group.get_end_effector_link()
    start_pose = ur5._group.get_current_pose(end_effector_link).pose
    waypoints = []
     
    # 将初始位姿加入路点列表
        
    waypoints.append(start_pose)
    wpose = deepcopy(start_pose)
    # 第二个路点需要向后运动0.2米，向右运动0.2米

    wpose.position.x -= 0
    wpose.position.y += 0
    wpose.position.z -= 0.3
    waypoints.append(deepcopy(wpose))
    # 设置第三个路点数据，并加入路点列表
    wpose.position.x -= 0.3
    wpose.position.y -=0
    wpose.position.z += 0
    waypoints.append(deepcopy(wpose))
   # 设置第4个路点数据，并加入路点列表
    wpose.position.x += 0
    wpose.position.y -=0
    wpose.position.z += 0.3
    waypoints.append(deepcopy(wpose))
    # 设置第4个路点数据，并加入路点列表
    wpose.position.x += 0.3
    wpose.position.y -=0
    wpose.position.z += 0
    waypoints.append(deepcopy(wpose))
    
    fraction = 0.0   #路径规划覆盖率
    maxtries = 100   #最大尝试规划次数
    attempts = 0     #已经尝试规划次数
                # 设置机器臂当前的状态作为运动初始状态
    ur5._group.set_start_state_to_current_state()
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = ur5._group.compute_cartesian_path (
                                waypoints,   # waypoint poses，路点列表
                                0.01,        # eef_step，终端步进值
                                0.0,         # jump_threshold，跳跃阈值
                                True)        # avoid_collisions，避障规划
        # 尝试次数累加
        attempts += 1
        # 打印运动规划进程
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")            
    # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        ur5._group.execute(plan)
        rospy.loginfo("Path execution complete.")
        ur5.set_joint_angles(pose1)
        rospy.sleep(20)
    # 如果路径规划失败，则打印失败信息
    else:
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
    flag_plan = ur5._group.go(wait=True)
    if (flag_plan == True):
        rospy.loginfo(
            '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
    else:
        rospy.logerr(
            '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')
    rospy.sleep(2)
    del ur5