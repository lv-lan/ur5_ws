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
if __name__ == '__main__':

    
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