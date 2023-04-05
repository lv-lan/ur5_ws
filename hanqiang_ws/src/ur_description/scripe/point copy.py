#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import copy

from math import pi
import copy

from copy import deepcopy

if __name__ == '__main__':
    start_pose =[0,0, 0]
    waypoints = []
     
    # 将初始位姿加入路点列表
    waypoints.append(start_pose)
    wpose = deepcopy(start_pose)
    # 第二个路点需要向后运动0.2米，向右运动0.2米
    print ("origin  :"+str(wpose))
    wpose[0] += 1
    wpose[1] += 1
    wpose[2] += 1
    waypoints.append(deepcopy(wpose))
    print ("origin  :"+str(waypoints))