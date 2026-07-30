#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path


class read_path_pub :

    def __init__(self):
        rospy.init_node('read_path_pub', anonymous=True)
        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1)

        self.global_path_msg=Path()
        self.global_path_msg.header.frame_id='/map'
        

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('beginner_tutorials')
        path_file = rospy.get_param('~path_file', 'ajou_mini_comp_global_path.txt')
        full_path = os.path.join(pkg_path, 'path', path_file)
        if not os.path.isfile(full_path):
            rospy.logwarn("Path file '%s' not found; falling back to kcity.txt", full_path)
            full_path = os.path.join(pkg_path, 'path', 'kcity.txt')
        self.f=open(full_path,'r')
        lines=self.f.readlines()

        for line in lines :
            
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            if len(tmp) > 2:
                read_pose.pose.position.z=float(tmp[2])
            read_pose.pose.orientation.w=1
            self.global_path_msg.poses.append(read_pose)
        
        self.f.close()

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
   
            self.global_path_pub.publish(self.global_path_msg)

            rate.sleep()

        

if __name__ == '__main__':
    try:
        test_track=read_path_pub()
    except rospy.ROSInterruptException:
        pass
