#!/usr/bin/env python
#coding=utf-8
from geometry_msgs.msg import PoseStamped
import rospy
import time
rospy.init_node('aa')
a=rospy.Publisher('target_pose',PoseStamped,queue_size=5)
p=PoseStamped()
p.pose.position.x=0.2
p.pose.position.y=-0.1
p.pose.position.z=-0.07
time.sleep(0.1)
a.publish(p)
time.sleep(1)
p.pose.position.x=0.27
p.pose.position.y=-0.0
p.pose.position.z=-0.07
time.sleep(1)
a.publish(p)
p.pose.position.x=0.15
p.pose.position.y=0.1
p.pose.position.z=-0.07
time.sleep(15)
a.publish(p)
