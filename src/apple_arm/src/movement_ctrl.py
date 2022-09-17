#!/usr/bin/env python
#coding=utf-8
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
import time
import tf


class Arm_Basic_Movements:
    '''
        机械臂执行运动的过程控制
    '''
    def __init__(self):
        #消息发布
        self.m_Pub=rospy.Publisher('target_trasient_pose',PoseStamped,queue_size=10)
        self.executing=False
        #消息接收
        self.listener=tf.TransformListener()
        rospy.Subscriber('/arm_pose_goal',PoseStamped,callback=self.MoveTo_Linear)  #目标位置
        rospy.Subscriber('arm_pose',PoseStamped,callback=self.Refresh_Pos)      #当前位置
        #缓存
        self.posNow=PoseStamped()
        self.step=rospy.get_param('arm_pos_acc_step',0.003)
        self.speed=rospy.get_param('arm_control_interval',0.03)
    def Refresh_Pos(self,msg):
        #更新位置
        self.posNow=msg

    def MoveTo_Linear(self,msg):
        '''
            按照线性方式运动到某个位置
        '''
        if self.executing: return
        self.executing=True
        #首先，获取机械臂的当前位置，作为其线性运动的起点
        x1,y1,z1=self.posNow.pose.position.x,\
            self.posNow.pose.position.y,\
            self.posNow.pose.position.z
        rospy.loginfo('[arm control] Get target at %f %f %f:'%(x1,y1,z1))
        #其次，获取机械臂的终点位置
        x2,y2,z2=msg.pose.position.x,\
            msg.pose.position.y,\
            msg.pose.position.z
        rospy.loginfo('[arm control] Get target at %f %f %f:'%(x2,y2,z2))
        #构建缓存变量
        cache=PoseStamped()
        stp=np.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1))/self.step
        print stp
        if stp==0: 
            self.executing=False
            return
        stp=int(stp)

        #计算dx，dy
        dx=(x2-x1)/stp
        dy=(y2-y1)/stp
        dz=(z2-z1)/stp
        #开始画线
        for i in range(stp):
            #延时
            time.sleep(self.speed)
            cache.pose.position.x=x1+dx*i
            cache.pose.position.y=y1+dy*i
            cache.pose.position.z=z1+dz*i
            cache.pose.orientation.y=2.33
            self.m_Pub.publish(cache)
        self.executing=False


if __name__=='__main__':
    rospy.init_node('arm_basic_movements')
    m=Arm_Basic_Movements()
    rospy.spin()