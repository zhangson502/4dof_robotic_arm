#!/usr/bin/env python
#coding=utf-8
import rospy
import cv2
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import numpy as np
class Catch_App:
    '''
        让机械臂抓取一个环境中的东西
    '''
    def __init__(self):
        #消息发布
        self.goal_Pub=rospy.Publisher('/arm_pose_goal',PoseStamped,queue_size=3)
        self.clip_Pub=rospy.Publisher('clip_val',Float32,queue_size=1,latch=True)

        #消息接收
        rospy.Subscriber('target_pose',PoseStamped,self.Catch)
        rospy.Subscriber('arm_pose',PoseStamped,callback=self.Refresh_Pos)      #当前位置

        #缓存
        self.pos=[0,0,0]    #当前位置
        self.tmpPose=0.15,0.15,0.1     #中转点位置
        self.boxPose=0.25,0.0,0.15       #盒子位置
        self.accuracy=rospy.get_param('~taret_accuracy',0.005)  #保证抓取精度
        self.clip_close_Val=rospy.get_param('~close_val',1.14)
        self.clip_open_Val=rospy.get_param('~open_val',0)

        #   移动到一个无关点
        self.GoToPos(0.25,0.0,0.1)

    def Catch(self,msg):
        '''
            执行机械臂的抓取过程
        '''
        #   移动到一个无关点
        self.GoToPos(0.25,0.0,0.1)
        #　移动到物体上面
        self.GoToPos(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z+0.12)
        #  抓取
        self.GoToPos(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
        #关闭夹子
        time.sleep(1)
        self.Close()
        time.sleep(1)
        #移动到一个无关点
        self.GoToPos(0.25,0.0,0.1)
        #放到盒子上
        self.GoToPos(-0.05,0.28,0.15)
        self.Open()
    
    def GoToPos(self,x=0.15,y=-0.15,z=0.1):
        '''
            移动爪子到一个位置
        '''
        cache=PoseStamped()
        cache.pose.position.x=x;cache.pose.position.y=y;cache.pose.position.z=z
        self.goal_Pub.publish(cache)
        while np.abs(cache.pose.position.x-self.pos[0])>self.accuracy or \
            np.abs(cache.pose.position.y-self.pos[1])>self.accuracy or \
            np.abs(cache.pose.position.z-self.pos[2])>self.accuracy:
            time.sleep(0.1)

        

    def Refresh_Pos(self,msg):
        '''
            更新位置信息
        '''
        self.pos=[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]

    
    def Open(self):
        '''#开启夹子'''
        self.clip_Pub.publish(self.clip_open_Val)

    def Close(self):
        '''夹紧夹子'''
        self.clip_Pub.publish(self.clip_close_Val)


if __name__=='__main__':
    rospy.init_node('catch_app')
    app=Catch_App()
    rospy.spin()

