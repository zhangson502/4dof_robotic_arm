#!/usr/bin/env python
#coding=utf-8

import cv2
import time
import numpy as np
import rospy
import pygame
import threading
from geometry_msgs.msg import PoseStamped
from pygame.surfarray import make_surface
class Cam_Analyzer:
    '''
        摄像头分析模块
    '''
    def __init__(self,cam_id='/dev/video0',width=500,height=500,\
        cx=250,cy=250,\
        leftBottom_Coor=[-0.35,0.35],\
        rightTop_Coor=[0.35,-0.35],\
        hsv_low=[0,90,150],\
        hsv_high=[255,230,255],\
        calib_X=-0.0,\
        calib_Y=0.0):

        #配置参数
        self.width=width
        self.height=height
        self.cx=cx
        self.cy=cy
        self.lB=leftBottom_Coor
        self.rT=rightTop_Coor
        self.calib_X=calib_X
        self.calib_Y=calib_Y

        self.Objs=[]

        

        # 初始化ROS模块
        rospy.init_node('cam_analyzer')
        self.tar_Pub=rospy.Publisher('target_pose',PoseStamped,queue_size=5)
        #初始化机械臂控制界面
        pygame.init()

        #初始化控制界面
        self.clock=pygame.time.Clock()
        self.win=pygame.display.set_mode((width,height))
        self.win.fill((50,50,50))
        pygame.display.update()

        # 初始化独立面板刷新线程
        self.t1=threading.Thread(target=self.t_UpdateImage,args=())
        self.t1.setDaemon(True)
        self.t1.start()
    
    def Pixel2RealXY(self,x,y):
        '''
            将像素坐标转化为机械臂的实际运动坐标

        '''
        real_Expansion_X=self.rT[0]-self.lB[0]
        real_Expansion_Y=self.rT[1]-self.lB[1]
        #获取每个像素点代表的实际环境下的坐标位置
        pixel_per_meter_X=real_Expansion_X/self.width
        pixel_per_meter_Y=real_Expansion_Y/self.height
        #开始转换
        real_X=x*pixel_per_meter_X+self.lB[0]+self.calib_X
        real_Y=y*pixel_per_meter_Y+self.lB[1]+self.calib_Y
        return real_X,real_Y
    
    def DrawCoordination(self):
        ''' 
            绘制坐标轴
        '''
        #绘制禁止抓取区域
        pygame.draw.circle(self.win,(120,120,120),(self.width/2,self.height/2), 250, width=0)
        pygame.draw.circle(self.win,(50,50,50),(self.width/2,self.height/2), 100, width=0)
        #先画横坐标
        pygame.draw.line(self.win,(50,200,250),(10,self.height/2),(self.width-10,self.height/2),2)
        #再画纵坐标
        pygame.draw.line(self.win,(50,200,250),(self.width/2,10),(self.width/2,self.height-10),2)
        
    
    def t_UpdateImage(self):
        '''
            更新可视化界面
        '''
        pygame.display.set_caption('Robotic arm control')
        clock=pygame.time.Clock()
        while not rospy.is_shutdown():
            clock.tick(60)
            self.win.fill((50,50,50))
            self.DrawCoordination()
            for pos in self.Objs:
               pygame.draw.circle(self.win,(255,255,0),pos, 10, width=2)
            self.DetectKeyEvent()
            pygame.display.update()
    def DetectKeyEvent(self):
        ''' 
            检测键盘事件 
        '''
        for event in pygame.event.get():  
            if event.type==pygame.MOUSEBUTTONDOWN:
                #判断鼠标左键点击事件
                if event.button==1:
                    x, y = pygame.mouse.get_pos()
                    self.Objs.append((x,y))
                    x,y=self.Pixel2RealXY(x,y)
                    p=PoseStamped()
                    p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=-0.075
                    self.tar_Pub.publish(p)

if __name__=='__main__':
    cam=Cam_Analyzer()
    rospy.spin()

        