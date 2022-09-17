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
    def __init__(self,cam_id='/dev/video0',width=1280,height=960,\
        cx=640,cy=480,\
        leftBottom_Coor=[0.12,0.10],\
        rightTop_Coor=[0.41,-0.105],\
        hsv_low=[0,90,150],\
        hsv_high=[255,230,255],\
        calib_X=-0.025,\
        calib_Y=0.01):

        #配置参数
        self.width=width
        self.height=height
        self.cx=cx
        self.cy=cy
        self.lB=leftBottom_Coor
        self.rT=rightTop_Coor
        self.calib_X=calib_X
        self.calib_Y=calib_Y

        self.frame=np.zeros((1,1))
        self.frame_Raw=np.zeros((1,1))
        self.HSV_LOW=np.array(hsv_low)
        self.HSV_HIGH=np.array(hsv_high)

        self.Objs=[]

        # 初始化摄像头模块病氦气摄像头采集线程
        self.haveFrame=False
        self.cap=cv2.VideoCapture(cam_id)
        self.t1=threading.Thread(target=self.t_CamAquire,args=())
        self.t1.setDaemon(True)
        self.t1.start()
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
        # 开启线程
        self.t2=threading.Thread(target=self.t_UpdateImage,args=())
        self.t2.setDaemon(True)
        self.t2.start()

        #开启图像处理线程
        self.t3=threading.Thread(target=self.t_ImageProcess,args=())
        self.t3.setDaemon(True)
        self.t3.start()
        
    
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
        #先画横坐标
        pygame.draw.line(self.win,(50,50,250),(100,self.height-100),(self.width-100,self.height-100),2)
        #再画纵坐标
        pygame.draw.line(self.win,(50,50,250),(100,100),(100,self.height-100),2)
        #绘制刻度线

    def t_CamAquire(self):
        '''
            获取摄像头原始数据
        '''
        
        while not rospy.is_shutdown():
            
            time.sleep(0.0001)
            ret,frame_Raw=self.cap.read()
            self.frame_Raw=cv2.resize(frame_Raw,(self.width,self.height))
            if ret: self.haveFrame=True
            else: self.haveFrame=False
    
    def t_UpdateImage(self):
        '''
            更新可视化界面
        '''
        pygame.display.set_caption('Robotic arm control')
        clock=pygame.time.Clock()
        while not rospy.is_shutdown():
            clock.tick(60)
            self.win.fill((50,50,50))
            if not self.haveFrame: continue
            #复制一份Frame
            self.frame=np.copy(self.frame_Raw)
            for x,y,box in self.Objs:
                cv2.drawContours(self.frame,[box],0,(0,0,255),2)
            surfFrame=cv2.transpose(cv2.cvtColor(np.copy(self.frame), cv2.COLOR_RGB2BGR))
            surf=make_surface(surfFrame)
            self.win.blit(surf,(0,0))
            self.DetectKeyEvent()
            pygame.display.update()
        
    def Cam_Process(self):
        '''
            处理摄像头数据,识别场景物体
        '''
        if not self.haveFrame: return
        #Debug,测试时间长度
        t0=time.time()
        '''
            处理摄像头数据
        '''
        frame=self.frame_Raw
        frame=cv2.resize(frame,(self.width,self.height))
        '''
            1. 首先,将场景中的物体色度空间转化为HSV空间,由于背景是白色(S<30),所以通过S可以提取出场景中的物体
        '''
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_Frame=cv2.inRange(frame,self.HSV_LOW,self.HSV_HIGH)
        '''
            2. 去噪,使用 cv2.morphologyEx（img，cv2.MORPH_CLOSE，内核）去掉内部黑色噪点
        '''
        kernel=np.ones((5,5),np.uint8)
        mask_Frame=cv2.morphologyEx(mask_Frame,cv2.MORPH_CLOSE,kernel,2)
        '''
            3. 再次使用 Erode 去掉白色区域
        '''
        kernel=np.ones((5,5),np.uint8)
        mask_Frame=cv2.erode(mask_Frame,kernel)
        distance_Frame=cv2.distanceTransform(mask_Frame,cv2.DIST_L2,5)
        ret,target_Frame = cv2.threshold(np.copy(distance_Frame),0.1*distance_Frame.max(),255,0)
        target_Frame=np.uint8(target_Frame)
        '''
            轮廓检测,确定每个物体的位置
        '''
        im2,contours,hierarchy = cv2.findContours(target_Frame, cv2.RETR_EXTERNAL, 2)
        '''
            按照轮廓标定物体位置
        '''

        self.Objs=[]
        for cont in contours:
            #过小范围的区域,忽略
            if cv2.contourArea(cont)<200: continue
            #计算中心点
            M = cv2.moments(cont)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            '''
                提取轮廓
            '''
            rect = cv2.minAreaRect(cont)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            self.Objs.append([cx,cy,box])
        '''
            调试数据
        '''
        cv2.imshow('Debug',cv2.resize(frame,(640,480)))
        cv2.waitKey(10)
        cv2.imshow('Debug2',cv2.resize(mask_Frame,(640,480)))

    def t_ImageProcess(self):
        '''
            独立线程,处理程序
        '''
        while not rospy.is_shutdown():
            time.sleep(0.03)
            self.Cam_Process()

    def DetectKeyEvent(self):
        ''' 
            检测键盘事件 
        '''
        for event in pygame.event.get():  
            if event.type==pygame.MOUSEBUTTONDOWN:
                #判断鼠标左键点击事件
                if event.button==1:
                    self.Cam_Process()
                    
                    x, y = pygame.mouse.get_pos()
                    print self.frame[y][x]
                    x,y=self.Pixel2RealXY(x,y)
                    p=PoseStamped()
                    p.pose.position.x=x;p.pose.position.y=y;p.pose.position.z=-0.075
                    self.tar_Pub.publish(p)

if __name__=='__main__':
    cam=Cam_Analyzer()
    rospy.spin()

        