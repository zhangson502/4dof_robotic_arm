#!/usr/bin/env python
#coding=utf-8
from arm_kinematic import *
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32,UInt8
from sensor_msgs.msg import JointState
import threading
class Arm_ROS_Bridge:
    '''
        机械臂姿态控制ROS接口：
        功能：
        1）接收Pose，设置机器人末端机械臂Joint
        2）接收Float，设置机械臂末端爪子的抓紧
    '''
    def __init__(self):
        '''
            初始化
        '''
        #   消息接收
        rospy.Subscriber('/target_trasient_pose',PoseStamped,callback=self.poseCb)     #接收目标位置
        rospy.Subscriber('/clip_val',Float32,callback=self.clipCb)     #接收到爪子数据
        
        #   发布消息
        self.j_Pub=rospy.Publisher('arm_joints',JointState,queue_size=5)
        self.p_Pub=rospy.Publisher('arm_pose',PoseStamped,queue_size=5,latch=True)

        #数据缓存
        self.jointState=JointState()        #关节
        self.poseState=PoseStamped()
        self.base_frame=rospy.get_param('~base_frame','/base_link')
        self.jointState.header.frame_id=self.base_frame
        self.jointState.name=['J0','J1','J2','J3','J4','clip']
        self.jointState.position=[0.0,0.0,0.0,0.0,0.0,0]    #6字节
        
        #开启机械臂末端位置发布线程
        self.t=threading.Thread(target=self.t_PosePush,args=())
        self.t.setDaemon(True)
        self.t.start()

    def t_PosePush(self):
        '''
            发送末端当前位置信息
        '''
        self.poseState.header.frame_id=self.base_frame
        #循环发布
        while True:
            self.poseState.pose.position.x,\
                self.poseState.pose.position.y,\
                self.poseState.pose.position.z,\
                self.poseState.pose.orientation.x,\
                self.poseState.pose.orientation.y,\
                self.poseState.pose.orientation.z=CalcPos(self.jointState.position)
            self.p_Pub.publish(self.poseState)
            time.sleep(0.1)

    def clipCb(self,msg):
        '''
            接收到夹子信息
        '''
        self.jointState.position[5]=msg.data
        self.PubJoints()

    def poseCb(self,msg):
        '''
            接收到位置信息
            机械臂会直接吧位置摆到这个姿态
            注意：
            这里msg的旋转是自定义的旋转，跟ROS的Orientation无关
        '''
        #计算机械臂的关节姿态
        j_Pos,j_Neg=InvKinematicResolve_4dof(msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z)

        #选择姿态
        if len(j_Pos)==0:
            rospy.logwarn('[arm-resolver] target invalid')
            return
        center_Index=int(len(j_Pos)/2)
        self.jointState.position[0:5]=j_Pos[center_Index]
        self.PubJoints()
        #发布Joints数据
    def PubJoints(self):
        '''
            发布机械臂关节数据
            在这里进行
        '''
        self.j_Pub.publish(self.jointState)

if __name__=='__main__':
    rospy.init_node('arm_joints_resolver')
    arm=Arm_ROS_Bridge()
    rospy.spin()