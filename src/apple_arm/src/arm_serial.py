#!/usr/bin/env python
#coding=utf-8
import serial
import numpy as np
import threading
import time
import rospy
from sensor_msgs.msg import JointState
class Serial_Arm:
    def __init__(self,port='/dev/ttyUSB0'):
        '''
            串口初始化
            port要更换为自己的接口
        '''
        self.ser=serial.Serial(port,9600,timeout=3.0) #定义一个串口类
        self.ser.close()
        self.vals=[255,255,127,127,127,127,127,127,127,127]        #定义一个数据包
        '''
            ROS接口初始化
            读取JointState类型数据
        '''
        rospy.init_node("arm_serial_driver", anonymous=True)  #False代表一次只能开启一个节点
        #接收消息
        rospy.Subscriber('/arm_joints',JointState,self.J_Update)
        
    
    def J_Update(self,msg):
        '''接收到虚拟环境中的机械臂关节数据'''
        #获得关节数据矩阵
        posArr=msg.position
        #检查关节数量是否匹配
        if len(posArr)!=6: 
            rospy.logerr('Data mismatch！')
            return
        #建立一个8变量的矩阵（主板最多支持8个关节）
        vals=[0,0,0,0,0,0,0,0]
         #把接受到的数据赋值给关节
        vals[2:8]=posArr
        vals[3]=-vals[3]
         #把接受到的数据赋值给关节
        self.Set_Arm_Values(vals)

    def Set_Arm_Values(self,dat):
        '''串口接口，把dat[8字节]发送出去'''
        if(len(dat))!=8:
            print('机械臂关节数量输入错误！')
            return
        vals=[]
        for i in dat:
            val = int(i * 255 / (np.pi*180/120) + 127)
            if val < 0: val = 0
            if val > 255: val = 255
            vals.append(val)
        self.vals[2:10]=vals
        pPush=self.vals
        self.ser.open()
        self.ser.write(pPush)
        self.ser.close()

if __name__=='__main__':
    s=Serial_Arm()
    rospy.spin()
    s.ser.close()
   
    




