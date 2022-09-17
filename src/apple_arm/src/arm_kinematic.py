#!/usr/bin/env python
#coding=utf-8
import numpy as np
import time
def InvKinematicResolve_4dof(
    x,y,z,roll,pitch,yaw,
    errXYZ=[0.001,0.001,0.001],
    errRPY=[3.14,0.78,3.14],
    L1=0.105,L2=0.098,L3=0.165,
    j0resvRange=[-1.5,1.5,0.01],
    j_Limits=1.8):
    '''
        机械臂的逆向解算器(4自由度机械臂)
        从X，Y，Z坐标到关节旋转值的解算
        @param:x,y,z,r,p,y    [六个值描述机械臂的位置和姿态]
        @param:errXYZ         [三维坐标系下x，y,z三轴误差]
        @param：erRPPPY        [三维旋转误差]
        @param: L1-L3           [机械臂三条link的长度]
        @param: j0resvRange     [解算的迭代范围]
        @j_Limits:                 [每个舵机的最大转角（120度）]
    '''
    Joints_Pos=[]   #关节数值的返回值(A姿态)
    Joints_Neg=[]   #关节数值的返回值（B姿态）
    #获取第一关节J0旋转值
    #J0的旋转为后续机械臂的旋转构建了一个新的2D平面
    j0=np.arctan2(y,x)
    #将新的映射平面等效为一个XY平面，那么机械臂末端映射在这个平面的横坐标为：
    if x==0:plateX=y
    else: plateX=x/np.cos(j0)     
    plateY=z        #平面的Y轴与之等效
    '''
        由三维坐标系映射到J0-J4，会有多个解的情况
        使用穷举法求解
    '''
    #构建一组j1列表
    j1_List=np.arange(j0resvRange[0],j0resvRange[1],j0resvRange[2],dtype=np.float32)
    #   基于平面三角函数计算j3的数值
    #   (a-L1*sinj1)^2+(b-L1*cosj1)^2=L2^2+L3^2-cos(j3)*L3*L2
    #   逆向求解
    ''' 首先，构造出L2，L3两条臂合成的三角形第三边边长'''
    fL1=np.sqrt(np.power(plateX-L1*np.sin(j1_List),2)+np.power(plateY-L1*np.cos(j1_List),2))
    

    ''' 利用余弦定理求解j3的数据'''
    #   J2,J3应该有两组解
    cosj3=(fL1*fL1-L2*L2-L3*L3)/(2*L2*L3)
    j3_List_Pos=np.arccos(cosj3)      #解A姿态
    j3_List_Neg=-j3_List_Pos                #解B姿态
    ''' 求解j2的数据 '''
    #首先计算辅助角度th_1
    costh_1=(fL1*fL1+L2*L2-L3*L3)/(2*fL1*L2)
    th_1_Pos=np.arccos(costh_1)             #解A姿态
    th_1_Neg=-th_1_Pos                      #解B姿态
    #构造辅助线fL2的长度
    fL2=np.sqrt(np.power(plateX,2)+np.power(plateY,2))
    #计算角度
    cos_th_2=(fL2*fL2-L1*L1-fL1*fL1)/(2*fL1*L1)
    th_2=np.arccos(cos_th_2)
    j2_List_Pos=th_2-th_1_Pos
    j2_List_Neg=th_2-th_1_Neg

    '''
        接下来进行回环检验，首先进行正姿态校验
    '''

    '''基于上述J0-J4，再次还原机械臂位置姿态'''
    #首先还原x，y平面的特征线长
    xy_Plate_x=L1*np.sin(j1_List)+L2*np.sin(j1_List+j2_List_Pos)+L3*np.sin(j1_List+j2_List_Pos+j3_List_Pos)
    #再还原X，Y，Z坐标
    x_Restored=xy_Plate_x*np.cos(j0)
    y_Restored=xy_Plate_x*np.sin(j0)
    z_Restored=L1*np.cos(j1_List)+L2*np.cos(j1_List+j2_List_Pos)+L3*np.cos(j1_List+j2_List_Pos+j3_List_Pos)
    '''基于上述J0-J4，再次还原机械臂旋转姿态'''
    yaw_Restored=j0
    pitch_Restored=j1_List+j2_List_Pos+j3_List_Pos
    roll_Restored=roll
    
    #校验
    for i in range(len(j1_List)):
        #如果无解，返回
        if np.isnan(j1_List[i]) or np.isnan(j2_List_Pos[i])  or np.isnan(j3_List_Pos[i]) : continue
        #如果关节超出限度，返回
        if np.abs(j1_List[i])>j_Limits or np.abs(j2_List_Pos[i])>j_Limits or np.abs(j3_List_Pos[i])>j_Limits: continue
        #print x_Restored[i],z_Restored[i],pitch_Restored[i]
        #如果坐标对不上，返回
        if(np.abs(x-x_Restored[i])>errXYZ[0] or np.abs(y-y_Restored[i])>errXYZ[1] or np.abs(z-z_Restored[i])>errXYZ[2]): continue
        if(np.abs(roll-roll_Restored)>errRPY[0] or np.abs(pitch-pitch_Restored[i])>errRPY[1] or np.abs(yaw-yaw_Restored)>errRPY[2]): continue
        Joints_Pos.append([j0,j1_List[i],j2_List_Pos[i],j3_List_Pos[i],roll])
    '''
        接下来进行负姿态校验
    '''
    '''基于上述J0-J4，再次还原机械臂位置姿态'''
    #首先还原x，y平面的特征线长
    xy_Plate_x=L1*np.sin(j1_List)+L2*np.sin(j1_List+j2_List_Neg)+L3*np.sin(j1_List+j2_List_Neg+j3_List_Neg)
    #再还原X，Y，Z坐标
    x_Restored=xy_Plate_x*np.cos(j0)
    y_Restored=xy_Plate_x*np.sin(j0)
    z_Restored=L1*np.cos(j1_List)+L2*np.cos(j1_List+j2_List_Neg)+L3*np.cos(j1_List+j2_List_Neg+j3_List_Neg)
    '''基于上述J0-J4，再次还原机械臂旋转姿态'''
    yaw_Restored=j0
    pitch_Restored=j1_List+j2_List_Neg+j3_List_Neg
    roll_Restored=roll
    #校验
    for i in range(len(j1_List)):
        #如果无解，返回
        if np.isnan(j1_List[i]) or np.isnan(j2_List_Neg[i])  or np.isnan(j3_List_Neg[i]) : continue
        #如果关节超出限度，返回
        if np.abs(j1_List[i])>j_Limits or np.abs(j2_List_Neg[i])>j_Limits or np.abs(j3_List_Neg[i])>j_Limits: continue
       
        #如果坐标对不上，返回
        if np.abs(x-x_Restored[i])>errXYZ[0] or np.abs(y-y_Restored[i])>errXYZ[1] or np.abs(z-z_Restored[i])>errXYZ[2] : continue
        if np.abs(roll-roll_Restored)>errRPY[0] or np.abs(pitch-pitch_Restored[i])>errRPY[1] or np.abs(yaw-yaw_Restored)>errRPY[2]: continue
        Joints_Neg.append([j0,j1_List[i],j2_List_Neg[i],j3_List_Neg[i],roll])
    return Joints_Pos,Joints_Neg

def CalcPos(j,L1=0.105,L2=0.098,L3=0.165):
    ''' 
        获取机械臂末端的当前位置
        j: 长度为6的列表
    '''
    xy_Plate_x=L1*np.sin(j[1])+L2*np.sin(j[1]+j[2])+L3*np.sin(j[1]+j[2]+j[3])
    #还原三轴姿态
    x_Restored=xy_Plate_x*np.cos(j[0])
    y_Restored=xy_Plate_x*np.sin(j[0])
    z_Restored=L1*np.cos(j[1])+L2*np.cos(j[1]+j[2])+L3*np.cos(j[1]+j[2]+j[3])
    yaw_Restored=j[0]
    pitch=j[1]+j[2]+j[3]
    roll=j[4]
    return x_Restored,y_Restored,z_Restored,yaw_Restored,pitch,roll

if __name__=='__main__':
    t=time.time()
    ret=InvKinematicResolve_4dof(0.3,0.00,0.0,0.0,0.78+1.57,0.0)
    print time.time()-t
