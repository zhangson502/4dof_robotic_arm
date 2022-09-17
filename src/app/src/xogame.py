#!/usr/bin/env python
#coding=utf-8
import rospy
import time
from std_msgs.msg import Float32MultiArray

class XO_Game:
    '''
        小游戏控制器
    '''
    def __init__(self,canvas_Origin=[0.27,-0.05],canvas_Size=0.1):
        '''
            初始化棋盘
        '''
        self.o_Pub=rospy.Publisher('/draw_circle',Float32MultiArray,queue_size=5)
        self.x_Pub=rospy.Publisher('/draw_x',Float32MultiArray,queue_size=5)
        self.l_Pub=rospy.Publisher('/draw_line',Float32MultiArray,queue_size=5)
        self.canvas_Origin=canvas_Origin
        self.canvas_Size=canvas_Size
        self.cell_Size=canvas_Size/3.0
    def DrawCircle(self,x,y):
        '''
          2  |      |  2
        _______________
          1  |      |  
        _______________
          0  |  1   |  2
        '''
        cache=Float32MultiArray()
        cx=(x+0.5)*self.cell_Size+self.canvas_Origin[0]
        cy=(x+0.5)*self.cell_Size+self.canvas_Origin[1]
        cache.data=[self.cell_Size*0.8,cx,cy]
        print cache
        self.o_Pub.publish(cache)

    def DrawX(self,x,y):
        '''
            画叉叉
        '''
        cache=Float32MultiArray()
        cx=(x+0.5)*self.cell_Size+self.canvas_Origin[0]
        cy=(x+0.5)*self.cell_Size+self.canvas_Origin[1]
        cache.data=[self.cell_Size*0.8,cx,cy]
        print cache
        self.x_Pub.publish(cache)
    def DrawPlate(self):
        cache=Float32MultiArray()
        #第一条线
        x1=self.canvas_Origin[0];y1=self.canvas_Origin[1]+self.cell_Size;x2=self.canvas_Origin[0]+self.canvas_Size;y2=y1
        cache.data=[x1,y1,x2,y2]
        print cache.data
        self.l_Pub.publish(cache)
        time.sleep(3)
        #第二条线
        x1=self.canvas_Origin[0];y1=self.canvas_Origin[1]+self.cell_Size*2;x2=self.canvas_Origin[0]+self.canvas_Size;y2=y1
        cache.data=[x1,y1,x2,y2]
        self.l_Pub.publish(cache)
        time.sleep(3)
        #第三条线
        x1=self.canvas_Origin[0]+self.cell_Size*1;y1=self.canvas_Origin[1];x2=x1;y2=y1+self.canvas_Size
        cache.data=[x1,y1,x2,y2]
        self.l_Pub.publish(cache)
        time.sleep(3)
        #第四条线
        x1=self.canvas_Origin[0]+self.cell_Size*2;y1=self.canvas_Origin[1];x2=x1;y2=y1+self.canvas_Size
        cache.data=[x1,y1,x2,y2]
        self.l_Pub.publish(cache)
        time.sleep(3)


if __name__=='__main__':
    rospy.init_node('app_xogame')
    game=XO_Game()
    time.sleep(1)
    # #game.DrawPlate()
    # game.DrawCircle(1,1)
    # time.sleep(8)
    # game.DrawCircle(1,0)
    # time.sleep(8)
    # game.DrawCircle(0,1)
    # time.sleep(5)
    # game.DrawCircle(0,0)
    # time.sleep(8)
    cache=Float32MultiArray()
    cache.data=[0.29,-0.06,0.32,0.06]
    game.l_Pub.publish(cache)
    time.sleep(4)
    cache.data=[0.32,0.06,0.33,-0.06]
    game.l_Pub.publish(cache)
    time.sleep(4)
    cache.data=[0.33,-0.06,0.29,-0.06]
    game.l_Pub.publish(cache)
    #rospy.spin()