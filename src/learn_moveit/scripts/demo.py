#!/usr/bin/env python
#coding=utf-8
'''
  这个程序原本是Acorn Pooley, Mike Lautman提供的教学程序
  程序版权所属原作者Acorn Pooley, Mike Lautman，Applehang仅作开源分享
  本程序由Applehang进行精简和汉化，内容与原教学程序不同，但更好理解～
  根据开源软件BSD协议，本程序禁止任何形式的任何商业化行为，转载需保留原作者及汉化信息
  由于弱智的Linux输入法问题，以下“机械必”=机械臂
'''


import sys
import copy
import rospy
import moveit_commander   #moveit_commander是MoveIt体系下Move Group的接口
import moveit_msgs.msg      #
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



def all_close(goal, actual, tolerance):
  """
  检测机械必是否已经按照预定到达了目标位置
  @param: goal      一组浮点数，检查机械必的目标位置
  @param: actual     机械必的当前位置
  @param: tolerance  容忍度，也就是允许误差
  @returns:   到达目标返回True，没到就返回False
  """
  #这里用了递归，首先将PoseStamp传递为Pose,然后将Pose转化为List，再进行处理
  #妙！
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """封装一个Movet Python接口的类"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    #首先初始化Movet Commander接口
    moveit_commander.roscpp_initialize(sys.argv)
    #anonymous=True，允许同时运行多个节点
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    #实例一个RobotCommander对象
    robot = moveit_commander.RobotCommander()
    #实例一个场景描述对象
    scene = moveit_commander.PlanningSceneInterface()
    ##Moveroup Commander对象
    move_group = moveit_commander.MoveGroupCommander("panda_arm")

    #这个在机械臂路径规划中没什么用，使用来在RVIZ中显示路径用的
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # 显示move_group规划的全局坐标系
    planning_frame = move_group.get_planning_frame()
    
    # 获得末端执行器的link
    eef_link = move_group.get_end_effector_link()
 
    # 获得机械臂中的所有执行组
    group_names = robot.get_group_names()

    # 显示整个机器人的当前状态
    print "============ 机械臂状态"
    print robot.get_current_state()

    # 缓存变量
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    """
      为机械臂设定目标关节状态，并控制机械按照指定关节状态运动
    """
    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = -pi/4
    joint_goal[3] = -pi/2
    joint_goal[4] = -pi/4
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # go方法开始执行机械臂运动，wait=True  等待机械臂运动完成
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    #检查关节状态，是否运行成功
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):
    """
      为机械臂设定末端关节的位置姿态，并控制机械臂将末端关节移动过去
    """
    move_group = self.move_group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    #在这里可以配置姿态
    #pose_goal.orientation.x=0.5
    #配置目标状态
    move_group.set_pose_target(pose_goal)
    #开始运动
    plan = move_group.go(wait=True)
    move_group.stop()
     # 运动后最好清除已经缓存的位置
    move_group.clear_pose_targets()
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.3
    pose_goal.position.z = 0.4
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    pose_goal.position.x = 0.6
    pose_goal.position.y = 0.3
    pose_goal.position.z = 0.4
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    pose_goal.position.x = 0.6
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    #检查运动的完成情况
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    # 沿着路径运动
    move_group = self.move_group
    #配置路径点
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1 
    wpose.position.y += scale * 0.2  
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1 
    waypoints.append(copy.deepcopy(wpose))

    # 让机械臂自动规划处一条运动路线
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # 路径点
                                       0.01,        # 运动步长
                                       0.0)         # jump_threshold

    # 在这里我们暂时并没有让机械臂运动
    return plan, fraction
  def display_trajectory(self, plan):
    """
      显示机器人的运动路径
    """
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    '''
      让机械臂执行Plan
      注意，机械臂的当前位置必须与Plan中的第一个位置点差得不能太远，否则机械臂会执行失败
    '''
    move_group = self.move_group
    move_group.execute(plan, wait=True)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    
    box_name = self.box_name
    scene = self.scene


def main():
  try:

    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ 按回车让机械臂摆出一个姿态！..."
    raw_input()
    tutorial.go_to_joint_state()

    print "============ 按回车让机械臂画一个“口”字..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ 按回车让机器人规划关键点路径"
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print "============ 按回车让机器人延路径运动"
    raw_input()
    tutorial.execute_plan(cartesian_plan)

    print "============ 演示完成！"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
