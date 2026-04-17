#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
自主装载机器人模块模拟节点
模拟各个模块的功能，用于测试状态机
"""

import rospy
import sys
import select
import termios
import tty
import random
import math
from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import PoseStamped
from autonomous_loader_msgs.msg import TaskCommand, ScoopPointRequest, TaskProgress

class ModuleSimulator:
    def __init__(self):
        rospy.init_node('module_simulator', anonymous=True)
        
        # 发布者
        self.work_device_completed_pub = rospy.Publisher('work_device/task_completed', Bool, queue_size=10)
        self.arm_raised_pub = rospy.Publisher('work_device/arm_raised', Bool, queue_size=10)
        self.bucket_lowered_pub = rospy.Publisher('work_device/bucket_lowered', Bool, queue_size=10)
        
        self.scoop_completed_pub = rospy.Publisher('scoop/scoop_completed', Bool, queue_size=10)
        
        self.navigation_planning_success_pub = rospy.Publisher('navigation/planning_success', Bool, queue_size=10)
        self.navigation_arrived_pub = rospy.Publisher('navigation/arrived', Bool, queue_size=10)
        self.distance_to_goal_pub = rospy.Publisher('navigation/distance_to_goal', Float64, queue_size=10)
        self.current_pose_pub = rospy.Publisher('navigation/current_pose', PoseStamped, queue_size=10)
        
        self.scoop_point_response_pub = rospy.Publisher('scoop_point/response', PoseStamped, queue_size=10)
        
        # 订阅者
        self.raise_arm_sub = rospy.Subscriber('work_device/raise_arm', Bool, self.raiseArmCallback)
        self.lower_bucket_sub = rospy.Subscriber('work_device/lower_bucket', Bool, self.lowerBucketCallback)
        
        self.start_scoop_sub = rospy.Subscriber('scoop/start_scoop', Bool, self.startScoopCallback)
        
        self.navigation_goal_sub = rospy.Subscriber('navigation/goal', PoseStamped, self.navigationGoalCallback)
        self.navigation_pause_sub = rospy.Subscriber('navigation/pause', Bool, self.navigationPauseCallback)
        
        self.scoop_point_request_sub = rospy.Subscriber('scoop_point/request', ScoopPointRequest, self.scoopPointRequestCallback)
        
        # 状态
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "map"
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation.w = 1.0
        
        self.target_pose = None
        self.navigation_active = False
        self.distance_to_goal = 0.0
        
        # 设置终端为非阻塞模式
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        rospy.loginfo("模块模拟器已启动")
        rospy.loginfo("按键说明:")
        rospy.loginfo("  s - 发送开始任务")
        rospy.loginfo("  p - 发送暂停任务")
        rospy.loginfo("  e - 发送结束任务")
        rospy.loginfo("  r - 模拟工作装置完成")
        rospy.loginfo("  c - 模拟铲料完成")
        rospy.loginfo("  n - 模拟导航到达")
        rospy.loginfo("  q - 退出")
        rospy.loginfo("  h - 显示帮助")
        
        # 创建定时器模拟导航
        self.nav_timer = rospy.Timer(rospy.Duration(0.1), self.simulateNavigation)
        
    def simulateNavigation(self, event):
        """模拟导航过程"""
        if self.navigation_active and self.target_pose:
            # 计算距离
            dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
            dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            self.distance_to_goal = distance
            
            # 发布距离信息
            distance_msg = Float64()
            distance_msg.data = distance
            self.distance_to_goal_pub.publish(distance_msg)
            
            # 模拟移动
            if distance > 0.5:  # 如果距离大于0.5米，继续移动
                move_speed = 0.5  # 0.5米/秒
                move_distance = min(move_speed * 0.1, distance - 0.5)  # 每0.1秒移动的距离
                
                # 计算移动方向
                if distance > 0:
                    move_x = (dx / distance) * move_distance
                    move_y = (dy / distance) * move_distance
                    
                    self.current_pose.pose.position.x += move_x
                    self.current_pose.pose.position.y += move_y
            else:
                # 到达目标
                self.navigation_active = False
                arrived_msg = Bool()
                arrived_msg.data = True
                self.navigation_arrived_pub.publish(arrived_msg)
                rospy.loginfo("导航到达目标位置")
            
            # 发布当前位置
            self.current_pose.header.stamp = rospy.Time.now()
            self.current_pose_pub.publish(self.current_pose)
    
    def raiseArmCallback(self, msg):
        """抬大臂指令回调"""
        if msg.data:
            rospy.loginfo("收到抬大臂指令")
            # 模拟延迟后完成
            rospy.Timer(rospy.Duration(2.0), self.raiseArmCompleted, oneshot=True)
    
    def raiseArmCompleted(self, event):
        """抬大臂完成"""
        completed_msg = Bool()
        completed_msg.data = True
        self.work_device_completed_pub.publish(completed_msg)
        
        raised_msg = Bool()
        raised_msg.data = True
        self.arm_raised_pub.publish(raised_msg)
        rospy.loginfo("抬大臂完成")
    
    def lowerBucketCallback(self, msg):
        """放平铲斗指令回调"""
        if msg.data:
            rospy.loginfo("收到放平铲斗指令")
            # 模拟延迟后完成
            rospy.Timer(rospy.Duration(1.5), self.lowerBucketCompleted, oneshot=True)
    
    def lowerBucketCompleted(self, event):
        """放平铲斗完成"""
        completed_msg = Bool()
        completed_msg.data = True
        self.work_device_completed_pub.publish(completed_msg)
        
        lowered_msg = Bool()
        lowered_msg.data = True
        self.bucket_lowered_pub.publish(lowered_msg)
        rospy.loginfo("放平铲斗完成")
    
    def startScoopCallback(self, msg):
        """开始铲料指令回调"""
        if msg.data:
            rospy.loginfo("收到开始铲料指令")
            # 模拟延迟后完成
            rospy.Timer(rospy.Duration(3.0), self.scoopCompleted, oneshot=True)
    
    def scoopCompleted(self, event):
        """铲料完成"""
        completed_msg = Bool()
        completed_msg.data = True
        self.scoop_completed_pub.publish(completed_msg)
        rospy.loginfo("铲料完成")
    
    def navigationGoalCallback(self, msg):
        """导航目标回调"""
        self.target_pose = msg
        self.navigation_active = True
        
        # 模拟路径规划成功
        planning_msg = Bool()
        planning_msg.data = True
        self.navigation_planning_success_pub.publish(planning_msg)
        
        rospy.loginfo("收到导航目标: (%.2f, %.2f)", 
                     msg.pose.position.x, msg.pose.position.y)
    
    def navigationPauseCallback(self, msg):
        """暂停导航回调"""
        if msg.data:
            self.navigation_active = False
            rospy.loginfo("导航已暂停")
    
    def scoopPointRequestCallback(self, msg):
        """铲料点请求回调"""
        rospy.loginfo("收到铲料点请求，料仓ID: %d", msg.bin_id)
        
        # 模拟延迟后响应
        rospy.Timer(rospy.Duration(1.0), lambda event: self.sendScoopPointResponse(msg.bin_id), oneshot=True)
    
    def sendScoopPointResponse(self, bin_id):
        """发送铲料点响应"""
        response = PoseStamped()
        response.header.frame_id = "map"
        response.header.stamp = rospy.Time.now()
        
        # 模拟铲料点（有时返回(0,0)）
        if random.random() < 0.3:  # 30%概率返回(0,0)
            response.pose.position.x = 0.0
            response.pose.position.y = 0.0
            rospy.loginfo("返回铲料点(0,0)")
        else:
            # 随机生成铲料点
            response.pose.position.x = random.uniform(5.0, 15.0)
            response.pose.position.y = random.uniform(5.0, 15.0)
            rospy.loginfo("返回铲料点: (%.2f, %.2f)", 
                         response.pose.position.x, response.pose.position.y)
        
        response.pose.position.z = 0.0
        response.pose.orientation.w = 1.0
        
        self.scoop_point_response_pub.publish(response)
    
    def sendStartTask(self):
        """发送开始任务"""
        task_msg = TaskCommand()
        task_msg.task_id = random.randint(1, 100)
        task_msg.bin_id = random.randint(1, 3)
        task_msg.hopper_id = random.randint(1, 3)
        task_msg.task_type = "scoop"
        
        # 发布任务
        start_task_pub = rospy.Publisher('scheduler/start_task', TaskCommand, queue_size=10)
        start_task_pub.publish(task_msg)
        
        rospy.loginfo("发送开始任务: ID=%d, 料仓=%d, 料斗=%d", 
                     task_msg.task_id, task_msg.bin_id, task_msg.hopper_id)
    
    def sendPauseTask(self):
        """发送暂停任务"""
        pause_msg = Bool()
        pause_msg.data = True
        
        pause_pub = rospy.Publisher('scheduler/pause_task', Bool, queue_size=10)
        pause_pub.publish(pause_msg)
        
        rospy.loginfo("发送暂停任务指令")
    
    def sendEndTask(self):
        """发送结束任务"""
        end_msg = Bool()
        end_msg.data = True
        
        end_pub = rospy.Publisher('scheduler/end_task', Bool, queue_size=10)
        end_pub.publish(end_msg)
        
        rospy.loginfo("发送结束任务指令")
    
    def simulateWorkDeviceCompletion(self):
        """模拟工作装置完成"""
        completed_msg = Bool()
        completed_msg.data = True
        self.work_device_completed_pub.publish(completed_msg)
        rospy.loginfo("模拟工作装置任务完成")
    
    def simulateScoopCompletion(self):
        """模拟铲料完成"""
        completed_msg = Bool()
        completed_msg.data = True
        self.scoop_completed_pub.publish(completed_msg)
        rospy.loginfo("模拟铲料任务完成")
    
    def simulateNavigationArrival(self):
        """模拟导航到达"""
        arrived_msg = Bool()
        arrived_msg.data = True
        self.navigation_arrived_pub.publish(arrived_msg)
        rospy.loginfo("模拟导航到达目标位置")
    
    def print_help(self):
        """打印帮助信息"""
        print("\n=== 模块模拟器帮助 ===")
        print("按键说明:")
        print("  s - 发送开始任务")
        print("  p - 发送暂停任务")
        print("  e - 发送结束任务")
        print("  r - 模拟工作装置完成")
        print("  c - 模拟铲料完成")
        print("  n - 模拟导航到达")
        print("  q - 退出")
        print("  h - 显示帮助")
        print("========================")
    
    def run(self):
        """主循环"""
        try:
            while not rospy.is_shutdown():
                # 检查是否有按键输入
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == 'q':
                        break
                    elif key == 's':
                        self.sendStartTask()
                    elif key == 'p':
                        self.sendPauseTask()
                    elif key == 'e':
                        self.sendEndTask()
                    elif key == 'r':
                        self.simulateWorkDeviceCompletion()
                    elif key == 'c':
                        self.simulateScoopCompletion()
                    elif key == 'n':
                        self.simulateNavigationArrival()
                    elif key == 'h':
                        self.print_help()
                        
        except KeyboardInterrupt:
            pass
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print("\n模块模拟器已退出")

if __name__ == '__main__':
    try:
        simulator = ModuleSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass
