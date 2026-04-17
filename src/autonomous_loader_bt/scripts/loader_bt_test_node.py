#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
自主装载机器人行为树测试节点
用于发送各种指令来测试行为树的功能
"""

import rospy
import sys
import select
import termios
import tty
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs

class LoaderBTTestNode:
    def __init__(self):
        rospy.init_node('loader_bt_test_node', anonymous=True)
        
        # 发布者
        self.end_task_pub = rospy.Publisher('loader_bt/end_task', Bool, queue_size=10)
        self.pause_task_pub = rospy.Publisher('loader_bt/pause_task', Bool, queue_size=10)
        self.add_task_pub = rospy.Publisher('loader_bt/add_task', PoseStamped, queue_size=10)
        
        # 订阅者
        self.status_sub = rospy.Subscriber('loader_bt/status', String, self.status_callback)
        
        # 状态
        self.current_status = "UNKNOWN"
        
        # 设置终端为非阻塞模式
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        rospy.loginfo("自主装载机器人行为树测试节点已启动")
        rospy.loginfo("按键说明:")
        rospy.loginfo("  e - 发送结束任务指令")
        rospy.loginfo("  p - 发送暂停任务指令")
        rospy.loginfo("  r - 发送恢复任务指令")
        rospy.loginfo("  t - 添加测试任务")
        rospy.loginfo("  q - 退出")
        rospy.loginfo("  h - 显示帮助")
        
        self.print_status()
        
    def status_callback(self, msg):
        self.current_status = msg.data
        
    def print_status(self):
        print(f"\r当前状态: {self.current_status}", end="", flush=True)
        
    def add_test_task(self):
        """添加测试任务"""
        task_msg = PoseStamped()
        task_msg.header.frame_id = "map"
        task_msg.header.stamp = rospy.Time.now()
        
        # 随机生成铲料点
        import random
        task_msg.pose.position.x = random.uniform(5.0, 15.0)
        task_msg.pose.position.y = random.uniform(5.0, 15.0)
        task_msg.pose.position.z = 0.0
        task_msg.pose.orientation.w = 1.0
        
        self.add_task_pub.publish(task_msg)
        rospy.loginfo(f"添加测试任务: ({task_msg.pose.position.x:.2f}, {task_msg.pose.position.y:.2f})")
        
    def send_end_task(self, end=True):
        """发送结束任务指令"""
        msg = Bool()
        msg.data = end
        self.end_task_pub.publish(msg)
        rospy.loginfo(f"发送结束任务指令: {'是' if end else '否'}")
        
    def send_pause_task(self, pause=True):
        """发送暂停任务指令"""
        msg = Bool()
        msg.data = pause
        self.pause_task_pub.publish(msg)
        rospy.loginfo(f"发送暂停任务指令: {'是' if pause else '否'}")
        
    def print_help(self):
        """打印帮助信息"""
        print("\n=== 自主装载机器人行为树测试节点 ===")
        print("按键说明:")
        print("  e - 发送结束任务指令")
        print("  p - 发送暂停任务指令")
        print("  r - 发送恢复任务指令")
        print("  t - 添加测试任务")
        print("  q - 退出")
        print("  h - 显示帮助")
        print("================================")
        
    def run(self):
        """主循环"""
        try:
            while not rospy.is_shutdown():
                # 检查是否有按键输入
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == 'q':
                        break
                    elif key == 'e':
                        self.send_end_task(True)
                    elif key == 'p':
                        self.send_pause_task(True)
                    elif key == 'r':
                        self.send_pause_task(False)
                    elif key == 't':
                        self.add_test_task()
                    elif key == 'h':
                        self.print_help()
                        
                # 更新状态显示
                self.print_status()
                
        except KeyboardInterrupt:
            pass
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            print("\n测试节点已退出")

if __name__ == '__main__':
    try:
        test_node = LoaderBTTestNode()
        test_node.run()
    except rospy.ROSInterruptException:
        pass
