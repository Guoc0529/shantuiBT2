#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
示例：如何通过ROS话题发送任务指令

使用方法：
    rosrun autonomous_loader_bt send_task_example.py
"""

import rospy
from autonomous_loader_msgs.msg import TaskCommand

def send_task(task_id, bin_id, hopper_id, task_type="scoop"):
    """
    发送任务命令
    
    参数:
        task_id: 任务ID
        bin_id: 料仓编号
        hopper_id: 料斗编号
        task_type: 任务类型（默认为"scoop"）
    """
    # 初始化节点
    rospy.init_node('task_sender', anonymous=True)
    
    # 创建发布者
    pub = rospy.Publisher('scheduler/start_task', TaskCommand, queue_size=10)
    
    # 等待发布者就绪
    rospy.sleep(0.5)
    
    # 创建任务消息
    task_msg = TaskCommand()
    task_msg.task_id = task_id
    task_msg.bin_id = bin_id
    task_msg.hopper_id = hopper_id
    task_msg.task_type = task_type
    
    # 发布任务
    pub.publish(task_msg)
    rospy.loginfo("已发送任务 - ID: %d, 料仓: %d, 料斗: %d, 类型: %s", 
                  task_id, bin_id, hopper_id, task_type)

if __name__ == '__main__':
    try:
        # 发送示例任务
        send_task(task_id=1, bin_id=1, hopper_id=1, task_type="scoop")
        send_task(task_id=2, bin_id=2, hopper_id=2, task_type="scoop")
        
        print("示例任务已发送")
    except rospy.ROSInterruptException:
        pass

