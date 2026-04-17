#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mock_spading_service.py
=======================

一个可交互的ROS服务节点，用于模拟 /spadingPose 服务。
它可以通过键盘按键在两种模式之间切换，以测试不同的行为树分支。

模式:
1.  **Zero Mode**: 返回全0姿态，用于测试异常或复位逻辑。
2.  **Real Point Mode**: 从预设的三个真实数据点（包含位置和姿态四元数）中随机选一个返回。

按键:
-   `t`: 在两种模式之间切换 (Toggle)。每次切入 Real Mode 会随机刷新一个目标点。
-   `q`: 退出脚本。
"""

import rospy
import sys
import select
import termios
import tty
import threading
import random
from geometry_msgs.msg import Pose
from autonomous_loader_msgs.srv import spadingPose, spadingPoseResponse

class MockSpadingService:
    def __init__(self):
        self.lock = threading.Lock()
        self.return_zero_mode = True
        
        # [修改] 定义三个完整的 Pose 数据 (位置 x,y,z + 姿态 x,y,z,w)
        # 数据来源：您上传的 RViz/Echo 截图
        self.target_poses = [
            # Point 1 (Img 1)
            {
                "pos": {"x": 4.68, "y": 23.01, "z": 0.0},
                "ori": {"x": 0, "y": 0, "z": 0, "w": 1}
            },
            # Point 2 (Img 2)
            {
                "pos": {"x": 0.73, "y": 11.42, "z": -2.81},
                "ori": {"x": 0, "y": 0, "z": 0, "w": 1}
            },
            # Point 3 (Img 3)
            # {
            #     "pos": {"x": 7.957, "y": 0.815, "z": -3.032},
            #     "ori": {"x": 0, "y": 0, "z": 0, "w": 1}
            # }
        ]
        
        # 当前选中的 Pose，初始化为第一个
        self.current_real_pose = self.target_poses[0]

        rospy.init_node('mock_spading_service')
        self.service = rospy.Service('/spadingPose', spadingPose, self.handle_request)
        
        self.print_status()
        rospy.loginfo("[MockSpadingService] /spadingPose service is ready.")

    def handle_request(self, req):
        """服务请求的回调函数。"""
        with self.lock:
            rospy.loginfo(f"[MockSpadingService] Received request for pileID: {req.pileID}")
            
            response = spadingPoseResponse()
            response.pose_output = Pose()

            if self.return_zero_mode:
                # Zero Mode: 全 0，Quaternion w=1 保持合法
                response.pose_output.position.x = 0.0
                response.pose_output.position.y = 0.0
                response.pose_output.position.z = 0.0
                response.pose_output.orientation.x = 0.0
                response.pose_output.orientation.y = 0.0
                response.pose_output.orientation.z = 0.0
                response.pose_output.orientation.w = 1.0
                rospy.loginfo("[MockSpadingService] Responding in Zero Mode.")
            else:
                # Real Mode: 填充位置和姿态
                p = self.current_real_pose["pos"]
                o = self.current_real_pose["ori"]
                
                response.pose_output.position.x = p["x"]
                response.pose_output.position.y = p["y"]
                response.pose_output.position.z = p["z"]
                
                response.pose_output.orientation.x = o["x"]
                response.pose_output.orientation.y = o["y"]
                response.pose_output.orientation.z = o["z"]
                response.pose_output.orientation.w = o["w"]
                
                rospy.loginfo(f"[MockSpadingService] Responding Real Pose -> Pos:({p['x']:.2f}, {p['y']:.2f}), Ori_z:{o['z']:.3f}")
            
            return response

    def toggle_mode(self):
        """切换返回模式。"""
        with self.lock:
            self.return_zero_mode = not self.return_zero_mode
            
            # [修改] 切换到 Real Mode 时，随机选择一个新的 Pose
            if not self.return_zero_mode:
                self.current_real_pose = random.choice(self.target_poses)
                
        self.print_status()

    def print_status(self):
        """打印当前状态。"""
        if self.return_zero_mode:
            mode_str = "Zero Mode (0,0)"
        else:
            p = self.current_real_pose["pos"]
            mode_str = f"Real Pose (x={p['x']:.2f}, y={p['y']:.2f})"
            
        rospy.loginfo(f"\n\033[93mCurrent Mode: {mode_str}. Press 't' to toggle, 'q' to quit.\033[0m")

def main():
    service_provider = MockSpadingService()
    
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.5)[0]:
                key = sys.stdin.read(1)
                if key == 't':
                    service_provider.toggle_mode()
                elif key == 'q':
                    rospy.loginfo("Exiting...")
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()