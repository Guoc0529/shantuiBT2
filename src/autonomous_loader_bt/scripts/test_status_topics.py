#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
test_status_topics.py
=====================

全自动测试脚本，用于验证：
1. /bt_action/status (动作状态消息)
2. /vehicle/task_status (任务状态上报)

功能：
- 自动监听两个 topic
- 自动下发任务触发状态变化
- 实时显示状态变化日志
- 结束时输出测试报告
"""

import rospy
import sys
import time
import threading
from std_msgs.msg import Bool, Int8, Float64, String, Int32MultiArray
from geometry_msgs.msg import PoseStamped, Pose
from shuju.srv import cangdou, cangdouRequest
from autonomous_loader_msgs.srv import spadingPose, spadingPoseResponse


class StatusTopicsTest:
    def __init__(self):
        rospy.init_node('status_topics_test', anonymous=True)

        self.status_data = []       # /bt_action/status 数据记录
        self.task_status_data = []  # /vehicle/task_status 数据记录
        self.lock = threading.Lock()

        # 统计
        self.stats = {
            'action_in_progress': 0,
            'action_completed': 0,
            'task_state_changes': set()
        }

        # 导航模拟状态
        self.nav_active = False
        self.nav_distance = 10.0
        self.nav_timer = None
        self.dump_phase_active = False

        # 铲料点
        self.scoop_point_received = None

        self.setup_subscribers()
        self.setup_publishers()
        self.setup_services()

        rospy.sleep(1.0)
        rospy.loginfo("[TEST] 状态话题测试节点已启动")

    def setup_subscribers(self):
        # 动作状态监听
        rospy.Subscriber('/bt_action/status', String, self.action_status_callback)
        # 任务状态监听 (实际话题名: /bt_navigation/task_status)
        rospy.Subscriber('/bt_navigation/task_status', Int32MultiArray, self.task_status_callback)
        # 导航指令监听
        rospy.Subscriber('/autonomous_loader_bt_node/navigation/goal', PoseStamped, self.navigation_goal_callback)
        # 抬臂指令
        rospy.Subscriber('/autonomous_loader_bt_node/work_device/raise_arm', Bool, self.raise_arm_callback)
        # 放铲斗指令
        rospy.Subscriber('/autonomous_loader_bt_node/work_device/lower_bucket', Bool, self.lower_bucket_callback)
        # 铲料/卸料指令
        rospy.Subscriber('/autonomous_loader_bt_node/scoop/start_scoop', Bool, self.scoop_callback)
        # 工作状态
        rospy.Subscriber('/autonomous_loader_bt_node/workstate', Int8, self.workstate_callback)

    def setup_publishers(self):
        self.arm_raised_pub = rospy.Publisher('/autonomous_loader_bt_node/work_device/arm_raised', Bool, queue_size=10)
        self.bucket_lowered_pub = rospy.Publisher('/autonomous_loader_bt_node/work_device/bucket_lowered', Bool, queue_size=10)
        self.scoop_completed_pub = rospy.Publisher('/autonomous_loader_bt_node/scoop/scoop_completed', Bool, queue_size=10)
        self.planning_success_pub = rospy.Publisher('/autonomous_loader_bt_node/navigation/planning_success', Bool, queue_size=10)
        self.arrived_pub = rospy.Publisher('/autonomous_loader_bt_node/navigation/arrived', Bool, queue_size=10)
        self.distance_pub = rospy.Publisher('/autonomous_loader_bt_node/navigation/distance_to_goal', Float64, queue_size=10)

    def setup_services(self):
        # 等待任务下发服务
        rospy.loginfo("[TEST] 等待服务 /autonomous_loader_bt_node/liaodou ...")
        try:
            rospy.wait_for_service('/autonomous_loader_bt_node/liaodou', timeout=10.0)
            self.task_srv = rospy.ServiceProxy('/autonomous_loader_bt_node/liaodou', cangdou)
            rospy.loginfo("[TEST] 服务 /autonomous_loader_bt_node/liaodou 已就绪")
        except rospy.ROSException as e:
            rospy.logwarn("[TEST] 无法连接服务: %s", str(e))
            self.task_srv = None

        # 提供铲料点服务
        self.spading_srv = rospy.Service('/spadingPose', spadingPose, self.spading_pose_callback)
        rospy.loginfo("[TEST] 铲料点服务 /spadingPose 已启动")

    def action_status_callback(self, msg):
        with self.lock:
            self.status_data.append({
                'time': rospy.Time.now().to_sec(),
                'status': msg.data
            })
            if '进行中' in msg.data:
                self.stats['action_in_progress'] += 1
            elif '完成' in msg.data or 'Success' in msg.data:
                self.stats['action_completed'] += 1
        rospy.loginfo("[ACTION] %s", msg.data)

    def task_status_callback(self, msg):
        if len(msg.data) < 4:
            return
        with self.lock:
            state = msg.data[0]
            timestamp = msg.data[1]
            task_id = msg.data[2]
            self.stats['task_state_changes'].add(state)
            self.task_status_data.append({
                'time': rospy.Time.now().to_sec(),
                'state': state,
                'timestamp': timestamp,
                'task_id': task_id
            })
        state_names = {
            0: "未启动", 1: "空闲中", 2: "自动作业中", 3: "临时作业中",
            4: "紧急停车中", 5: "避障停车中", 6: "远程接管中",
            7: "结束作业中", 8: "已结束"
        }
        name = state_names.get(state, "未知")
        rospy.loginfo("[TASK_STATUS] state=%d(%s) task_id=%d", state, name, task_id)

    def workstate_callback(self, msg):
        rospy.loginfo("[WORKSTATE] 参数值: %d", msg.data)

    def navigation_goal_callback(self, msg):
        goal_x, goal_y = msg.pose.position.x, msg.pose.position.y
        rospy.loginfo("[NAV] 收到导航目标: (%.2f, %.2f)", goal_x, goal_y)

        # 发送规划成功
        self.planning_success_pub.publish(Bool(data=True))

        # 模拟距离递减
        self.nav_distance = 10.0
        self.nav_active = True

        if self.nav_timer:
            self.nav_timer.shutdown()
        self.nav_timer = rospy.Timer(rospy.Duration(0.5), self.nav_timer_callback)

    def nav_timer_callback(self, event):
        if not self.nav_active:
            return
        self.distance_pub.publish(Float64(data=self.nav_distance))
        self.nav_distance -= 0.5
        if self.nav_distance <= 0:
            self.nav_distance = 0
            self.nav_active = False
            self.arrived_pub.publish(Bool(data=True))
            rospy.loginfo("[NAV] 到达目标")

    def raise_arm_callback(self, msg):
        if msg.data:
            rospy.loginfo("[ACTION] 抬臂指令")
            rospy.sleep(1.5)
            self.arm_raised_pub.publish(Bool(data=True))
            rospy.loginfo("[ACTION] 抬臂完成")

    def lower_bucket_callback(self, msg):
        if msg.data:
            rospy.loginfo("[ACTION] 放铲斗指令")
            rospy.sleep(1.0)
            self.bucket_lowered_pub.publish(Bool(data=True))
            rospy.loginfo("[ACTION] 放铲斗完成")

    def scoop_callback(self, msg):
        if msg.data:
            rospy.loginfo("[ACTION] 铲料/卸料指令")
            rospy.sleep(2.0)
            self.scoop_completed_pub.publish(Bool(data=True))
            rospy.loginfo("[ACTION] 动作完成")

    def spading_pose_callback(self, req):
        rospy.loginfo("[SERVICE] 铲料点请求, pileID=%d", req.pileID)
        pose = Pose()
        pose.position.x = 10.5 + req.pileID * 2
        pose.position.y = 10.5 + req.pileID * 2
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        self.scoop_point_received = (pose.position.x, pose.position.y)
        rospy.loginfo("[SERVICE] 返回铲料点: (%.2f, %.2f)", pose.position.x, pose.position.y)
        resp = spadingPoseResponse()
        resp.pose_output = pose
        return resp

    def send_task(self, task_id, cang, dou):
        if not self.task_srv:
            rospy.logerr("[TEST] 任务服务不可用")
            return False
        try:
            req = cangdouRequest()
            req.taskID = task_id
            req.cang = cang
            req.dou = dou
            resp = self.task_srv(req)
            rospy.loginfo("[TEST] 任务下发: taskID=%d, cang=%d, dou=%d, 响应=%s",
                         task_id, cang, dou, resp.huiying)
            return resp.huiying
        except Exception as e:
            rospy.logerr("[TEST] 任务下发失败: %s", str(e))
            return False

    def print_header(self):
        print("\n" + "="*70)
        print("              状态话题测试 - 自动测试模式")
        print("="*70)
        print("监听话题:")
        print("  1. /bt_action/status       - 动作状态消息")
        print("  2. /vehicle/task_status   - 任务状态上报 [state, timestamp, task_id, reserved]")
        print("="*70)

    def print_report(self):
        print("\n" + "="*70)
        print("                    测试报告")
        print("="*70)

        with self.lock:
            print("\n[动作状态统计]")
            print(f"  进行中消息: {self.stats['action_in_progress']} 条")
            print(f"  完成消息:   {self.stats['action_completed']} 条")

            print("\n[任务状态变化]")
            state_names = {
                0: "未启动", 1: "空闲中", 2: "自动作业中", 3: "临时作业中",
                4: "紧急停车中", 5: "避障停车中", 6: "远程接管中",
                7: "结束作业中", 8: "已结束"
            }
            for s in sorted(self.stats['task_state_changes']):
                print(f"  {s}: {state_names.get(s, '未知')}")

            print("\n[任务状态详情 (最近10条)]")
            for item in self.task_status_data[-10:]:
                name = state_names.get(item['state'], "未知")
                print(f"  [{item['time']:.1f}] state={item['state']}({name}) task_id={item['task_id']}")

        print("\n" + "="*70)
        print("测试完成")
        print("="*70)

    def run(self, duration=60):
        self.print_header()
        rospy.loginfo("[TEST] 测试将运行 %d 秒", duration)

        # 下发第一个任务
        rospy.sleep(2.0)
        rospy.loginfo("[TEST] ===== 下发测试任务 1 =====")
        self.send_task(task_id=1, cang=1, dou=1)

        # 等待一段时间
        rospy.sleep(duration / 2)

        # 下发第二个任务（如果有）
        # rospy.loginfo("[TEST] ===== 下发测试任务 2 =====")
        # self.send_task(task_id=1002, cang=2, dou=1)

        # 等待剩余时间
        rospy.sleep(duration / 2)

        self.print_report()
        return True


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='状态话题测试')
    parser.add_argument('--duration', '-d', type=int, default=60,
                       help='测试持续时间(秒), 默认60秒')
    args = parser.parse_args()

    try:
        test = StatusTopicsTest()
        test.run(duration=args.duration)
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n测试被用户中断")
