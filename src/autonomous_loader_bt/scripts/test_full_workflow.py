#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
完整工作流程测试脚本
测试状态机的完整功能，包括：
1. 通过Service调用下发任务
2. 监控参数变化（canggoal, workstate）
3. 监控话题状态
4. 自动模拟模块回令完成整个流程
"""

import rospy
import sys
import math
from std_msgs.msg import Bool, Int8, Float64, String
from geometry_msgs.msg import PoseStamped, Pose
from shuju.srv import cangdou, cangdouRequest
from autonomous_loader_msgs.srv import spadingPose

class FullWorkflowTest:
    def __init__(self):
        rospy.init_node('full_workflow_test', anonymous=True)
        
        # 等待服务就绪
        rospy.loginfo("等待服务 /autonomous_loader_bt_node/liaodou 就绪...")
        rospy.wait_for_service('/autonomous_loader_bt_node/liaodou', timeout=10.0)
        self.task_service = rospy.ServiceProxy('/autonomous_loader_bt_node/liaodou', cangdou)
        rospy.loginfo("服务 /autonomous_loader_bt_node/liaodou 已就绪")
        
        # 提供铲料点服务（模拟）
        self.spading_pose_service = rospy.Service('/spadingPose', spadingPose, self.spadingPoseCallback)
        rospy.loginfo("铲料点服务 /spadingPose 已启动")
        
        # 订阅者：监控状态
        self.workstate_sub = rospy.Subscriber('/autonomous_loader_bt_node/workstate', Int8, self.workstate_callback)
        self.status_sub = rospy.Subscriber('/autonomous_loader_bt_node/state_machine/status', String, self.status_callback)
        
        # 订阅者：接收状态机指令
        self.raise_arm_sub = rospy.Subscriber('/autonomous_loader_bt_node/work_device/raise_arm', Bool, self.raiseArmCallback)
        self.lower_bucket_sub = rospy.Subscriber('/autonomous_loader_bt_node/work_device/lower_bucket', Bool, self.lowerBucketCallback)
        self.start_scoop_sub = rospy.Subscriber('/autonomous_loader_bt_node/scoop/start_scoop', Bool, self.startScoopCallback)
        self.navigation_goal_sub = rospy.Subscriber('/autonomous_loader_bt_node/navigation/goal', PoseStamped, self.navigationGoalCallback)
        self.navigation_pause_sub = rospy.Subscriber('/autonomous_loader_bt_node/navigation/pause', Bool, self.navigationPauseCallback)
        
        # 发布者：模拟模块回令
        self.arm_raised_pub = rospy.Publisher('/autonomous_loader_bt_node/work_device/arm_raised', Bool, queue_size=10)
        self.bucket_lowered_pub = rospy.Publisher('/autonomous_loader_bt_node/work_device/bucket_lowered', Bool, queue_size=10)
        self.scoop_completed_pub = rospy.Publisher('/autonomous_loader_bt_node/scoop/scoop_completed', Bool, queue_size=10)
        self.navigation_planning_success_pub = rospy.Publisher('/autonomous_loader_bt_node/navigation/planning_success', Bool, queue_size=10)
        self.navigation_arrived_pub = rospy.Publisher('/autonomous_loader_bt_node/navigation/arrived', Bool, queue_size=10)
        self.distance_to_goal_pub = rospy.Publisher('/autonomous_loader_bt_node/navigation/distance_to_goal', Float64, queue_size=10)
        
        # 状态
        self.current_workstate = -1
        self.current_status = "UNKNOWN"
        self.canggoal_value = None
        self.workstate_value = None
        
        # 导航模拟状态
        self.nav_active = False
        self.nav_distance = 10.0
        self.nav_timer = None
        self.dump_phase_active = False
        
        # 记录最近一次铲料点与导航目标
        self.scoop_point_received = None
        self.scoop_point_is_zero = False
        self.previous_nav_target_x = None
        self.previous_nav_target_y = None
        self.nav_target_x = None
        self.nav_target_y = None
        
        # 测试状态
        self.test_step = 0
        self.test_passed = True
        self.test_messages = []

        # 指令接收标志
        self.received_commands = {
            'navigation_goal': None,
            'navigation_pause': False
        }

        # 工作流步骤跟踪
        self.workflow_steps = {
            'raise_arm': False,
            'navigation_to_scoop': False,
            'lower_bucket': False,
            'scoop': False,
            'navigation_to_hopper': False,
            'dump': False
        }
        self.workflow_step_labels = {
            'raise_arm': "Raise Arm",
            'navigation_to_scoop': "Navigation to Scoop Point",
            'lower_bucket': "Lower Bucket",
            'scoop': "Scoop",
            'navigation_to_hopper': "Navigation to Hopper",
            'dump': "Dump"
        }
        
        # 等待节点初始化
        rospy.sleep(1.0)

    def workstate_callback(self, msg):
        """工作状态回调"""
        self.current_workstate = msg.data
        # rospy.loginfo("收到工作状态: %d (0空闲 1执行中 2暂停中 3结束中 4已结束)", msg.data)

    def record_workflow_step(self, key, details=""):
        if key not in self.workflow_steps:
            return
        if self.workflow_steps[key]:
            return
        self.workflow_steps[key] = True
        label = self.workflow_step_labels.get(key, key)
        timestamp = rospy.Time.now().to_sec()
        rospy.loginfo("[WORKFLOW] [%.2f] %s%s", timestamp, label, (": " + details) if details else "")
        self.test_messages.append(f"✓ {label}{(': ' + details) if details else ''}")

    def status_callback(self, msg):
        """状态机状态回调"""
        self.current_status = msg.data
            
    def raiseArmCallback(self, msg):
        """抬大臂指令回调"""
        if msg.data:
            self.record_workflow_step('raise_arm')
            rospy.loginfo("[测试] 收到抬大臂指令，2秒后发送完成回令")
            rospy.Timer(rospy.Duration(2.0), self.sendArmRaised, oneshot=True)
            
    def sendArmRaised(self, event):
        """发送抬大臂完成回令"""
        msg = Bool()
        msg.data = True
        self.arm_raised_pub.publish(msg)
        rospy.loginfo("[测试] 发送抬大臂完成回令")
        
    def lowerBucketCallback(self, msg):
        """放平铲斗指令回调"""
        if msg.data:
            self.record_workflow_step('lower_bucket')
            rospy.loginfo("[测试] 收到放平铲斗指令，1.5秒后发送完成回令")
            rospy.Timer(rospy.Duration(1.5), self.sendBucketLowered, oneshot=True)
            
    def sendBucketLowered(self, event):
        """发送放平铲斗完成回令"""
        msg = Bool()
        msg.data = True
        self.bucket_lowered_pub.publish(msg)
        rospy.loginfo("[测试] 发送放平铲斗完成回令")
        
    def startScoopCallback(self, msg):
        """开始铲料指令回调"""
        if msg.data:
            if self.dump_phase_active:
                self.record_workflow_step('dump')
                rospy.loginfo("[测试] 收到卸料指令，3秒后发送完成回令")
            else:
                self.record_workflow_step('scoop')
                rospy.loginfo("[测试] 收到开始铲料指令，3秒后发送完成回令")
            rospy.Timer(rospy.Duration(3.0), self.sendScoopCompleted, oneshot=True)
            
    def sendScoopCompleted(self, event):
        """发送铲料完成回令"""
        msg = Bool()
        msg.data = True
        self.scoop_completed_pub.publish(msg)
        rospy.loginfo("[测试] 发送铲料完成回令")
        if self.dump_phase_active:
            self.record_workflow_step('dump', "Completion acknowledgment sent")
            self.dump_phase_active = False
        
    def navigationPauseCallback(self, msg):
        """导航暂停指令回调"""
        self.received_commands['navigation_pause'] = msg.data
        rospy.loginfo("[SIM] Received navigation pause command: %s", "pause" if msg.data else "resume")

    def navigationGoalCallback(self, msg):
        """Navigation goal callback"""
        rospy.loginfo("[SIM] navigationGoalCallback called")
        self.received_commands['navigation_goal'] = msg
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        rospy.loginfo("[SIM] Received navigation goal: (%.2f, %.2f)", goal_x, goal_y)
        
        # Determine navigation phase based on goal position
        is_hopper = abs(goal_x - 5.0) < 2.0 and abs(goal_y - 5.0) < 2.0
        is_scoop_point = False
        if self.scoop_point_received:
            expected_x, expected_y = self.scoop_point_received
            is_scoop_point = abs(goal_x - expected_x) < 2.0 and abs(goal_y - expected_y) < 2.0
        if not is_scoop_point:
            # Fallback check
            is_scoop_point = abs(goal_x - 10.5) < 2.0 and abs(goal_y - 10.5) < 2.0

        # Try to detect navigation steps immediately in callback
        # This ensures we don't miss steps if navigation_goal is reset before workflow loop checks
        if hasattr(self, '_workflow_steps_completed'):
            steps_completed = self._workflow_steps_completed
            
            # Check if it's scoop point (most common case)
            if not steps_completed.get('navigation_to_scoop', False) and is_scoop_point:
                steps_completed['navigation_to_scoop'] = True
                expect_bin_center = self.scoop_point_is_zero if self.scoop_point_received else False
                step_num = 2 if not expect_bin_center else 3
                self.log_step(step_num, 7, "Navigation to Scoop Point", 
                            f"Goal: ({goal_x:.2f}, {goal_y:.2f})")
            
            # Check if it's bin center (only if scoop point is (0,0))
            if not steps_completed.get('navigation_to_bin', False):
                if self.scoop_point_is_zero and self.scoop_point_received:
                    if abs(goal_x - 10.0) < 2.0 and abs(goal_y - 10.0) < 2.0:
                        steps_completed['navigation_to_bin'] = True
                        self.log_step(2, 7, "Navigation to Bin Center", 
                                    f"Goal: ({goal_x:.2f}, {goal_y:.2f})")
            
            # Check if it's hopper (start dump phase)
            if not steps_completed.get('navigation_to_hopper', False) and is_hopper:
                steps_completed['navigation_to_hopper'] = True
                self.log_step(6, 7, "Navigation to Hopper", 
                            f"Goal: ({goal_x:.2f}, {goal_y:.2f})")

        if is_scoop_point:
            self.record_workflow_step('navigation_to_scoop', f"Goal=({goal_x:.2f}, {goal_y:.2f})")

        if is_hopper:
            self.record_workflow_step('navigation_to_hopper', f"Goal=({goal_x:.2f}, {goal_y:.2f})")
        
        # [Original planning_success publish was here - it's been moved down]
        
        # Calculate initial distance based on current position and goal
        # In real system, navigation module calculates actual distance from current pose to goal
        # For simulation, we estimate distance or use a reasonable starting value
        if is_hopper:
            # For hopper navigation (dump phase), calculate distance from previous target (scoop point) to hopper
            # IMPORTANT: Save previous target BEFORE updating nav_target_x/y
            # Use previous_nav_target_x/y if available, otherwise use current nav_target_x/y
            previous_x = self.previous_nav_target_x if self.previous_nav_target_x is not None else self.nav_target_x
            previous_y = self.previous_nav_target_y if self.previous_nav_target_y is not None else self.nav_target_y
            
            rospy.loginfo("[SIM] Dump phase: Checking distance calculation...")
            rospy.loginfo("[SIM]   - previous_nav_target: (%.2f, %.2f)", 
                         self.previous_nav_target_x if self.previous_nav_target_x is not None else -999, 
                         self.previous_nav_target_y if self.previous_nav_target_y is not None else -999)
            rospy.loginfo("[SIM]   - current nav_target: (%.2f, %.2f)", 
                         self.nav_target_x if self.nav_target_x is not None else -999,
                         self.nav_target_y if self.nav_target_y is not None else -999)
            rospy.loginfo("[SIM]   - Using previous: (%.2f, %.2f)", previous_x, previous_y)
            
            if previous_x is not None and previous_y is not None:
                # Calculate approximate distance from previous target (scoop point) to hopper
                # In real system, navigation module calculates actual distance from current pose
                dx = goal_x - previous_x
                dy = goal_y - previous_y
                estimated_distance = math.sqrt(dx*dx + dy*dy)
                # Ensure minimum distance for dump phase simulation
                # But also check: if estimated distance is very small (< 1m), it means we're already close
                # In that case, we should still use a reasonable distance for simulation
                if estimated_distance < 1.0:
                    rospy.logwarn("[SIM] Dump phase: Estimated distance is very small (%.2f m), using minimum 8m for simulation", estimated_distance)
                    initial_distance = 8.0
                else:
                    initial_distance = max(estimated_distance, 8.0)  # At least 8m for proper simulation
                rospy.loginfo("[SIM] Dump phase: Previous target: (%.2f, %.2f)", previous_x, previous_y)
                rospy.loginfo("[SIM] Dump phase: Hopper target: (%.2f, %.2f)", goal_x, goal_y)
                rospy.loginfo("[SIM] Dump phase: Estimated distance: %.2f m", estimated_distance)
                rospy.loginfo("[SIM] Dump phase: Using initial distance: %.2f m", initial_distance)
            else:
                # Fallback: use typical distance from scoop point to hopper
                initial_distance = 10.0  # Default fallback
                rospy.logwarn("[SIM] Dump phase: No previous target available, using default distance: %.2f m", initial_distance)
            
            # Activate dump phase when navigating to hopper
            self.dump_phase_active = True
            self._dump_phase_start_time = rospy.Time.now()
            self._dump_phase_distance_reset = True
            rospy.loginfo("[SIM] Dump phase activated: navigating to hopper, distance will decrease from %.2f m to 0 m", initial_distance)
        else:
            # For scoop point navigation (scoop phase), use standard starting distance
            initial_distance = 10.0
            # Deactivate dump phase if it was active
            if hasattr(self, 'dump_phase_active'):
                self.dump_phase_active = False
            rospy.loginfo("[SIM] Scoop phase: navigating to scoop point, distance will decrease from %.2f m to 0 m", initial_distance)
        
        # Save previous target before updating (for distance calculation in next navigation)
        self.previous_nav_target_x = self.nav_target_x
        self.previous_nav_target_y = self.nav_target_y
        
        # Start navigation simulation with calculated distance
        self.nav_target_x = goal_x
        self.nav_target_y = goal_y
        self.nav_distance = initial_distance
        self.nav_active = True
        
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!! BEGINNING OF FIX !!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        # FIX: 发布初始距离 *先于* 发布规划成功.
        # 这可以防止行为树在测试脚本发布起始距离 (默认为 0.0) 之前就检查距离的竞争条件。
        distance_msg = Float64()
        distance_msg.data = self.nav_distance
        self.distance_to_goal_pub.publish(distance_msg)
        rospy.loginfo("[SIM] Published initial distance: %.2f m", self.nav_distance)
        rospy.sleep(0.05) # 给予一个极短的延迟，确保消息被 ROS master 处理

        # 现在, *在* 距离被设置后, 再发送规划成功
        planning_msg = Bool()
        planning_msg.data = True
        self.navigation_planning_success_pub.publish(planning_msg)
        rospy.loginfo("[SIM] Sent navigation planning success")

        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!! END OF FIX !!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        # Stop previous timer if exists
        if self.nav_timer is not None:
            self.nav_timer.shutdown()
        
        # Start new navigation simulation timer
        self.nav_timer = rospy.Timer(rospy.Duration(0.2), self.update_navigation_simulation)
        
    def update_navigation_simulation(self, event):
        """更新导航模拟（定时器回调）"""
        if not self.nav_active:
            return
            
        # 发布距离
        distance_msg = Float64()
        distance_msg.data = self.nav_distance
        self.distance_to_goal_pub.publish(distance_msg)
        
        # 距离递减
        self.nav_distance -= 0.2
        if self.nav_distance < 0.1:
            self.nav_distance = 0.0
            self.nav_active = False
            
            # 到达目标
            arrived_msg = Bool()
            arrived_msg.data = True
            self.navigation_arrived_pub.publish(arrived_msg)
            rospy.loginfo("[测试] 导航到达目标位置")
        
    def spadingPoseCallback(self, req):
        """铲料点服务回调"""
        rospy.loginfo("[测试] 收到铲料点请求，pileID (cang): %d", req.pileID)
        
        # 根据pileID返回铲料点（简单模拟，返回料仓附近的位置）
        # 根据配置文件，cang=1 在 (10, 10)，cang=2 在 (15, 15)，cang=3 在 (20, 20)
        scoop_positions = {
            1: (10.5, 10.5, 0.0),
            2: (15.5, 15.5, 0.0),
            3: (20.5, 20.5, 0.0),
        }
        
        pose = Pose()
        if req.pileID in scoop_positions:
            x, y, z = scoop_positions[req.pileID]
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            rospy.loginfo("[测试] 返回铲料点: (%.2f, %.2f) for pileID=%d", x, y, req.pileID)
        else:
            # 默认位置
            pose.position.x = 10.0 + req.pileID * 5.0
            pose.position.y = 10.0 + req.pileID * 5.0
            pose.position.z = 0.0
            rospy.loginfo("[测试] 返回默认铲料点: (%.2f, %.2f) for pileID=%d", 
                         pose.position.x, pose.position.y, req.pileID)
        
        pose.orientation.w = 1.0

        # 记录铲料点信息，供后续导航阶段识别
        self.scoop_point_received = (pose.position.x, pose.position.y)
        self.scoop_point_is_zero = (abs(pose.position.x) < 0.1 and abs(pose.position.y) < 0.1)
        
        # 创建响应对象
        resp = spadingPose._response_class()
        resp.pose_output = pose
        return resp
        
    def check_parameter(self, param_name, expected_value=None, timeout=5.0):
        """检查参数值"""
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            try:
                value = rospy.get_param('/autonomous_loader_bt_node/' + param_name, None)
                if value is not None:
                    if expected_value is None or value == expected_value:
                        rospy.loginfo("[测试] 参数 %s = %s", param_name, value)
                        return value
            except:
                pass
            rospy.sleep(0.1)
        
        rospy.logwarn("[测试] 参数 %s 检查超时或值不匹配", param_name)
        return None
        
    def test_service_call(self, task_id, cang, dou):
        """测试服务调用"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("[测试] 步骤 %d: 调用服务下发任务", self.test_step)
        rospy.loginfo("任务ID: %d, 料仓(cang): %d, 料斗(dou): %d", task_id, cang, dou)
        
        try:
            req = cangdouRequest()
            if task_id < -128 or task_id > 127:
                rospy.logwarn("[测试] taskID=%d 超出 int8 范围，已自动裁剪", task_id)
                task_id = max(min(task_id, 127), -128)
            req.taskID = task_id
            req.cang = cang
            req.dou = dou
            
            resp = self.task_service(req)
            
            if resp.huiying:
                rospy.loginfo("[测试] ✓ 服务调用成功，收到应答 huiying=true")
                self.test_messages.append("✓ 服务调用成功")
                return True
            else:
                rospy.logwarn("[测试] ✗ 服务调用失败，huiying=false")
                self.test_messages.append("✗ 服务调用失败")
                self.test_passed = False
                return False
                
        except Exception as e:
            rospy.logerr("[测试] ✗ 服务调用异常: %s", str(e))
            self.test_messages.append(f"✗ 服务调用异常: {str(e)}")
            self.test_passed = False
            return False
            
    def test_canggoal_parameter(self, expected_cang):
        """测试 canggoal 参数"""
        rospy.loginfo("\n[测试] 步骤 %d: 检查 canggoal 参数", self.test_step)
        self.test_step += 1
        
        value = self.check_parameter('canggoal', expected_cang, timeout=3.0)
        
        if value == expected_cang:
            rospy.loginfo("[测试] ✓ canggoal 参数正确: %d", value)
            self.test_messages.append(f"✓ canggoal 参数正确: {value}")
            return True
        else:
            rospy.logwarn("[测试] ✗ canggoal 参数错误或超时，期望: %d, 实际: %s", 
                         expected_cang, value)
            self.test_messages.append(f"✗ canggoal 参数错误")
            self.test_passed = False
            return False
            
    def test_workstate(self, expected_states=None, timeout=10.0):
        """测试工作状态"""
        rospy.loginfo("\n[测试] 步骤 %d: 监控工作状态", self.test_step)
        self.test_step += 1
        
        start_time = rospy.Time.now()
        states_seen = []
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.current_workstate >= 0:
                if self.current_workstate not in states_seen:
                    states_seen.append(self.current_workstate)
                    rospy.loginfo("[测试] 工作状态变化: %d", self.current_workstate)
                    
            # 检查参数服务器
            try:
                param_value = rospy.get_param('/autonomous_loader_bt_node/workstate', None)
                if param_value is not None:
                    rospy.loginfo("[测试] workstate 参数: %d", param_value)
            except:
                pass
                
            rospy.sleep(0.5)
            
        rospy.loginfo("[测试] 观察到的工作状态序列: %s", states_seen)
        self.test_messages.append(f"工作状态序列: {states_seen}")
        
        if expected_states:
            for expected in expected_states:
                if expected not in states_seen:
                    rospy.logwarn("[测试] ✗ 未观察到预期状态: %d", expected)
                    self.test_passed = False
                    return False
                    
        rospy.loginfo("[测试] ✓ 工作状态监控完成")
        return True
        
    def run_full_test(self):
        """运行完整测试"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("开始完整工作流程测试")
        rospy.loginfo("="*60)
        
        # 测试1: 服务调用
        self.test_step = 1
        test_cang = 2
        test_dou = 1
        test_task_id = 101
        
        if not self.test_service_call(test_task_id, test_cang, test_dou):
            rospy.logerr("测试失败: 服务调用失败")
            return False
            
        rospy.sleep(1.0)
        
        # 测试2: 检查 canggoal 参数
        if not self.test_canggoal_parameter(test_cang):
            rospy.logerr("测试失败: canggoal 参数错误")
            return False
            
        rospy.sleep(1.0)
        
        # 测试3: 监控工作状态
        if not self.test_workstate(expected_states=[1], timeout=5.0):
            rospy.logwarn("工作状态监控可能不完整，但继续测试")
            
        # 等待流程执行
        rospy.loginfo("\n[测试] 等待流程执行（自动模拟模块回令）...")
        rospy.loginfo("[测试] 预计需要30-60秒完成整个流程")
        rospy.sleep(60.0)

        # 校验关键步骤
        rospy.loginfo("[测试] 校验铲料/卸料关键步骤...")
        for key, label in self.workflow_step_labels.items():
            if self.workflow_steps.get(key, False):
                rospy.loginfo("[测试] ✓ %s 已完成", label)
            else:
                rospy.logwarn("[测试] ✗ %s 未完成", label)
                self.test_messages.append(f"✗ {label} 未完成")
                self.test_passed = False
        
        # 测试总结
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("测试总结")
        rospy.loginfo("="*60)
        for msg in self.test_messages:
            rospy.loginfo(msg)
            
        if self.test_passed:
            rospy.loginfo("\n[测试] ✓ 所有测试通过！")
        else:
            rospy.logwarn("\n[测试] ✗ 部分测试失败")
            
        return self.test_passed
        
    def run_interactive_test(self):
        """运行交互式测试"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("交互式测试模式")
        rospy.loginfo("="*60)
        rospy.loginfo("输入命令:")
        rospy.loginfo("  test <taskID> <cang> <dou>  - 发送测试任务")
        rospy.loginfo("  check canggoal             - 检查 canggoal 参数")
        rospy.loginfo("  check workstate            - 检查 workstate 参数")
        rospy.loginfo("  status                     - 显示当前状态")
        rospy.loginfo("  quit                       - 退出")
        
        while not rospy.is_shutdown():
            try:
                cmd = input("\n> ").strip().split()
                
                if not cmd:
                    continue
                    
                if cmd[0] == 'test' and len(cmd) == 4:
                    task_id = int(cmd[1])
                    cang = int(cmd[2])
                    dou = int(cmd[3])
                    self.test_service_call(task_id, cang, dou)
                    rospy.sleep(1.0)
                    self.test_canggoal_parameter(cang)
                    
                elif cmd[0] == 'check' and len(cmd) == 2:
                    param_name = cmd[1]
                    value = self.check_parameter(param_name, timeout=2.0)
                    if value is not None:
                        rospy.loginfo("参数 %s = %s", param_name, value)
                    else:
                        rospy.logwarn("无法获取参数 %s", param_name)
                        
                elif cmd[0] == 'status':
                    rospy.loginfo("当前工作状态: %d", self.current_workstate)
                    rospy.loginfo("当前状态机状态: %s", self.current_status)
                    
                elif cmd[0] == 'quit':
                    break
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logerr("命令执行错误: %s", str(e))

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='完整工作流程测试脚本')
    parser.add_argument('--interactive', '-i', action='store_true', 
                       help='交互式测试模式')
    parser.add_argument('--auto', '-a', action='store_true', 
                       help='自动测试模式（默认）')
    args = parser.parse_args()
    
    try:
        test = FullWorkflowTest()
        
        if args.interactive:
            test.run_interactive_test()
        else:
            test.run_full_test()
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("测试异常: %s", str(e))
        sys.exit(1)
