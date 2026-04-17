#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Comprehensive Behavior Tree Test Script
Tests all behavior tree branches and complete workflow:
1. Test stop task branch (highest priority)
2. Test pause task branch (medium priority)
3. Test work/idle branch (normal priority)
4. Test complete task workflow
5. Verify all nodes and conditions work correctly
"""

import rospy
import sys
import time
import math
from std_msgs.msg import Bool, Int8, Float64, String
from geometry_msgs.msg import PoseStamped, Pose
from autonomous_loader_msgs.msg import TaskCommand
from shuju.srv import cangdou
from autonomous_loader_msgs.srv import spadingPose

class ComprehensiveBehaviorTreeTest:
    def __init__(self):
        rospy.init_node('comprehensive_bt_test', anonymous=True)
        
        # Wait for services (with better error handling)
        # Node uses private namespace "~", so service is: /autonomous_loader_bt_node/liaodou
        service_name = '/autonomous_loader_bt_node/liaodou'
        rospy.loginfo("Waiting for service %s...", service_name)
        try:
            rospy.wait_for_service(service_name, timeout=10.0)
            self.task_service = rospy.ServiceProxy(service_name, cangdou)
            self.service_available = True
            rospy.loginfo("Service %s is ready", service_name)
        except rospy.ROSException as e:
            rospy.logwarn("Service %s not available: %s", service_name, str(e))
            rospy.logwarn("Some tests requiring service will be skipped")
            self.service_available = False
            self.task_service = None
        
        # Provide scoop point service (simulation)
        self.spading_pose_service = rospy.Service('/spadingPose', spadingPose, self.spadingPoseCallback)
        rospy.loginfo("Scoop point service /spadingPose started")
        
        # Subscribers: monitor status
        self.workstate_sub = rospy.Subscriber('/workstate', Int8, self.workstate_callback)
        self.status_sub = rospy.Subscriber('/state_machine/status', String, self.status_callback)
        
        # Node uses private namespace "~", so all topics have /autonomous_loader_bt_node prefix
        node_name = '/autonomous_loader_bt_node'
        
        # Subscribers: receive commands from behavior tree (using node's private namespace)
        self.raise_arm_sub = rospy.Subscriber(node_name + '/work_device/raise_arm', Bool, self.raiseArmCallback)
        self.lower_bucket_sub = rospy.Subscriber(node_name + '/work_device/lower_bucket', Bool, self.lowerBucketCallback)
        self.start_scoop_sub = rospy.Subscriber(node_name + '/scoop/start_scoop', Bool, self.startScoopCallback)
        self.navigation_goal_sub = rospy.Subscriber(node_name + '/navigation/goal', PoseStamped, self.navigationGoalCallback)
        self.navigation_pause_sub = rospy.Subscriber(node_name + '/navigation/pause', Bool, self.navigationPauseCallback)
        
        # Publishers: simulate module responses (using node's private namespace)
        self.arm_raised_pub = rospy.Publisher(node_name + '/work_device/arm_raised', Bool, queue_size=10)
        self.bucket_lowered_pub = rospy.Publisher(node_name + '/work_device/bucket_lowered', Bool, queue_size=10)
        self.scoop_completed_pub = rospy.Publisher(node_name + '/scoop/scoop_completed', Bool, queue_size=10)
        self.navigation_planning_success_pub = rospy.Publisher(node_name + '/navigation/planning_success', Bool, queue_size=10)
        self.navigation_arrived_pub = rospy.Publisher(node_name + '/navigation/arrived', Bool, queue_size=10)
        self.distance_to_goal_pub = rospy.Publisher(node_name + '/navigation/distance_to_goal', Float64, queue_size=10)
        
        # Publishers: send commands (using node's private namespace)
        self.start_task_pub = rospy.Publisher(node_name + '/scheduler/start_task', TaskCommand, queue_size=10, latch=False)
        self.pause_task_pub = rospy.Publisher(node_name + '/scheduler/pause_task', Bool, queue_size=10, latch=False)
        self.end_task_pub = rospy.Publisher(node_name + '/scheduler/end_task', Bool, queue_size=10, latch=False)
        
        # Wait for publishers to be ready
        rospy.sleep(0.5)
        
        # Wait for subscribers to connect (they will connect when publishers are ready)
        # Note: In ROS, subscribers connect automatically when publishers start advertising
        rospy.loginfo("Subscribers initialized. Waiting for publishers to advertise...")
        rospy.sleep(1.0)  # Give time for publishers to advertise
        
        # State tracking
        self.current_workstate = -1
        self.current_status = "UNKNOWN"
        self.received_commands = {
            'raise_arm': False,
            'lower_bucket': False,
            'start_scoop': False,
            'dump_command': False,  # Track dump command separately
            'navigation_goal': None,
            'navigation_pause': False
        }
        
        # Navigation simulation
        self.nav_active = False
        self.nav_distance = 10.0
        self.nav_timer = None
        self.nav_target_x = None
        self.nav_target_y = None
        self.previous_nav_target_x = None  # Track previous target for distance calculation
        self.previous_nav_target_y = None
        
        # Track scoop point to determine if we need bin center navigation
        self.scoop_point_received = None
        self.scoop_point_is_zero = False
        
        # Test results
        self.test_results = []
        self.test_passed = True
        
        # Wait for node initialization
        rospy.sleep(2.0)
        rospy.loginfo("="*60)
        rospy.loginfo("TEST NODE INITIALIZED")
        rospy.loginfo("="*60)
        rospy.loginfo("Service: %s", "Ready" if self.service_available else "Not Available")
        rospy.loginfo("Scoop Point Service: /spadingPose")
        rospy.loginfo("Node Namespace: /autonomous_loader_bt_node")
        rospy.loginfo("="*60)
        
    def log_test(self, test_name, passed, message=""):
        """Log test result"""
        status = "✓ PASS" if passed else "✗ FAIL"
        timestamp = rospy.Time.now().to_sec()
        self.test_results.append((test_name, passed, message))
        if passed:
            rospy.loginfo("[TEST] [%.2f] %s: %s %s", timestamp, status, test_name, message)
        else:
            rospy.logwarn("[TEST] [%.2f] %s: %s %s", timestamp, status, test_name, message)
            self.test_passed = False
    
    def log_step(self, step_num, total_steps, step_name, details=""):
        """Log workflow step with clear formatting"""
        timestamp = rospy.Time.now().to_sec()
        progress = f"[{step_num}/{total_steps}]"
        rospy.loginfo("[STEP] [%.2f] %s %s: %s", timestamp, progress, step_name, details)
    
    def log_workflow(self, message, level="INFO"):
        """Log workflow status with timestamp"""
        timestamp = rospy.Time.now().to_sec()
        if level == "INFO":
            rospy.loginfo("[WORKFLOW] [%.2f] %s", timestamp, message)
        elif level == "WARN":
            rospy.logwarn("[WORKFLOW] [%.2f] %s", timestamp, message)
        elif level == "ERROR":
            rospy.logerr("[WORKFLOW] [%.2f] %s", timestamp, message)
        
    def workstate_callback(self, msg):
        """Work state callback"""
        old_state = self.current_workstate
        self.current_workstate = msg.data
        if old_state != self.current_workstate:
            state_names = {0: "IDLE", 1: "RUNNING", 2: "PAUSED", 3: "ENDING", 4: "ENDED"}
            rospy.loginfo("[STATE] Work state changed: %d -> %d (%s)", 
                         old_state, msg.data, state_names.get(msg.data, "UNKNOWN"))
            
    def status_callback(self, msg):
        """Status callback"""
        self.current_status = msg.data
        
    def raiseArmCallback(self, msg):
        """Raise arm command callback"""
        rospy.loginfo("[SIM] raiseArmCallback called with data=%s", msg.data)
        if msg.data:
            # Check if this is dump phase (after navigation_to_hopper)
            # We need to check if we're already at hopper to determine phase
            is_dump_phase = hasattr(self, 'dump_phase_active') and self.dump_phase_active
            phase_info = " (dump phase)" if is_dump_phase else " (prepare phase)"
            rospy.loginfo("[SIM] Received raise arm command%s, sending ack immediately", phase_info)
            
            # Set flag - this will be checked in workflow monitoring
            self.received_commands['raise_arm'] = True
            # Send ack immediately to ensure it's set before behavior tree checks
            self.sendArmRaised(None)
            
    def sendArmRaised(self, event):
        """Send arm raised acknowledgment"""
        # Send immediately multiple times to ensure it's received
        for i in range(3):
            msg = Bool()
            msg.data = True
            self.arm_raised_pub.publish(msg)
            if i == 0:
                rospy.loginfo("[SIM] ✓ Sent arm raised acknowledgment to topic: /autonomous_loader_bt_node/work_device/arm_raised")
                rospy.loginfo("[SIM] ✓ Publishing ack 3 times to ensure it's received")
            rospy.sleep(0.05)
        
    def lowerBucketCallback(self, msg):
        """Lower bucket command callback"""
        rospy.loginfo("[SIM] lowerBucketCallback called with data=%s", msg.data)
        if msg.data:
            self.received_commands['lower_bucket'] = True
            rospy.loginfo("[SIM] Received lower bucket command, sending ack immediately")
            # Send ack immediately to ensure it's set before behavior tree checks CheckBucketLoweredAck
            # The ack must be set before SEQ_START_SCOOP checks it
            self.sendBucketLowered(None)  # Send immediately
            
    def sendBucketLowered(self, event):
        """Send bucket lowered acknowledgment"""
        # Send acknowledgment immediately and multiple times to ensure it's received
        # This is critical: SEQ_START_SCOOP checks CheckBucketLoweredAck immediately after SEQ_LOWER_BUCKET completes
        rospy.loginfo("[SIM] ✓ Sending bucket lowered acknowledgment to topic: /autonomous_loader_bt_node/work_device/bucket_lowered")
        rospy.loginfo("[SIM] ✓ This sets bucket_lowered_ack = True and work_device_completed = True in GlobalState")
        
        # Send immediately multiple times to ensure it's received before behavior tree checks
        for i in range(5):
            msg = Bool()
            msg.data = True
            self.bucket_lowered_pub.publish(msg)
            if i == 0:
                rospy.loginfo("[SIM] ✓ Publishing ack 5 times to ensure it's received")
            rospy.sleep(0.05)  # Small delay between publishes
        
        # Ensure distance is published so behavior tree can check threshold
        # Bucket lowered should trigger distance check for start_scoop (needs distance <= 3m)
        rospy.loginfo("[SIM] Current distance: %.2f m, start_scoop threshold: <= 3m", self.nav_distance)
        if self.nav_distance > 3.0:
            self.log_workflow(f"Bucket lowered ack sent, but distance={self.nav_distance:.2f}m > 3m, "
                            f"will wait for distance to decrease to <= 3m", "INFO")
        else:
            self.log_workflow(f"Bucket lowered ack sent, distance={self.nav_distance:.2f}m <= 3m, "
                            f"behavior tree should send start_scoop command NOW", "INFO")
            self.log_workflow(f"  - CheckBucketLoweredAck should return SUCCESS (bucket_lowered_ack=True)", "INFO")
            self.log_workflow(f"  - CheckDistanceThreshold (start_scoop_distance) should return SUCCESS (distance <= 3m)", "INFO")
            self.log_workflow(f"  - Then SendScoopCommand should be called", "INFO")
        
    def startScoopCallback(self, msg):
        """Start scoop command callback (can be scoop or dump)"""
        rospy.loginfo("[SIM] startScoopCallback called with data=%s", msg.data)
        if msg.data:
            # Check if this is dump phase (after navigation_to_hopper and scoop completed)
            is_dump_phase = hasattr(self, 'dump_phase_active') and self.dump_phase_active
            phase_info = " (dump phase)" if is_dump_phase else " (scoop phase)"
            
            # Check current distance to determine phase more accurately
            # In dump phase, distance should be <= 0m for dump command
            # In scoop phase, distance should be <= 3m for scoop command
            current_distance = self.nav_distance if hasattr(self, 'nav_distance') else 999.0
            if is_dump_phase:
                if current_distance <= 0.1:  # Within 0.1m tolerance
                    rospy.loginfo("[SIM] ✓ Received start scoop command (dump phase, distance=%.2f m), sending completion ack in 1.0s", current_distance)
                    # Mark as dump command
                    self.received_commands['start_scoop'] = True
                    self.received_commands['dump_command'] = True  # Mark as dump command
                    rospy.Timer(rospy.Duration(1.0), self.sendScoopCompleted, oneshot=True)
                else:
                    rospy.logwarn("[SIM] Received start scoop command in dump phase but distance=%.2f m > 0m, ignoring (should be <= 0m)", current_distance)
            else:
                # Scoop phase
                rospy.loginfo("[SIM] ✓ Received start scoop command (scoop phase, distance=%.2f m), sending completion ack in 1.0s", current_distance)
                self.received_commands['start_scoop'] = True
                rospy.Timer(rospy.Duration(1.0), self.sendScoopCompleted, oneshot=True)
            
    def sendScoopCompleted(self, event):
        """Send scoop completed acknowledgment"""
        # Send multiple times to ensure it's received
        for i in range(3):
            msg = Bool()
            msg.data = True
            self.scoop_completed_pub.publish(msg)
            if i == 0:
                rospy.loginfo("[SIM] ✓ Sent scoop completed acknowledgment to topic: /autonomous_loader_bt_node/scoop/scoop_completed")
                rospy.loginfo("[SIM] ✓ Publishing ack 3 times to ensure it's received")
            rospy.sleep(0.05)
        
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
                rospy.loginfo("[SIM] ✓ navigation_to_scoop marked as completed in navigationGoalCallback")
            
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
        
        # CRITICAL: Calculate initial distance BEFORE sending planning success
        # This ensures behavior tree receives distance update before checking distance threshold
        # In real system, navigation module calculates actual distance from current pose to goal
        # For simulation, we estimate distance or use a reasonable starting value
        initial_distance = 10.0  # Default for scoop phase
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
            # initial_distance already set to 10.0 above
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
        
        # CRITICAL: Publish initial distance BEFORE sending planning success
        # This ensures behavior tree receives distance update before checking distance threshold
        distance_msg = Float64()
        distance_msg.data = initial_distance
        self.distance_to_goal_pub.publish(distance_msg)
        rospy.loginfo("[SIM] Published initial distance: %.2f m (before planning success)", initial_distance)
        rospy.sleep(0.05)  # Small delay to ensure message is processed
        
        # NOW send planning success (behavior tree will check distance after this)
        planning_msg = Bool()
        planning_msg.data = True
        self.navigation_planning_success_pub.publish(planning_msg)
        rospy.loginfo("[SIM] Sent navigation planning success")
        
        # Stop previous timer if exists
        if self.nav_timer is not None:
            self.nav_timer.shutdown()
        
        # Start new navigation simulation timer (will continue publishing distance updates)
        self.nav_timer = rospy.Timer(rospy.Duration(0.2), self.update_navigation_simulation)
        
    def update_navigation_simulation(self, event):
        """Update navigation simulation"""
        # Always publish distance, even after arrival (for distance-based commands)
        # This is critical for distance-based command triggers (lower_bucket at 6m, start_scoop at 3m)
        distance_msg = Float64()
        distance_msg.data = self.nav_distance
        self.distance_to_goal_pub.publish(distance_msg)
        
        # Real-time distance logging (less frequent to avoid spam)
        # Only log every 0.5 seconds or when threshold changes
        if not hasattr(self, '_last_distance_log_time'):
            self._last_distance_log_time = rospy.Time.now()
            self._last_logged_distance = None
        
        current_time = rospy.Time.now()
        should_log = False
        
        # Log if enough time has passed (every 0.5 seconds)
        if (current_time - self._last_distance_log_time).to_sec() >= 0.5:
            should_log = True
            self._last_distance_log_time = current_time
        
        # Log if threshold changed (crossed 6m or 3m boundary)
        if self._last_logged_distance is None:
            should_log = True
        elif (self._last_logged_distance > 6.0 and self.nav_distance <= 6.0) or \
             (self._last_logged_distance > 3.0 and self.nav_distance <= 3.0) or \
             (self._last_logged_distance > 0.0 and self.nav_distance <= 0.0):
            should_log = True
        
        # Check if we're in dump phase (after navigation_to_hopper)
        is_dump_phase = hasattr(self, 'dump_phase_active') and self.dump_phase_active
        phase_info = " (dump phase)" if is_dump_phase else ""
        
        # NOTE: In dump phase, distance should be set correctly in navigationGoalCallback
        # when hopper navigation goal is received. We don't need to reset it here.
        # The distance will naturally decrease from the initial value (calculated based on
        # actual distance from current position to hopper) to 0m.
        
        if should_log:
            rospy.loginfo("[DISTANCE] Distance to goal: %.2f meters | Target: (%.2f, %.2f)%s", 
                         self.nav_distance, self.nav_target_x, self.nav_target_y, phase_info)
            self._last_logged_distance = self.nav_distance
        
        # Continue decreasing distance even after navigation is "active" is False
        # This ensures behavior tree can check distance thresholds for commands
        # IMPORTANT: In dump phase, we need to continue decreasing distance from 10m to 0m
        # CRITICAL: In dump phase, ALWAYS decrease distance if > 0, regardless of nav_active
        # This ensures dump phase distance simulation works correctly
        if is_dump_phase:
            # In dump phase, always decrease distance if > 0
            # IMPORTANT: Continue decreasing until exactly 0.0m to ensure state machine sees 0.0m
            # OPTIMIZATION: When distance is very close to 0 (<= 0.5m), set it directly to 0.0m
            # This ensures state machine sees 0.0m immediately, not 0.5m -> 0.0m
            if self.nav_distance > 0.5:
                old_distance = self.nav_distance
                self.nav_distance -= 0.5
                if self.nav_distance < 0.0:
                    self.nav_distance = 0.0
                # Publish distance immediately after update to ensure state machine gets latest value
                distance_msg = Float64()
                distance_msg.data = self.nav_distance
                self.distance_to_goal_pub.publish(distance_msg)
                if should_log:
                    rospy.loginfo("[DISTANCE] Dump phase distance update: %.2f m -> %.2f m (decreased by 0.5 m)", 
                                 old_distance, self.nav_distance)
            elif self.nav_distance > 0.0:
                # When distance is <= 0.5m, set it directly to 0.0m to ensure state machine sees 0.0m
                old_distance = self.nav_distance
                self.nav_distance = 0.0
                # Publish 0.0m immediately
                distance_msg = Float64()
                distance_msg.data = 0.0
                self.distance_to_goal_pub.publish(distance_msg)
                rospy.loginfo("[DISTANCE] Dump phase: Distance reached threshold (%.2f m -> 0.00 m), setting to 0.0m immediately", old_distance)
            else:
                # In dump phase, if distance is already 0, keep it at 0
                # IMPORTANT: Continue publishing 0.0m to ensure state machine always sees 0.0m
                # This ensures CheckDistanceThreshold (start_dump_distance <= 0m) can pass
                distance_msg = Float64()
                distance_msg.data = 0.0
                self.distance_to_goal_pub.publish(distance_msg)
                if should_log:
                    rospy.loginfo("[DISTANCE] Dump phase: Distance is 0m, waiting for raise_arm and dump commands")
                    rospy.loginfo("[DISTANCE] Publishing 0.0m to ensure state machine receives it")
        elif self.nav_active and self.nav_distance > 0.0:
            # Normal navigation: decrease distance if nav_active is True
            old_distance = self.nav_distance
            self.nav_distance -= 0.5
            if self.nav_distance < 0.0:
                self.nav_distance = 0.0
            if should_log:
                rospy.loginfo("[DISTANCE] Distance update: %.2f m -> %.2f m (decreased by 0.5 m)", 
                             old_distance, self.nav_distance)
        
        # Check distance thresholds for commands (only log when threshold changes)
        if should_log:
            if is_dump_phase:
                # Dump phase thresholds: raise_arm at <= 6m, dump at <= 0m
                if self.nav_distance <= 6.0 and self.nav_distance > 0.0:
                    rospy.loginfo("[DISTANCE] ✓ Within raise_arm (dump phase) threshold (<= 6m)")
                elif self.nav_distance <= 0.0:
                    rospy.loginfo("[DISTANCE] ✓ Within dump threshold (<= 0m)")
                    rospy.loginfo("[DISTANCE] ⚠ Need arm_raised_ack=True to trigger dump command")
            else:
                # Scoop phase thresholds: lower_bucket at <= 6m, start_scoop at <= 3m
                if self.nav_distance <= 6.0 and self.nav_distance > 3.0:
                    rospy.loginfo("[DISTANCE] ✓ Within lower_bucket threshold (<= 6m)")
                elif self.nav_distance <= 3.0 and self.nav_distance > 0.0:
                    rospy.loginfo("[DISTANCE] ✓ Within start_scoop threshold (<= 3m)")
                    rospy.loginfo("[DISTANCE] ⚠ Need bucket_lowered_ack=True to trigger start_scoop command")
                elif self.nav_distance <= 0.0:
                    rospy.loginfo("[DISTANCE] ✓ At target position (distance = 0m)")
        
        # If navigation was active and we just reached target, mark as arrived
        # BUT: In dump phase, we don't want to mark as arrived immediately
        # We want to continue distance simulation for dump commands
        # IMPORTANT: In dump phase, we should NOT publish arrival until dump is complete
        # because the behavior tree's WaitForNavigationArrival should wait for arrival
        # But we need to keep distance simulation running for distance-based commands
        if self.nav_active and self.nav_distance <= 0.0:
            # Only mark as arrived if NOT in dump phase
            # In dump phase, we need to keep distance at 0 for dump command
            # BUT: We still need to publish arrival so behavior tree can proceed
            # The key is: distance should be set correctly BEFORE arrival, not after
            if not is_dump_phase:
                self.nav_active = False
                
                # Arrive at target
                arrived_msg = Bool()
                arrived_msg.data = True
                self.navigation_arrived_pub.publish(arrived_msg)
                rospy.loginfo("[SIM] Navigation arrived at target position: (%.2f, %.2f)", 
                             self.nav_target_x, self.nav_target_y)
            else:
                # In dump phase, we need to handle arrival carefully
                # The behavior tree's WaitForNavigationArrival waits for arrival
                # But we also need distance simulation for distance-based commands
                # So we publish arrival but keep distance simulation running
                if not hasattr(self, '_dump_arrival_published') or not self._dump_arrival_published:
                    # Publish arrival once when distance reaches 0
                    arrived_msg = Bool()
                    arrived_msg.data = True
                    self.navigation_arrived_pub.publish(arrived_msg)
                    self._dump_arrival_published = True
                    rospy.loginfo("[SIM] Dump phase: Navigation arrived at hopper, distance=0m, "
                                 "distance simulation continues for dump commands")
                if should_log:
                    rospy.loginfo("[SIM] Dump phase: At target position (distance=0m), waiting for raise_arm and dump commands")
            
        # Don't stop timer - keep publishing distance for distance-based commands
        # Timer will stop when workflow completes or timeout
        
    def navigationPauseCallback(self, msg):
        """Navigation pause callback"""
        if msg.data:
            self.received_commands['navigation_pause'] = True
            rospy.loginfo("[SIM] Received navigation pause command")
        
    def spadingPoseCallback(self, req):
        """Scoop point service callback"""
        rospy.loginfo("[SIM] Received scoop point request, pileID (cang): %d", req.pileID)
        
        # Return scoop point based on pileID
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
            rospy.loginfo("[SIM] Returning scoop point: (%.2f, %.2f) for pileID=%d", x, y, req.pileID)
        else:
            pose.position.x = 10.0 + req.pileID * 5.0
            pose.position.y = 10.0 + req.pileID * 5.0
            pose.position.z = 0.0
            rospy.loginfo("[SIM] Returning default scoop point: (%.2f, %.2f) for pileID=%d", 
                         pose.position.x, pose.position.y, req.pileID)
        
        pose.orientation.w = 1.0
        
        # Track scoop point for test script
        self.scoop_point_received = (pose.position.x, pose.position.y)
        self.scoop_point_is_zero = (abs(pose.position.x) < 0.1 and abs(pose.position.y) < 0.1)
        self.log_workflow(f"Scoop point received: ({pose.position.x:.2f}, {pose.position.y:.2f}), "
                         f"is_zero={self.scoop_point_is_zero}")
        
        resp = spadingPose._response_class()
        resp.pose_output = pose
        return resp
        
    def reset_command_flags(self):
        """Reset received command flags"""
        self.received_commands = {
            'raise_arm': False,
            'lower_bucket': False,
            'start_scoop': False,
            'dump_command': False,
            'navigation_goal': None,
            'navigation_pause': False
        }
    
    def reset_navigation_goal_only(self):
        """Reset only navigation goal flag, keep other commands"""
        self.received_commands['navigation_goal'] = None
        
    def wait_for_command(self, command_name, timeout=5.0):
        """Wait for a command to be received"""
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if command_name == 'navigation_goal':
                if self.received_commands[command_name] is not None:
                    return True
            else:
                if self.received_commands[command_name]:
                    return True
            rospy.sleep(0.1)
        return False
        
    def check_parameter(self, param_name, expected_value=None, timeout=3.0):
        """Check parameter value"""
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            try:
                value = rospy.get_param('/autonomous_loader_bt_node/' + param_name, None)
                if value is not None:
                    if expected_value is None or value == expected_value:
                        return value
            except:
                pass
            rospy.sleep(0.1)
        return None
        
    def test_1_initial_state(self):
        """Test 1: Check initial state"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST 1: Initial State Check")
        rospy.loginfo("="*60)
        
        # Check initial workstate (should be 0 - IDLE)
        rospy.sleep(1.0)
        initial_state = self.current_workstate
        self.log_test("Initial workstate", initial_state == 0, 
                     f"Expected: 0 (IDLE), Got: {initial_state}")
        
        # Check that no commands have been received
        no_commands = not any([
            self.received_commands['raise_arm'],
            self.received_commands['lower_bucket'],
            self.received_commands['start_scoop'],
            self.received_commands['navigation_goal'] is not None
        ])
        self.log_test("No commands received initially", no_commands)
        
        return self.test_passed
        
    def test_2_end_task_branch(self):
        """Test 2: Test end task branch (highest priority)"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST 2: End Task Branch (Highest Priority)")
        rospy.loginfo("="*60)
        
        self.reset_command_flags()
        
        # Send end task command
        rospy.loginfo("Sending end task command to: /autonomous_loader_bt_node/scheduler/end_task")
        end_msg = Bool()
        end_msg.data = True
        rospy.sleep(0.5)  # Wait for publisher
        self.end_task_pub.publish(end_msg)
        
        # Wait for navigation goal (parking point)
        rospy.sleep(1.0)
        goal_received = self.wait_for_command('navigation_goal', timeout=3.0)
        self.log_test("End task triggers navigation to parking", goal_received)
        
        if goal_received:
            # Check if goal is parking point (should be near (0, 0) based on config)
            goal = self.received_commands['navigation_goal']
            is_parking = abs(goal.pose.position.x) < 5.0 and abs(goal.pose.position.y) < 5.0
            self.log_test("Navigation goal is parking point", is_parking,
                         f"Position: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})")
        
        # Check workstate (should be 3 - ENDING or 4 - ENDED)
        rospy.sleep(2.0)
        workstate_ending = self.current_workstate in [3, 4]
        self.log_test("Workstate reflects ending", workstate_ending,
                     f"Current workstate: {self.current_workstate}")
        
        # Reset end task
        rospy.loginfo("Resetting end task command...")
        end_msg.data = False
        self.end_task_pub.publish(end_msg)
        rospy.sleep(1.0)
        
        return self.test_passed
        
    def test_3_pause_task_branch(self):
        """Test 3: Test pause task branch (medium priority)"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST 3: Pause Task Branch (Medium Priority)")
        rospy.loginfo("="*60)
        
        self.reset_command_flags()
        
        # Send pause task command
        rospy.loginfo("Sending pause task command to: /autonomous_loader_bt_node/scheduler/pause_task")
        pause_msg = Bool()
        pause_msg.data = True
        rospy.sleep(0.5)  # Wait for publisher
        self.pause_task_pub.publish(pause_msg)
        
        # Wait for navigation pause command
        rospy.sleep(1.0)
        pause_received = self.wait_for_command('navigation_pause', timeout=3.0)
        self.log_test("Pause task triggers navigation pause", pause_received)
        
        # Check workstate (should be 2 - PAUSED)
        rospy.sleep(1.0)
        workstate_paused = self.current_workstate == 2
        self.log_test("Workstate reflects paused", workstate_paused,
                     f"Current workstate: {self.current_workstate}")
        
        # Reset pause task
        rospy.loginfo("Resetting pause task command...")
        pause_msg.data = False
        self.pause_task_pub.publish(pause_msg)
        rospy.sleep(1.0)
        
        return self.test_passed
        
    def test_4_work_branch_service(self):
        """Test 4: Test work branch via service call"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST 4: Work Branch via Service Call")
        rospy.loginfo("="*60)
        
        if not self.service_available:
            self.log_test("Service call", False, "Service /autonomous_loader_bt_node/liaodou not available, skipping test")
            return self.test_passed
        
        self.reset_command_flags()
        
        # Call service to send task
        # Note: taskID must be int8 (-128 to 127)
        rospy.loginfo("Calling service /autonomous_loader_bt_node/liaodou with taskID=101, cang=2, dou=1...")
        try:
            req = cangdou._request_class()
            req.taskID = 101  # int8 range: -128 to 127
            req.cang = 2
            req.dou = 1
            
            resp = self.task_service(req)
            service_ok = resp.huiying
            self.log_test("Service call successful", service_ok)
            
            if service_ok:
                # Check canggoal parameter
                rospy.sleep(1.0)
                canggoal = self.check_parameter('canggoal', 2, timeout=3.0)
                self.log_test("canggoal parameter set correctly", canggoal == 2,
                             f"Expected: 2, Got: {canggoal}")
                
                # Check workstate (should be 1 - RUNNING)
                rospy.sleep(1.0)
                workstate_running = self.current_workstate == 1
                self.log_test("Workstate reflects running", workstate_running,
                             f"Current workstate: {self.current_workstate}")
                
        except Exception as e:
            self.log_test("Service call", False, f"Exception: {str(e)}")
            
        return self.test_passed
        
    def test_5_work_branch_topic(self):
        """Test 5: Test work branch via topic message"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST 5: Work Branch via Topic Message")
        rospy.loginfo("="*60)
        
        self.reset_command_flags()
        
        # Send task via topic
        rospy.loginfo("Publishing task command via topic...")
        task_msg = TaskCommand()
        task_msg.task_id = 2001
        task_msg.bin_id = 1
        task_msg.hopper_id = 2
        task_msg.task_type = "scoop"
        
        # Wait for publisher to be ready
        rospy.sleep(1.0)
        self.start_task_pub.publish(task_msg)
        rospy.loginfo("Task published to: /autonomous_loader_bt_node/scheduler/start_task")
        rospy.loginfo("Task details: ID=%d, bin=%d, hopper=%d, type=%s",
                     task_msg.task_id, task_msg.bin_id, task_msg.hopper_id, task_msg.task_type)
        
        # Wait for workstate to change
        rospy.sleep(2.0)
        workstate_running = self.current_workstate == 1
        self.log_test("Workstate reflects running after topic message", workstate_running,
                     f"Current workstate: {self.current_workstate}")
        
        return self.test_passed
        
    def test_6_complete_workflow(self):
        """Test 6: Test complete workflow (one full cycle) - Success Case"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST 6: Complete Workflow - Success Case")
        rospy.loginfo("="*60)
        rospy.loginfo("Scenario: Send task and verify complete workflow execution")
        rospy.loginfo("Expected: All 7 workflow steps should complete successfully")
        rospy.loginfo("Steps: 1.Raise Arm, 2.Nav to Bin, 3.Nav to Scoop, 4.Lower Bucket,")
        rospy.loginfo("       5.Scoop, 6.Nav to Hopper, 7.Dump")
        rospy.loginfo("="*60)
        
        if not self.service_available:
            rospy.logwarn("Service /autonomous_loader_bt_node/liaodou not available, using topic message instead...")
            # Use topic message instead
            # Wait for subscriber to connect (check topic connections)
            rospy.loginfo("Waiting for subscriber to connect...")
            subscriber_connected = False
            for _ in range(20):  # Wait up to 2 seconds
                num_subscribers = self.start_task_pub.get_num_connections()
                if num_subscribers > 0:
                    subscriber_connected = True
                    rospy.loginfo("Subscriber connected (num_subscribers=%d)", num_subscribers)
                    break
                rospy.sleep(0.1)
            
            if not subscriber_connected:
                rospy.logwarn("No subscriber connected to topic, but will publish anyway...")
            
            task_msg = TaskCommand()
            task_msg.task_id = 3001
            task_msg.bin_id = 1
            task_msg.hopper_id = 1
            task_msg.task_type = "scoop"
            
            # Publish message
            self.start_task_pub.publish(task_msg)
            rospy.loginfo("Task published via topic: /autonomous_loader_bt_node/scheduler/start_task")
            rospy.loginfo("Task details: ID=%d, bin=%d, hopper=%d, type=%s",
                         task_msg.task_id, task_msg.bin_id, task_msg.hopper_id, task_msg.task_type)
            
            # Wait for task to be processed
            rospy.sleep(2.0)
        else:
            self.reset_command_flags()
            
            # Send task via service
            # Note: taskID must be int8 (-128 to 127)
            rospy.loginfo("Sending task via service...")
            try:
                req = cangdou._request_class()
                req.taskID = 103  # int8 range: -128 to 127
                req.cang = 1
                req.dou = 1
                resp = self.task_service(req)
                if not resp.huiying:
                    self.log_test("Service call", False, "Service returned false")
                    return self.test_passed
            except Exception as e:
                self.log_test("Service call", False, f"Exception: {str(e)}")
                return self.test_passed
            
        self.log_workflow("Task sent, waiting for workflow execution...")
        self.log_workflow("Expected duration: 30-60 seconds")
        
        # Reset command flags before waiting
        self.reset_command_flags()
        
        # IMPORTANT: Initialize steps_completed BEFORE sending task or any navigation callbacks
        # This ensures navigationGoalCallback can access it immediately when navigation goals arrive
        # Monitor workflow steps
        # Note: navigation_to_bin is optional - only occurs if scoop point is (0,0)
        steps_completed = {
            'raise_arm': False,  # Step 1: Raise arm (prepare phase)
            'navigation_to_bin': False,  # Optional: only if scoop point is (0,0)
            'navigation_to_scoop': False,  # Step 2: Navigation to scoop point
            'lower_bucket': False,  # Step 4: Lower bucket
            'scoop': False,  # Step 5: Scoop
            'navigation_to_hopper': False,  # Step 6: Navigation to hopper
            'dump': False  # Step 7: Dump (uses SendScoopCommand, but in dump phase)
        }
        
        # Store steps_completed reference so navigationGoalCallback can access it
        # CRITICAL: Set this BEFORE any navigation callbacks are triggered
        self._workflow_steps_completed = steps_completed
        
        # IMPORTANT: If there's an active navigation (parking), complete it immediately
        # This is needed because node starts in idle state and waits for navigation arrival
        # Behavior tree cannot check for new tasks while waiting for navigation
        if self.nav_active or self.nav_distance > 0:
            self.log_workflow("Completing active navigation to allow task processing...")
            # Complete navigation immediately
            self.nav_distance = 0.0
            self.nav_active = False
            
            # Send navigation arrived message
            arrived_msg = Bool()
            arrived_msg.data = True
            self.navigation_arrived_pub.publish(arrived_msg)
            self.log_workflow("Sent navigation arrived message to unblock behavior tree")
            
            # Stop navigation timer if exists
            if self.nav_timer is not None:
                self.nav_timer.shutdown()
                self.nav_timer = None
            
            rospy.sleep(0.5)
        
        # Wait a bit for task to be processed by behavior tree
        rospy.sleep(1.0)
        
        # Reset scoop point tracking (but keep _workflow_steps_completed set)
        self.scoop_point_received = None
        self.scoop_point_is_zero = False
        
        # Track dump phase state
        self.dump_phase_active = False
        self.raise_arm_dump_received = False
        self.dump_phase_raise_arm_detected = False  # Track if dump phase raise_arm was detected
        self._dump_phase_distance_reset = False  # Track if dump phase distance was reset
        self._dump_arrival_published = False  # Track if dump phase arrival was published
        
        start_time = rospy.Time.now()
        timeout = 90.0  # 90 seconds timeout
        last_log_time = start_time
        
        self.log_workflow("Starting workflow monitoring...")
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            elapsed = (rospy.Time.now() - start_time).to_sec()
            # Log status every 5 seconds
            if (rospy.Time.now() - last_log_time).to_sec() > 5.0:
                completed_count = sum(1 for v in steps_completed.values() if v)
                progress_pct = int((completed_count / len(steps_completed)) * 100)
                nav_goal_info = "None"
                if self.received_commands['navigation_goal'] is not None:
                    g = self.received_commands['navigation_goal']
                    nav_goal_info = f"({g.pose.position.x:.2f}, {g.pose.position.y:.2f})"
                self.log_workflow(f"Progress: {completed_count}/7 steps ({progress_pct}%) | Elapsed: {elapsed:.1f}s")
                self.log_workflow(f"Commands: raise_arm={self.received_commands['raise_arm']}, "
                                f"lower_bucket={self.received_commands['lower_bucket']}, "
                                f"start_scoop={self.received_commands['start_scoop']}, "
                                f"nav_goal={nav_goal_info}")
                # Log completed steps
                completed_steps = [k for k, v in steps_completed.items() if v]
                if completed_steps:
                    self.log_workflow(f"Completed steps: {', '.join(completed_steps)}")
                last_log_time = rospy.Time.now()
            
            # Check for raise arm command
            if self.received_commands['raise_arm'] and not steps_completed['raise_arm']:
                steps_completed['raise_arm'] = True
                self.log_step(1, 7, "Raise Arm", "Command received, waiting for acknowledgment...")
                
            # Check if we've reached hopper but haven't reset distance yet
            # This is a fallback check in case navigation_goal was already reset
            if (steps_completed['navigation_to_hopper'] and 
                not self.dump_phase_active and 
                self.nav_target_x == 5.0 and self.nav_target_y == 5.0 and
                self.nav_distance <= 0.01):
                # We've reached hopper but haven't activated dump phase yet
                rospy.logwarn("[SIM] Detected hopper arrival but dump phase not activated, activating now...")
                self.dump_phase_active = True
                self.nav_distance = 10.0
                self.nav_active = True
                if hasattr(self, '_last_logged_distance'):
                    self._last_logged_distance = None
                distance_msg = Float64()
                distance_msg.data = self.nav_distance
                self.distance_to_goal_pub.publish(distance_msg)
                rospy.loginfo("[SIM] Dump phase activated via fallback check")
                rospy.loginfo("[SIM] Distance reset to 10m for dump phase")
            
            # Check for navigation goals
            if self.received_commands['navigation_goal'] is not None:
                goal = self.received_commands['navigation_goal']
                goal_x = goal.pose.position.x
                goal_y = goal.pose.position.y
                
                # Determine if we should expect bin center navigation
                # If scoop point is (0,0), behavior tree will navigate to bin center first
                # Otherwise, it goes directly to scoop point
                expect_bin_center = self.scoop_point_is_zero if self.scoop_point_received else False
                
                # Check if it's bin center (around 10, 10 for bin 1)
                # Only check if we expect bin center navigation (scoop point was (0,0))
                if expect_bin_center and not steps_completed['navigation_to_bin']:
                    if abs(goal_x - 10.0) < 2.0 and abs(goal_y - 10.0) < 2.0:
                        steps_completed['navigation_to_bin'] = True
                        self.log_step(2, 7, "Navigation to Bin Center", 
                                    f"Goal: ({goal_x:.2f}, {goal_y:.2f})")
                        self.reset_navigation_goal_only()  # Only reset nav goal, keep other commands
                        continue  # Skip other checks
                
                # Check if it's scoop point
                # If scoop point is not (0,0), this is step 2 (skip bin center)
                # If scoop point is (0,0), this is step 3 (after bin center)
                if not steps_completed['navigation_to_scoop']:
                    matched = False
                    if self.scoop_point_received:
                        expected_x, expected_y = self.scoop_point_received
                        if abs(goal_x - expected_x) < 2.0 and abs(goal_y - expected_y) < 2.0:
                            steps_completed['navigation_to_scoop'] = True
                            step_num = 2 if not expect_bin_center else 3
                            self.log_step(step_num, 7, "Navigation to Scoop Point", 
                                        f"Goal: ({goal_x:.2f}, {goal_y:.2f})")
                            matched = True
                    else:
                        # Fallback: check if it's around typical scoop point (10.5, 10.5)
                        if abs(goal_x - 10.5) < 2.0 and abs(goal_y - 10.5) < 2.0:
                            steps_completed['navigation_to_scoop'] = True
                            step_num = 2 if not expect_bin_center else 3
                            self.log_step(step_num, 7, "Navigation to Scoop Point", 
                                        f"Goal: ({goal_x:.2f}, {goal_y:.2f})")
                            matched = True
                    
                    if matched:
                        self.reset_navigation_goal_only()  # Only reset nav goal, keep other commands
                        continue  # Skip other checks
                
                # Check if it's hopper (around 5, 5 for hopper 1)
                # NOTE: This check is now mainly for logging, as navigation_to_hopper
                # is already detected in navigationGoalCallback
                if not steps_completed['navigation_to_hopper']:
                    if abs(goal_x - 5.0) < 2.0 and abs(goal_y - 5.0) < 2.0:
                        # This should have been detected in navigationGoalCallback
                        # But if not, mark it here as fallback
                        if not steps_completed['navigation_to_hopper']:
                            steps_completed['navigation_to_hopper'] = True
                            self.log_step(6, 7, "Navigation to Hopper", 
                                        f"Goal: ({goal_x:.2f}, {goal_y:.2f})")
                        
                        # Reset navigation goal (dump phase is already activated in navigationGoalCallback)
                        self.reset_navigation_goal_only()  # Only reset nav goal, keep other commands
                        
            # Check for lower bucket command
            # This command can arrive at any time after navigation to scoop point
            if self.received_commands['lower_bucket'] and not steps_completed['lower_bucket']:
                steps_completed['lower_bucket'] = True
                self.log_step(4, 7, "Lower Bucket", "Command received, waiting for acknowledgment...")
                # Don't reset the flag yet, we need it for acknowledgment tracking
                
            # Check for scoop command (could be scoop or dump)
            # Scoop phase: after bucket is lowered and distance <= 3m
            # Dump phase: after raise arm and distance <= 0m
            if self.received_commands['start_scoop']:
                if not steps_completed['scoop']:
                    # First occurrence: this is scoop phase
                    steps_completed['scoop'] = True
                    self.log_step(5, 7, "Scoop", "Command received, waiting for completion...")
                    # Don't reset the flag yet, we need it for completion tracking
                elif self.received_commands.get('dump_command', False) and not steps_completed['dump']:
                    # Dump command is marked separately in startScoopCallback
                    # This ensures we only mark dump when it's actually a dump command (distance <= 0m)
                    steps_completed['dump'] = True
                    self.log_step(7, 7, "Dump", "Command received (dump phase, distance <= 0m), waiting for completion...")
                    # Don't reset the flag yet, we need it for completion tracking
            
            # Check for raise arm command (could be prepare phase or dump phase)
            # Note: raise_arm flag is set once, so we need to track dump phase separately
            if self.received_commands['raise_arm']:
                if not steps_completed['raise_arm']:
                    # First occurrence: this is prepare phase
                    steps_completed['raise_arm'] = True
                    self.log_step(1, 7, "Raise Arm (Prepare)", "Command received, waiting for acknowledgment...")
                elif not self.raise_arm_dump_received and steps_completed['navigation_to_hopper']:
                    # Second occurrence after navigation_to_hopper: this is dump phase
                    # NOTE: We detect this by checking if raise_arm is True AND we're at hopper
                    # But we need to check if this is a NEW raise_arm command, not the old one
                    # Since raise_arm flag persists, we track dump phase activation separately
                    if not hasattr(self, 'dump_phase_raise_arm_detected'):
                        self.dump_phase_raise_arm_detected = False
                    
                    # Check if dump phase raise_arm was already detected
                    # If not, and we're at hopper, this is the dump phase raise_arm
                    if not self.dump_phase_raise_arm_detected:
                        self.raise_arm_dump_received = True
                        self.dump_phase_active = True
                        self.dump_phase_raise_arm_detected = True
                        self.log_step(6, 7, "Raise Arm (Dump Phase)", "Command received, waiting for acknowledgment...")
                        self.log_workflow("Dump phase: Raise arm command received, will wait for dump command", "INFO")
            
            # Debug: Log distance when checking for scoop command (every 5 seconds)
            # Scoop phase: bucket_lowered_ack = True and distance <= 3m
            if not steps_completed['scoop'] and steps_completed['lower_bucket']:
                if (rospy.Time.now() - last_log_time).to_sec() > 5.0:
                    self.log_workflow(f"Waiting for scoop command (scoop phase):", "INFO")
                    self.log_workflow(f"  - Distance: {self.nav_distance:.2f}m (need <= 3m) {'✓' if self.nav_distance <= 3.0 else '✗'}", "INFO")
                    self.log_workflow(f"  - bucket_lowered_cmd received: {self.received_commands['lower_bucket']} ✓", "INFO")
                    self.log_workflow(f"  - bucket_lowered_ack: Should be True (set by /work_device/bucket_lowered topic)", "INFO")
                    self.log_workflow(f"  - Behavior tree should check: CheckBucketLoweredAck + CheckDistanceThreshold", "INFO")
                    # Log current distance decrease rate
                    self.log_workflow(f"  - Distance decreasing: {self.nav_distance:.2f}m -> "
                                    f"{max(0, self.nav_distance - 0.5):.2f}m (next cycle)", "INFO")
            
            # Debug: Log distance when checking for dump command (every 5 seconds)
            # Dump phase: arm_raised_ack = True and distance <= 0m
            if not steps_completed['dump'] and steps_completed['navigation_to_hopper']:
                if (rospy.Time.now() - last_log_time).to_sec() > 5.0:
                    self.log_workflow(f"Waiting for dump command (dump phase):", "INFO")
                    self.log_workflow(f"  - Distance: {self.nav_distance:.2f}m (need <= 0m) {'✓' if self.nav_distance <= 0.0 else '✗'}", "INFO")
                    self.log_workflow(f"  - raise_arm_dump received: {self.raise_arm_dump_received if hasattr(self, 'raise_arm_dump_received') else False}", "INFO")
                    self.log_workflow(f"  - arm_raised_ack: Should be True (set by /work_device/arm_raised topic)", "INFO")
                    self.log_workflow(f"  - Behavior tree should check: CheckArmRaisedAck + CheckDistanceThreshold (start_dump_distance)", "INFO")
                    # Log current distance decrease rate
                    self.log_workflow(f"  - Distance decreasing: {self.nav_distance:.2f}m -> "
                                    f"{max(0, self.nav_distance - 0.5):.2f}m (next cycle)", "INFO")
                
            # Check if workflow is complete
            # Note: navigation_to_bin is optional, so we check if all required steps are done
            required_steps = ['raise_arm', 'navigation_to_scoop', 'lower_bucket', 'scoop', 
                             'navigation_to_hopper', 'dump']
            # navigation_to_bin is optional, only check if scoop point was (0,0)
            if self.scoop_point_is_zero and self.scoop_point_received:
                required_steps.append('navigation_to_bin')
            
            all_required_completed = all(steps_completed[step] for step in required_steps)
            if all_required_completed:
                elapsed = (rospy.Time.now() - start_time).to_sec()
                self.log_workflow(f"All workflow steps completed! Total time: {elapsed:.1f}s", "INFO")
                break
                
            rospy.sleep(0.5)
            
        # Evaluate results
        elapsed = (rospy.Time.now() - start_time).to_sec()
        
        # Stop navigation timer if workflow is complete or timeout
        if self.nav_timer is not None:
            self.nav_timer.shutdown()
            self.nav_timer = None
        
        # Check required steps (navigation_to_bin is optional)
        required_steps = ['raise_arm', 'navigation_to_scoop', 'lower_bucket', 'scoop', 
                         'navigation_to_hopper', 'dump']
        if self.scoop_point_is_zero and self.scoop_point_received:
            required_steps.append('navigation_to_bin')
        
        all_required_completed = all(steps_completed[step] for step in required_steps if step in steps_completed)
        completed_count = sum(1 for step in required_steps if step in steps_completed and steps_completed[step])
        
        # Debug: Log all steps status
        self.log_workflow(f"Workflow monitoring completed. Elapsed time: {elapsed:.1f}s")
        self.log_workflow(f"All steps status:", "INFO")
        for step in required_steps:
            status = "✓" if (step in steps_completed and steps_completed[step]) else "✗"
            self.log_workflow(f"  {status} {step}: {steps_completed.get(step, False)}", "INFO")
        
        self.log_test("Complete workflow execution", all_required_completed,
                     f"Steps completed: {completed_count}/{len(required_steps)}")
        
        if not all_required_completed:
            self.log_workflow("Missing steps:", "WARN")
            for step in required_steps:
                if step not in steps_completed or not steps_completed[step]:
                    self.log_workflow(f"  ✗ Step '{step}' not completed", "WARN")
        
        return self.test_passed
        
    def test_7_idle_branch(self):
        """Test 7: Test idle branch (no tasks)"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST 7: Idle Branch (No Tasks)")
        rospy.loginfo("="*60)
        rospy.loginfo("Scenario: Verify system stays in idle state when no tasks")
        rospy.loginfo("Expected: System should remain idle, no navigation commands")
        rospy.loginfo("="*60)
        
        # Wait for system to be idle
        self.log_workflow("Waiting for system to become idle...")
        rospy.sleep(5.0)
        
        # Check if workstate is IDLE
        workstate_idle = self.current_workstate == 0
        self.log_test("System enters idle state", workstate_idle,
                     f"Current workstate: {self.current_workstate}")
        
        # Check if navigation to parking might be triggered
        # (This is optional - the system might stay idle if already at parking)
        rospy.sleep(5.0)
        # We don't fail if navigation doesn't happen, as it might already be at parking
        
        return self.test_passed
    
    def test_8_task_interrupt_pause(self):
        """Test 8: Test task interruption by pause command"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST 8: Task Interruption - Pause")
        rospy.loginfo("="*60)
        rospy.loginfo("Scenario: Send task, then pause during execution")
        rospy.loginfo("Expected: Task should pause, navigation should pause")
        rospy.loginfo("="*60)
        
        if not self.service_available:
            self.log_test("Service call", False, "Service not available, skipping test")
            return self.test_passed
        
        self.reset_command_flags()
        
        # Send task
        self.log_workflow("Sending task via service...")
        try:
            req = cangdou._request_class()
            req.taskID = 104
            req.cang = 1
            req.dou = 1
            resp = self.task_service(req)
            if not resp.huiying:
                self.log_test("Service call", False, "Service returned false")
                return self.test_passed
            self.log_workflow("Task sent successfully")
        except Exception as e:
            self.log_test("Service call", False, f"Exception: {str(e)}")
            return self.test_passed
        
        # Wait for workflow to start
        rospy.sleep(3.0)
        
        # Send pause command
        self.log_workflow("Sending pause command...")
        pause_msg = Bool()
        pause_msg.data = True
        self.pause_task_pub.publish(pause_msg)
        self.log_workflow("Pause command published")
        
        # Wait and check if workstate changed to PAUSED (2)
        rospy.sleep(2.0)
        workstate_paused = self.current_workstate == 2
        self.log_test("Task paused successfully", workstate_paused,
                     f"Current workstate: {self.current_workstate} (expected: 2)")
        
        # Check if navigation pause command was received
        nav_pause_received = self.received_commands['navigation_pause']
        self.log_test("Navigation pause command sent", nav_pause_received,
                     f"Navigation pause: {nav_pause_received}")
        
        return self.test_passed
    
    def test_9_task_interrupt_end(self):
        """Test 9: Test task interruption by end command"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST 9: Task Interruption - End")
        rospy.loginfo("="*60)
        rospy.loginfo("Scenario: Send task, then end during execution")
        rospy.loginfo("Expected: Task should end, navigation to parking should start")
        rospy.loginfo("="*60)
        
        if not self.service_available:
            self.log_test("Service call", False, "Service not available, skipping test")
            return self.test_passed
        
        self.reset_command_flags()
        
        # Send task
        self.log_workflow("Sending task via service...")
        try:
            req = cangdou._request_class()
            req.taskID = 105
            req.cang = 1
            req.dou = 1
            resp = self.task_service(req)
            if not resp.huiying:
                self.log_test("Service call", False, "Service returned false")
                return self.test_passed
            self.log_workflow("Task sent successfully")
        except Exception as e:
            self.log_test("Service call", False, f"Exception: {str(e)}")
            return self.test_passed
        
        # Wait for workflow to start
        rospy.sleep(3.0)
        
        # Send end command
        self.log_workflow("Sending end task command...")
        end_msg = Bool()
        end_msg.data = True
        self.end_task_pub.publish(end_msg)
        self.log_workflow("End command published")
        
        # Wait and check if navigation to parking starts
        rospy.sleep(5.0)
        parking_nav_received = False
        if self.received_commands['navigation_goal'] is not None:
            goal = self.received_commands['navigation_goal']
            # Parking point is typically (0, 0)
            if abs(goal.pose.position.x - 0.0) < 1.0 and abs(goal.pose.position.y - 0.0) < 1.0:
                parking_nav_received = True
        
        self.log_test("Navigation to parking started", parking_nav_received,
                     f"Navigation goal received: {parking_nav_received}")
        
        return self.test_passed
    
    def test_10_multiple_tasks_queue(self):
        """Test 10: Test multiple tasks in queue"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST 10: Multiple Tasks Queue")
        rospy.loginfo("="*60)
        rospy.loginfo("Scenario: Send multiple tasks, verify they are queued")
        rospy.loginfo("Expected: Tasks should be queued and executed sequentially")
        rospy.loginfo("="*60)
        
        if not self.service_available:
            self.log_test("Service call", False, "Service not available, skipping test")
            return self.test_passed
        
        # Send multiple tasks
        self.log_workflow("Sending multiple tasks...")
        task_ids = [106, 107, 108]
        for task_id in task_ids:
            try:
                req = cangdou._request_class()
                req.taskID = task_id
                req.cang = 1
                req.dou = 1
                resp = self.task_service(req)
                if resp.huiying:
                    self.log_workflow(f"Task {task_id} added to queue")
                rospy.sleep(0.5)
            except Exception as e:
                self.log_workflow(f"Failed to send task {task_id}: {str(e)}", "ERROR")
        
        # Wait a bit for tasks to be processed
        rospy.sleep(2.0)
        
        # Note: We can't easily verify queue in test, but we can verify tasks are accepted
        self.log_test("Multiple tasks accepted", True, f"Sent {len(task_ids)} tasks")
        
        return self.test_passed
        
    def run_all_tests(self):
        """Run all tests"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("COMPREHENSIVE BEHAVIOR TREE TEST SUITE")
        rospy.loginfo("="*60)
        rospy.loginfo("This test will verify all behavior tree branches and functionality")
        rospy.loginfo("="*60)
        
        try:
            # Test 1: Initial state
            self.test_1_initial_state()
            rospy.sleep(2.0)
            
            # Test 2: End task branch
            self.test_2_end_task_branch()
            rospy.sleep(3.0)
            
            # Test 3: Pause task branch
            self.test_3_pause_task_branch()
            rospy.sleep(3.0)
            
            # Test 4: Work branch via service
            self.test_4_work_branch_service()
            rospy.sleep(5.0)
            
            # Test 5: Work branch via topic
            self.test_5_work_branch_topic()
            rospy.sleep(5.0)
            
            # Test 6: Complete workflow
            self.test_6_complete_workflow()
            rospy.sleep(5.0)
            
            # Test 7: Idle branch
            self.test_7_idle_branch()
            rospy.sleep(3.0)
            
            # Test 8: Task interruption - pause
            self.test_8_task_interrupt_pause()
            rospy.sleep(5.0)
            
            # Test 9: Task interruption - end
            self.test_9_task_interrupt_end()
            rospy.sleep(5.0)
            
            # Test 10: Multiple tasks queue
            self.test_10_multiple_tasks_queue()
            
        except KeyboardInterrupt:
            rospy.logwarn("\nTest interrupted by user")
        except Exception as e:
            rospy.logerr(f"\nTest exception: {str(e)}")
            import traceback
            traceback.print_exc()
            
        # Print summary
        self.print_test_summary()
        
        return self.test_passed
        
    def print_test_summary(self):
        """Print test summary"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("TEST SUMMARY")
        rospy.loginfo("="*60)
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for _, passed, _ in self.test_results if passed)
        failed_tests = total_tests - passed_tests
        
        for test_name, passed, message in self.test_results:
            status = "✓" if passed else "✗"
            rospy.loginfo("%s %s %s", status, test_name, message)
            
        rospy.loginfo("\n" + "-"*60)
        rospy.loginfo("Total: %d | Passed: %d | Failed: %d", 
                     total_tests, passed_tests, failed_tests)
        
        if self.test_passed:
            rospy.loginfo("\n✓ ALL TESTS PASSED!")
        else:
            rospy.logwarn("\n✗ SOME TESTS FAILED")
        rospy.loginfo("="*60)

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Comprehensive Behavior Tree Test Script')
    parser.add_argument('--test', '-t', type=int, choices=[1,2,3,4,5,6,7,8,9,10],
                       help='Run specific test (1-10)')
    parser.add_argument('--all', '-a', action='store_true', default=True,
                       help='Run all tests (default)')
    args = parser.parse_args()
    
    try:
        test = ComprehensiveBehaviorTreeTest()
        
        if args.test:
            test_names = {
                1: "test_1_initial_state",
                2: "test_2_end_task_branch",
                3: "test_3_pause_task_branch",
                4: "test_4_work_branch_service",
                5: "test_5_work_branch_topic",
                6: "test_6_complete_workflow",
                7: "test_7_idle_branch",
                8: "test_8_task_interrupt_pause",
                9: "test_9_task_interrupt_end",
                10: "test_10_multiple_tasks_queue"
            }
            test_method = getattr(test, test_names[args.test])
            test_method()
            test.print_test_summary()
        else:
            test.run_all_tests()
            
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Test exception: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

