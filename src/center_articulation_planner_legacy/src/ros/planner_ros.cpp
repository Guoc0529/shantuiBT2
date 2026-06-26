#include "center_articulation_planner/ros/planner_ros.hpp"
#include "center_articulation_planner/ros/ros_io.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <sstream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>
#include <cstdint>

/*
  - 如果只看当前启用的代码路径，并假设过程中没有 Error，那么“内部自主规划”的
    链路是单一的：Idle �?PlanningGlobalPath �?AdjustingDirection �?Moving �?
    Completed。onReach() 里的“只测一段”早退让循环被短路，所以在一条路径内不会看到额外分支�?
    
  - 仍然存在第二条无错误链路：当外部模块直接推�?/global_path_ts 时，状态机会从
    Idle（或任何状态）立即跳到 Moving，执行完同样�?Moving �?Completed�?
    这条支路跳过了全局规划和方向调整，适用于“外部已经规划好”的模式�?

    TODO:
    1. �?is_forward 改为“命令广�?+ 分节点反馈”模型（/is_forward_cmd + /is_forward_state/<node>），
    不再走单话题/单service：（已加入is_forward反馈�?
*/

namespace
{
  // Bring namespaced TaskType into this TU-local helper scope
  using jhzx::center_articulation_planner::TaskType;
  constexpr std::uint8_t kBrakeRelease = 0;
  constexpr std::uint8_t kBrake30Percent = 77; // round(255 * 0.30)

  // 状态机的纯内部helper，不在其他任何源文件中使�?
  const char *plannerStateToString(PlannerState state)
  {
    switch (state)
    {
    case PlannerState::Idle:
      return "Idle";
    case PlannerState::PlanningGlobalPath:
      return "PlanningGlobalPath";
    case PlannerState::AdjustingDirection:
      return "AdjustingDirection";
    case PlannerState::AligningInPlace:
      return "AligningInPlace";
    case PlannerState::Moving:
      return "Moving";
    case PlannerState::CheckingGoal:
      return "CheckingGoal";
    case PlannerState::BackingOutUnload:
      return "BackingOutUnload";
    case PlannerState::Completed:
      return "Completed";
    case PlannerState::Error:
      return "Error";
    default:
      return "Unknown";
    }
  }

  const char *taskTypeToString(TaskType task)
  {
    switch (task)
    {
    case TaskType::InitialShovel:
      return "InitialShovel";
    case TaskType::NormalShovel:
      return "NormalShovel";
    case TaskType::InitialUnload:
      return "InitialUnload";
    case TaskType::NormalUnload:
      return "NormalUnload";
    case TaskType::MoveAround:
      return "MoveAround";
    case TaskType::Unknown:
    default:
      return "Unknown";
    }
  }

  // ============================================================
  // RAII 守卫类：确保规划参数被恢�?
  // ============================================================
  // 什么是 RAII�?
  // RAII = Resource Acquisition Is Initialization（资源获取即初始化）
  // 核心思想：在对象构造时获取资源，在对象析构时释放资�?
  //
  // 为什么要用它�?
  // - C++ 保证：当对象离开作用域时，一定会调用析构函数
  // - 即使代码中�?return、throw 异常，析构函数也会被调用
  // - 这样就能确保"清理工作"（恢复参数）一定会执行
  // ============================================================
  class DirectionSwitchParamGuard
  {
  public:
    // 类型别名，便于调�?setMaxDirectionSwitches
    using PlannerType = jhzx::center_articulation_planner::PlannerROS;

    // 构造函数：保存 planner 指针和原始�?
    DirectionSwitchParamGuard(PlannerType *planner, int original_value)
        : planner_(planner), original_value_(original_value), should_restore_(true)
    {
    }

    // 析构函数：恢复原始参数�?
    // 无论函数如何退出（正常return、异常、提前return），这里都会执行
    ~DirectionSwitchParamGuard()
    {
      if (should_restore_ && planner_)
      {
        planner_->setMaxDirectionSwitches(original_value_);
      }
    }

    // 禁止拷贝（防止意外复制导致多次恢复）
    DirectionSwitchParamGuard(const DirectionSwitchParamGuard &) = delete;
    DirectionSwitchParamGuard &operator=(const DirectionSwitchParamGuard &) = delete;

  private:
    PlannerType *planner_;
    int original_value_;
    bool should_restore_;
  };

} // namespace

namespace jhzx::center_articulation_planner
{
  void PlannerROS::initialize(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    pnh.param<std::string>("plan_service", plan_service_name_, plan_service_name_);
    pnh.param<std::string>("set_params_service", set_params_service_name_, set_params_service_name_);
    pnh.param("planner_service_timeout", planner_service_timeout_, planner_service_timeout_);
    pnh.param("direction_switch_timeout_sec", direction_switch_timeout_sec_, direction_switch_timeout_sec_);
    pnh.param("direction_feedback_window_sec", direction_feedback_window_sec_, direction_feedback_window_sec_);
    pnh.param("direction_switch_settle_delay_sec",
              direction_switch_settle_delay_sec_,
              direction_switch_settle_delay_sec_);
    pnh.param("final_goal_feedback_required", final_goal_feedback_required_, final_goal_feedback_required_);
    pnh.param("pile_low_threshold_percent", pile_low_threshold_percent_, pile_low_threshold_percent_);
    pnh.param("pile_low_offset_x_m", pile_low_offset_x_m_, pile_low_offset_x_m_);
    pnh.param("unload_goal_offset_x_m", unload_goal_offset_x_m_, unload_goal_offset_x_m_);
    pnh.param("unload_trigger_distance", unload_trigger_distance_, unload_trigger_distance_);
    pnh.param("load_early_complete_distance_m", load_early_complete_distance_m_, load_early_complete_distance_m_);
    pnh.param("debug_mode", debug_mode_, debug_mode_);
    pnh.param("backout_yaw_threshold_rad", backout_yaw_threshold_rad_, backout_yaw_threshold_rad_);
    pnh.param("backout_distance_m", backout_distance_m_, backout_distance_m_);
    pnh.param("backout_lateral_threshold_m", backout_lateral_threshold_m_, backout_lateral_threshold_m_);
    pnh.param("backout_lateral_yaw_threshold_rad", backout_lateral_yaw_threshold_rad_, backout_lateral_yaw_threshold_rad_);
    pnh.param("goal_guard_distance_m", goal_guard_distance_m_, goal_guard_distance_m_);
    pnh.param("goal_guard_angle_deg", goal_guard_angle_deg_, goal_guard_angle_deg_);
    pnh.param("goal_guard_forward_deadzone_m", goal_guard_forward_deadzone_m_, goal_guard_forward_deadzone_m_);
    pnh.param("align_pos_thresh_m", align_pos_thresh_m_, align_pos_thresh_m_);
    pnh.param("align_lateral_thresh_m", align_lateral_thresh_m_, align_pos_thresh_m_);  // 默认与 align_pos_thresh_m_ 相同
    pnh.param("align_heading_thresh_deg", align_heading_thresh_deg_, align_heading_thresh_deg_);
    pnh.param("large_lateral_replan_thresh_m", large_lateral_replan_thresh_m_, large_lateral_replan_thresh_m_);
    pnh.param("max_remaining_replan_count", max_remaining_replan_count_, max_remaining_replan_count_);

    planner_backend_ = std::make_unique<RemotePlannerBackend>(nh,
                                                              plan_service_name_,
                                                              set_params_service_name_,
                                                              planner_service_timeout_);
    local_planner_ptr_ = std::make_shared<local_planner::LocalPlanner>(nh);
    ROS_INFO_STREAM("PlannerROS using remote planner services: plan=" << plan_service_name_
                    << ", set_params=" << set_params_service_name_
                    << ", timeout=" << planner_service_timeout_ << "s");
  }

  // Router, 较为低频的，moving状态分离到一个高频的timer中处理了
  void PlannerROS::onTimer()
  {
    // === 状态机 ===
    switch (state_)
    {
    case PlannerState::Idle:
      ROS_INFO_THROTTLE(5.0, "Planner is idle, waiting for goal...");
      break;
    case PlannerState::PlanningGlobalPath:
      // 计算转折�?
      ROS_INFO_THROTTLE(3.0, "PlanningGlobalPath...");
      planning_global_path();
      break;
    
    case PlannerState::AdjustingDirection:
      if (!nav_path_.empty() && ros_io_)
      {
        ros_io_->publishSegmentedPath(nav_path_.front());
      }
      // 调整车辆当前状态适配前进还是后退
      ROS_INFO_THROTTLE(3.0, "Adjusting vehicle direction...");
      adjust_vehicle_direction();
      break;
    case PlannerState::AligningInPlace:
      ROS_INFO_THROTTLE(3.0, "Aligning in place...");
      if (!align_service_called_)
      {
        align_service_called_ = true;
        if (ros_io_)
        {
          // 计算目标角度（基于当前航向偏差，单位：度）
          // /way_point_shan_tui 是基于车辆坐标系的lookahead点
          double target_angle = 0.0;
          if (has_shan_tui_waypoint_)
          {
            // 获取车辆坐标系中的lookahead航向角
            const double raw_heading = tf2::getYaw(shan_tui_waypoint_.pose.orientation);
            target_angle = std::atan2(std::sin(raw_heading), std::cos(raw_heading)) * 180.0 / M_PI;
          }
          else
          {
            ROS_WARN("AligningInPlace: /way_point_shan_tui not available, using 0 deg");
          }

          // 调用新API：传递前/后车架标志 + 目标角度
          bool use_rear = (active_pose_source_ == PoseSource::Rear);
          align_target_angle_deg_ = static_cast<float>(target_angle);
          ROS_WARN("AligningInPlace: calling align_in_place service (target_angle=%.1f deg, use_rear=%d)",
                   align_target_angle_deg_,
                   static_cast<int>(use_rear));
          ros_io_->callAlignInPlace(use_rear, align_target_angle_deg_);
          ros_io_->startMovingTimer();

          ROS_INFO("Called align_in_place with target_angle=%.1f deg, use_rear=%d",
                   align_target_angle_deg_,
                   static_cast<int>(use_rear));
        }
        transitionTo(PlannerState::Moving, "align_in_place completed");
      }
      break;

    case PlannerState::Moving:
      // 移动中，等待到达通知
      // 因为moving需求频率较高，所以开另一个moving timer去了
      if (nav_path_.empty())
      {
        ROS_INFO_THROTTLE(3.0, "Moving... path not ready");
      }
      else
      {
        const std::size_t total = (total_segments_ == 0) ? nav_path_.size() + segment_index_ : total_segments_;
        ROS_INFO_THROTTLE(3.0,
                          "Moving... total=%zu, current=%zu",
                          total,
                          segment_index_ + 1);
      }
      break;
    case PlannerState::BackingOutUnload:
      if (!buildBackoutPaths())
      {
        setError(ErrorCode::NavPathUnavailable, "buildBackoutPaths failed");
      }
      else
      {
        transitionTo(PlannerState::AdjustingDirection, "backout paths ready");
      }
      break;
    case PlannerState::CheckingGoal:
    {
      // 开始计时
      if (checking_goal_start_time_.isZero())  
      {
        checking_goal_start_time_ = ros::Time::now();
      }

      // 两个goal都收到，且都成功
      if (reach_goal_received_ && final_goal_received_ && final_goal_ok_)
      {
        transitionToCompletedOrBackout("goal and final goal confirmed");
        break;
      }

      // timeout
      const double elapsed = (ros::Time::now() - checking_goal_start_time_).toSec();
      if (elapsed > checking_goal_timeout_sec_)
      {
        final_goal_ok_ = false;
        final_error_code_ = static_cast<std::uint8_t>(ErrorCode::FinalGoalCheckFailed);
        final_error_msg_ = "timeout waiting for goal_reached/final_goal_reached";
        transitionTo(PlannerState::Error, final_error_msg_);
        break;
      }
      ROS_INFO_THROTTLE(2.0,
                        "CheckingGoal: waiting confirmations (reach=%d, final=%d, elapsed=%.2fs, timeout=%.2fs)",
                        static_cast<int>(reach_goal_received_),
                        static_cast<int>(final_goal_received_),
                        elapsed,
                        checking_goal_timeout_sec_);
      break;
    }
    case PlannerState::Completed:
      // 规划完成
      ROS_INFO_THROTTLE(1.0, "Completed. Waiting for new goal...");
      break;
    case PlannerState::Error:
      // 出错状态，无动�?
    {
      const std::string msg = formatErrorStateMessage();
      ROS_ERROR_THROTTLE(3.0, "%s", msg.c_str());
      break;
    }
    default:
      ROS_ERROR_THROTTLE(5.0, "PlannerROS 进入未知状�? %d", static_cast<int>(state_));
      break;
    }
  }

  void PlannerROS::onMovingTimer()
  {
    // 仅在 Moving 状态下工作
    if (state_ != PlannerState::Moving)
    {
      return;
    }
    if (!has_distance_)
    {
      setError(ErrorCode::NoDistanceFeedback);
      return;
    }
    local_path_.poses.clear();  // 先清�?

    // 如果使用后车架坐标，clip 时将朝向翻转 180 �?
    geometry_msgs::Pose pose_for_clip = current_pose_.pose;
    if (active_pose_source_ == PoseSource::Rear)
    {
      double yaw = tf2::getYaw(pose_for_clip.orientation);
      yaw = std::atan2(std::sin(yaw + M_PI), std::cos(yaw + M_PI));
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose_for_clip.orientation = tf2::toMsg(q);
    }

    // 按前视点截断当前路径的一�?
    // 把截断的路径喂给locplanner进行优化
    local_path_ = clipPathByLookahead(nav_path_.front(), pose_for_clip);
    // 交给局部规划器进行（可选）平滑/优化，TODO: 为啥注释掉？
    // local_planner_ptr_->plan(local_path_);
    const double remaining = remainingDistanceToGoal();
    const double seg_remaining = distanceToCurrentSegmentGoal();

    // 近终点不可达守卫，仅在 debug 模式生效
    const bool guard_window_active = debug_mode_ &&
                                     goal_guard_distance_m_ > 0.0 &&
                                     seg_remaining <= goal_guard_distance_m_;
    goal_guard_active_ = guard_window_active;
    if (goal_guard_active_ && !goal_guard_logged_)
    {
      ROS_INFO("Goal guard enabled (distance<=%.2f m, angle>%.1f deg, forward_deadzone=%.2f m)",
               goal_guard_distance_m_,
               goal_guard_angle_deg_,
               goal_guard_forward_deadzone_m_);
      goal_guard_logged_ = true;
    }
    if (goal_guard_active_)
    {
      double forward_m = 0.0;
      double lateral_m = 0.0;
      double angle_deg = 0.0;
      std::string reason;
      const bool unreachable = nearGoalUnreachable(seg_remaining, forward_m, lateral_m, angle_deg, reason);
      if (unreachable)
      {
        ROS_WARN_THROTTLE(0.5,
                          "Goal guard check: seg_remaining=%.2f m, forward=%.2f m, lateral=%.2f m, yaw_err=%.1f deg, status=%s (%s)",
                          seg_remaining,
                          forward_m,
                          lateral_m,
                          angle_deg,
                          "UNREACHABLE",
                          reason.c_str());
      }
      else
      {
        ROS_INFO_THROTTLE(0.5,
                          "Goal guard check: seg_remaining=%.2f m, forward=%.2f m, lateral=%.2f m, yaw_err=%.1f deg, status=%s (%s)",
                          seg_remaining,
                          forward_m,
                          lateral_m,
                          angle_deg,
                          "ok",
                          reason.c_str());
      }
      if (unreachable)
      {
        const bool is_last_segment = (nav_path_.size() == 1);

        // 姿态差计算（rear 源需加 pi）
        double yaw_curr = tf2::getYaw(current_pose_.pose.orientation);
        if (active_pose_source_ == PoseSource::Rear)
        {
          yaw_curr = std::atan2(std::sin(yaw_curr + M_PI), std::cos(yaw_curr + M_PI));
        }
        double yaw_goal = 0.0;
        if (!nav_path_.empty() && !nav_path_.front().poses.empty())
        {
          yaw_goal = tf2::getYaw(nav_path_.front().poses.back().pose.orientation);
        }
        const double yaw_err = std::atan2(std::sin(yaw_curr - yaw_goal), std::cos(yaw_curr - yaw_goal));
        const double yaw_err_deg = std::abs(yaw_err * 180.0 / M_PI);

        if (is_last_segment)
        {
          if (ros_io_)
          {
            ros_io_->stopMovingTimer();
            ros_io_->publishBrake(kBrake30Percent);
          }

          if (yaw_err_deg > 45.0)
          {
            setError(ErrorCode::FinalGoalCheckFailed, "goal guard yaw mismatch");
            return;
          }

          reach_goal_received_ = true;
          final_goal_received_ = true;
          final_goal_ok_ = true;
          final_distance_error_ = 0.0;
          final_angle_error_ = yaw_err;
          final_error_code_ = 0;
          final_error_msg_.clear();
          transitionTo(PlannerState::Completed, "goal guard reached final segment");
          return;
        }

        // 非最后一段：停控，跳到下一段
        if (!nav_path_.empty())
        {
          nav_path_.erase(nav_path_.begin());
        }
        if (!segment_backward_flags_.empty())
        {
          segment_backward_flags_.erase(segment_backward_flags_.begin());
        }
        if (!segment_lengths_.empty())
        {
          segment_lengths_.erase(segment_lengths_.begin());
        }
        ++segment_index_;
        armLastUnloadDistanceRefreshGate("goal guard skipped into last unload segment");
        armLastLoadDistanceRefreshGate("goal guard skipped into last load segment");
        goal_guard_active_ = false;
        goal_guard_logged_ = false;

        if (!segment_backward_flags_.empty())
        {
          const bool next_backward = segment_backward_flags_.front();
          setPoseSource(next_backward ? PoseSource::Rear : PoseSource::Front);
          updateCurrentPoseFromActiveSource();
        }
        else
        {
          togglePoseSource(segment_index_);
        }

        transitionTo(PlannerState::AdjustingDirection, "goal guard skipped current segment");
        return;
      }
    }

    // 局部规划器发布局部路�?
    if (ros_io_)
    {
      ros_io_->publishLocalPath(local_path_);
    }

    if (ros_io_)
    {
      const bool is_load_task = (current_task_ == TaskType::NormalShovel ||
                                 current_task_ == TaskType::InitialShovel);
      const bool is_unload_task = (current_task_ == TaskType::NormalUnload ||
                                   current_task_ == TaskType::InitialUnload);
      const bool is_last_segment = (nav_path_.size() == 1);

      if (is_unload_task && is_last_segment && waiting_last_segment_distance_refresh_)
      {
        tryClearLastSegmentDistanceRefreshGate();
        if (waiting_last_segment_distance_refresh_)
        {
          const double jump = distance_to_goal_ - prev_segment_tail_distance_;
          ROS_WARN_THROTTLE(0.2,
                            "[unload_debug] waiting last-segment distance refresh: dist=%.3f prev_tail=%.3f jump=%.3f min=%.3f",
                            distance_to_goal_,
                            prev_segment_tail_distance_,
                            jump,
                            last_segment_jump_min_m_);
        }
      }

      if (is_unload_task && is_last_segment)
      {
        ROS_WARN_THROTTLE(0.2, "[unload_debug] remaining=%.3f", remaining);
      }

      if (is_load_task && is_last_segment && waiting_load_last_segment_distance_refresh_)
      {
        tryClearLastLoadDistanceRefreshGate();
        if (waiting_load_last_segment_distance_refresh_)
        {
          const double jump = distance_to_goal_ - prev_load_segment_tail_distance_;
          ROS_WARN_THROTTLE(0.2,
                            "[load_debug] waiting last-segment distance refresh: dist=%.3f prev_tail=%.3f jump=%.3f min=%.3f",
                            distance_to_goal_,
                            prev_load_segment_tail_distance_,
                            jump,
                            last_segment_jump_min_m_);
        }
      }

      ros_io_->publishActionFeedback(remaining, true);

      if (is_load_task &&
          is_last_segment &&
          !waiting_load_last_segment_distance_refresh_ &&
          load_early_complete_distance_m_ > 0.0 &&
          remaining <= load_early_complete_distance_m_)
      {
        ros_io_->stopMovingTimer();
        final_goal_ok_ = true;
        final_distance_error_ = 0.0;
        final_angle_error_ = 0.0;
        final_error_code_ = 0;
        final_error_msg_.clear();
        transitionTo(PlannerState::Completed, "load early complete near goal");
        return;
      }

      if (is_unload_task &&
          is_last_segment &&
          !waiting_last_segment_distance_refresh_ &&
          !unload_flag_sent_ &&
          remaining <= unload_trigger_distance_)
      {
        ros_io_->publishIsUnload(true);
        unload_flag_sent_ = true;
        ROS_INFO("Publishing is_unload=true (remaining=%.2f m)", remaining);
      }
    }
  }

  void PlannerROS::armLastUnloadDistanceRefreshGate(const std::string &reason)
  {
    const bool is_unload_task = (current_task_ == TaskType::NormalUnload ||
                                 current_task_ == TaskType::InitialUnload);
    if (!is_unload_task || nav_path_.size() != 1)
    {
      return;
    }

    waiting_last_segment_distance_refresh_ = true;
    prev_segment_tail_distance_ = has_distance_ ? distance_to_goal_ : 0.0;
    ROS_INFO("Entered last unload segment: gating is_unload until distance jump > %.2f m (prev_tail=%.3f, reason=%s)",
             last_segment_jump_min_m_,
             prev_segment_tail_distance_,
             reason.c_str());
  }

  void PlannerROS::armLastLoadDistanceRefreshGate(const std::string &reason)
  {
    const bool is_load_task = (current_task_ == TaskType::NormalShovel ||
                               current_task_ == TaskType::InitialShovel);
    if (!is_load_task || nav_path_.size() != 1)
    {
      return;
    }

    waiting_load_last_segment_distance_refresh_ = true;
    prev_load_segment_tail_distance_ = has_distance_ ? distance_to_goal_ : 0.0;
    ROS_INFO("Entered last load segment: gating early completion until distance jump > %.2f m (prev_tail=%.3f, reason=%s)",
             last_segment_jump_min_m_,
             prev_load_segment_tail_distance_,
             reason.c_str());
  }

  void PlannerROS::tryClearLastSegmentDistanceRefreshGate()
  {
    if (!waiting_last_segment_distance_refresh_ || nav_path_.size() != 1 || !has_distance_)
    {
      return;
    }

    const double jump = distance_to_goal_ - prev_segment_tail_distance_;
    if (jump <= last_segment_jump_min_m_)
    {
      return;
    }

    waiting_last_segment_distance_refresh_ = false;
    ROS_WARN("Last unload segment distance refreshed: dist=%.3f prev_tail=%.3f jump=%.3f (> %.3f)",
             distance_to_goal_,
             prev_segment_tail_distance_,
             jump,
             last_segment_jump_min_m_);
  }

  void PlannerROS::tryClearLastLoadDistanceRefreshGate()
  {
    if (!waiting_load_last_segment_distance_refresh_ || nav_path_.size() != 1 || !has_distance_)
    {
      return;
    }

    const double jump = distance_to_goal_ - prev_load_segment_tail_distance_;
    if (jump <= last_segment_jump_min_m_)
    {
      return;
    }

    waiting_load_last_segment_distance_refresh_ = false;
    ROS_WARN("Last load segment distance refreshed: dist=%.3f prev_tail=%.3f jump=%.3f (> %.3f)",
             distance_to_goal_,
             prev_load_segment_tail_distance_,
             jump,
             last_segment_jump_min_m_);
  }
  
  // 状态转移，实际的状态转移都从这个函数走，便于track
  std::string PlannerROS::formatErrorStateMessage() const
  {
    std::ostringstream oss;
    oss << "Planner in error state";

    if (final_error_code_ != 0)
    {
      oss << " [code=" << static_cast<int>(final_error_code_) << "]";
    }

    if (!final_error_msg_.empty())
    {
      oss << ": " << final_error_msg_;
    }
    else
    {
      oss << ": unknown error";
    }

    oss << ". Please reset by sending a new goal.";
    return oss.str();
  }

  void PlannerROS::transitionTo(PlannerState next_state, const std::string &reason)
  {
    if (state_ == next_state)
    {
      return;
    }

    const PlannerState previous = state_;
    state_ = next_state;

    // DEBUG
    std::string message = "Planner state transition: ";
    message += plannerStateToString(previous);
    message += " -> ";
    message += plannerStateToString(next_state);
    if (!reason.empty())
    {
      message += " (" + reason + ")";
    }
    const bool goal_guard_completed =
        (next_state == PlannerState::Completed && reason == "goal guard reached final segment");
    if (goal_guard_completed)
    {
      ROS_WARN("%s", message.c_str());
    }
    else
    {
      ROS_INFO("%s", message.c_str());
    }

    // 【优化】离开Moving状态时，确保发布controller_started=false
    if (next_state != PlannerState::Error && ros_io_)
    {
      ros_io_->setDiagnosticOk(plannerStateToString(next_state));
    }

    if (previous == PlannerState::Moving && next_state != PlannerState::Moving)
    {
      if (ros_io_)
      {
        ros_io_->publishControllerStarted(false);
        ROS_INFO("Published controller_started=false (left Moving state: %s -> %s)",
                 plannerStateToString(previous),
                 plannerStateToString(next_state));
      }
    }

    // 结束时重�?
    if (next_state == PlannerState::Completed)
    {
      onEnterCompleted();
    }
    else if (next_state == PlannerState::Moving)
    {
      if (ros_io_)
      {
        ros_io_->publishControllerStarted(true);
        ROS_INFO("Published controller_started=true to /controller_started");
      }
    }
    else if (next_state == PlannerState::Error)
    {
      if (final_error_msg_.empty() && !reason.empty())
      {
        final_error_msg_ = reason;
      }
      ROS_ERROR("%s", formatErrorStateMessage().c_str());
      if (ros_io_)
      {
        ros_io_->publishIsUnload(false);
        // controller_started=false已在上面的离开Moving逻辑中处理
        ros_io_->publishActionResult(false,
                                     final_distance_error_,
                                     final_angle_error_,
                                     final_error_code_,
                                     final_error_msg_);
      }
    }
  }

  void PlannerROS::onEnterCompleted()
  {
    if (ros_io_)
    {
      // ros_io_->publishIsUnload(false);
      ros_io_->publishControllerStarted(false);
      ROS_INFO("Published controller_started=false because of completion.");
      ros_io_->publishActionResult(true,
                                   final_distance_error_,
                                   final_angle_error_,
                                   final_error_code_,
                                   final_error_msg_);
    }
    reset();
  }

  void PlannerROS::setError(ErrorCode code, const std::string &detail)
  {
    final_error_code_ = static_cast<std::uint8_t>(code);
    const std::string base_msg = defaultErrorMessage(code);
    if (detail.empty())
    {
      final_error_msg_ = base_msg;
    }
    else
    {
      final_error_msg_ = base_msg + ": " + detail;
    }

    if (ros_io_)
    {
      switch (code)
      {
      case ErrorCode::PlanFailed:
        ros_io_->setDiagnosticError("7101", final_error_msg_);
        break;
      case ErrorCode::BackendNull:
      case ErrorCode::PlanServiceUnavailable:
        ros_io_->setDiagnosticError("7102", final_error_msg_);
        break;
      case ErrorCode::NoDistanceFeedback:
        ros_io_->setDiagnosticError("7103", final_error_msg_);
        break;
      default:
        break;
      }
    }

    transitionTo(PlannerState::Error, final_error_msg_);
  }

  // 规划、获取、分段路�?
  void PlannerROS::planning_global_path()
  {
    if (!planner_backend_)
    {
      ROS_ERROR("Remote planner backend is not initialized.");
      if (ros_io_)
      {
        ros_io_->publishActionFeedback(999.0, false);
      }
      setError(ErrorCode::BackendNull);
      return;
    }

    geometry_msgs::PoseStamped plan_start = current_pose_;
    bool using_front_offset = false;
    if (use_offset_front_at_unload_start_)
    {
      if (!makeOffsetStartFromFront(1.2, plan_start))
      {
        if (ros_io_)
        {
          ros_io_->publishActionFeedback(999.0, false);
        }
        return;
      }
      using_front_offset = true;
    }
    ROS_INFO("Planning start: x=%.2f, y=%.2f, yaw=%.2f (using_front_offset=%d)",
             plan_start.pose.position.x,
             plan_start.pose.position.y,
             tf2::getYaw(plan_start.pose.orientation),
             static_cast<int>(using_front_offset));

    goal_adjusted_for_pile_ = false;
    bool goal_adjusted_for_pile = false;
    geometry_msgs::PoseStamped plan_goal = buildPlanGoal(goal_adjusted_for_pile);

    if (ros_io_)
    {
      geometry_msgs::PoseStamped service_start = plan_start;
      service_start.header.stamp = ros::Time::now();
      ros_io_->publishPlanServiceStart(service_start);

      geometry_msgs::PoseStamped service_goal = plan_goal;
      service_goal.header.stamp = ros::Time::now();
      ros_io_->publishPlanServiceGoal(service_goal);
    }

    const int8_t task_type = remotePlannerTaskType();

    PlanAttemptResult best_attempt;
    PlanAttemptResult primary_attempt = runPlanAttempt(plan_start, plan_goal, "primary", task_type);
    if (primary_attempt.success)
    {
      best_attempt = std::move(primary_attempt);
    }
    else
    {
      ROS_WARN("Primary planning failed (%s).", primary_attempt.message.c_str());

      if (isUnloadTask())
      {
        ROS_WARN("Trying unload swapped start-goal fallback before offset-based fallbacks.");
        PlanAttemptResult swapped_attempt =
            runPlanAttempt(plan_goal, plan_start, "fallback_unload_swap_start_goal", task_type);
        if (swapped_attempt.success)
        {
          reversePlanForTracking(swapped_attempt);
          best_attempt = std::move(swapped_attempt);
          ROS_INFO("Planning succeeded with unload swapped start-goal fallback.");
        }
        else
        {
          ROS_WARN("Unload swapped start-goal fallback failed: %s", swapped_attempt.message.c_str());
        }
      }

      if (!best_attempt.success)
      {
        ROS_WARN("Trying fallback offsets toward articulation center.");
        geometry_msgs::PoseStamped offset_start;
        if (makeOffsetStartTowardCenter(1.5, offset_start))
        {
          const double yaw_step = M_PI / 9.0;
          const std::array<double, 3> yaw_offsets{{0.0, yaw_step, -yaw_step}};
          for (const double yaw_delta : yaw_offsets)
          {
            geometry_msgs::PoseStamped candidate_start = yawOffsetPose(offset_start, yaw_delta);
            const char *yaw_label = (std::abs(yaw_delta) < 1e-6) ? "0deg" : (yaw_delta > 0.0 ? "+20deg" : "-20deg");
            std::string tag = std::string("fallback_offset_") + yaw_label;
            PlanAttemptResult attempt = runPlanAttempt(candidate_start, plan_goal, tag, task_type);
            if (!attempt.success)
            {
              ROS_WARN("Plan attempt %s failed: %s", tag.c_str(), attempt.message.c_str());
              continue;
            }
            if (!best_attempt.success || attempt.total_length < best_attempt.total_length)
            {
              best_attempt = std::move(attempt);
            }
          }
        }
        else
        {
          ROS_WARN("Failed to compute offset start pose for fallback planning.");
        }
      }
    }

    if (!best_attempt.success)
    {
      ROS_WARN("All swap/offset fallbacks failed, trying with increased direction switches (2)...");

      PlanAttemptResult increased_switch_attempt =
          tryPlanWithIncreasedSwitches(plan_start, plan_goal, task_type);

      if (increased_switch_attempt.success)
      {
        best_attempt = std::move(increased_switch_attempt);
        ROS_INFO("Planning succeeded with increased direction switches (max_switches=2)");
      }
      else
      {
        ROS_ERROR("All planning attempts failed (including increased direction switches)");
        if (ros_io_)
        {
          ros_io_->publishActionFeedback(999.0, false);
        }
        const std::string detail =
            primary_attempt.message.empty() ? "all planning attempts failed" : primary_attempt.message;
        setError(primary_attempt.error_code, detail);
        return;
      }
    }

    if (best_attempt.tag != "primary")
    {
      ROS_WARN("Using fallback plan '%s' (total length=%.2f m)",
               best_attempt.tag.c_str(),
               best_attempt.total_length);
    }
    else
    {
      ROS_INFO("Primary plan selected (total length=%.2f m)", best_attempt.total_length);
    }

    const std::string selected_tag = best_attempt.tag;
    const double planning_time_sec = best_attempt.planning_time_sec;
    applyPlanAttempt(std::move(best_attempt), goal_adjusted_for_pile, true);

    if (planning_time_sec > 0.0)
    {
      ROS_INFO_STREAM("Remote planner succeeded in " << planning_time_sec << "s ("
                                                     << selected_tag << ")");
    }
    armLastUnloadDistanceRefreshGate("planning produced single unload segment");
    armLastLoadDistanceRefreshGate("planning produced single load segment");
    const std::string reason = selected_tag.empty() ? "global path ready"
                                                    : "global path ready via " + selected_tag;
    transitionTo(PlannerState::AdjustingDirection, reason);
  }

  bool PlannerROS::buildSegmentsFromPlan(const nav_msgs::Path &path,
                                         const std::vector<int8_t> &direction_flags,
                                         const std::vector<int32_t> &segment_starts,
                                         const std::vector<int32_t> &segment_ends)
  {
    nav_path_.clear();
    segment_backward_flags_.clear();
    segment_lengths_.clear();
    return buildSegmentsFromPlan(path,
                                 direction_flags,
                                 segment_starts,
                                 segment_ends,
                                 nav_path_,
                                 segment_backward_flags_,
                                 segment_lengths_);
  }

  bool PlannerROS::buildSegmentsFromPlan(const nav_msgs::Path &path,
                                         const std::vector<int8_t> &direction_flags,
                                         const std::vector<int32_t> &segment_starts,
                                         const std::vector<int32_t> &segment_ends,
                                         std::vector<nav_msgs::Path> &out_nav_path,
                                         std::vector<bool> &out_backward_flags,
                                         std::vector<double> &out_segment_lengths)
  {
    out_nav_path.clear();
    out_backward_flags.clear();
    out_segment_lengths.clear();

    if (path.poses.empty())
    {
      return false;
    }

    // 确保分段索引一一对应
    if (segment_starts.empty() || segment_starts.size() != segment_ends.size())
    {
      ROS_ERROR("Segment indices missing or size mismatch (starts=%zu, ends=%zu).",
                segment_starts.size(), segment_ends.size());
      return false;
    }

    // 确保方向标志与分段索引一一对应
    if (direction_flags.empty() || direction_flags.size() != segment_starts.size())
    {
      ROS_ERROR("Direction flags missing or size mismatch (flags=%zu, segments=%zu).",
                direction_flags.size(), segment_starts.size());
      return false;
    }

    // lamda:  先创建一个空�?seg，然后按索引区间把对应的路径�?
    // 拷贝进去，并根据 backward 调整朝向。输�?nav_msgs::Path
    const auto addSegment = [&](std::size_t start_idx, std::size_t end_idx, bool backward) {
      nav_msgs::Path seg;
      fillSegment(seg, path, start_idx, end_idx, backward);
      if (!seg.poses.empty())
      {
        out_nav_path.push_back(std::move(seg));
        out_backward_flags.push_back(backward);
        out_segment_lengths.push_back(computePathLength(out_nav_path.back()));
      }
    };

    bool has_valid_segment = false;
    // 遍历所有分段索引，大路径分成小路径
    for (std::size_t i = 0; i < segment_starts.size(); ++i)
    {
      const auto s = static_cast<std::size_t>(std::max(0, segment_starts[i]));
      const auto e = static_cast<std::size_t>(std::max(0, segment_ends[i]));
      //  如果起止下标越界或起点大于终点，跳过
      if (s >= path.poses.size() || e >= path.poses.size() || s > e)
      {
        ROS_WARN("Skipping invalid segment [%d, %d] for path size %zu",
                 segment_starts[i], segment_ends[i], path.poses.size());
        continue;
      }
      const bool backward = isBackwardFlag(direction_flags[i]); // 每段对应一个标�?
      addSegment(s, e, backward);  // 实际切片
      has_valid_segment = true;
    }

    if (!has_valid_segment)
    {
      ROS_ERROR("No valid segments were built from planner response.");
      return false;
    }

    return !out_nav_path.empty() &&
           out_nav_path.size() == out_backward_flags.size() &&
           out_nav_path.size() == out_segment_lengths.size();
  }

  PlannerROS::PlanAttemptResult PlannerROS::runPlanAttempt(const geometry_msgs::PoseStamped &start,
                                                           const geometry_msgs::PoseStamped &goal,
                                                           const std::string &tag,
                                                           int8_t task_type)
  {
    PlanAttemptResult attempt;
    attempt.tag = tag;

    if (!planner_backend_)
    {
      attempt.message = "Remote planner backend is not initialized.";
      attempt.error_code = ErrorCode::BackendNull;
      return attempt;
    }

    const RemotePlanResult plan_result = planner_backend_->plan(start, goal, task_type);
    if (!plan_result.call_ok)
    {
      attempt.message = "Remote plan call failed: " + plan_result.message;
      attempt.error_code = ErrorCode::PlanServiceUnavailable;
      return attempt;
    }
    if (!plan_result.success)
    {
      attempt.message = "Remote planner reported failure: " + plan_result.message;
      attempt.error_code = ErrorCode::PlanFailed;
      return attempt;
    }
    if (plan_result.path.poses.empty())
    {
      attempt.message = "Remote planner returned an empty path.";
      attempt.error_code = ErrorCode::EmptyPath;
      return attempt;
    }

    std::vector<nav_msgs::Path> nav_segments;
    std::vector<bool> backward_flags;
    std::vector<double> segment_lengths;
    if (!buildSegmentsFromPlan(plan_result.path,
                               plan_result.direction_flags,
                               plan_result.segment_starts,
                               plan_result.segment_ends,
                               nav_segments,
                               backward_flags,
                               segment_lengths))
    {
      attempt.message = "Failed to build segmented path from remote planner response.";
      attempt.error_code = ErrorCode::SegmentBuildFailed;
      return attempt;
    }

    attempt.success = true;
    attempt.message = plan_result.message;
    attempt.global_path = plan_result.path;
    attempt.nav_segments = std::move(nav_segments);
    attempt.backward_flags = std::move(backward_flags);
    attempt.segment_lengths = std::move(segment_lengths);
    attempt.total_length = std::accumulate(attempt.segment_lengths.begin(),
                                           attempt.segment_lengths.end(),
                                           0.0);
    attempt.planning_time_sec = plan_result.planning_time_sec;
    return attempt;
  }

  geometry_msgs::PoseStamped PlannerROS::buildPlanGoal(bool &goal_adjusted_for_pile)
  {
    geometry_msgs::PoseStamped plan_goal = current_goal_pose_;

    if (current_task_ == TaskType::NormalUnload || current_task_ == TaskType::InitialUnload)
    {
      plan_goal.pose.position.x += unload_goal_offset_x_m_;
      plan_goal.header.stamp = ros::Time::now();
    }

    goal_adjusted_for_pile = false;
    if (current_status_ == 0 && ros_io_)
    {
      const int volume = ros_io_->getPileVolume(current_id_);
      if (volume >= 0 && volume < pile_low_threshold_percent_)
      {
        const double original_x = plan_goal.pose.position.x;
        plan_goal.pose.position.x -= pile_low_offset_x_m_;
        plan_goal.header.stamp = ros::Time::now();
        goal_adjusted_for_pile = true;

        ROS_INFO("Pile %u volume low (%d%% < %d%%), adjusted goal x: %.2f -> %.2f (offset %.2fm)",
                 static_cast<unsigned>(current_id_),
                 volume,
                 pile_low_threshold_percent_,
                 original_x,
                 plan_goal.pose.position.x,
                 plan_goal.pose.position.x - original_x);
      }
      else if (volume >= pile_low_threshold_percent_)
      {
        ROS_INFO("Pile %u volume sufficient (%d%% >= %d%%), using original goal (x=%.2f)",
                 static_cast<unsigned>(current_id_),
                 volume,
                 pile_low_threshold_percent_,
                 plan_goal.pose.position.x);
      }
    }

    return plan_goal;
  }

  int8_t PlannerROS::remotePlannerTaskType() const
  {
    return (last_status_ == 0) ? 4 : 0;
  }

  bool PlannerROS::isUnloadTask() const
  {
    return current_task_ == TaskType::NormalUnload ||
           current_task_ == TaskType::InitialUnload;
  }

  void PlannerROS::reversePlanForTracking(PlanAttemptResult &attempt)
  {
    std::reverse(attempt.global_path.poses.begin(), attempt.global_path.poses.end());

    for (auto &segment : attempt.nav_segments)
    {
      std::reverse(segment.poses.begin(), segment.poses.end());
    }
    std::reverse(attempt.nav_segments.begin(), attempt.nav_segments.end());

    std::reverse(attempt.backward_flags.begin(), attempt.backward_flags.end());
    for (std::size_t i = 0; i < attempt.backward_flags.size(); ++i)
    {
      attempt.backward_flags[i] = !attempt.backward_flags[i];
    }

    std::reverse(attempt.segment_lengths.begin(), attempt.segment_lengths.end());
  }
  void PlannerROS::applyPlanAttempt(PlanAttemptResult attempt,
                                    bool goal_adjusted_for_pile,
                                    bool reset_segment_index)
  {
    goal_adjusted_for_pile_ = goal_adjusted_for_pile;
    global_path_ = std::move(attempt.global_path);
    nav_path_ = std::move(attempt.nav_segments);
    segment_backward_flags_ = std::move(attempt.backward_flags);
    segment_lengths_ = std::move(attempt.segment_lengths);
    total_segments_ = nav_path_.size();
    local_path_.poses.clear();
    goal_guard_active_ = false;
    goal_guard_logged_ = false;
    if (reset_segment_index)
    {
      segment_index_ = 0;
    }

    if (ros_io_)
    {
      ros_io_->publishGlobalPath(global_path_);
    }

    if ((current_task_ == TaskType::NormalUnload || current_task_ == TaskType::InitialUnload) &&
        !nav_path_.empty())
    {
      geometry_msgs::PoseStamped original_goal = current_goal_pose_;
      original_goal.header.stamp = ros::Time::now();
      global_path_.poses.push_back(original_goal);
      nav_path_.back().poses.push_back(original_goal);
      ROS_INFO_STREAM("Appended original unload goal to path tail: x="
                      << original_goal.pose.position.x << " y=" << original_goal.pose.position.y
                      << " yaw=" << tf2::getYaw(original_goal.pose.orientation));
    }

    if (goal_adjusted_for_pile_ && !nav_path_.empty())
    {
      geometry_msgs::PoseStamped original_goal = current_goal_pose_;
      original_goal.header.stamp = ros::Time::now();
      global_path_.poses.push_back(original_goal);
      nav_path_.back().poses.push_back(original_goal);

      ROS_INFO_STREAM("Appended original shovel goal to path tail: x="
                      << original_goal.pose.position.x << " y="
                      << original_goal.pose.position.y
                      << " yaw=" << tf2::getYaw(original_goal.pose.orientation)
                      << " (restored after pile adjustment)");
    }
  }

  bool PlannerROS::makeOffsetStartTowardCenter(double back_dist, geometry_msgs::PoseStamped &out_start)
  {
    // helper, 把当前激活的位姿向铰接中心移�?
    const bool use_rear = (active_pose_source_ == PoseSource::Rear);
    const geometry_msgs::PoseStamped *source = nullptr;
    if (use_rear && has_rear_pose_)
    {
      source = &rear_pose_;
    }
    else if (!use_rear && has_front_pose_)
    {
      source = &front_pose_;
    }
    else if (!current_pose_.header.frame_id.empty())
    {
      source = &current_pose_;
    }

    if (!source)
    {
      ROS_WARN("makeOffsetStartTowardCenter: no valid pose available (active=%s)",
               use_rear ? "rear" : "front");
      return false;
    }

    out_start = *source;
    out_start.header.frame_id = "map";
    out_start.header.stamp = ros::Time::now();

    const double yaw = tf2::getYaw(source->pose.orientation);
    const double sign = use_rear ? 1.0 : -1.0; // 前车架后退、后车架前进
    out_start.pose.position.x = source->pose.position.x + sign * std::cos(yaw) * back_dist;
    out_start.pose.position.y = source->pose.position.y + sign * std::sin(yaw) * back_dist;
    return true;
  }

  geometry_msgs::PoseStamped PlannerROS::yawOffsetPose(const geometry_msgs::PoseStamped &pose,
                                                       double yaw_delta_rad)
  {
    geometry_msgs::PoseStamped out = pose;
    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double wrapped = std::atan2(std::sin(yaw + yaw_delta_rad), std::cos(yaw + yaw_delta_rad));
    tf2::Quaternion q;
    q.setRPY(0, 0, wrapped);
    out.pose.orientation = tf2::toMsg(q);
    return out;
  }

  bool PlannerROS::handleSetPlannerParams(hybrid_a_star::SetPlannerParams::Request &req,
                                          hybrid_a_star::SetPlannerParams::Response &res)
  {
    const bool has_update = req.set_max_direction_switches;
    if (!has_update)
    {
      res.success = false;
      res.message = "no fields set";
      return true;
    }

    if (sendPlannerParams(req))
    {
      res.success = true;
      res.message = "ok";
    }
    else
    {
      res.success = false;
      res.message = "forwarding to planner failed";
    }
    return true;
  }

  void PlannerROS::fillSegment(nav_msgs::Path &segment,
                               const nav_msgs::Path &path,
                               std::size_t start_idx,
                               std::size_t end_idx,
                               bool backward)
  {
    /* 仅被buildSegmentsFromPlan调用
    把全局规划�?nav_msgs::Path 中的一个区间（start_idx �?end_idx，含头尾）拷贝成
    单独的子路径，并按照前进/后退标志确认每个姿态的朝向，供后续调整车头方向、分段执行�?
    */
    if (path.poses.empty() || start_idx >= path.poses.size() || end_idx >= path.poses.size() || start_idx > end_idx)
    {
      return;
    }

    // 复制header, 预留空间，复制路径点
    segment.header = path.header;
    segment.header.stamp = ros::Time::now();
    segment.poses.reserve(end_idx - start_idx + 1);
    for (std::size_t i = start_idx; i <= end_idx; ++i)
    {
      segment.poses.push_back(path.poses[i]);
    }

    // lamda: 把任意弧度角规范�?[-π, π]
    const auto wrapYaw = [](double yaw) {
      return std::atan2(std::sin(yaw), std::cos(yaw));
    };

    // 不是后退，直接把这段返回
    if (!backward)
    {
      return;
    }

    // 后退，修改路径点航向180
    for (std::size_t i = 0; i < segment.poses.size(); ++i)
    {
      double yaw = tf2::getYaw(segment.poses[i].pose.orientation);
      yaw = wrapYaw(yaw + M_PI);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      segment.poses[i].pose.orientation = tf2::toMsg(q);
    }
  }

  bool PlannerROS::sendPlannerParams(hybrid_a_star::SetPlannerParams::Request &req)
  {
    if (!planner_backend_)
    {
      ROS_WARN("Remote planner backend not available; cannot send params.");
      return false;
    }
    std::string err;
    if (!planner_backend_->setPlannerParams(req, err))
    {
      ROS_ERROR("Failed to send planner params: %s", err.c_str());
      return false;
    }
    return true;
  }

  // 带自定义超时的版本（用于 fallback 场景，需要快速响应）
  bool PlannerROS::sendPlannerParams(hybrid_a_star::SetPlannerParams::Request &req,
                                     double timeout_sec)
  {
    if (!planner_backend_)
    {
      ROS_WARN("Remote planner backend not available; cannot send params.");
      return false;
    }
    std::string err;
    if (!planner_backend_->setPlannerParams(req, err, timeout_sec))
    {
      ROS_ERROR("Failed to send planner params (timeout=%.3fs): %s", timeout_sec, err.c_str());
      return false;
    }
    return true;
  }

  // ============================================================
  // setMaxDirectionSwitches: 设置远程规划器的最大换向次�?
  // ============================================================
  // 步骤说明�?
  // 1. 构�?Service 请求消息
  // 2. 设置标志位为 true（表示要修改这个参数�?
  // 3. 设置具体的�?
  // 4. 调用 sendPlannerParams 发送给远程规划器（使用100ms超时�?
  // ============================================================
  bool PlannerROS::setMaxDirectionSwitches(int value)
  {
    hybrid_a_star::SetPlannerParams::Request req;

    // set_max_direction_switches = true 表示"我要修改这个参数"
    // 如果�?false，远程规划器会忽�?max_direction_switches 的�?
    req.set_max_direction_switches = true;
    req.max_direction_switches = value;

    // 使用 100ms 超时，快速响�?
    // 0.1 �?= 100 毫秒
    constexpr double kSetParamsTimeoutSec = 0.1;
    const bool success = sendPlannerParams(req, kSetParamsTimeoutSec);

    if (success)
    {
      ROS_INFO("Successfully set max_direction_switches to %d", value);
    }
    else
    {
      ROS_WARN("Failed to set max_direction_switches to %d", value);
    }

    return success;
  }

  // ============================================================
  // tryPlanWithIncreasedSwitches: 使用增加的换向次数尝试规�?
  // ============================================================
  // 这个函数做什么？
  // 1. 把换向次数从 1 改成 2
  // 2. 尝试规划
  // 3. 无论成功失败，都把换向次数改�?1
  // ============================================================
  PlannerROS::PlanAttemptResult PlannerROS::tryPlanWithIncreasedSwitches(
      const geometry_msgs::PoseStamped &start,
      const geometry_msgs::PoseStamped &goal,
      int8_t task_type)
  {
    constexpr int kOriginalSwitches = 1;   // 原始�?
    constexpr int kIncreasedSwitches = 2;  // 增加后的�?

    // ------------------------------------------------------------
    // 步骤 1：尝试设置新的换向次�?
    // ------------------------------------------------------------
    if (!setMaxDirectionSwitches(kIncreasedSwitches))
    {
      // 设置失败，直接返回失败结�?
      PlanAttemptResult result;
      result.success = false;
      result.error_code = ErrorCode::PlanServiceUnavailable;
      result.message = "failed to set increased direction switches";
      result.tag = "fallback_increased_switches";
      return result;
    }

    // ------------------------------------------------------------
    // 步骤 2：创建参数恢复守�?
    // ------------------------------------------------------------
    // 关键点：这个对象在函数结束时会自动销�?
    // 销毁时会调用析构函数，从而恢�?max_direction_switches = 1
    //
    // 为什么不�?try-catch�?
    // - RAII 更简洁、更安全
    // - 即使后续代码有多�?return 点，也能保证恢复
    // ------------------------------------------------------------
    DirectionSwitchParamGuard guard(this, kOriginalSwitches);

    // ------------------------------------------------------------
    // 步骤 3：使用增加的换向次数尝试规划
    // ------------------------------------------------------------
    ROS_INFO("Attempting plan with max_direction_switches = %d", kIncreasedSwitches);

    PlanAttemptResult result = runPlanAttempt(start, goal, "fallback_increased_switches", task_type);

    // ------------------------------------------------------------
    // 步骤 4：函数返�?
    // ------------------------------------------------------------
    // 注意：这里不需要显式调�?setMaxDirectionSwitches(1)
    // 因为 guard 对象离开作用域时会自动恢�?
    //
    // 执行顺序�?
    // 1. return result; 准备返回
    // 2. guard 对象销毁，调用 ~DirectionSwitchParamGuard()
    // 3. 析构函数中调�?setMaxDirectionSwitches(1)
    // 4. 函数真正返回
    // ------------------------------------------------------------

    return result;

    // guard 在这里被销毁，自动恢复参数
  }

  bool PlannerROS::isBackwardFlag(int8_t flag)
  {
    // 规划算法的方向flag与is_forward标志不一致，使用这个函数转换
    // direction_flag == 0 表示前进，其余（�?）表示后退
    return flag != 0;
  }

  // 在进�?Moving 之前，先根据“当前路径段”和“车辆姿态反馈”判断这段应该前进还是后退
  // 把车调整到正确的坐标系上
  void PlannerROS::adjust_vehicle_direction()
  {
    if (nav_path_.empty())
    {
      ROS_WARN("adjust_vehicle_direction: nav_path_ is empty");
      setError(ErrorCode::NavPathUnavailable);
      return;
    }

    // 确保从规划服务器得到的标志位列表与分段列表长度严格一�?
    if (segment_backward_flags_.empty() || segment_backward_flags_.size() != nav_path_.size())
    {
      ROS_ERROR("Direction flags missing or misaligned with nav_path_ (flags=%zu, segments=%zu).",
                segment_backward_flags_.size(), nav_path_.size());
      setError(ErrorCode::MissingDirectionFlags);
      return;
    }

    // 0: 先把这段路径的终点发布出�?
    if (ros_io_)
    {
      ros_io_->publishNavPoint(nav_path_.front().poses.back());
    }


    // Compute desired direction and finalize immediately (no feedback wait).
    desired_forward_ = !segment_backward_flags_.front();  // 来自规划返回的方向标�?
    const PoseSource segment_source = desired_forward_ ? PoseSource::Front : PoseSource::Rear;
    setPoseSource(segment_source);
    updateCurrentPoseFromActiveSource();

    ROS_INFO("Requesting direction switch (forward=%d)", static_cast<int>(desired_forward_));
    publishIsForwardCommand(desired_forward_);
    finalizeDirectionSwitch();
    return;
  }

  // 发布方向变换命令
  void PlannerROS::publishIsForwardCommand(bool target_forward)
  {
    if (!ros_io_)
    {
      ROS_WARN_THROTTLE(2.0, "RosIo not available, unable to publish is_forward command.");
      return;
    }
    ros_io_->publishIsForward(target_forward);
    ROS_INFO_THROTTLE(1.0, "Publishing is_forward command: %s", target_forward ? "true" : "false");
  }

  void PlannerROS::onIsForwardState(const ::center_articulation_planner_legacy::IsForwardState &msg)
  {
    // 计划�?
    if (!waiting_direction_feedback_)
    {
      return;
    }

    // 返回标志位不匹配
    if (msg.forward != desired_forward_)
    {
      ROS_WARN_THROTTLE(1.0, "Ignoring is_forward_state with mismatched forward=%d (expected %d)",
                        static_cast<int>(msg.forward), static_cast<int>(desired_forward_));
      return;
    }

    // 时间不匹�?
    const double age = std::abs((ros::Time::now() - msg.header.stamp).toSec());
    if (age > direction_feedback_window_sec_)
    {
      ROS_WARN_THROTTLE(1.0, "Ignoring is_forward_state from %s due to staleness (age=%.2fs, window=%.2fs)",
                        msg.node_name.c_str(), age, direction_feedback_window_sec_);
      return;
    }

    node_success_[msg.node_name] = msg.success;

    // 切换不成�?
    if (!msg.success)
    {
      waiting_direction_feedback_ = false;
      is_switching_direction_ = false;
      ROS_ERROR("Direction switch rejected by node '%s'", msg.node_name.c_str());
      setError(ErrorCode::DirectionSwitchRejected, msg.node_name);
      return;
    }

    if (allRequiredNodesSucceeded())
    {
      finalizeDirectionSwitch();
    }
  }

  void PlannerROS::onActionGoal(const ::autonomous_loader_msgs::NavigateGoal &goal)
  {
    reset();
    current_goal_pose_ = goal.end_pose;
    last_status_ = goal.last_status;
    current_status_ = goal.current_status;

    // 新增：保�?id
    last_id_ = goal.last_id;
    current_id_ = goal.current_id;

    if (ros_io_)
    {
      ros_io_->publishIsUnload(false);
    }

    // Clear brake on new action; controller_started publishes on Moving entry.
    if (ros_io_)
    {
      ros_io_->publishBrake(kBrakeRelease);
      ROS_INFO("Published brake release for new action");
    }
    brake_engaged_ = false;

    const TaskType task = decodeTask(last_status_, current_status_);  // 先解码任�?

    const bool is_normal_shovel_or_unload = (task == TaskType::NormalShovel || task == TaskType::NormalUnload);  //标准卸料/铲料任务标志�?
    const bool last_was_unload = (last_status_ == 1);  // ��һ��������ж��
    use_offset_front_at_unload_start_ = (task == TaskType::NormalShovel);  // 标准卸料任务中，alternative起点标志�?
    use_offset_front_at_unload_start_ = false;

    const bool start_from_rear = is_normal_shovel_or_unload || (task == TaskType::MoveAround && last_was_unload);
    initial_pose_source_ = start_from_rear ? PoseSource::Rear : PoseSource::Front;  // �������� + ��һ����ж�Ͼ����滮���

    segment_index_ = 0;   // 重置段计�?
    togglePoseSource(segment_index_);  // 将起始位姿源按启发设�?

    ROS_INFO("Action goal stored (last_status=%u, current_status=%u, last_id=%u, current_id=%u, task=%s)",
             static_cast<unsigned>(last_status_),
             static_cast<unsigned>(current_status_),
             static_cast<unsigned>(last_id_),
             static_cast<unsigned>(current_id_),
             taskTypeToString(task));
    kickoffTaskFromAction(task);   // 进入状态机
  }

  void PlannerROS::onDistance(const std_msgs::Float64 &msg)
  {
    distance_to_goal_ = msg.data;
    has_distance_ = true;
    tryClearLastSegmentDistanceRefreshGate();
    tryClearLastLoadDistanceRefreshGate();
  }

  // 普通的goal_reached�?from goal checker
  // 注意：存储的分段路径�?的头元素只在 �?回调里被移除
  void PlannerROS::onReach(const std_msgs::Bool &reach_msg)
  {
    // 不在moving不处�?
    if (state_ != PlannerState::Moving)
    {
      return;
    }

    // 没有到达不处�?
    if (!reach_msg.data)
    {
      return; // TEMP: ignore /goal_reached false; remove after upstream fix lands
      setError(ErrorCode::GoalReachedRejected);
      return;
    }

    // 保险，所有路径跑完了还是进了这里会尝试弹空list（理论上不会进这里）
    if (nav_path_.empty())
    {
      ROS_WARN("onReach: nav_path_ is empty!!!");
      transitionTo(PlannerState::CheckingGoal, "no remaining path segments");
      return;
    }

    // 确认是否最后一�?
    const bool is_last_segment = nav_path_.size() == 1;
    if (is_last_segment && waiting_last_segment_distance_refresh_)
    {
      ROS_WARN_THROTTLE(0.5,
                        "Ignoring /goal_reached on last segment before /distance_to_goal refresh "
                        "(dist=%.3f, prev_tail=%.3f)",
                        distance_to_goal_,
                        prev_segment_tail_distance_);
      return;
    }

    // 去掉已经到达的路径段
    if (!nav_path_.empty())
    {
      nav_path_.erase(nav_path_.begin());  // 如果是最后一段，这里会成空的
    }
    if (!segment_backward_flags_.empty())
    {
      segment_backward_flags_.erase(segment_backward_flags_.begin());
    }
    if (!segment_lengths_.empty())
    {
      segment_lengths_.erase(segment_lengths_.begin());
    }
    ++segment_index_;
    goal_guard_active_ = false;
    goal_guard_logged_ = false;

    armLastUnloadDistanceRefreshGate("onReach entered last unload segment");
    armLastLoadDistanceRefreshGate("onReach entered last load segment");

    // 没有剩余段了，说明所有全局规划好的段都走完了
    if (is_last_segment && nav_path_.empty())
    {
      reach_goal_received_ = true;
      checking_goal_start_time_ = ros::Time::now();
      final_goal_ok_ = true;
      final_distance_error_ = 0.0;
      // 如果 final_goal 已先到，角度误差已由 onFinalGoalReached 填入，不覆盖；
      // 否则清零，等待 onFinalGoalReached 填入
      if (!final_goal_received_)
      {
        final_angle_error_ = 0.0;
      }
      final_error_code_ = 0;
      final_error_msg_.clear();
      if (ros_io_)
      {
        ros_io_->stopMovingTimer();
      }
      ROS_INFO("Last segment reached. Entering CheckingGoal...");
      // 如果final已经先到且成功，直接完成；否则进入等待
      if (final_goal_received_)
      {
        transitionToCompletedOrBackout("goal and final goal confirmed");
      }
      else
      {
        transitionTo(PlannerState::CheckingGoal, "last segment reached");
      }
      return;
    }

    // 没走完的逻辑
    // 下一段沿用“起始坐标系 + 奇偶段交替”的规则
    if (!segment_backward_flags_.empty())
    {
      const bool next_backward = segment_backward_flags_.front();
      setPoseSource(next_backward ? PoseSource::Rear : PoseSource::Front);
      updateCurrentPoseFromActiveSource();
    }
    else
    {
      ROS_WARN("onReach: direction flags missing for next segment, falling back to togglePoseSource");
      togglePoseSource(segment_index_);
    }
    // 调整方向进行跟踪
    // backout 上下文：seg1（后退）完成，seg2（重新进入）即将开始，
    // 重置 unload_flag_sent_ 使 onMovingTimer 能在接近终点时重发 is_unload=true
    if (backout_retry_count_ > 0)
    {
      unload_flag_sent_ = false;
      ROS_INFO("[BackoutUnload] seg1 done, reset unload_flag_sent_ for re-entry segment");
    }
    transitionTo(PlannerState::AdjustingDirection, "next segment ready");
  }

  // 终点微调给出的最终goal reached
  void PlannerROS::onFinalGoalReached(const geometry_msgs::PoseStamped &msg)
  {
    const bool in_checking = (state_ == PlannerState::CheckingGoal);
    const bool in_moving_last_segment = (state_ == PlannerState::Moving && nav_path_.size() == 1);
    if (!in_checking && !in_moving_last_segment)
    {
      // 仅在最后一段或检查阶段处理，其他阶段忽略
      return;
    }

    // 检查并忽略可能的过于老的信息
    const ros::Time stamp = msg.header.stamp.isZero() ? ros::Time::now() : msg.header.stamp;
    const double age = (ros::Time::now() - stamp).toSec();
    if (age > checking_goal_timeout_sec_)
    {
      ROS_WARN_THROTTLE(2.0,
                        "Ignoring final_goal_reached due to staleness (age=%.2fs > %.2fs)",
                        age,
                        checking_goal_timeout_sec_);
      return;
    }

    // 解码：x=dx, y=dy, z=dyaw, wx=success
    const double dx = msg.pose.position.x;
    const double dy = msg.pose.position.y;
    final_distance_error_ = std::hypot(dx, dy);
    final_angle_error_ = msg.pose.position.z;
    final_y_error_     = dy;
    const bool success = msg.pose.orientation.x > 0.5;
    final_goal_ok_ = success;
    final_goal_received_ = true;

    // IMPORTANT: 暂时屏蔽此处
    if (!success)
    {
      return;
      setError(ErrorCode::FinalGoalCheckFailed);
      return;
    }

    ROS_INFO("Final goal check: success=%d, dist_err=%.3f (x=%.3f, y=%.3f), angle_err=%.3f",
             static_cast<int>(success),
             final_distance_error_,
             dx,
             dy,
             final_angle_error_);
    
    // 临时：每次卸料踩刹车
    if (current_status_ == 1 && !brake_engaged_)
    {
      if (ros_io_)
      {
        ros_io_->publishBrake(kBrake30Percent);
        ROS_INFO("Published brake command: value=%u (%.1f%%) on unload final goal",
                 static_cast<unsigned>(kBrake30Percent),
                 100.0 * static_cast<double>(kBrake30Percent) / 255.0);
      }
      brake_engaged_ = true;
    }

    // TODO：确保不携带错误状态？
    final_error_code_ = 0;
    final_error_msg_.clear();

    // 同样的，如果进这里的时候已经有goal_reached，就进结束
    if (reach_goal_received_)
    {
      transitionToCompletedOrBackout("final goal confirmed");
      return;
    }

    // final先到，等待goal_reached
    if (in_moving_last_segment && !in_checking)
    {
      transitionTo(PlannerState::CheckingGoal, "final goal arrived, waiting for reach");
    }
  }

  // 把输入任务解码成enum成员
  TaskType PlannerROS::decodeTask(uint8_t last_status, uint8_t current_status) const
  {
    if (last_status == 2 && current_status == 0)
    {
      return TaskType::InitialShovel;
    }
    if (last_status == 1 && current_status == 0)
    {
      return TaskType::NormalShovel;
    }
    if (last_status == 2 && current_status == 1)
    {
      return TaskType::InitialUnload;
    }
    if (last_status == 0 && current_status == 1)
    {
      return TaskType::NormalUnload;
    }
    if ((last_status == 1 && current_status == 2) ||
        (last_status == 0 && current_status == 2) ||
        (last_status == 2 && current_status == 2))
    {
      return TaskType::MoveAround;
    }

    ROS_WARN("Unknown status combination (last_status=%u, current_status=%u), defaulting to Unknown.",
             static_cast<unsigned>(last_status),
             static_cast<unsigned>(current_status));
    return TaskType::Unknown;
  }

  // 从Action确定状态机的entry
  void PlannerROS::kickoffTaskFromAction(TaskType task)
  {
    // records the selected TaskType
    current_task_ = task;

    // ============================================================
    // 【新增】检查是否需要停车（current_id == 0�?
    // ============================================================
    if (current_id_ == 0)
    {
      ROS_INFO("current_id=0, entering parking mode");

      if (ros_io_)
      {
        // 1. controller_started=false (stop)
        ros_io_->publishControllerStarted(false);
        ROS_INFO("Published controller_started=false");

        // 2. 发布刹车 30%
        ros_io_->publishBrake(kBrake30Percent);
        ROS_INFO("Published brake command: value=%u (30%%)",
                 static_cast<unsigned>(kBrake30Percent));
      }
      brake_engaged_ = true;

      // 3. 设置成功的结果信息，直接进入 Completed 状�?
      final_goal_ok_ = true;
      final_distance_error_ = 0.0;
      final_angle_error_ = 0.0;
      final_error_code_ = 0;
      final_error_msg_.clear();

      transitionTo(PlannerState::Completed, "parking completed (current_id=0)");
      return;
    }

    // ============================================================
    // 原有逻辑：处理正常任�?
    // ============================================================

    // for unknown combos parks the planner in Idle
    if (task == TaskType::Unknown)
    {
      transitionTo(PlannerState::Idle, "unknown task from action");
      return;
    }

    // for known tasks resets buffers/timers and
    std::string reason = "action task ";
    reason += taskTypeToString(task);
    reason += " received";
    transitionTo(PlannerState::PlanningGlobalPath, reason);
  }

  // 清除所有的buffer，并确保 periodic onMovingTimer() no longer runs
  // 只清空路径缓存、停止定时器，不动地图，也不动混合A*的缓�?
  void PlannerROS::reset()
  {
    current_index_ = 0;
    segment_index_ = 0;
    active_pose_source_ = initial_pose_source_;
    nav_path_.clear();
    segment_backward_flags_.clear();
    segment_lengths_.clear();
    total_segments_ = 0;
    local_path_.poses.clear();
    global_path_.poses.clear();
    has_distance_ = false;
    waiting_direction_feedback_ = false;
    align_service_called_ = false;
    align_target_angle_deg_ = 0.0;
    align_check_done_ = false;
    remaining_replan_count_ = 0;
    node_success_.clear();
    is_switching_direction_ = false;
    waiting_cmd_time_ = ros::Time(0);
    reach_goal_received_ = false;
    final_goal_received_ = false;
    final_goal_ok_ = true;
    final_distance_error_ = 0.0;
    final_angle_error_ = 0.0;
    final_y_error_ = 0.0;
    final_error_code_ = 0;
    final_error_msg_.clear();
    checking_goal_start_time_ = ros::Time(0);
    use_offset_front_at_unload_start_ = false;
    brake_engaged_ = false;
    unload_flag_sent_ = false;
    waiting_last_segment_distance_refresh_ = false;
    waiting_load_last_segment_distance_refresh_ = false;
    prev_segment_tail_distance_ = 0.0;
    prev_load_segment_tail_distance_ = 0.0;
    goal_guard_active_ = false;
    goal_guard_logged_ = false;
    backout_retry_count_ = 0;
    // 重置时停止移动阶段定时器
    if (ros_io_)
    {
      ros_io_->stopMovingTimer();
    }
  }

  // costmap的回调，只接收并设置map一�?
  void PlannerROS::onMap(const nav_msgs::OccupancyGrid &map_msg)
  {
    (void)map_msg;
    ROS_INFO_THROTTLE(5.0, "Map callback received but remote planner manages map; ignoring.");
    if (ros_io_)
    {
      ros_io_->shutdownMapSubscription();
    }
  }

  bool PlannerROS::allRequiredNodesSucceeded() const
  {
    for (const auto *node : required_nodes_)
    {
      const auto it = node_success_.find(node);
      if (it == node_success_.end() || !it->second)
      {
        return false;
      }
    }
    return true;
  }

  bool PlannerROS::directionSwitchTimedOut() const
  {
    if (!waiting_direction_feedback_ || direction_switch_timeout_sec_ <= 0.0)
    {
      return false;
    }
    const double elapsed = (ros::Time::now() - direction_switch_start_time_).toSec();
    return elapsed > direction_switch_timeout_sec_;
  }

  double PlannerROS::computePathLength(const nav_msgs::Path &path) const
  {
    double total = 0.0;
    if (path.poses.size() < 2)
    {
      return total;
    }
    for (std::size_t i = 1; i < path.poses.size(); ++i)
    {
      const auto &p0 = path.poses[i - 1].pose.position;
      const auto &p1 = path.poses[i].pose.position;
      total += std::hypot(p1.x - p0.x, p1.y - p0.y);
    }
    return total;
  }

  double PlannerROS::remainingDistanceToGoal() const
  {
    if (nav_path_.empty() || segment_lengths_.size() != nav_path_.size())
    {
      return has_distance_ ? distance_to_goal_ : 0.0;
    }

    const double current_remaining = has_distance_ ? distance_to_goal_ : segment_lengths_.front();
    double future_remaining = 0.0;
    for (std::size_t i = 1; i < segment_lengths_.size(); ++i)
    {
      future_remaining += segment_lengths_[i];
    }
    return current_remaining + future_remaining;
  }

  double PlannerROS::distanceToCurrentSegmentGoal() const
  {
    if (nav_path_.empty() || nav_path_.front().poses.empty())
    {
      return std::numeric_limits<double>::infinity();
    }
    const auto &goal_pos = nav_path_.front().poses.back().pose.position;
    const auto &curr_pos = current_pose_.pose.position;
    return std::hypot(goal_pos.x - curr_pos.x, goal_pos.y - curr_pos.y);
  }

  bool PlannerROS::nearGoalUnreachable(double remaining_dist,
                                       double &forward_m,
                                       double &lateral_m,
                                       double &angle_deg,
                                       std::string &reason) const
  {
    const nav_msgs::Path *path_ptr = nullptr;
    if (!nav_path_.empty() && !nav_path_.front().poses.empty())
    {
      path_ptr = &nav_path_.front();
    }
    else if (!global_path_.poses.empty())
    {
      path_ptr = &global_path_;
    }

    if (!path_ptr)
    {
      reason = "guard: no path available";
      forward_m = 0.0;
      lateral_m = 0.0;
      angle_deg = 0.0;
      return false;
    }

    const geometry_msgs::PoseStamped &goal_pose = path_ptr->poses.back();

    geometry_msgs::Pose eval_pose = current_pose_.pose;
    double yaw = tf2::getYaw(eval_pose.orientation);
    if (active_pose_source_ == PoseSource::Rear)
    {
      yaw = std::atan2(std::sin(yaw + M_PI), std::cos(yaw + M_PI));
    }

    const double dx = goal_pose.pose.position.x - eval_pose.position.x;
    const double dy = goal_pose.pose.position.y - eval_pose.position.y;

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    forward_m = cos_yaw * dx + sin_yaw * dy;
    lateral_m = -sin_yaw * dx + cos_yaw * dy;

    const double angle_rad = std::atan2(lateral_m, forward_m);
    angle_deg = angle_rad * 180.0 / M_PI;

    const double distance_to_goal = std::hypot(dx, dy);
    const double effective_distance = std::min(distance_to_goal, remaining_dist);
    if (effective_distance > goal_guard_distance_m_)
    {
      reason = "outside guard radius";
      return false;
    }

    const double guard_angle_rad = goal_guard_angle_deg_ * M_PI / 180.0;
    const bool behind = forward_m < -goal_guard_forward_deadzone_m_;
    const bool out_of_wedge = std::abs(angle_rad) > guard_angle_rad;

    if (!behind && !out_of_wedge)
    {
      reason = "within guard wedge";
      return false;
    }

    if (behind && out_of_wedge)
    {
      reason = "goal behind and outside guard wedge";
    }
    else if (behind)
    {
      reason = "goal behind guard deadzone";
    }
    else
    {
      reason = "goal outside guard wedge";
    }
    return true;
  }

  PlannerROS::PoseSource PlannerROS::togglePoseSource(std::size_t segment_idx)
  {
    // 偶数段沿用起始坐标系，奇数段切换到相反的坐标�?
    const bool use_initial = (segment_idx % 2 == 0);
    const PoseSource flipped = (initial_pose_source_ == PoseSource::Front) ? PoseSource::Rear : PoseSource::Front;
    const PoseSource desired = use_initial ? initial_pose_source_ : flipped;
    setPoseSource(desired);
    updateCurrentPoseFromActiveSource();
    return desired;
  }

  void PlannerROS::setPoseSource(PoseSource source)
  {
    active_pose_source_ = source;
    ROS_INFO("Active pose source set to %s.", source == PoseSource::Rear ? "rear" : "front");
  }

  // 前后向改变完毕，到moving状态前的流�?
  void PlannerROS::finalizeDirectionSwitch()
  {
    waiting_direction_feedback_ = false;   // 禁用回调
    is_switching_direction_ = false;       // 初始化换向标志位
    node_success_.clear();                 // 清除记录的其他节点反�?
    is_forward_state_ = desired_forward_;  // 节点内部方向标志位同�?

    // 根据反馈方向同步坐标源
    setPoseSource(desired_forward_ ? PoseSource::Front : PoseSource::Rear);
    updateCurrentPoseFromActiveSource();

    // 进moving, 给杜威发controller_started车才会动
    if (direction_switch_settle_delay_sec_ > 0.0)
    {
      ros::Duration(direction_switch_settle_delay_sec_).sleep(); // IMPORTANT: delay before entering Moving to let goal consumers react
    }

    if (ros_io_)
    {
      if (!nav_path_.empty() && !nav_path_.front().poses.empty())
      {
        // 发一个终点给goal checker
        geometry_msgs::PoseStamped segment_end = nav_path_.front().poses.back();
        segment_end.header.stamp = ros::Time::now();
        ros_io_->publishNavPoint(segment_end);

        // 如果是最后一段，同步发一个终点给微调
        if (nav_path_.size() == 1)
        {
          geometry_msgs::PoseStamped final_goal = segment_end;
          if (!global_path_.poses.empty())
          {
            final_goal = global_path_.poses.back();
          }
          final_goal.header.stamp = ros::Time::now();
          ros_io_->publishPlannerGoal(final_goal);
        }

        // 【新增】提前计算并发布局部路径，以便生成way_point_shan_tui
        // 如果使用后车架坐标，clip 时将朝向翻转 180 度
        geometry_msgs::Pose pose_for_clip = current_pose_.pose;
        if (active_pose_source_ == PoseSource::Rear)
        {
          double yaw = tf2::getYaw(pose_for_clip.orientation);
          yaw = std::atan2(std::sin(yaw + M_PI), std::cos(yaw + M_PI));
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          pose_for_clip.orientation = tf2::toMsg(q);
        }

        nav_msgs::Path preview_local_path = clipPathByLookahead(nav_path_.front(), pose_for_clip);
        ros_io_->publishLocalPath(preview_local_path);

        // 等待way_point_shan_tui更新（给订阅者一些处理时间）
        ros::Duration(0.15).sleep();
        ROS_INFO("Published preview local path before align check (poses=%zu)", preview_local_path.poses.size());
      }

      if (direction_switch_settle_delay_sec_ > 0.0)
      {
        ros::Duration(direction_switch_settle_delay_sec_).sleep(); // IMPORTANT: delay before entering Moving to let goal consumers react
      }
    }

    // 进入 Moving 前基于距离/航向阈值触发原地对齐
    if (handlePreMoveDecision())
    {
      return;
    }

    if (ros_io_)
    {
      ros_io_->startMovingTimer();
    }
    transitionTo(PlannerState::Moving, "direction aligned by feedback");
  }

  bool PlannerROS::computePathTrackingError(const nav_msgs::Path &path,
                                            const geometry_msgs::PoseStamped &pose,
                                            PathTrackingError &tracking_error)
  {
    tracking_error = PathTrackingError{};
    if (path.poses.empty())
    {
      return false;
    }

    if (path.poses.size() == 1)
    {
      const auto &path_pose = path.poses.front().pose;
      const double yaw = tf2::getYaw(path_pose.orientation);
      const double vehicle_yaw = tf2::getYaw(pose.pose.orientation);
      const double dx = pose.pose.position.x - path_pose.position.x;
      const double dy = pose.pose.position.y - path_pose.position.y;
      tracking_error.valid = true;
      tracking_error.lateral_error_m = -std::sin(yaw) * dx + std::cos(yaw) * dy;
      tracking_error.longitudinal_error_m = std::cos(yaw) * dx + std::sin(yaw) * dy;
      tracking_error.heading_error_deg =
          std::atan2(std::sin(vehicle_yaw - yaw), std::cos(vehicle_yaw - yaw)) * 180.0 / M_PI;
      return true;
    }

    findNearestSegment(path, pose.pose.position, tracking_error.best_seg, tracking_error.best_t);
    if (tracking_error.best_seg + 1 >= path.poses.size())
    {
      return false;
    }

    const auto &p0 = path.poses[tracking_error.best_seg].pose.position;
    const auto &p1 = path.poses[tracking_error.best_seg + 1].pose.position;
    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    const double seg_len = std::hypot(vx, vy);
    const double seg_yaw = (seg_len > 1e-6) ? std::atan2(vy, vx)
                                            : tf2::getYaw(path.poses[tracking_error.best_seg].pose.orientation);
    const double proj_x = p0.x + tracking_error.best_t * vx;
    const double proj_y = p0.y + tracking_error.best_t * vy;
    const double dx = pose.pose.position.x - proj_x;
    const double dy = pose.pose.position.y - proj_y;
    const double vehicle_yaw = tf2::getYaw(pose.pose.orientation);

    tracking_error.valid = true;
    tracking_error.lateral_error_m = -std::sin(seg_yaw) * dx + std::cos(seg_yaw) * dy;
    tracking_error.longitudinal_error_m = std::cos(seg_yaw) * dx + std::sin(seg_yaw) * dy;
    tracking_error.heading_error_deg =
        std::atan2(std::sin(vehicle_yaw - seg_yaw), std::cos(vehicle_yaw - seg_yaw)) * 180.0 / M_PI;
    return true;
  }

  PlannerROS::PreMoveDecision PlannerROS::evaluatePreMoveDecision(PathTrackingError &tracking_error,
                                                                  double &align_position_error,
                                                                  double &align_position_thresh,
                                                                  std::string &align_error_type,
                                                                  double &heading_deg)
  {
    tracking_error = PathTrackingError{};
    align_position_error = 0.0;
    align_position_thresh = 0.0;
    align_error_type = "position error";
    heading_deg = 0.0;

    if (nav_path_.empty() || nav_path_.front().poses.empty())
    {
      return PreMoveDecision::Proceed;
    }

    if (segment_index_ > 0 && computePathTrackingError(nav_path_.front(), current_pose_, tracking_error))
    {
      ROS_WARN("Pre-move path tracking (segment=%zu): lateral=%.2f m, longitudinal=%.2f m, heading=%.1f deg, replan_thresh=%.2f m",
               segment_index_,
               tracking_error.lateral_error_m,
               tracking_error.longitudinal_error_m,
               tracking_error.heading_error_deg,
               large_lateral_replan_thresh_m_);

      if (remaining_replan_count_ < max_remaining_replan_count_ &&
          std::abs(tracking_error.lateral_error_m) >= large_lateral_replan_thresh_m_)
      {
        return PreMoveDecision::ReplanRemaining;
      }
    }

    if (!debug_mode_)
    {
      return PreMoveDecision::Proceed;
    }

    if (!has_shan_tui_waypoint_)
    {
      ROS_WARN_THROTTLE(2.0, "Skipping AligningInPlace: /way_point_shan_tui not received.");
      return PreMoveDecision::Proceed;
    }

    const auto &start_pos = nav_path_.front().poses.front().pose.position;
    const auto &curr_pos = current_pose_.pose.position;
    if (segment_index_ == 0)
    {
      const double dx = start_pos.x - curr_pos.x;
      const double dy = start_pos.y - curr_pos.y;
      align_position_error = std::hypot(dx, dy);
      align_position_thresh = align_pos_thresh_m_;
      align_error_type = "euclidean distance";
    }
    else if (tracking_error.valid)
    {
      align_position_error = std::abs(tracking_error.lateral_error_m);
      align_position_thresh = align_lateral_thresh_m_;
      align_error_type = "lateral error";
      ROS_INFO_THROTTLE(1.0,
                        "Segment %zu position decomposition: lateral=%.2f m, longitudinal=%.2f m, path_heading=%.1f deg",
                        segment_index_,
                        tracking_error.lateral_error_m,
                        tracking_error.longitudinal_error_m,
                        tracking_error.heading_error_deg);
    }
    else
    {
      const double dx_map = curr_pos.x - start_pos.x;
      const double dy_map = curr_pos.y - start_pos.y;
      const double yaw_start = tf2::getYaw(nav_path_.front().poses.front().pose.orientation);
      const double lateral_error = -dx_map * std::sin(yaw_start) + dy_map * std::cos(yaw_start);
      const double longitudinal_error = dx_map * std::cos(yaw_start) + dy_map * std::sin(yaw_start);
      align_position_error = std::abs(lateral_error);
      align_position_thresh = align_lateral_thresh_m_;
      align_error_type = "lateral error";
      ROS_INFO_THROTTLE(1.0,
                        "Segment %zu position decomposition: lateral=%.2f m, longitudinal=%.2f m (start yaw=%.1f deg)",
                        segment_index_,
                        lateral_error,
                        longitudinal_error,
                        yaw_start * 180.0 / M_PI);
    }

    const double raw_heading = tf2::getYaw(shan_tui_waypoint_.pose.orientation);
    const double heading = std::atan2(std::sin(raw_heading), std::cos(raw_heading));
    heading_deg = heading * 180.0 / M_PI;
    const double heading_abs_deg = std::abs(heading_deg);

    ROS_WARN("Check if AligningInPlace is needed (segment=%zu): %s=%.2f m (thresh=%.2f m), heading=%.1f deg (abs=%.1f deg, thresh=%.1f deg)",
             segment_index_,
             align_error_type.c_str(),
             align_position_error,
             align_position_thresh,
             heading_deg,
             heading_abs_deg,
             align_heading_thresh_deg_);

    const bool position_trigger = (align_position_error > align_position_thresh);
    const bool heading_trigger = (heading_abs_deg > align_heading_thresh_deg_);
    return (position_trigger || heading_trigger) ? PreMoveDecision::Align
                                                 : PreMoveDecision::Proceed;
  }

  bool PlannerROS::replanRemainingPath(const PathTrackingError &tracking_error)
  {
    if (segment_index_ == 0)
    {
      return false;
    }

    if (remaining_replan_count_ >= max_remaining_replan_count_)
    {
      ROS_WARN("Skipping remaining-path replan: replan count %d reached limit %d.",
               remaining_replan_count_,
               max_remaining_replan_count_);
      return false;
    }

    updateCurrentPoseFromActiveSource();
    geometry_msgs::PoseStamped plan_start = current_pose_;
    plan_start.header.stamp = ros::Time::now();
    bool goal_adjusted_for_pile = false;
    geometry_msgs::PoseStamped plan_goal = buildPlanGoal(goal_adjusted_for_pile);

    ROS_WARN("Replanning remaining path on segment %zu due to large lateral deviation: lateral=%.2f m, longitudinal=%.2f m, heading=%.1f deg",
             segment_index_,
             tracking_error.lateral_error_m,
             tracking_error.longitudinal_error_m,
             tracking_error.heading_error_deg);

    if (ros_io_)
    {
      geometry_msgs::PoseStamped service_start = plan_start;
      service_start.header.stamp = ros::Time::now();
      ros_io_->publishPlanServiceStart(service_start);

      geometry_msgs::PoseStamped service_goal = plan_goal;
      service_goal.header.stamp = ros::Time::now();
      ros_io_->publishPlanServiceGoal(service_goal);
    }

    PlanAttemptResult attempt = runPlanAttempt(plan_start,
                                               plan_goal,
                                               "remaining_replan",
                                               remotePlannerTaskType());
    if (!attempt.success)
    {
      const std::string detail = "remaining path replan failed: " + attempt.message;
      ROS_ERROR("%s", detail.c_str());
      setError(attempt.error_code, detail);
      return true;
    }

    const double planning_time_sec = attempt.planning_time_sec;
    applyPlanAttempt(std::move(attempt), goal_adjusted_for_pile, true);
    ++remaining_replan_count_;

    if (planning_time_sec > 0.0)
    {
      ROS_INFO_STREAM("Remaining-path replan succeeded in " << planning_time_sec << "s");
    }
    armLastUnloadDistanceRefreshGate("remaining path replanned");
    armLastLoadDistanceRefreshGate("remaining path replanned");
    transitionTo(PlannerState::AdjustingDirection, "remaining path replanned after large lateral deviation");
    return true;
  }

  bool PlannerROS::handlePreMoveDecision()
  {
    PathTrackingError tracking_error;
    double align_position_error = 0.0;
    double align_position_thresh = 0.0;
    std::string align_error_type;
    double heading_deg = 0.0;

    const PreMoveDecision decision =
        evaluatePreMoveDecision(tracking_error,
                                align_position_error,
                                align_position_thresh,
                                align_error_type,
                                heading_deg);

    if (decision == PreMoveDecision::Proceed)
    {
      return false;
    }

    if (decision == PreMoveDecision::ReplanRemaining)
    {
      return replanRemainingPath(tracking_error);
    }

    const bool position_trigger = (align_position_error > align_position_thresh);
    const bool heading_trigger = (std::abs(heading_deg) > align_heading_thresh_deg_);
    const char *trigger_reason = "position or heading deviation before moving";
    if (position_trigger && heading_trigger)
    {
      trigger_reason = "position and heading deviation before moving";
    }
    else if (position_trigger)
    {
      trigger_reason = "position deviation before moving";
    }
    else
    {
      trigger_reason = "heading deviation before moving";
    }

    ROS_WARN("Entering AligningInPlace (segment=%zu, position_trigger=%d, heading_trigger=%d, %s=%.2f m, pos_thresh=%.2f m, heading=%.1f deg, heading_thresh=%.1f deg)",
             segment_index_,
             static_cast<int>(position_trigger),
             static_cast<int>(heading_trigger),
             align_error_type.c_str(),
             align_position_error,
             align_position_thresh,
             heading_deg,
             align_heading_thresh_deg_);

    align_service_called_ = false;
    align_target_angle_deg_ = static_cast<float>(heading_deg);
    transitionTo(PlannerState::AligningInPlace, trigger_reason);
    return true;
  }

  bool PlannerROS::maybeEnterAligningInPlace()
  {
    // 总开关：非 debug 模式直接跳过
    if (!debug_mode_)
    {
      return false;
    }

    // 需要有待执行的段和 lookahead waypoint
    if (nav_path_.empty() || nav_path_.front().poses.empty())
    {
      return false;
    }
    if (!has_shan_tui_waypoint_)
    {
      ROS_WARN_THROTTLE(2.0, "Skipping AligningInPlace: /way_point_shan_tui not received.");
      return false;
    }

    // 获取段起点位置和朝向
    const auto &start_pos = nav_path_.front().poses.front().pose.position;
    const auto &curr_pos = current_pose_.pose.position;

    double position_error = 0.0;
    double position_thresh = 0.0;
    std::string error_type;

    if (segment_index_ == 0)
    {
      // 第一个segment：使用欧氏距离（原有逻辑）
      const double dx = start_pos.x - curr_pos.x;
      const double dy = start_pos.y - curr_pos.y;
      position_error = std::hypot(dx, dy);
      position_thresh = align_pos_thresh_m_;
      error_type = "euclidean distance";
    }
    else
    {
      // 后续segment：使用横向误差（基于段起点切线坐标系）
      const double dx_map = curr_pos.x - start_pos.x;
      const double dy_map = curr_pos.y - start_pos.y;

      // 提取段起点的朝向角（全局坐标系下）
      const double yaw_start = tf2::getYaw(nav_path_.front().poses.front().pose.orientation);

      // 计算Frenet坐标系下的横向偏差（lateral error）
      // lateral = -dx * sin(yaw) + dy * cos(yaw)
      const double lateral_error = -dx_map * std::sin(yaw_start) + dy_map * std::cos(yaw_start);

      // 可选：也计算纵向误差用于日志
      const double longitudinal_error = dx_map * std::cos(yaw_start) + dy_map * std::sin(yaw_start);

      position_error = std::abs(lateral_error);
      position_thresh = align_lateral_thresh_m_;
      error_type = "lateral error";

      // 详细日志：包含横向和纵向误差
      ROS_INFO_THROTTLE(1.0,
                        "Segment %zu position decomposition: lateral=%.2f m, longitudinal=%.2f m (start yaw=%.1f deg)",
                        segment_index_,
                        lateral_error,
                        longitudinal_error,
                        yaw_start * 180.0 / M_PI);
    }

    // 航向判定：使用 /way_point_shan_tui 的航向
    const double raw_heading = tf2::getYaw(shan_tui_waypoint_.pose.orientation);
    const double heading = std::atan2(std::sin(raw_heading), std::cos(raw_heading));
    const double heading_deg = heading * 180.0 / M_PI;
    const double heading_abs_deg = std::abs(heading_deg);

    ROS_WARN("Check if AligningInPlace is needed (segment=%zu): %s=%.2f m (thresh=%.2f m), heading=%.1f deg (abs=%.1f deg, thresh=%.1f deg)",
             segment_index_,
             error_type.c_str(),
             position_error,
             position_thresh,
             heading_deg,
             heading_abs_deg,
             align_heading_thresh_deg_);

    const bool position_trigger = (position_error > position_thresh);
    const bool heading_trigger = (heading_abs_deg > align_heading_thresh_deg_);
    if (!position_trigger && !heading_trigger)
    {
      return false;
    }

    const char *trigger_reason = "position or heading deviation before moving";
    if (position_trigger && heading_trigger)
    {
      trigger_reason = "position and heading deviation before moving";
    }
    else if (position_trigger)
    {
      trigger_reason = "position deviation before moving";
    }
    else
    {
      trigger_reason = "heading deviation before moving";
    }

    ROS_WARN("Entering AligningInPlace (segment=%zu, position_trigger=%d, heading_trigger=%d, %s=%.2f m, pos_thresh=%.2f m, heading=%.1f deg, heading_thresh=%.1f deg)",
             segment_index_,
             static_cast<int>(position_trigger),
             static_cast<int>(heading_trigger),
             error_type.c_str(),
             position_error,
             position_thresh,
             heading_deg,
             align_heading_thresh_deg_);

    align_service_called_ = false;
    align_target_angle_deg_ = static_cast<float>(heading_deg);
    transitionTo(PlannerState::AligningInPlace, trigger_reason);
    return true;
  }

  // ============================================================
  // BackingOutUnload：退出-重入流程
  // ============================================================

  void PlannerROS::transitionToCompletedOrBackout(const std::string &reason)
  {
    if (maybeEnterBackoutUnload()) return;
    transitionTo(PlannerState::Completed, reason);
  }

  bool PlannerROS::maybeEnterBackoutUnload()
  {
    const bool is_unload = (current_task_ == TaskType::NormalUnload ||
                            current_task_ == TaskType::InitialUnload);
    if (!is_unload) return false;

    const bool yaw_trigger     = std::abs(final_angle_error_) > backout_yaw_threshold_rad_;
    const bool lateral_trigger = (std::abs(final_y_error_)    > backout_lateral_threshold_m_ &&
                                   std::abs(final_angle_error_) > backout_lateral_yaw_threshold_rad_);

    if (!yaw_trigger && !lateral_trigger) return false;

    if (backout_retry_count_ >= kMaxBackoutRetries)
    {
      ROS_ERROR("[BackoutUnload] max retries (%d) exceeded, yaw_err=%.3f rad — proceeding to Completed",
                kMaxBackoutRetries, final_angle_error_);
      return false;
    }
    ++backout_retry_count_;

    if (yaw_trigger)
      ROS_ERROR("[BackoutUnload] YAW trigger: yaw_err=%.3f > %.3f rad (attempt %d/%d)",
               final_angle_error_, backout_yaw_threshold_rad_,
               backout_retry_count_, kMaxBackoutRetries);
    else
      ROS_ERROR("[BackoutUnload] LATERAL trigger: y_err=%.3f > %.3f m, yaw_err=%.3f > %.3f rad (attempt %d/%d)",
               final_y_error_, backout_lateral_threshold_m_,
               final_angle_error_, backout_lateral_yaw_threshold_rad_,
               backout_retry_count_, kMaxBackoutRetries);

    transitionTo(PlannerState::BackingOutUnload, "yaw error too large at unload goal");
    return true;
  }

  bool PlannerROS::buildBackoutPaths()
  {
    const double goal_x   = current_goal_pose_.pose.position.x;
    const double goal_y   = current_goal_pose_.pose.position.y;
    const std::string fid = current_goal_pose_.header.frame_id;
    const double exit_x   = goal_x + backout_distance_m_;

    // 局部重置：清路径缓存，保留 task / goal 上下文
    nav_path_.clear();
    segment_backward_flags_.clear();
    segment_lengths_.clear();
    global_path_.poses.clear();       // 清除，让 finalizeDirectionSwitch 用 segment_end 作为 planner goal
    total_segments_  = 0;
    segment_index_   = 0;
    reach_goal_received_   = false;
    final_goal_received_   = false;
    final_goal_ok_         = true;
    final_distance_error_  = 0.0;
    final_angle_error_     = 0.0;
    unload_flag_sent_      = true;    // 禁止 backout 期间重发 is_unload=true
    if (ros_io_)
    {
      ros_io_->publishIsUnload(false);
      ROS_INFO("[BackoutUnload] Published is_unload=false (seg1 start)");
    }
    waiting_last_segment_distance_refresh_ = false;
    waiting_load_last_segment_distance_refresh_ = false;
    goal_guard_active_     = false;
    goal_guard_logged_     = false;

    const int n = std::max(2, static_cast<int>(backout_distance_m_ / backout_step_m_) + 1);

    // --- 段 1：后退（后车架为头，yaw=0，路径点从 goal_x 到 exit_x）---
    nav_msgs::Path seg1;
    seg1.header.frame_id = fid;
    seg1.header.stamp    = ros::Time::now();
    for (int i = 0; i < n; ++i)
    {
      const double x = goal_x + backout_distance_m_ * static_cast<double>(i) / (n - 1);
      geometry_msgs::PoseStamped ps;
      ps.header = seg1.header;
      ps.pose.position.x = x;
      ps.pose.position.y = goal_y;
      ps.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, 0.0);           // yaw = 0（x 轴正向）
      ps.pose.orientation = tf2::toMsg(q);
      seg1.poses.push_back(ps);
    }

    // --- 段 2：前进回去（前车架为头，yaw=π，路径点从 exit_x 回到 goal_x）---
    nav_msgs::Path seg2;
    seg2.header = seg1.header;
    seg2.header.stamp = ros::Time::now();
    for (int i = n - 1; i >= 0; --i)
    {
      const double x = goal_x + backout_distance_m_ * static_cast<double>(i) / (n - 1);
      geometry_msgs::PoseStamped ps;
      ps.header = seg2.header;
      ps.pose.position.x = x;
      ps.pose.position.y = goal_y;
      ps.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, M_PI);          // yaw = π（x 轴负向）
      ps.pose.orientation = tf2::toMsg(q);
      seg2.poses.push_back(ps);
    }

    nav_path_               = {seg1, seg2};
    segment_backward_flags_ = {true, false};   // seg1 后退，seg2 前进
    segment_lengths_        = {computePathLength(seg1), computePathLength(seg2)};
    total_segments_         = 2;

    desired_forward_ = false;
    setPoseSource(PoseSource::Rear);
    updateCurrentPoseFromActiveSource();

    ROS_INFO("[BackoutUnload] seg1=%zu pts (backward, yaw=0), seg2=%zu pts (forward, yaw=pi)"
             ", goal=(%.2f,%.2f), exit_x=%.2f, attempt=%d/%d",
             seg1.poses.size(), seg2.poses.size(),
             goal_x, goal_y, exit_x,
             backout_retry_count_, kMaxBackoutRetries);
    return true;
  }

} // namespace center_articulation_planner
