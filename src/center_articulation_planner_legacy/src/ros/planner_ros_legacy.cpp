#include "center_articulation_planner/ros/planner_ros.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>

// Helper inventory (legacy planning utilities retained for state machine):
// - estimateSegmentForward: checks if current pose aligns with the queued path segment direction.
// - clipPathByLookahead: trims the nav path using lookahead distance and optional Hermite merge.
// - findNearestSegment: projects the vehicle pose onto the closest path segment.
// - findLookaheadTarget: advances along the path to locate the target pose and heading.
// - generateHermiteMerge: produces a Hermite spline between the current pose and lookahead goal.
// - extractPathSegment: copies the remaining path points while respecting a distance budget.

namespace jhzx::center_articulation_planner
{
  void PlannerROS::onGoal(const geometry_msgs::PoseStamped &goal_msg)
  {
    ROS_WARN("LEGACY: onGoal(test only) triggered, goal should be input by action.");
    ROS_INFO("Get goal from TOPIC, try planning from current position...");
    initial_pose_source_ = PoseSource::Front;
    segment_index_ = 0;
    reset();
    togglePoseSource(segment_index_);
    current_goal_pose_ = goal_msg;
    // 有目标点了，切换到规划状态
    transitionTo(PlannerState::PlanningGlobalPath, "goal received");
  }

  // TODO: 未来改为订阅聚合后的 /is_forward_state/<node> 反馈，而不是命令/状态同一个话题
  void PlannerROS::onIsForward(const std_msgs::Bool &msg)
  {
    is_forward_state_ = msg.data;
  }

  // TODO：落地“命令广播 + 分节点状态反馈”设计（/is_forward_cmd + /is_forward_state/<node>），替代单话题切换
  void PlannerROS::onFrontPose(const geometry_msgs::PoseStamped &pose_msg)
  {
    // 更新前车架位姿
    front_pose_ = pose_msg;
    front_pose_.header.frame_id = "map";
    has_front_pose_ = true;

    updateCurrentPoseFromActiveSource();
  }

  void PlannerROS::onRearPose(const geometry_msgs::PoseStamped &pose_msg)
  {
    // 更新后车架位姿
    rear_pose_ = pose_msg;
    rear_pose_.header.frame_id = "map";
    has_rear_pose_ = true;

    updateCurrentPoseFromActiveSource();
  }

  void PlannerROS::onShanTuiWaypoint(const geometry_msgs::PoseStamped &pose_msg)
  {
    shan_tui_waypoint_ = pose_msg;
    shan_tui_waypoint_.header.frame_id = "map";
    has_shan_tui_waypoint_ = true;
  }

  // 按照当前激活的前/后车架，更新规划起点位姿
  void PlannerROS::updateCurrentPoseFromActiveSource()
  {
    const bool use_rear = (active_pose_source_ == PoseSource::Rear);
    const geometry_msgs::PoseStamped *selected = nullptr;  // 位姿指针
    if (use_rear && has_rear_pose_)  // 指针切换选择位姿源
    {
      selected = &rear_pose_;
    }
    else if (!use_rear && has_front_pose_)
    {
      selected = &front_pose_;
    }

    if (!selected)
    {
      ROS_WARN_THROTTLE(2.0, "Active pose source (%s) not available yet.",
                        use_rear ? "rear" : "front");
      return;
    }

    current_pose_ = *selected;  // 更新位姿成员

    // 保险判断
    if (is_switching_direction_)
    {
      double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
      double diff = std::abs(current_yaw - last_switch_yaw_);
      while (diff > M_PI) diff -= 2.0 * M_PI;
      diff = std::abs(diff);

      // 如果角度变化超过 90 度 ，说明坐标系已经翻转
      if (diff > 1.56) 
      {
        is_switching_direction_ = false;
        ROS_INFO("Pose flip detected (diff: %.2f). Direction switch confirmed.", diff);
      }
    }
  }

  bool PlannerROS::makeOffsetStartFromFront(double back_dist, geometry_msgs::PoseStamped &out_start)
  {
    if (!has_front_pose_)
    {
      transitionTo(PlannerState::Error, "no front pose");
      return false;
    }

    // 不管输入的out_start是什么，都直接把前车架后方back_dist处坐标传出
    out_start = front_pose_;
    out_start.header.frame_id = "map";
    out_start.header.stamp = ros::Time::now();

    const double yaw = tf2::getYaw(front_pose_.pose.orientation);
    out_start.pose.position.x = front_pose_.pose.position.x - std::cos(yaw) * back_dist;
    out_start.pose.position.y = front_pose_.pose.position.y - std::sin(yaw) * back_dist;
    out_start.pose.orientation = front_pose_.pose.orientation;
    return true;
  }

  // TODO: 之前的测试应该是走的这个接口？
  void PlannerROS::onExternalGlobalPath(const nav_msgs::Path &path_msg)
  {    
    nav_path_.clear();
    nav_path_.push_back(path_msg);
    segment_lengths_.clear();
    segment_lengths_.push_back(computePathLength(path_msg));
    initial_pose_source_ = PoseSource::Front;  // 默认使用前车架
    segment_index_ = 0;
    togglePoseSource(segment_index_);
    transitionTo(PlannerState::Moving, "external global path received"); // 直接进入 Moving 状态
    ROS_INFO_THROTTLE(2.0, "Received external /global_path .");
  }

  // 基于路径yaw与姿态角的相对关系估计该段需不需要换向行驶
  bool PlannerROS::estimateSegmentForward()
  {
    if (nav_path_.empty())
    {
      ROS_WARN("estimateSegmentForward: nav_path_ is empty");
      return false;
    }

    const auto &path = nav_path_.front();
    if (path.poses.empty())
    {
      ROS_WARN("estimateSegmentForward: current path segment is empty");
      return false;
    }

    double path_yaw = tf2::getYaw(path.poses.front().pose.orientation);
    double vehicle_yaw = tf2::getYaw(current_pose_.pose.orientation);

    double diff = path_yaw - vehicle_yaw;
    while (diff > M_PI)
      diff -= 2.0 * M_PI;
    while (diff < -M_PI)
      diff += 2.0 * M_PI;

    bool is_aligned = (std::abs(diff) < M_PI_2);
    return (is_forward_state_ == is_aligned);
  }

  nav_msgs::Path PlannerROS::clipPathByLookahead(const nav_msgs::Path &path,
                                                 const geometry_msgs::Pose &curr,
                                                 double x_meter,
                                                 double lateral_offset_threshold,
                                                 double merge_distance_scale,
                                                 double merge_min_distance,
                                                 int merge_curve_points,
                                                 double merge_forward_distance,
                                                 double yaw_trigger_deg)
  {
    nav_msgs::Path out;
    out.header.frame_id = path.header.frame_id;
    out.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped curr_ps;
    curr_ps.header = out.header;
    curr_ps.pose = curr;
    out.poses.push_back(curr_ps);

    if (path.poses.empty() || x_meter <= 0.0)
    {
      return out;
    }

    size_t best_seg = 0;
    double best_t = 0.0;
    findNearestSegment(path, curr.position, best_seg, best_t);

    size_t target_seg = best_seg;
    double target_t = best_t;
    double fwd_x = 0.0, fwd_y = 0.0, fwd_yaw = 0.0;
    findLookaheadTarget(path, best_seg, best_t, merge_forward_distance, target_seg, target_t, fwd_x, fwd_y, fwd_yaw);

    double merge_added_len = 0.0;
    if (merge_curve_points > 1)
    {
      merge_added_len = generateHermiteMerge(curr, fwd_x, fwd_y, fwd_yaw, merge_distance_scale, merge_min_distance, merge_curve_points, out);
    }
    else if (std::hypot(fwd_x - curr.position.x, fwd_y - curr.position.y) > 1e-6)
    {
      geometry_msgs::PoseStamped fwd_ps;
      fwd_ps.header = out.header;
      fwd_ps.pose.position.x = fwd_x;
      fwd_ps.pose.position.y = fwd_y;
      fwd_ps.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, fwd_yaw);
      fwd_ps.pose.orientation = tf2::toMsg(q);

      const geometry_msgs::PoseStamped &prev = out.poses.back();
      merge_added_len += std::hypot(fwd_ps.pose.position.x - prev.pose.position.x,
                                    fwd_ps.pose.position.y - prev.pose.position.y);
      out.poses.push_back(fwd_ps);
    }

    double budget = std::max(0.0, x_meter - merge_added_len);
    extractPathSegment(path, target_seg, target_t, budget, out);

    return out;
  }

  void PlannerROS::findNearestSegment(const nav_msgs::Path &path, const geometry_msgs::Point &curr_pos, size_t &best_seg, double &best_t)
  {
    best_seg = 0;
    best_t = 0.0;
    double best_d2 = std::numeric_limits<double>::infinity();
    const size_t N = path.poses.size();

    if (N == 1)
    {
      return;
    }

    const double cx = curr_pos.x;
    const double cy = curr_pos.y;

    for (size_t i = 0; i + 1 < N; ++i)
    {
      const auto &p0 = path.poses[i].pose.position;
      const auto &p1 = path.poses[i + 1].pose.position;
      const double vx = p1.x - p0.x;
      const double vy = p1.y - p0.y;
      const double wx = cx - p0.x;
      const double wy = cy - p0.y;
      const double vv = vx * vx + vy * vy;
      if (vv < 1e-12)
        continue;
      double t = (wx * vx + wy * vy) / vv;
      t = std::max(0.0, std::min(1.0, t));
      const double proj_x = p0.x + t * vx;
      const double proj_y = p0.y + t * vy;
      const double dx = cx - proj_x;
      const double dy = cy - proj_y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2)
      {
        best_d2 = d2;
        best_seg = i;
        best_t = t;
      }
    }
  }

  void PlannerROS::findLookaheadTarget(const nav_msgs::Path &path, size_t start_seg, double start_t, double forward_dist,
                                       size_t &target_seg, double &target_t, double &target_x, double &target_y, double &target_yaw)
  {
    target_seg = start_seg;
    target_t = start_t;
    const size_t N = path.poses.size();
    double advance = std::max(0.0, forward_dist);

    size_t si = start_seg;
    double st = start_t;

    while (si + 1 < N)
    {
      const auto &pp0 = path.poses[si].pose.position;
      const auto &pp1 = path.poses[si + 1].pose.position;
      const double sdx = pp1.x - pp0.x;
      const double sdy = pp1.y - pp0.y;
      const double slen = std::hypot(sdx, sdy);
      if (slen < 1e-9)
      {
        ++si;
        st = 0.0;
        continue;
      }
      const double remain = (1.0 - st) * slen;
      if (advance <= remain + 1e-12)
      {
        const double dt = (slen > 0.0) ? (advance / slen) : 0.0;
        target_seg = si;
        target_t = st + dt;
        target_x = pp0.x + target_t * sdx;
        target_y = pp0.y + target_t * sdy;
        target_yaw = std::atan2(sdy, sdx);
        return;
      }
      advance -= remain;
      ++si;
      st = 0.0;
    }

    if (advance > 1e-9 || si + 1 >= N)
    {
      target_seg = (N >= 2) ? (N - 2) : 0;
      target_t = 1.0;
      const auto &lp0 = path.poses[target_seg].pose.position;
      const auto &lp1 = path.poses[target_seg + 1].pose.position;
      target_x = lp1.x;
      target_y = lp1.y;
      target_yaw = std::atan2(lp1.y - lp0.y, lp1.x - lp0.x);
    }
  }

  double PlannerROS::generateHermiteMerge(const geometry_msgs::Pose &curr, double target_x, double target_y, double target_yaw,
                                          double scale, double min_dist, int points, nav_msgs::Path &out_path)
  {
    double added_len = 0.0;
    const double cx = curr.position.x;
    const double cy = curr.position.y;
    const double start_yaw = tf2::getYaw(curr.orientation);

    const double dist_to_target = std::hypot(target_x - cx, target_y - cy);
    const double L = dist_to_target * std::max(scale, 0.8);

    const double sx = cx, sy = cy;
    const double ex = target_x, ey = target_y;
    const double end_yaw = target_yaw;

    const double tx0 = std::cos(start_yaw) * L;
    const double ty0 = std::sin(start_yaw) * L;
    const double tx1 = std::cos(end_yaw) * L;
    const double ty1 = std::sin(end_yaw) * L;

    auto hermite = [&](double t) {
      const double h00 = 2 * t * t * t - 3 * t * t + 1;
      const double h10 = t * t * t - 2 * t * t + t;
      const double h01 = -2 * t * t * t + 3 * t * t;
      const double h11 = t * t * t - t * t;
      geometry_msgs::PoseStamped ps;
      ps.header = out_path.header;
      ps.pose.position.x = h00 * sx + h10 * tx0 + h01 * ex + h11 * tx1;
      ps.pose.position.y = h00 * sy + h10 * ty0 + h01 * ey + h11 * ty1;
      ps.pose.position.z = 0.0;
      return ps;
    };

    for (int k = 1; k <= points; ++k)
    {
      const double t = static_cast<double>(k) / points;
      geometry_msgs::PoseStamped ps = hermite(t);

      double yaw = end_yaw;
      if (!out_path.poses.empty())
      {
        const auto &prev = out_path.poses.back();
        const double dx = ps.pose.position.x - prev.pose.position.x;
        const double dy = ps.pose.position.y - prev.pose.position.y;
        const double d = std::hypot(dx, dy);
        if (d > 1e-6)
        {
          yaw = std::atan2(dy, dx);
          added_len += d;
        }
      }
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      ps.pose.orientation = tf2::toMsg(q);
      out_path.poses.push_back(ps);
    }
    return added_len;
  }

  void PlannerROS::extractPathSegment(const nav_msgs::Path &path, size_t start_seg, double start_t, double length_budget, nav_msgs::Path &out_path)
  {
    double accum = 0.0;
    size_t i = start_seg;
    double t0 = start_t;
    const size_t N = path.poses.size();

    while (i + 1 < N)
    {
      const auto &p0 = path.poses[i].pose.position;
      const auto &p1 = path.poses[i + 1].pose.position;
      const double seg_dx = p1.x - p0.x;
      const double seg_dy = p1.y - p0.y;
      const double seg_len = std::hypot(seg_dx, seg_dy);

      if (seg_len < 1e-9)
      {
        ++i;
        t0 = 0.0;
        continue;
      }

      const double seg_remain = (1.0 - t0) * seg_len;
      if (accum + seg_remain < length_budget - 1e-9)
      {
        geometry_msgs::PoseStamped next_ps = path.poses[i + 1];
        next_ps.header = out_path.header;
        out_path.poses.push_back(next_ps);
        accum += seg_remain;
        ++i;
        t0 = 0.0;
      }
      else
      {
        const double remain = std::max(0.0, length_budget - accum);
        const double dt = std::max(0.0, std::min(1.0, remain / seg_len));
        const double t = t0 + dt;

        geometry_msgs::PoseStamped inter;
        inter.header = out_path.header;
        inter.pose.position.x = p0.x + t * seg_dx;
        inter.pose.position.y = p0.y + t * seg_dy;
        inter.pose.position.z = 0.0;

        const double yaw = std::atan2(seg_dy, seg_dx);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        inter.pose.orientation = tf2::toMsg(q);

        if (out_path.poses.empty() ||
            std::hypot(inter.pose.position.x - out_path.poses.back().pose.position.x,
                       inter.pose.position.y - out_path.poses.back().pose.position.y) > 1e-6)
        {
          out_path.poses.push_back(inter);
        }
        break;
      }
    }
  }

} // namespace jhzx::center_articulation_planner
