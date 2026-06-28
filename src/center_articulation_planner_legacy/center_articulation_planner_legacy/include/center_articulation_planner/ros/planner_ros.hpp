// 这个planner是规划调度的核心
// 负责全局路径的规划的调用、分割、局部路径调用以及发�?
// 最后的输出是局部路径，给到控制模块

#pragma once
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <array>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "center_articulation_planner_legacy/IsForwardState.h"
#include "hybrid_a_star/SetPlannerParams.h"
#include "center_articulation_planner/core/errors.hpp"
#include "center_articulation_planner/core/data_type.hpp"
#include <autonomous_loader_msgs/NavigateAction.h>
#include "center_articulation_planner/ros/remote_planner_backend.hpp"

#include <local_planner/local_planner.hpp>

namespace jhzx::center_articulation_planner
{
  enum class TaskType
  {
    Unknown,
    InitialShovel,
    NormalShovel,
    InitialUnload,
    NormalUnload,
    MoveAround
  };

  class RosIo;

  class PlannerROS
  {
  public:
    PlannerROS() = default;
    void initialize(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    void setRosIo(RosIo *ros_io) { ros_io_ = ros_io; }

    void onTimer();
    void onMovingTimer();
    void onGoal(const geometry_msgs::PoseStamped &goal_msg);
    void onFrontPose(const geometry_msgs::PoseStamped &pose_msg);
    void onRearPose(const geometry_msgs::PoseStamped &pose_msg);
    void onShanTuiWaypoint(const geometry_msgs::PoseStamped &pose_msg);
    void onReach(const std_msgs::Bool &reach_msg);
    void onIsForward(const std_msgs::Bool &msg);
    void onMap(const nav_msgs::OccupancyGrid &map_msg);
    void onExternalGlobalPath(const nav_msgs::Path &path_msg);
    void onActionGoal(const ::autonomous_loader_msgs::NavigateGoal &goal);
    void onDistance(const std_msgs::Float64 &msg);
    void onIsForwardState(const ::center_articulation_planner_legacy::IsForwardState &msg);
    void onFinalGoalReached(const geometry_msgs::PoseStamped &msg);
    void setError(ErrorCode code, const std::string &detail = "");
    bool handleSetPlannerParams(hybrid_a_star::SetPlannerParams::Request &req,
                                hybrid_a_star::SetPlannerParams::Response &res);
    // 设置远程规划器的最大换向次数（公开给内�?RAII 守卫使用�?
    bool setMaxDirectionSwitches(int value);

  private:
    struct PlanAttemptResult
    {
      bool success{false};
      ErrorCode error_code{ErrorCode::PlanFailed};
      std::string message;
      nav_msgs::Path global_path;
      std::vector<nav_msgs::Path> nav_segments;
      std::vector<bool> backward_flags;
      std::vector<double> segment_lengths;
      double total_length{std::numeric_limits<double>::infinity()};
      double planning_time_sec{0.0};
      std::string tag;
    };

    struct PathTrackingError
    {
      bool valid{false};
      std::size_t best_seg{0};
      double best_t{0.0};
      double lateral_error_m{0.0};
      double longitudinal_error_m{0.0};
      double heading_error_deg{0.0};
    };

    enum class PreMoveDecision
    {
      Proceed,
      Align,
      ReplanRemaining
    };

    bool buildSegmentsFromPlan(const nav_msgs::Path &path,
                               const std::vector<int8_t> &direction_flags,
                               const std::vector<int32_t> &segment_starts,
                               const std::vector<int32_t> &segment_ends);
    bool buildSegmentsFromPlan(const nav_msgs::Path &path,
                               const std::vector<int8_t> &direction_flags,
                               const std::vector<int32_t> &segment_starts,
                               const std::vector<int32_t> &segment_ends,
                               std::vector<nav_msgs::Path> &out_nav_path,
                               std::vector<bool> &out_backward_flags,
                               std::vector<double> &out_segment_lengths);
    PlanAttemptResult runPlanAttempt(const geometry_msgs::PoseStamped &start,
                                     const geometry_msgs::PoseStamped &goal,
                                     const std::string &tag,
                                     int8_t task_type = 0);
    geometry_msgs::PoseStamped buildPlanGoal(bool &goal_adjusted_for_pile);
    void applyUnloadPlanGoalOffset(geometry_msgs::PoseStamped &plan_goal);
    void applyGoalBodyOffset(geometry_msgs::PoseStamped &pose,
                             double forward_m,
                             double lateral_m);
    int8_t remotePlannerTaskType() const;
    void applyPlanAttempt(PlanAttemptResult attempt,
                          bool goal_adjusted_for_pile,
                          bool reset_segment_index);
    bool makeOffsetStartTowardCenter(double back_dist, geometry_msgs::PoseStamped &out_start);
    geometry_msgs::PoseStamped yawOffsetPose(const geometry_msgs::PoseStamped &pose, double yaw_delta_rad);
    void fillSegment(nav_msgs::Path &segment,
                     const nav_msgs::Path &path,
                     std::size_t start_idx,
                     std::size_t end_idx,
                     bool backward);
    bool sendPlannerParams(hybrid_a_star::SetPlannerParams::Request &req);
    // 带自定义超时的版本（用于 fallback 场景�?
    bool sendPlannerParams(hybrid_a_star::SetPlannerParams::Request &req,
                           double timeout_sec);
    static bool isBackwardFlag(int8_t flag);

    // ============================================================
    // Fallback: 增加换向次数的相关函�?
    // ============================================================
    // 使用增加的换向次数尝试规划（最后的 fallback 手段�?
    // 会临时将 max_direction_switches �?1 改为 2
    // 尝试规划后，无论成功与否都恢复为 1
    PlanAttemptResult tryPlanWithIncreasedSwitches(
        const geometry_msgs::PoseStamped &start,
        const geometry_msgs::PoseStamped &goal,
        int8_t task_type = 0);

    nav_msgs::Path global_path_;
    std::vector<nav_msgs::Path> nav_path_;
    std::vector<bool> segment_backward_flags_;
    std::vector<double> segment_lengths_;
    std::size_t total_segments_{0};
    nav_msgs::Path local_path_;

    geometry_msgs::PoseStamped current_goal_pose_;
    geometry_msgs::PoseStamped front_pose_;
    geometry_msgs::PoseStamped rear_pose_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped shan_tui_waypoint_;
    bool has_front_pose_{false};
    bool has_rear_pose_{false};
    bool has_shan_tui_waypoint_{false};
    bool align_check_done_{false};
    bool use_offset_front_at_unload_start_{false};
    bool goal_adjusted_for_pile_{false};  // 新增：标记是否因料堆调整了终�?
    bool final_goal_feedback_required_{true};
    bool reach_goal_received_{false};
    bool final_goal_received_{false};
    ros::Time checking_goal_start_time_;
    double checking_goal_timeout_sec_{5.0};
    enum class PoseSource
    {
      Front,
      Rear
    };
    PoseSource initial_pose_source_{PoseSource::Front};
    PoseSource active_pose_source_{PoseSource::Front};

    std::shared_ptr<local_planner::LocalPlanner> local_planner_ptr_;
    std::unique_ptr<class RemotePlannerBackend> planner_backend_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    PlannerState state_{PlannerState::Idle};
    size_t current_index_{0};
    size_t segment_index_{0};
    bool is_forward_state_{true};
    bool desired_forward_{true};
    bool is_switching_direction_{false};
    double last_switch_yaw_{0.0};
    bool waiting_direction_feedback_{false};
    bool align_service_called_{false};
    float align_target_angle_deg_{0.0};  // 目标转向角度 (°)
    ros::Time waiting_cmd_time_;
    std::unordered_map<std::string, bool> node_success_;
    ros::Time direction_switch_start_time_;
    double direction_switch_timeout_sec_{3.0};
    double direction_feedback_window_sec_{1.0};
    double direction_switch_settle_delay_sec_{0.0};
    static constexpr std::array<const char *, 2> required_nodes_{{"controller", "goal_checker"}};
    bool final_goal_ok_{true};
    double final_distance_error_{0.0};
    double final_angle_error_{0.0};
    std::uint8_t final_error_code_{0};
    std::string final_error_msg_;
    uint8_t last_status_{0};
    uint8_t current_status_{0};
    uint8_t last_id_{0};      // 新增：上一个铲�?卸料 id
    uint8_t current_id_{0};   // 新增：当前铲�?卸料 id（为0时进入停车模式）
    TaskType current_task_{TaskType::Unknown};
    bool brake_engaged_{false};
    // 中转点切换 drive_mode + brake 的流水线标志（0 秒等待，一次性 onTick 内全部完成）
    bool need_inter_segment_drive_mode_switch_{false};
    bool inter_segment_brake_engaged_{false};
    ros::Timer inter_segment_settle_timer_;  // 占位：当前方案 0 秒等待，未实际使用
    double distance_to_goal_{0.0};
    bool has_distance_{false};
    bool unload_flag_sent_{false};
    double unload_trigger_distance_{6.0};
    double load_early_complete_distance_m_{0.0};
    bool waiting_last_segment_distance_refresh_{false};
    bool waiting_load_last_segment_distance_refresh_{false};
    double prev_segment_tail_distance_{0.0};
    double prev_load_segment_tail_distance_{0.0};
    double last_segment_jump_min_m_{2.0};
    int pile_low_threshold_percent_{60};
    double pile_low_offset_x_m_{2.0};
    double unload_goal_offset_x_m_{3.0};
    bool debug_mode_{true};
    // --- BackingOutUnload ---
    int    backout_retry_count_{0};
    double backout_yaw_threshold_rad_{0.1};
    double backout_distance_m_{12.0};
    double backout_step_m_{0.2};
    double final_y_error_{0.0};
    double backout_lateral_threshold_m_{0.4};
    double backout_lateral_yaw_threshold_rad_{0.8};
    static constexpr int kMaxBackoutRetries = 2;
    double goal_guard_distance_m_{0.0};
    double goal_guard_angle_deg_{110.0};
    double goal_guard_forward_deadzone_m_{0.25};
    double align_pos_thresh_m_{1.0};
    double align_lateral_thresh_m_{1.0};  // 横向误差阈值（用于非首段）
    double align_heading_thresh_deg_{30.0};
    double large_lateral_replan_thresh_m_{1.5};
    int max_remaining_replan_count_{1};
    int remaining_replan_count_{0};
    bool goal_guard_active_{false};
    bool goal_guard_logged_{false};
    std::string plan_service_name_{"/plan_hybrid_path"};
    std::string set_params_service_name_{"/run_hybrid_astar/set_planner_params"};
    double planner_service_timeout_{3.0};

    void reset();
    void planning_global_path();
    void adjust_vehicle_direction();

    bool estimateSegmentForward();
    void publishIsForwardCommand(bool target_forward);
    void findNearestSegment(const nav_msgs::Path &path, const geometry_msgs::Point &curr_pos, size_t &best_seg, double &best_t);
    void findLookaheadTarget(const nav_msgs::Path &path, size_t start_seg, double start_t, double forward_dist,
                             size_t &target_seg, double &target_t, double &target_x, double &target_y, double &target_yaw);
    double generateHermiteMerge(const geometry_msgs::Pose &curr, double target_x, double target_y, double target_yaw,
                                double scale, double min_dist, int points, nav_msgs::Path &out_path);
    void extractPathSegment(const nav_msgs::Path &path, size_t start_seg, double start_t, double length_budget, nav_msgs::Path &out_path);

    // 从 curr 在 path 上的最近投影点起，沿 path 向前裁剪 x_meter 弧长（无车位姿连线）
    nav_msgs::Path clipPathByLookahead(const nav_msgs::Path &path,
                                       const geometry_msgs::Pose &curr,
                                       double x_meter = 8.0);
    bool nearGoalUnreachable(double remaining_dist,
                             double &forward_m,
                             double &lateral_m,
                             double &angle_deg,
                             std::string &reason) const;

    RosIo *ros_io_{nullptr};
    void transitionTo(PlannerState next_state, const std::string &reason = "");
    std::string formatErrorStateMessage() const;
    void onEnterCompleted();
    TaskType decodeTask(uint8_t last_status, uint8_t current_status) const;
    void kickoffTaskFromAction(TaskType task);
    void finalizeDirectionSwitch();
    bool maybeEnterAligningInPlace();
    bool computePathTrackingError(const nav_msgs::Path &path,
                                  const geometry_msgs::PoseStamped &pose,
                                  PathTrackingError &tracking_error);
    PreMoveDecision evaluatePreMoveDecision(PathTrackingError &tracking_error,
                                            double &align_position_error,
                                            double &align_position_thresh,
                                            std::string &align_error_type,
                                            double &heading_deg);
    bool handlePreMoveDecision();
    bool replanRemainingPath(const PathTrackingError &tracking_error);
    bool allRequiredNodesSucceeded() const;
    bool directionSwitchTimedOut() const;
    PoseSource togglePoseSource(std::size_t segment_idx);
    void setPoseSource(PoseSource source);
    void updateCurrentPoseFromActiveSource();
    bool makeOffsetStartFromFront(double back_dist, geometry_msgs::PoseStamped &out_start);
    double computePathLength(const nav_msgs::Path &path) const;
    double remainingDistanceToGoal() const;
    double distanceToCurrentSegmentGoal() const;
    void armLastUnloadDistanceRefreshGate(const std::string &reason);
    void tryClearLastSegmentDistanceRefreshGate();
    void armLastLoadDistanceRefreshGate(const std::string &reason);
    void tryClearLastLoadDistanceRefreshGate();

    void transitionToCompletedOrBackout(const std::string &reason);
    bool maybeEnterBackoutUnload();
    bool buildBackoutPaths();
  };

} // namespace center_articulation_planner
