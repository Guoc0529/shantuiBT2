#include "center_articulation_planner/ros/ros_io.hpp"
#include "center_articulation_planner/ros/planner_ros.hpp"

#include <boost/bind.hpp>

#include <my_msgs/pileInfo.h>
#include <utility>

namespace jhzx::center_articulation_planner
{

  RosIo::RosIo(ros::NodeHandle nh, ros::NodeHandle pnh, PlannerROS &planner)
      : nh_(std::move(nh)), pnh_(std::move(pnh)), planner_(planner)
  {
    planner_.setRosIo(this);
    planner_.initialize(nh_, pnh_);

    loadParameters();
    setupPublishers();
    setupSubscribers();
    setupServices();
    setupTimers();
    setupActionServer();

    ROS_INFO("PlannerROS initialized.");
  }

  void RosIo::loadParameters()
  {    
    pnh_.param<std::string>("reach_topic", reach_topic_, reach_topic_);
    pnh_.param<std::string>("is_forward_topic", is_forward_topic_, is_forward_topic_);
    pnh_.param<std::string>("global_path_topic", global_path_topic_, global_path_topic_);
    pnh_.param<std::string>("segmented_path_topic", segmented_path_topic_, segmented_path_topic_);
    pnh_.param<std::string>("local_path_topic", local_path_topic_, local_path_topic_);
    pnh_.param<std::string>("front_pose_topic", front_pose_topic_, front_pose_topic_);
    pnh_.param<std::string>("rear_pose_topic", rear_pose_topic_, rear_pose_topic_);
    pnh_.param<std::string>("way_point_shan_tui_topic", way_point_shan_tui_topic_, way_point_shan_tui_topic_);
    pnh_.param<std::string>("map_topic", map_topic_, map_topic_);
    pnh_.param<std::string>("distance_topic", distance_topic_, distance_topic_);
    pnh_.param<std::string>("navigate_action_name", action_name_, action_name_);
    pnh_.param<std::string>("plan_service_start_topic", plan_service_start_topic_, plan_service_start_topic_);
    pnh_.param<std::string>("plan_service_goal_topic", plan_service_goal_topic_, plan_service_goal_topic_);
    pnh_.param<std::string>("goal_topic", goal_topic_, goal_topic_);                         // For goal_checker
    pnh_.param<std::string>("legacy_goal_topic", legacy_goal_topic_, legacy_goal_topic_);    // For legacy test interface
    pnh_.param<std::string>("planner_goal_topic", planner_goal_topic_, planner_goal_topic_); // For 终点微调调整
    pnh_.param<std::string>("final_goal_reached_topic", final_goal_reached_topic_, final_goal_reached_topic_);
    pnh_.param<std::string>("shache_topic", shache_topic_, shache_topic_);
    pnh_.param<std::string>("is_unload_topic", is_unload_topic_, is_unload_topic_);
    if (!pnh_.getParam("controller_started_topic", controller_started_topic_))
    {
      // TODO：删掉这�?      
      pnh_.param<std::string>("controller_stop_topic", controller_started_topic_, controller_started_topic_);
    }
    pnh_.param<std::string>("pile_info_service", pile_info_service_, pile_info_service_);
    pnh_.param<std::string>("align_in_place_service", align_in_place_service_, align_in_place_service_);
    pnh_.param("map_subscribe_enabled", map_subscribe_enabled_, map_subscribe_enabled_);
  }

  void RosIo::setupPublishers()
  {
    path_pub_ = nh_.advertise<nav_msgs::Path>(global_path_topic_, 1);
    segmented_path_pub_ = nh_.advertise<nav_msgs::Path>(segmented_path_topic_, 1);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>(local_path_topic_, 1);
    nav_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1);
    planner_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(planner_goal_topic_, 1);
    plan_service_start_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(plan_service_start_topic_, 1, true);
    plan_service_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(plan_service_goal_topic_, 1, true);
    is_forward_pub_ = nh_.advertise<std_msgs::Bool>(is_forward_topic_, 1);
    is_unload_pub_ = nh_.advertise<std_msgs::Bool>(is_unload_topic_, 1);
    controller_started_pub_ = nh_.advertise<std_msgs::Bool>(controller_started_topic_, 1);
    brake_pub_ = nh_.advertise<std_msgs::UInt8>(shache_topic_, 1);
  }

  void RosIo::setupSubscribers()
  {
    sub_goal_ = nh_.subscribe(legacy_goal_topic_, 1, &RosIo::goalCallback, this);
    sub_reach_ = nh_.subscribe(reach_topic_, 1, &RosIo::reachCallback, this);
    sub_front_pose_ = nh_.subscribe(front_pose_topic_, 1, &RosIo::frontPoseCallback, this);
    sub_rear_pose_ = nh_.subscribe(rear_pose_topic_, 1, &RosIo::rearPoseCallback, this);
    sub_shan_tui_waypoint_ = nh_.subscribe(way_point_shan_tui_topic_, 1, &RosIo::shanTuiWaypointCallback, this);
    sub_is_forward_ = nh_.subscribe(is_forward_topic_, 1, &RosIo::isForwardCallback, this);
    sub_is_forward_state_ = nh_.subscribe(is_forward_state_topic_, 3, &RosIo::isForwardStateCallback, this);
    if (map_subscribe_enabled_)
    {
      sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>(map_topic_, 1, &RosIo::mapCallback, this);
    }
    else
    {
      ROS_WARN("Map subscription disabled (map_subscribe_enabled=false); remote planner is expected to manage maps.");
    }
    sub_global_path_ = nh_.subscribe<nav_msgs::Path>("/global_path_ts", 1, &RosIo::globalPathCallback, this);
    sub_distance_ = nh_.subscribe(distance_topic_, 1, &RosIo::distanceCallback, this);
    sub_final_goal_reached_ = nh_.subscribe<geometry_msgs::PoseStamped>(final_goal_reached_topic_, 1, &RosIo::finalGoalReachedCallback, this);
  }

  void RosIo::setupServices()
  {
    set_params_srv_ = pnh_.advertiseService("set_planner_params",
                                            &RosIo::setPlannerParamsCallback,
                                            this);
    pile_info_client_ = nh_.serviceClient<my_msgs::pileInfo>(pile_info_service_);
    align_in_place_client_ = nh_.serviceClient<center_articulation_planner_legacy::AlignInPlace>(align_in_place_service_);
  }

  void RosIo::setupTimers()
  {
    timer_ = nh_.createTimer(ros::Duration(0.2), &RosIo::timerCallback, this);
    moving_timer_ = nh_.createTimer(ros::Duration(0.05), &RosIo::movingTimerCallback, this, false, false);
  }

  void RosIo::setupActionServer()
  {
    // Manual goal/preempt callbacks to allow the state machine to decide when to finish the action.
    action_server_ = std::make_unique<NavigateActionServer>(nh_, action_name_, false);
    action_server_->registerGoalCallback(boost::bind(&RosIo::handleActionGoal, this));
    action_server_->registerPreemptCallback(boost::bind(&RosIo::handleActionPreempt, this));
    action_server_->start();
    ROS_INFO_STREAM("Navigate action server started on action name: " << action_name_);
  }

  void RosIo::publishGlobalPath(const nav_msgs::Path &path)
  {
    path_pub_.publish(path);
  }

  void RosIo::publishSegmentedPath(const nav_msgs::Path &path)
  {
    segmented_path_pub_.publish(path);
  }

  void RosIo::publishLocalPath(const nav_msgs::Path &path)
  {
    local_path_pub_.publish(path);
  }

  void RosIo::publishNavPoint(const geometry_msgs::PoseStamped &pose)
  {
    nav_point_pub_.publish(pose);
  }

  void RosIo::publishPlannerGoal(const geometry_msgs::PoseStamped &pose)
  {
    planner_goal_pub_.publish(pose);
  }

  void RosIo::publishPlanServiceStart(const geometry_msgs::PoseStamped &pose)
  {
    plan_service_start_pub_.publish(pose);
  }

  void RosIo::publishPlanServiceGoal(const geometry_msgs::PoseStamped &pose)
  {
    plan_service_goal_pub_.publish(pose);
  }

  void RosIo::publishIsForward(bool is_forward)
  {
    std_msgs::Bool msg;
    msg.data = is_forward;
    is_forward_pub_.publish(msg);
  }

  void RosIo::publishIsUnload(bool is_unload)
  {
    std_msgs::Bool msg;
    msg.data = is_unload;
    is_unload_pub_.publish(msg);
  }

  void RosIo::publishBrake(std::uint8_t value)
  {
    std_msgs::UInt8 msg;
    msg.data = value;
    brake_pub_.publish(msg);
  }

  void RosIo::publishControllerStarted(bool started)
  {
    std_msgs::Bool msg;
    msg.data = started;
    controller_started_pub_.publish(msg);
  }

  bool RosIo::callAlignInPlace(bool use_rear_pose_source, float target_angle)
  {
    center_articulation_planner_legacy::AlignInPlace srv;
    srv.request.use_rear_pose_source = use_rear_pose_source;
    srv.request.target_angle = target_angle;
    if (!align_in_place_client_.call(srv))
    {
      ROS_WARN("align_in_place service call failed.");
      return false;
    }
    if (!srv.response.success)
    {
      ROS_WARN("align_in_place service reported failure: %s", srv.response.message.c_str());
    }
    else
    {
      ROS_INFO("align_in_place succeeded (target_angle=%.1f, final_angle=%.1f deg, use_rear=%d)",
               static_cast<double>(target_angle),
               srv.response.final_angle_deg,
               static_cast<int>(use_rear_pose_source));
    }
    return srv.response.success;
  }

  int RosIo::getPileVolume(uint8_t pile_id)
  {
    // 超时设置�?.1 �?    
    constexpr double kServiceTimeout = 0.1;

    // 等待 service 可用
    if (!pile_info_client_.waitForExistence(ros::Duration(kServiceTimeout)))
    {
      ROS_WARN("pile_info service not available after %.1fs timeout (pileID=%u)",
               kServiceTimeout,
               static_cast<unsigned>(pile_id));
      return -1;
    }

    // 构造请�?    
    my_msgs::pileInfo srv;
    srv.request.pileID = pile_id;

    // 调用 service
    if (!pile_info_client_.call(srv))
    {
      ROS_WARN("Failed to call pile_info service for pileID=%u",
               static_cast<unsigned>(pile_id));
      return -1;
    }

    // 详细日志：记录料堆余�?    
    ROS_INFO("Pile %u volume: %u%%",
             static_cast<unsigned>(pile_id),
             static_cast<unsigned>(srv.response.volume));

    return static_cast<int>(srv.response.volume);
  }

  void RosIo::startMovingTimer()
  {
    if (moving_timer_.isValid())
    {
      moving_timer_.start();
    }
  }

  void RosIo::stopMovingTimer()
  {
    if (moving_timer_.isValid())
    {
      moving_timer_.stop();
    }
  }

  void RosIo::shutdownMapSubscription()
  {
    if (sub_map_)
    {
      sub_map_.shutdown();
    }
  }

  void RosIo::timerCallback(const ros::TimerEvent &)
  {
    planner_.onTimer();
  }

  void RosIo::movingTimerCallback(const ros::TimerEvent &)
  {
    planner_.onMovingTimer();
  }

  void RosIo::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
  {
    if (goal_msg)
    {
      planner_.onGoal(*goal_msg);
    }
  }

  void RosIo::reachCallback(const std_msgs::Bool::ConstPtr &reach_msg)
  {
    if (reach_msg)
    {
      planner_.onReach(*reach_msg);
    }
  }

  void RosIo::frontPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
  {
    if (pose_msg)
    {
      planner_.onFrontPose(*pose_msg);
    }
  }

  void RosIo::rearPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
  {
    if (pose_msg)
    {
      planner_.onRearPose(*pose_msg);
    }
  }

  void RosIo::shanTuiWaypointCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
  {
    if (pose_msg)
    {
      planner_.onShanTuiWaypoint(*pose_msg);
    }
  }

  void RosIo::isForwardCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    if (msg)
    {
      planner_.onIsForward(*msg);
    }
  }

  void RosIo::isForwardStateCallback(const ::center_articulation_planner_legacy::IsForwardState::ConstPtr &msg)
  {
    if (msg)
    {
      planner_.onIsForwardState(*msg);
    }
  }

  void RosIo::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
  {
    if (map_msg)
    {
      planner_.onMap(*map_msg);
    }
  }

  void RosIo::globalPathCallback(const nav_msgs::Path::ConstPtr &path_msg)
  {
    if (path_msg)
    {
      planner_.onExternalGlobalPath(*path_msg);
    }
  }

  void RosIo::publishActionResult(bool success,
                                  double final_distance_error,
                                  double final_angle_error,
                                  std::uint8_t error_code,
                                  const std::string &error_msg)
  {
    if (!action_server_ || !has_active_goal_)
    {
      ROS_WARN("No active action goal to publish result for.");
      return;
    }

    ::autonomous_loader_msgs::NavigateResult result;
    result.success = success;
    result.final_distance_error = final_distance_error;
    result.final_angle_error = final_angle_error;
    result.error_code = error_code;
    result.error_msg = error_msg;

    if (success)
    {
      action_server_->setSucceeded(result, "Planner completed successfully.");
    }
    else
    {
      const std::string msg = error_msg.empty() ? "Planner entered error state." : error_msg;
      action_server_->setAborted(result, msg);
    }
    has_active_goal_ = false;
  }

  void RosIo::publishActionFeedback(double distance, bool plan_succeeded)
  {
    if (!action_server_ || !has_active_goal_)
    {
      return;
    }
    ::autonomous_loader_msgs::NavigateFeedback feedback;
    feedback.distance_to_goal = distance;
    feedback.plan_succeeded = plan_succeeded;
    action_server_->publishFeedback(feedback);
  }

  void RosIo::handleActionGoal()
  {
    if (!action_server_)
    {
      return;
    }

    if (action_server_->isActive() && has_active_goal_)
    {
      ::autonomous_loader_msgs::NavigateResult preempt_result;
      preempt_result.success = false;
      action_server_->setPreempted(preempt_result, "New goal preempted previous active goal.");
      has_active_goal_ = false;
    }

    const auto goal = action_server_->acceptNewGoal();
    if (!goal)
    {
      ROS_WARN("Navigate action goal is null, ignoring.");
      return;
    }

    has_active_goal_ = true;
    ROS_INFO_STREAM("Action goal received: last_status=" << static_cast<int>(goal->last_status)
                    << " current_status=" << static_cast<int>(goal->current_status));
    planner_.onActionGoal(*goal);
  }

  void RosIo::handleActionPreempt()
  {
    if (!action_server_ || !has_active_goal_)
    {
      return;
    }
    ::autonomous_loader_msgs::NavigateResult preempt_result;
    preempt_result.success = false;
    action_server_->setPreempted(preempt_result, "Goal preempted by client.");
    has_active_goal_ = false;
  }

  bool RosIo::setPlannerParamsCallback(hybrid_a_star::SetPlannerParams::Request &req,
                                       hybrid_a_star::SetPlannerParams::Response &res)
  {
    return planner_.handleSetPlannerParams(req, res);
  }

  void RosIo::distanceCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    if (msg)
    {
      planner_.onDistance(*msg);
    }
  }

  void RosIo::finalGoalReachedCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (msg)
    {
      planner_.onFinalGoalReached(*msg);
    }
  }

} // namespace jhzx::center_articulation_planner
