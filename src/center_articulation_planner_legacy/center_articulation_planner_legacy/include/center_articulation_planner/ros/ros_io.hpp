#pragma once

#include <memory>
#include <string>
#include <cstdint>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

#include "center_articulation_planner_legacy/AlignInPlace.h"
#include "center_articulation_planner_legacy/IsForwardState.h"
#include "hybrid_a_star/SetPlannerParams.h"
#include <autonomous_loader_msgs/NavigateAction.h>

namespace jhzx::center_articulation_planner
{

  class PlannerROS;
  using NavigateActionServer = actionlib::SimpleActionServer<::autonomous_loader_msgs::NavigateAction>;

  class RosIo
  {
  public:
    RosIo(ros::NodeHandle nh, ros::NodeHandle pnh, PlannerROS &planner);

    void publishGlobalPath(const nav_msgs::Path &path);
    void publishSegmentedPath(const nav_msgs::Path &path);
    void publishLocalPath(const nav_msgs::Path &path);
    void publishNavPoint(const geometry_msgs::PoseStamped &pose);
    void publishPlannerGoal(const geometry_msgs::PoseStamped &pose);
    void publishPlanServiceStart(const geometry_msgs::PoseStamped &pose);
    void publishPlanServiceGoal(const geometry_msgs::PoseStamped &pose);
    void publishIsForward(bool is_forward);
    void publishIsUnload(bool is_unload);
    void publishBrake(std::uint8_t value);
    void publishDriveMode(std::uint8_t value);
    void publishControllerStarted(bool started);  // controller_started command (true=enable, false=stop)
    void publishActionFeedback(double distance, bool plan_succeeded = false);
    bool callAlignInPlace(bool use_rear_pose_source, float target_angle);

    // 新增：查询料堆余�?    
    // 返回值：0-100 表示余料百分比，-1 表示查询失败或超�?    
    int getPileVolume(uint8_t pile_id);

    void startMovingTimer();
    void stopMovingTimer();
    void shutdownMapSubscription();
    void publishActionResult(bool success,
                             double final_distance_error = 0.0,
                             double final_angle_error = 0.0,
                             std::uint8_t error_code = 0,
                             const std::string &error_msg = "");

  private:
    void loadParameters();
    void setupPublishers();
    void setupSubscribers();
    void setupServices();
    void setupTimers();
    void setupActionServer();

    void timerCallback(const ros::TimerEvent &event);
    void movingTimerCallback(const ros::TimerEvent &event);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg);
    void reachCallback(const std_msgs::Bool::ConstPtr &reach_msg);
    void frontPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    void rearPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    void shanTuiWaypointCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    void isForwardCallback(const std_msgs::Bool::ConstPtr &msg);
    void isForwardStateCallback(const ::center_articulation_planner_legacy::IsForwardState::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
    void globalPathCallback(const nav_msgs::Path::ConstPtr &path_msg);
    void distanceCallback(const std_msgs::Float64::ConstPtr &msg);
    void finalGoalReachedCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void handleActionGoal();
    void handleActionPreempt();
    bool setPlannerParamsCallback(hybrid_a_star::SetPlannerParams::Request &req,
                                  hybrid_a_star::SetPlannerParams::Response &res);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    PlannerROS &planner_;

    ros::Timer timer_;
    ros::Timer moving_timer_;

    ros::Subscriber sub_goal_;
    ros::Subscriber sub_reach_;
    ros::Subscriber sub_front_pose_;
    ros::Subscriber sub_rear_pose_;
    ros::Subscriber sub_shan_tui_waypoint_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_is_forward_;
    ros::Subscriber sub_is_forward_state_;
    ros::Subscriber sub_global_path_;
    ros::Subscriber sub_distance_;
    ros::Subscriber sub_final_goal_reached_;
    ros::ServiceServer set_params_srv_;
    ros::ServiceClient pile_info_client_;  // 新增：料堆信�?service client
    ros::ServiceClient align_in_place_client_;

    ros::Publisher is_forward_pub_;
    ros::Publisher nav_point_pub_;
    ros::Publisher planner_goal_pub_;
    ros::Publisher plan_service_start_pub_;
    ros::Publisher plan_service_goal_pub_;
    ros::Publisher path_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher segmented_path_pub_;
    ros::Publisher brake_pub_;
    ros::Publisher is_unload_pub_;
    ros::Publisher controller_started_pub_;  // controller_started command publisher
    ros::Publisher drive_mode_pub_;          // drive_mode request publisher (inter-segment)

    std::string planner_goal_topic_{"/planner_goal"};

    std::string plan_service_start_topic_{"/plan_service_start"};
    std::string plan_service_goal_topic_{"/plan_service_goal"};
    std::string map_topic_{"/map"};
    std::string is_forward_topic_{"/is_forward"};
    std::string is_forward_state_topic_{"/is_forward_state"};
    std::string goal_topic_{"/planner_goal"};
    std::string legacy_goal_topic_{"/goal_chao"};
    std::string reach_topic_{"/goal_reached"};
    std::string front_pose_topic_{"/Odometry_front"};
    std::string rear_pose_topic_{"/Odometry_rear"};
    std::string way_point_shan_tui_topic_{"/way_point_shan_tui"};
    std::string distance_topic_{"/distance_to_goal"};
    std::string final_goal_reached_topic_{"/final_goal_reached"};
    std::string global_path_topic_{"/global_path"};
    std::string segmented_path_topic_{"/segmented_path"};
    std::string local_path_topic_{"/local_path"};
    std::string shache_topic_{"/shache"};
    std::string is_unload_topic_{"/is_unload"};
    std::string controller_started_topic_{"/controller_started"};  // controller_started topic
    std::string drive_mode_topic_{"/ACU_DrvModeReq"};  // drive_mode request topic (inter-segment)
    std::string pile_info_service_{"/pileVolume"};  // 新增：料堆信�?service
    std::string align_in_place_service_{"/align_in_place"};
    std::string action_name_{"navigate"};
    bool map_subscribe_enabled_{false};

    std::unique_ptr<NavigateActionServer> action_server_;
    bool has_active_goal_{false};
  };

} // namespace jhzx::center_articulation_planner



