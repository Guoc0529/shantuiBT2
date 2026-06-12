#ifndef ROS_TOPIC_MANAGER_H
#define ROS_TOPIC_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <functional>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <actionlib/client/simple_action_client.h>
#include <autonomous_loader_msgs/NavigateAction.h>
#include "autonomous_loader_msgs/spadingPose.h"

namespace autonomous_loader_bt
{

enum class NavigationStatus : int8_t
{
    SCOOP = 0,
    DUMP = 1,
    OTHER = 2
};

class ROSTopicManager
{
public:
    using NavigateClient = actionlib::SimpleActionClient<autonomous_loader_msgs::NavigateAction>;

    static const char* statusToLabel(NavigationStatus s)
    {
        switch (s)
        {
            case NavigationStatus::SCOOP: return "Scoop";
            case NavigationStatus::DUMP:  return "Dump";
            default:                      return "Other";
        }
    }

    static ROSTopicManager& getInstance()
    {
        static ROSTopicManager instance;
        return instance;
    }

    void initialize(ros::NodeHandle& nh)
    {
        nh_ = nh;

        spading_pose_client_ = nh.serviceClient<autonomous_loader_msgs::spadingPose>("/spadingPose");

        goal_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/bt_navigation/current_goal", 1, true);
        action_status_pub_ = nh.advertise<std_msgs::String>("/bt_action/status", 10, true);

        std::string action_name;
        nh_.param<std::string>("navigate_action_name", action_name, std::string("/navigate"));
        nav_client_.reset(new NavigateClient(action_name, true));
        ROS_INFO("Waiting for %s action server...", action_name.c_str());
        nav_client_->waitForServer();
        ROS_INFO("%s action server connected.", action_name.c_str());

        ROS_INFO("ROS Topic Manager initialized.");
    }

    void publishActionStatus(const std::string& message)
    {
        std_msgs::String msg;
        msg.data = message;
        action_status_pub_.publish(msg);
        ROS_INFO_STREAM("\033[32m[BT_ACTION]\033[0m " << message);
    }

    void setDistanceUpdateCallback(const std::function<void(double)>& cb) { on_distance_update_ = cb; }
    void setArrivalCallback(const std::function<void(bool)>& cb) { on_arrival_ = cb; }
    void setResultCallback(const std::function<void(const autonomous_loader_msgs::NavigateResult&)>& cb) { on_result_ = cb; }
    void setPlanningFeedbackCallback(const std::function<void(bool)>& cb) { on_planning_feedback_ = cb; }
    void setNavCancelConfirmedCallback(const std::function<void()>& cb) { on_nav_cancel_confirmed_ = cb; }
    void setNavCancelRequestedCallback(const std::function<void()>& cb) { on_nav_cancel_requested_ = cb; }
    void setPoseUpdateCallback(const std::function<void(const geometry_msgs::PoseStamped&)>& cb) { on_pose_update_ = cb; }

    // Send a navigation goal — returns immediately (non-blocking).
    // Done callback fires via AsyncSpinner in main thread; no fork needed.
    void sendMoveBaseGoal(const geometry_msgs::PoseStamped& goal,
        NavigationStatus status, uint8_t current_id, uint8_t last_id)
    {
        if (!nav_client_) {
            throw std::runtime_error("nav_client_ is null!");
        }
        if (!nav_client_->isServerConnected()) {
            throw std::runtime_error("Navigation Action Server is disconnected!");
        }

        goal_publisher_.publish(goal);
        last_goal_ = goal;

        autonomous_loader_msgs::NavigateGoal nav_goal;
        nav_goal.end_pose = goal;
        nav_goal.last_status = static_cast<uint8_t>(last_status_);
        nav_goal.current_status = static_cast<uint8_t>(status);
        nav_goal.current_id = current_id;
        nav_goal.last_id = last_id;
        ROS_INFO("[NAV] Sending goal (%.2f, %.2f), status: %d -> %d",
                 goal.pose.position.x, goal.pose.position.y,
                 static_cast<int>(nav_goal.last_status),
                 static_cast<int>(nav_goal.current_status));

        if (on_arrival_) on_arrival_(false);

        // Cancel any previous goals first
        try {
            nav_client_->cancelAllGoals();
        } catch (const std::exception& e) {
            ROS_WARN("[NAV] cancelAllGoals threw: %s", e.what());
        }

        // All actionlib callbacks fire via AsyncSpinner in main thread — safe to call directly
        nav_client_->sendGoal(
            nav_goal,
            // doneCb — fires via AsyncSpinner in main thread
            [this, status](const actionlib::SimpleClientGoalState& state,
                           const autonomous_loader_msgs::NavigateResultConstPtr& res)
            {
                bool was_preempted = (state == actionlib::SimpleClientGoalState::PREEMPTED);
                ROS_INFO("[NAV] doneCb: state=%s, preempted=%d, has_result=%d",
                         state.toString().c_str(), was_preempted, !!res);

                if (was_preempted) {
                    notifyNavCancelConfirmed();
                }

                autonomous_loader_msgs::NavigateResult result;
                result.success = false;

                if (res) {
                    result = *res;
                }

                if (on_result_) on_result_(result);
                if (on_arrival_) on_arrival_(true);

                if (state == actionlib::SimpleClientGoalState::SUCCEEDED && result.success) {
                    last_status_ = status;
                    ROS_INFO("[NAV] Nav goal succeeded, last_status updated to %d",
                             static_cast<int>(status));
                }
            },
            // activeCb
            NavigateClient::SimpleActiveCallback(),
            // feedbackCb — fires via AsyncSpinner in main thread
            [this](const autonomous_loader_msgs::NavigateFeedbackConstPtr& fb)
            {
                if (!fb) return;
                if (on_distance_update_) on_distance_update_(fb->distance_to_goal);
                if (on_planning_feedback_) {
                    on_planning_feedback_(fb->plan_succeeded);
                }
            }
        );

        ROS_INFO("[NAV] sendMoveBaseGoal returned (callbacks via AsyncSpinner)");
    }

    bool isNavChildAlive() const { return false; }  // No child process

    void cancelMoveBaseGoal()
    {
        if (nav_client_ && nav_client_->isServerConnected()) {
            nav_cancel_confirmed_ = false;
            nav_client_->cancelAllGoals();
            if (on_nav_cancel_requested_) on_nav_cancel_requested_();
        }
    }

    void notifyNavCancelConfirmed()
    {
        nav_cancel_confirmed_ = true;
        if (on_nav_cancel_confirmed_) on_nav_cancel_confirmed_();
    }
    bool isNavCancelConfirmed() const { return nav_cancel_confirmed_; }
    void resetNavCancelConfirmed() { nav_cancel_confirmed_ = false; }

    bool requestScoopPoint(uint8_t pileID, geometry_msgs::PoseStamped& result)
    {
        autonomous_loader_msgs::spadingPose srv;
        srv.request.pileID = pileID;

        if (spading_pose_client_.call(srv))
        {
            result.header.frame_id = "map";
            result.header.stamp = ros::Time::now();
            result.pose = srv.response.pose_output;
            return true;
        }
        return false;
    }

    void setLastStatus(NavigationStatus status) { last_status_ = status; }

    // ===== 避障相关：鸣笛和双闪 =====
    void initializeObstacleControls(ros::NodeHandle& nh)
    {
        horn_pub_ = nh.advertise<std_msgs::UInt8>("/ACU_Honr", 1, true);
        turn_left_pub_ = nh.advertise<std_msgs::UInt8>("/ACU_TurnLeftLgt", 1, true);
        turn_right_pub_ = nh.advertise<std_msgs::UInt8>("/ACU_TurnRightLgt", 1, true);
        ROS_INFO("Obstacle control publishers initialized (Horn, Hazard lights)");
    }

    void hornOn()
    {
        std_msgs::UInt8 msg;
        msg.data = 1;
        horn_pub_.publish(msg);
        ROS_INFO("[OBSTACLE] Horn ON");
    }

    void hornOff()
    {
        std_msgs::UInt8 msg;
        msg.data = 0;
        horn_pub_.publish(msg);
        ROS_INFO("[OBSTACLE] Horn OFF");
    }

    void hazardLightsOn()
    {
        std_msgs::UInt8 msg;
        msg.data = 1;
        turn_left_pub_.publish(msg);
        turn_right_pub_.publish(msg);
        ROS_INFO("[OBSTACLE] Hazard lights ON (left+right blinkers)");
    }

    void hazardLightsOff()
    {
        std_msgs::UInt8 msg;
        msg.data = 0;
        turn_left_pub_.publish(msg);
        turn_right_pub_.publish(msg);
        ROS_INFO("[OBSTACLE] Hazard lights OFF");
    }

    void publishHazardLights(bool on)
    {
        on ? hazardLightsOn() : hazardLightsOff();
    }

    void sendObstacleNavigationGoal(const geometry_msgs::PoseStamped& goal)
    {
        goal_publisher_.publish(goal);
        last_goal_ = goal;
        ROS_INFO("[OBSTACLE] Sending obstacle navigation goal (%.2f, %.2f)",
                 goal.pose.position.x, goal.pose.position.y);
    }

private:
    ROSTopicManager()
        : last_status_(NavigationStatus::OTHER), nav_cancel_confirmed_(false) {}

    ros::NodeHandle nh_;
    ros::ServiceClient spading_pose_client_;
    ros::Publisher goal_publisher_;
    ros::Publisher action_status_pub_;
    ros::Publisher horn_pub_;
    ros::Publisher turn_left_pub_;
    ros::Publisher turn_right_pub_;
    std::unique_ptr<NavigateClient> nav_client_;
    geometry_msgs::PoseStamped last_goal_;

    std::function<void(double)> on_distance_update_;
    std::function<void(bool)> on_arrival_;
    std::function<void(const autonomous_loader_msgs::NavigateResult&)> on_result_;
    std::function<void(bool)> on_planning_feedback_;
    std::function<void()> on_nav_cancel_confirmed_;
    std::function<void()> on_nav_cancel_requested_;
    std::function<void(const geometry_msgs::PoseStamped&)> on_pose_update_;

    NavigationStatus last_status_;
    bool nav_cancel_confirmed_;
};

}  // namespace autonomous_loader_bt

#endif  // ROS_TOPIC_MANAGER_H
