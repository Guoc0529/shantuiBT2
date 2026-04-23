#ifndef ROS_TOPIC_MANAGER_H
#define ROS_TOPIC_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <functional>
#include <cstdint>
#include <actionlib/client/simple_action_client.h>
#include <autonomous_loader_msgs/NavigateAction.h>
#include <cmath>
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

    // Helper to print human-readable navigation status labels
    static const char* statusToLabel(NavigationStatus s)
    {
        switch (s)
        {
            case NavigationStatus::SCOOP: return "铲料";
            case NavigationStatus::DUMP:  return "卸料";
            default:                      return "其他";
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
        
        // Service client for scoop point
        spading_pose_client_ = nh.serviceClient<autonomous_loader_msgs::spadingPose>("/spadingPose");
        
        // Publisher for the current navigation goal
        goal_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/bt_navigation/current_goal", 1, true); // Latching publisher
        
        // Publisher for action status messages
        action_status_pub_ = nh.advertise<std_msgs::String>("/bt_action/status", 10, true);
        
        // Action client for custom navigation
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
        ROS_INFO("[BT_ACTION] %s", message.c_str());
    }

    // Callbacks for wiring feedback to GlobalState
    void setDistanceUpdateCallback(const std::function<void(double)>& cb) { on_distance_update_ = cb; }
    void setArrivalCallback(const std::function<void(bool)>& cb) { on_arrival_ = cb; }
    void setResultCallback(const std::function<void(const autonomous_loader_msgs::NavigateResult&)>& cb) { on_result_ = cb; }
    void setPlanningFeedbackCallback(const std::function<void(bool)>& cb) { on_planning_feedback_ = cb; }
    // Pose updates not required in distance-reduction mode; keeping API for compatibility
    void setPoseUpdateCallback(const std::function<void(const geometry_msgs::PoseStamped&)>& cb) { on_pose_update_ = cb; }
    
    // Navigation APIs (using custom Navigate.action)
    void sendMoveBaseGoal(const geometry_msgs::PoseStamped& goal,
                          NavigationStatus status, uint8_t current_id, uint8_t last_id)
    {
        // Publish the goal to the dedicated topic
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
        // last_status_ will be updated in doneCb when navigation succeeds
        // last_status_ = status;
        
        if (on_arrival_) on_arrival_(false);
        
        nav_client_->sendGoal(
            nav_goal,
            // doneCb
            [this, status](const actionlib::SimpleClientGoalState& state,
                   const autonomous_loader_msgs::NavigateResultConstPtr& res)
            {
                if (res) {
                    if (on_result_) on_result_(*res);
                    // The action is done, so we have "arrived" at a result.
                    // The quality of that result is checked by the BT itself.
                    if (on_arrival_) on_arrival_(true);
                    // Update last_status_ only if navigation succeeded
                    if(state == actionlib::SimpleClientGoalState::SUCCEEDED && res->success)
                    {
                        last_status_ = status;
                    }
                } else {
                    // 如果结果为空，说明Action失败，创建一个默认的失败结果
                    autonomous_loader_msgs::NavigateResult default_res;
                    default_res.success = false;
                    if (on_result_) on_result_(default_res);
                    // The action is done, so we have "arrived" at a result.
                    if (on_arrival_) on_arrival_(true);
                }
            },
            // activeCb
            NavigateClient::SimpleActiveCallback(),
            // feedbackCb
            [this](const autonomous_loader_msgs::NavigateFeedbackConstPtr& fb)
            {
                if (!fb) return;
                // Always update distance
                if (on_distance_update_) on_distance_update_(fb->distance_to_goal);

                // Additionally, check for the planning feedback. 
                // The action server should send this immediately after planning.
                if (on_planning_feedback_) {
                    on_planning_feedback_(fb->plan_succeeded);
                }
            }
        );
    }

    void cancelMoveBaseGoal()
    {
        if (nav_client_ && nav_client_->isServerConnected()) {
            nav_client_->cancelAllGoals();
        }
    }
    
    // Service call for scoop point
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

    void setLastStatus(NavigationStatus status)
    {
        last_status_ = status;
    }

private:
    ROSTopicManager() = default;

    ros::NodeHandle nh_;
    ros::ServiceClient spading_pose_client_;
    ros::Publisher goal_publisher_; // Publisher for the current navigation goal
    ros::Publisher action_status_pub_; // Publisher for action status messages
    std::unique_ptr<NavigateClient> nav_client_;
    geometry_msgs::PoseStamped last_goal_;

    std::function<void(double)> on_distance_update_;
    std::function<void(bool)> on_arrival_;
    std::function<void(const autonomous_loader_msgs::NavigateResult&)> on_result_;
    std::function<void(const geometry_msgs::PoseStamped&)> on_pose_update_;
    std::function<void(bool)> on_planning_feedback_;

    NavigationStatus last_status_ = NavigationStatus::OTHER;
};

} // namespace autonomous_loader_bt

#endif // ROS_TOPIC_MANAGER_H