#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "center_articulation_planner_legacy/PlanHybridPath.h"
#include "hybrid_a_star/SetPlannerParams.h"

namespace jhzx::center_articulation_planner
{
  struct RemotePlanResult
  {
    bool call_ok{false};
    bool success{false};
    std::string message;
    nav_msgs::Path path;
    std::vector<int8_t> direction_flags;
    std::vector<int32_t> segment_starts;
    std::vector<int32_t> segment_ends;
    double planning_time_sec{0.0};
  };

  class RemotePlannerBackend
  {
  public:
    RemotePlannerBackend(ros::NodeHandle nh,
                         std::string plan_service_name,
                         std::string set_params_service_name,
                         double timeout_sec);

    RemotePlanResult plan(const geometry_msgs::PoseStamped &start,
                          const geometry_msgs::PoseStamped &goal,
                          int8_t task_type = 0);

    bool setPlannerParams(hybrid_a_star::SetPlannerParams::Request &req,
                          std::string &err);

    // 带自定义超时的版本（用于 fallback 场景，需要快速响应）
    // timeout_sec: 等待 service 的超时时间（秒）
    bool setPlannerParams(hybrid_a_star::SetPlannerParams::Request &req,
                          std::string &err,
                          double timeout_sec);

  private:
    bool waitForService(ros::ServiceClient &client, const std::string &name) const;
    // 带自定义超时的版本
    bool waitForService(ros::ServiceClient &client, const std::string &name, double timeout_sec) const;

    ros::NodeHandle nh_;
    ros::ServiceClient plan_client_;
    ros::ServiceClient set_params_client_;
    std::string plan_service_name_;
    std::string set_params_service_name_;
    ros::Duration timeout_;
  };
} // namespace jhzx::center_articulation_planner
