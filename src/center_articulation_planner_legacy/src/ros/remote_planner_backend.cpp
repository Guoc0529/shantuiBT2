#include "center_articulation_planner/ros/remote_planner_backend.hpp"

#include <utility>

/*
 - Holds service names, timeout, and two ROS service clients for PlanHybridPath and SetPlannerParams. 

  - plan() waits for the plan service, sends start/goal, and returns
    RemotePlanResult with path + segmentation flags + timing; call_ok only
    becomes true when the service call succeeds. 

  - setPlannerParams() forwards parameter updates to the remote planner service
    and surfaces service/response failures via err. 

  - 状态机里, PlannerROS::planning_global_path() 使用这个类
    during PlanningGlobalPath 获取 global plan, then builds segments and
    transitions to AdjustingDirection or Error.
*/

namespace jhzx::center_articulation_planner
{
  RemotePlannerBackend::RemotePlannerBackend(ros::NodeHandle nh,
                                             std::string plan_service_name,
                                             std::string set_params_service_name,
                                             double timeout_sec)
      : nh_(std::move(nh)),
        plan_service_name_(std::move(plan_service_name)),
        set_params_service_name_(std::move(set_params_service_name)),
        timeout_(timeout_sec)
  {
    plan_client_ = nh_.serviceClient<center_articulation_planner_legacy::PlanHybridPath>(plan_service_name_);
    set_params_client_ = nh_.serviceClient<hybrid_a_star::SetPlannerParams>(set_params_service_name_);
  }

  bool RemotePlannerBackend::waitForService(ros::ServiceClient &client, const std::string &name) const
  {
    if (client.exists())
    {
      return true;
    }
    ROS_WARN_STREAM("Waiting for service " << name << " for up to " << timeout_.toSec() << "s");
    return client.waitForExistence(timeout_);
  }

  // 带自定义超时的版本
  bool RemotePlannerBackend::waitForService(ros::ServiceClient &client,
                                            const std::string &name,
                                            double timeout_sec) const
  {
    if (client.exists())
    {
      return true;
    }
    ROS_WARN_STREAM("Waiting for service " << name << " for up to " << timeout_sec << "s");
    return client.waitForExistence(ros::Duration(timeout_sec));
  }

  RemotePlanResult RemotePlannerBackend::plan(const geometry_msgs::PoseStamped &start,
                                              const geometry_msgs::PoseStamped &goal,
                                              int8_t task_type)
  {
    RemotePlanResult result;
    center_articulation_planner_legacy::PlanHybridPath srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.task_type = task_type;

    if (!waitForService(plan_client_, plan_service_name_))
    {
      result.message = "Plan service unavailable";
      return result;
    }

    if (!plan_client_.call(srv))
    {
      result.message = "Plan service call failed";
      return result;
    }

    result.call_ok = true;
    result.success = srv.response.success;
    result.message = srv.response.message;
    result.path = std::move(srv.response.path);
    result.direction_flags = std::move(srv.response.direction_flags);
    result.segment_starts = std::move(srv.response.segment_starts);
    result.segment_ends = std::move(srv.response.segment_ends);
    result.planning_time_sec = srv.response.planning_time_sec;
    return result;
  }

  bool RemotePlannerBackend::setPlannerParams(hybrid_a_star::SetPlannerParams::Request &req,
                                              std::string &err)
  {
    hybrid_a_star::SetPlannerParams srv;
    srv.request = req;

    if (!waitForService(set_params_client_, set_params_service_name_))
    {
      err = "SetPlannerParams service unavailable";
      return false;
    }

    if (!set_params_client_.call(srv))
    {
      err = "SetPlannerParams service call failed";
      return false;
    }

    if (!srv.response.success)
    {
      err = srv.response.message;
      return false;
    }
    return true;
  }

  // 带自定义超时的版本（用于 fallback 场景）
  bool RemotePlannerBackend::setPlannerParams(hybrid_a_star::SetPlannerParams::Request &req,
                                              std::string &err,
                                              double timeout_sec)
  {
    hybrid_a_star::SetPlannerParams srv;
    srv.request = req;

    if (!waitForService(set_params_client_, set_params_service_name_, timeout_sec))
    {
      err = "SetPlannerParams service unavailable (timeout=" + std::to_string(timeout_sec) + "s)";
      return false;
    }

    if (!set_params_client_.call(srv))
    {
      err = "SetPlannerParams service call failed";
      return false;
    }

    if (!srv.response.success)
    {
      err = srv.response.message;
      return false;
    }
    return true;
  }

} // namespace jhzx::center_articulation_planner
