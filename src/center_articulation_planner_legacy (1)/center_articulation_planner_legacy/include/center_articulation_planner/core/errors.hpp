#pragma once

#include <cstdint>

namespace jhzx::center_articulation_planner
{
  // 统一的错误码定义（从 20 开始）
  enum class ErrorCode : std::uint8_t
  {
    BackendNull = 20,               // 规划后端指针为空
    PlanServiceUnavailable = 21,    // 规划服务不可用/调用失败
    PlanFailed = 22,                // 规划服务返回失败
    EmptyPath = 23,                 // 规划结果为空
    SegmentBuildFailed = 24,        // 路径分段失败
    NavPathUnavailable = 25,        // nav_path_ 为空
    MissingDirectionFlags = 26,     // 缺少方向标志
    DirectionSwitchTimeout = 27,    // 换向反馈超时
    DirectionSwitchRejected = 28,   // 节点拒绝换向
    FinalGoalCheckFailed = 29,      // 最终目标校验失败
    GoalReachedRejected = 30,       // goal_reached 反馈为 false
    NoDistanceFeedback = 31         // 无外部距离反馈
  };

  inline const char *defaultErrorMessage(ErrorCode code)
  {
    switch (code)
    {
    case ErrorCode::BackendNull:
      return "remote planner backend is null";
    case ErrorCode::PlanServiceUnavailable:
      return "plan service unavailable";
    case ErrorCode::PlanFailed:
      return "plan failed";
    case ErrorCode::EmptyPath:
      return "planner sends empty path";
    case ErrorCode::SegmentBuildFailed:
      return "segment path build failed";
    case ErrorCode::NavPathUnavailable:
      return "segment path list empty";
    case ErrorCode::MissingDirectionFlags:
      return "plan missing direction flags";
    case ErrorCode::DirectionSwitchTimeout:
      return "direction switch feedback timeout";
    case ErrorCode::DirectionSwitchRejected:
      return "direction switch rejected";
    case ErrorCode::FinalGoalCheckFailed:
      return "final goal check did not pass";
    case ErrorCode::GoalReachedRejected:
      return "goal_reached reported false";
    case ErrorCode::NoDistanceFeedback:
      return "distance topic no info";
    default:
      return "unknown error";
    }
  }
} // namespace jhzx::center_articulation_planner
