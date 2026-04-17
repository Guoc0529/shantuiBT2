#ifndef AUTONOMOUS_LOADER_BT_NODES_H
#define AUTONOMOUS_LOADER_BT_NODES_H

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <queue>
#include <mutex>
#include <map>
#include <autonomous_loader_msgs/NavigateAction.h>

namespace autonomous_loader_bt
{

// 任务结构体
struct Task
{
    int task_id;
    int bin_id;
    int hopper_id;
    std::string task_type;
    
    Task() : task_id(0), bin_id(0), hopper_id(0) {}
    Task(int id, int bin, int hopper, const std::string& type)
        : task_id(id), bin_id(bin), hopper_id(hopper), task_type(type) {}
};

// 配置管理器
class ConfigManager
{
public:
    static ConfigManager& getInstance()
    {
        static ConfigManager instance;
        return instance;
    }
    
    bool loadConfig(const std::string& config_file);
    
    geometry_msgs::PoseStamped getBinCenter(int bin_id);
    geometry_msgs::PoseStamped getHopperPose(int hopper_id);
    geometry_msgs::PoseStamped getParkingPose(const std::string& parking_name = "main_parking");
    double getDistanceThreshold(const std::string& threshold_name);
    double getTimeout(const std::string& timeout_name);
    
private:
    ConfigManager() = default;
    bool parsePoseEntry(const YAML::Node& entry,
                        geometry_msgs::PoseStamped& pose,
                        const std::string& context);

    YAML::Node config_;
    bool config_loaded_ = false;
};

// 全局状态管理
class GlobalState
{
public:
    static GlobalState& getInstance()
    {
        static GlobalState instance;
        return instance;
    }
    
    void addTask(const Task& task);
    bool hasNewTask() const;
    Task getNextTask();
    void clearTasks();
    void setCurrentTask(const Task& task);
    void setPreviousTask(const Task& task);
    Task getCurrentTask() const;
    Task getPreviousTask() const;
    void clearCurrentTask();

    // Task start counter
    void incrementTasksStarted();
    int getTasksStarted() const;
    
    void setPauseTask(bool pause);
    void setEndTask(bool end);
    void setCancelTask(bool cancel);
    bool isPauseTask() const;
    bool isEndTask() const;
    bool isCancelTask() const;
    
    void setNavigationArrived(bool arrived);
    void setDistanceToGoal(double distance);
    void setCurrentPose(const geometry_msgs::PoseStamped& pose);

    bool isNavigationArrived() const;
    double getDistanceToGoal() const;
    geometry_msgs::PoseStamped getCurrentPose() const;
    
    // Distance reduction (segment-based) APIs
    void resetDistanceReduction();
    void updateDistanceReduction(double current_distance_to_goal);
    double getMaxDistanceReduction() const;
    
    void setScoopPoint(const geometry_msgs::PoseStamped& point);
    geometry_msgs::PoseStamped getScoopPoint() const;
    bool isScoopPointZero() const;
    
    void updateTaskProgress(const std::string& phase, float progress, const std::string& message);
    
    void resetNavigationArrived();
    void resetDistanceToGoal();

    void setNavigationResult(const autonomous_loader_msgs::NavigateResult& result);
    autonomous_loader_msgs::NavigateResult getNavigationResult() const;

    // Planning feedback APIs
    void setPlanningFeedback(bool success);
    bool hasReceivedPlanningFeedback() const;
    bool wasPlanningSuccessful() const;
    void resetPlanningFeedback();
    
private:
    GlobalState() = default;
    mutable std::mutex mutex_;
    
    std::queue<Task> task_queue_;
    Task current_task_;
    Task previous_task_;
    int tasks_started_ = 0;
    bool pause_task_ = false;
    bool end_task_ = false;
    bool cancel_task_ = false;
    
    bool navigation_arrived_ = false;
    double distance_to_goal_ = 1000.0;
    geometry_msgs::PoseStamped current_pose_;

    // Distance reduction tracking for current navigation segment
    double segment_start_distance_ = -1.0; // D0, <0 means uninitialized
    double segment_max_reduction_ = 0.0;   // max(D0 - D)
    
    geometry_msgs::PoseStamped scoop_point_;
    
    std::string current_phase_;
    float current_progress_ = 0.0;
    std::string status_message_;

    // Navigation action result
    autonomous_loader_msgs::NavigateResult nav_result_;

    // Planning feedback from Navigate.action
    bool planning_feedback_received_ = false;
    bool planning_successful_ = false;
};

// ========== 条件节点 ==========
class CheckEndTask : public BT::ConditionNode
{
public:
    CheckEndTask(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CheckPauseTask : public BT::ConditionNode
{
public:
    CheckPauseTask(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CheckCancelTask : public BT::ConditionNode
{
public:
    CheckCancelTask(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CheckEndTaskAfterCycle : public BT::ConditionNode
{
public:
    CheckEndTaskAfterCycle(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CheckNewTask : public BT::ConditionNode
{
public:
    CheckNewTask(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CheckNavigationArrived : public BT::ConditionNode
{
public:
    CheckNavigationArrived(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("message", "", "Context message for logging") }; }
    BT::NodeStatus tick() override;
};

class CheckDistanceThreshold : public BT::ConditionNode
{
public:
    CheckDistanceThreshold(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("threshold_name")}; }
    BT::NodeStatus tick() override;
};

class CheckScoopPointZero : public BT::ConditionNode
{
public:
    CheckScoopPointZero(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CheckForNextCycle : public BT::ConditionNode
{
public:
    CheckForNextCycle(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CheckDistanceSinceScoop : public BT::ConditionNode
{
public:
    CheckDistanceSinceScoop(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("distance_gt", 0.0, "Distance threshold (legacy)"),
            BT::InputPort<std::string>("threshold_name", "", "Config threshold name")
        };
    }
    BT::NodeStatus tick() override;
};

class CheckDistanceFromLastHopper : public BT::ConditionNode
{
public:
    CheckDistanceFromLastHopper(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("distance_gt", 0.0, "Distance threshold (legacy)"),
            BT::InputPort<std::string>("threshold_name", "", "Config threshold name")
        };
    }
    BT::NodeStatus tick() override;
};

// Check if ArmBucketState has already reached 10 (operation completed)
class CheckArmBucketDone : public BT::ConditionNode
{
public:
    CheckArmBucketDone(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<int>("expect_code", 3, "Expected last ABS command code to consider done") }; }
    BT::NodeStatus tick() override;
};

class CheckArmBucketCommandSent : public BT::ConditionNode
{
public:
    CheckArmBucketCommandSent(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<int>("expect_code", 3, "Expected last ABS command code to check if sent") }; }
    BT::NodeStatus tick() override;
};

class IsFirstTask : public BT::ConditionNode
{
public:
    IsFirstTask(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class BlackboardCheck : public BT::ConditionNode
{
public:
    BlackboardCheck(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts(){return {BT::InputPort<std::string>("key"), BT::InputPort<bool>("expected")};}
    BT::NodeStatus tick() override;
};

class CheckPlanningSuccess : public BT::ConditionNode
{
public:
    CheckPlanningSuccess(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<bool>("plan_succeeded") }; }
    BT::NodeStatus tick() override;
};

class CheckArrivalQuality : public BT::ConditionNode
{
public:
    CheckArrivalQuality(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("distance_error"),
                 BT::InputPort<double>("angle_error"),
                 BT::OutputPort<bool>("quality_ok") };
    }
    BT::NodeStatus tick() override;
};

class CheckErrorCode : public BT::ConditionNode
{
public:
    CheckErrorCode(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class CheckNavStatus : public BT::ConditionNode
{
public:
    CheckNavStatus(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<int>("status"), BT::InputPort<int>("expected") }; }
    BT::NodeStatus tick() override;
};


// ========== 动作节点 ==========
class SetWorkState : public BT::SyncActionNode
{
public:
    SetWorkState(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<int>("value") }; }
    BT::NodeStatus tick() override;
};

class SetArmBucketState : public BT::SyncActionNode
{
public:
    SetArmBucketState(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return { BT::InputPort<int>("code") }; }
    BT::NodeStatus tick() override;
};

class WaitForArmBucketCompletion : public BT::StatefulActionNode
{
public:
    WaitForArmBucketCompletion(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() { 
        return { 
            BT::InputPort<double>("timeout_s", 0.0, "Timeout in seconds"),
            BT::InputPort<std::string>("message", "", "Context message for logging")
        }; 
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ros::Time start_time_;
    double timeout_s_ = 0.0;
};

class StartPointAccumulation : public BT::SyncActionNode
{
public:
    StartPointAccumulation(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class StopPointAccumulation : public BT::SyncActionNode
{
public:
    StopPointAccumulation(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class SetPointAccumulationStart : public BT::SyncActionNode
{
public:
    SetPointAccumulationStart(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class WaitForPointAccumulationDone : public BT::StatefulActionNode
{
public:
    WaitForPointAccumulationDone(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() { 
        return { 
            BT::InputPort<double>("timeout_s", 0.0, "Timeout in seconds"),
            BT::InputPort<std::string>("message", "", "Context message for logging")
        }; 
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ros::Time start_time_;
    double timeout_s_ = 0.0;
};

class RequestScoopPoint : public BT::SyncActionNode
{
public:
    RequestScoopPoint(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class SetGoalToBinCenter : public BT::SyncActionNode
{
public:
    SetGoalToBinCenter(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {BT::OutputPort<geometry_msgs::PoseStamped>("goal")}; }
    BT::NodeStatus tick() override;
};

class ResetPlanningFeedback : public BT::SyncActionNode
{
public:
    ResetPlanningFeedback(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class WaitForPlanningFeedback : public BT::StatefulActionNode
{
public:
    WaitForPlanningFeedback(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ros::Time start_time_;
    double timeout_s_ = 2.0; // Default timeout
};

class SendNavigationGoal : public BT::SyncActionNode
{
public:
    SendNavigationGoal(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {BT::InputPort<geometry_msgs::PoseStamped>("goal"), BT::InputPort<int>("status")}; }
    BT::NodeStatus tick() override;
};

class SetGoalToScoopPoint : public BT::SyncActionNode
{
public:
    SetGoalToScoopPoint(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {BT::OutputPort<geometry_msgs::PoseStamped>("goal")}; }
    BT::NodeStatus tick() override;
};

class SetGoalToHopper : public BT::SyncActionNode
{
public:
    SetGoalToHopper(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {BT::OutputPort<geometry_msgs::PoseStamped>("goal")}; }
    BT::NodeStatus tick() override;
};

class SetGoalToParking : public BT::SyncActionNode
{
public:
    SetGoalToParking(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {BT::OutputPort<geometry_msgs::PoseStamped>("goal")}; }
    BT::NodeStatus tick() override;
};

class CancelNavigation : public BT::SyncActionNode
{
public:
    CancelNavigation(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CancelArmBucketAction : public BT::SyncActionNode
{
public:
    CancelArmBucketAction(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CommitToEndMission : public BT::SyncActionNode
{
public:
    CommitToEndMission(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class ClearCurrentTask : public BT::SyncActionNode
{
public:
    ClearCurrentTask(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class PauseNavigation : public BT::SyncActionNode
{
public:
    PauseNavigation(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class UpdateTaskProgress : public BT::SyncActionNode
{
public:
    UpdateTaskProgress(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("phase"), BT::InputPort<float>("progress"), BT::InputPort<std::string>("message")}; }
    BT::NodeStatus tick() override;
};

class WaitForNavigationArrival : public BT::StatefulActionNode
{
public:
    WaitForNavigationArrival(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() { 
        return { BT::InputPort<std::string>("message", "", "Context message for logging") }; 
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

class GetAndSetCurrentTask : public BT::SyncActionNode
{
public:
    GetAndSetCurrentTask(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class ResetNavigationArrived : public BT::SyncActionNode
{
public:
    ResetNavigationArrived(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class ResetDistanceReduction : public BT::SyncActionNode
{
public:
    ResetDistanceReduction(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class ResetDistanceToGoal : public BT::SyncActionNode
{
public:
    ResetDistanceToGoal(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class RequestManualIntervention : public BT::SyncActionNode
{
public:
    RequestManualIntervention(const std::string& name, const BT::NodeConfiguration& config);
        static BT::PortsList providedPorts() 
    {
        return { BT::InputPort<std::string>("message"),
                 BT::InputPort<int>("current_status", -1, "The current navigation status code") };
    }
    BT::NodeStatus tick() override;
private:
    ros::NodeHandle nh_;
    ros::Publisher estop_pub_;
    ros::Publisher status_pub_;
};

class WaitForManualIntervention : public BT::StatefulActionNode
{
public:
    WaitForManualIntervention(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    void interventionCallback(const std_msgs::Bool::ConstPtr& msg);
    ros::NodeHandle nh_;
    ros::Subscriber intervention_sub_;
    ros::Publisher estop_pub_;
    std::atomic<bool> intervention_complete_;
};

class WaitForNavigationResult : public BT::StatefulActionNode
{
public:
    WaitForNavigationResult(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<bool>("success"),
                 BT::OutputPort<double>("final_distance_error"),
                 BT::OutputPort<double>("final_angle_error"),
                 BT::OutputPort<uint8_t>("error_code"),
                 BT::OutputPort<std::string>("error_msg") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};


// ========== 注册函数 ==========
void RegisterNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh);

} // namespace autonomous_loader_bt

#endif // AUTONOMOUS_LOADER_BT_NODES_H
