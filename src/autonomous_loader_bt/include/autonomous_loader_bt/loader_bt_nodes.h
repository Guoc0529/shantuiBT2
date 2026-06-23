#ifndef AUTONOMOUS_LOADER_BT_NODES_H
#define AUTONOMOUS_LOADER_BT_NODES_H

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <queue>
#include <mutex>
#include <map>
#include <atomic>
#include <autonomous_loader_msgs/NavigateAction.h>
#include <std_msgs/UInt8.h>
#include <shuju/TaskCtrl.h>

namespace autonomous_loader_bt
{

// 任务结构体
struct Task
{
    int task_id;
    int bin_id;
    int hopper_id;
    int task_type;
    std::string task_type_str;

    Task() : task_id(0), bin_id(0), hopper_id(0), task_type(0), task_type_str("auto") {}
    Task(int id, int bin, int hopper, int type, const std::string& type_str = "scoop")
        : task_id(id), bin_id(bin), hopper_id(hopper), task_type(type), task_type_str(type_str) {}
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
    void setStartTask(bool start);
    void setFirstTaskWaitingCtrlcmd(bool waiting);
    bool isFirstTaskWaitingCtrlcmd() const;
    void setEmergencyStop(bool stop);
    void setIsEnding(bool ending);
    void setHaltRequested(bool halt);
    bool isPauseTask() const;
    bool isEndTask() const;
    bool isCancelTask() const;
    bool isStartTask() const;
    bool isEmergencyStop() const;
    bool isEnding() const;
    bool isHaltRequested() const;
    void clearHaltRequested();

    // Task ID storage (int for atomic operations)
    void setTaskId(int task_id);
    int getTaskIdInt() const;
    
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
    
    // Arm/Bucket action code APIs
    void setArmBucketCode(int code);
    int getArmBucketCode() const;

    // Navigation action cancel confirmation
    void setNavCancelConfirmed(bool confirmed);
    bool isNavCancelConfirmed() const;

    // Work state APIs (2=auto, 3=temp)
    void setWorkState(int state);
    int getWorkState() const;

    // ===== 避障相关 API =====
    // obstacle_type: 0=无, 1=obstacle_1, 2=obstacle_2, 3=backstart
    void setObstacleType(int type);
    int getObstacleType() const;
    void setObstacleTarget(const geometry_msgs::PoseStamped& target);
    geometry_msgs::PoseStamped getObstacleTarget() const;
    void setRestoreRequested(bool restore);
    bool isRestoreRequested() const;
    void setObstacleTriggered(bool triggered);
    bool isObstacleTriggered() const;
    // obstacle_pending: 收到 ctrlCmd=3 后标记，等待当前任务完成
    void setObstaclePending(bool pending);
    bool isObstaclePending() const;
    // obstacle_from_ctrl3: 标记是否来自 ctrlCmd=3 触发的避障
    void setObstacleFromCtrl3(bool from_ctrl3);
    bool isObstacleFromCtrl3() const;
    void setObstacleCtrl3Immediate(bool immediate);
    bool isObstacleCtrl3Immediate() const;
    void setObstacleCtrl3Acknowledged(bool ack);
    bool isObstacleCtrl3Acknowledged() const;

    // ===== 遥控接管相关 API =====
    void setRemoteControlSnapshot(int work_state, const geometry_msgs::PoseStamped& pose);
    void clearRemoteControlSnapshot();
    bool hasRemoteControlSnapshot() const;
    int getSnapshotWorkState() const;
    geometry_msgs::PoseStamped getSnapshotPose() const;
    void setRemoteControlActive(bool active);
    bool isRemoteControlActive() const;
    void setRemoteControlRecovering(bool recovering);
    bool isRemoteControlRecovering() const;

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
    bool start_task_ = false;
    bool emergency_stop_ = false;
    bool is_ending_ = false;
    bool halt_requested_ = false;
    
    bool navigation_arrived_ = false;
    double distance_to_goal_ = 1000.0;
    bool nav_cancel_confirmed_ = false;  // set true when action done callback fires after cancel
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
    
    // Arm/Bucket action code
    int arm_bucket_code_ = 0;

    // Work state (2=auto working, 3=temp working)
    int work_state_ = 2;  // default to auto working

    // 第一个任务等待 ctrlCmd=1 标志
    bool first_task_waiting_ctrlcmd_ = true;  // true=第一个任务需要等待 ctrlCmd=1

    // 避障相关状态
    int obstacle_type_ = 0;  // 0=无, 1=obstacle_1, 2=obstacle_2, 3=backstart
    geometry_msgs::PoseStamped obstacle_target_;
    bool restore_requested_ = false;
    bool obstacle_triggered_ = false;
    bool obstacle_pending_ = false;  // 收到 ctrlCmd=3，等待任务完成后执行避障
    bool obstacle_from_ctrl3_ = false;  // 标记是否来自 ctrlCmd=3
    bool obstacle_ctrl3_immediate_ = false;  // ctrlCmd=3 即时响应标志（鸣笛双闪）
    bool obstacle_ctrl3_acknowledged_ = false;  // ctrlCmd=3 即时响应已执行

    // 遥控接管相关状态
    bool remote_control_active_ = false;  // 是否处于遥控接管状态
    bool remote_control_recovering_ = false;  // 是否正在恢复遥控接管
    bool has_remote_control_snapshot_ = false;  // 是否有遥控接管前的快照
    int snapshot_work_state_ = 1;  // 遥控接管前的工作状态
    geometry_msgs::PoseStamped snapshot_pose_;  // 遥控接管前的位置快照

    // Current task ID for status reporting
    std::atomic<int> current_task_id_{-1};
};

// ========== 任务状态上报器 ==========
// 功能：车端每 1s 上报一次任务状态，每次更改任务状态立即上报一次
class TaskStatusReporter
{
public:
    static TaskStatusReporter& instance() {
        static TaskStatusReporter inst;
        return inst;
    }

    // 任务状态枚举
    enum State {
        NOT_STARTED = 0,      // 未启动
        IDLE = 1,             // 空闲中
        AUTO_WORKING = 2,     // 自动作业中
        TEMP_WORKING = 3,     // 临时作业中
        EMERGENCY_STOP = 4,    // 紧急停车中
        OBSTACLE_STOP = 5,    // 避障停车中
        REMOTE_CONTROL = 6,    // 远程接管中
        ENDING = 7,           // 结束作业中
        ENDED = 8             // 已结束
    };

    // 初始化（需要在 ros::init 之后调用）
    void init(ros::NodeHandle& nh) {
        if (initialized_) return;

        status_pub_ = nh.advertise<std_msgs::Int32MultiArray>("/vehicle/task_status", 10, true);
        timer_ = nh.createTimer(ros::Duration(1.0), &TaskStatusReporter::timerCallback, this);
        initialized_ = true;
        setState(TaskStatusReporter::IDLE);
        ROS_INFO("[TaskStatusReporter] Initialized, publishing to /vehicle/task_status");
    }

    // 设置状态（状态变更立即上报）
    void setState(int state, const std::string& task_id = "") {
        int old_state = current_state_.load();
        current_state_.store(state);
        
        if (!task_id.empty()) {
            task_id_.store(std::stoi(task_id));
        }
        
        if (old_state != state) {
            // ROS_INFO("[TaskStatusReporter] State changed: %d -> %d", old_state, state);
            report();  // 状态变更立即上报
        }
    }

    // 设置任务编号
    void setTaskId(int task_id) {
        task_id_.store(task_id);
    }

    // 获取当前状态
    int getState() {
        return current_state_.load();
    }

    // 获取当前任务编号
    int getTaskIdInt() {
        return task_id_.load();
    }

private:
    TaskStatusReporter() : current_state_(0), task_id_(-1), initialized_(false) {}

    TaskStatusReporter(const TaskStatusReporter&) = delete;
    TaskStatusReporter& operator=(const TaskStatusReporter&) = delete;

    void timerCallback(const ros::TimerEvent&) {
        report();
    }

    void report() {
        std_msgs::Int32MultiArray msg;
        msg.data.resize(4);
        msg.data[0] = current_state_.load();                    // 状态码
        msg.data[1] = static_cast<int32_t>(ros::Time::now().sec);  // Unix 时间戳
        msg.data[2] = task_id_.load();                          // 任务编号
        msg.data[3] = 0;                                         // 预留
        status_pub_.publish(msg);
    }

    ros::Publisher status_pub_;
    ros::Timer timer_;
    std::atomic<int> current_state_;
    std::atomic<int> task_id_;
    bool initialized_;
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

// 检查 start_flag，等待 ctrlCmd=1 开始任务
class CheckStartTask : public BT::ConditionNode
{
public:
    CheckStartTask(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
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

// ===== 避障相关节点 =====
class CheckObstacleTriggered : public BT::ConditionNode
{
public:
    CheckObstacleTriggered(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class CheckRestoreRequested : public BT::ConditionNode
{
public:
    CheckRestoreRequested(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

// 检查是否有待执行的避障任务（ctrlCmd=3 触发，等待任务完成后执行）
class CheckObstaclePending : public BT::ConditionNode
{
public:
    CheckObstaclePending(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class HornAndHazard : public BT::SyncActionNode
{
public:
    HornAndHazard(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

// 检查 ctrlCmd=3 触发标志并执行即时响应（鸣笛+双闪）
class CheckCtrl3Immediate : public BT::SyncActionNode
{
public:
    CheckCtrl3Immediate(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class HazardLightsOff : public BT::SyncActionNode
{
public:
    HazardLightsOff(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class SetObstacleTarget : public BT::SyncActionNode
{
public:
    SetObstacleTarget(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

// 将 obstacle_pending 转换为 obstacle_triggered（表示正在执行避障）
class ActivateObstaclePending : public BT::SyncActionNode
{
public:
    ActivateObstaclePending(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

// 简单设置 obstacle_triggered=true（由 CheckCtrl3Immediate 在任务结束后触发外层导航）
class TriggerObstacleAvoidance : public BT::SyncActionNode
{
public:
    TriggerObstacleAvoidance(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class NavigateToObstacle : public BT::StatefulActionNode
{
public:
    NavigateToObstacle(const std::string& name, const BT::NodeConfiguration& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ros::Time start_time_;
};

class WaitForRestore : public BT::ConditionNode
{
public:
    WaitForRestore(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

class ClearRestoreRequested : public BT::SyncActionNode
{
public:
    ClearRestoreRequested(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts() { return {}; }
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

class CancelNavigation : public BT::StatefulActionNode
{
public:
    CancelNavigation(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), cancelled_(false) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("wait_s", 0.5, "Seconds to wait after cancel before returning SUCCESS") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override { cancelled_ = false; }
private:
    bool cancelled_;
    ros::Time start_time_;
    ros::Duration wait_duration_;
};

class WaitForNavigationCancelComplete : public BT::StatefulActionNode
{
public:
    WaitForNavigationCancelComplete(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), waiting_(false) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("timeout_s", 2.0, "Max seconds to wait for cancel confirmation") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override { waiting_ = false; }
private:
    bool waiting_;
    ros::Time start_time_;
    ros::Duration timeout_duration_;
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

class PublishOverrideRequest : public BT::SyncActionNode
{
public:
    PublishOverrideRequest(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("phase", "", "当前作业阶段"),
            BT::InputPort<std::string>("message", "", "显示给操作员的信息")
        };
    }
    BT::NodeStatus tick() override;
private:
    ros::NodeHandle nh_;
    ros::Publisher override_request_pub_;
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

class WaitForManualOverride : public BT::StatefulActionNode
{
public:
    WaitForManualOverride(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout", 600.0, "接管超时时间（秒）") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    void interventionCallback(const std_msgs::Bool::ConstPtr& msg);
    ros::NodeHandle nh_;
    ros::Subscriber intervention_sub_;
    ros::Publisher vehicle_pose_pub_;
    ros::Publisher estop_pub_;
    ros::Time start_time_;
    double timeout_s_ = 600.0;
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

// ========== 遥控接管相关节点 ==========

// 订阅 vcu_drive_mode，检测遥控接管状态
class VcuDriveModeListener : public BT::SyncActionNode
{
public:
    VcuDriveModeListener(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
private:
    void driveModeCallback(const std_msgs::UInt8::ConstPtr& msg);
    ros::NodeHandle nh_;
    ros::Subscriber vcu_drive_mode_sub_;
    int current_drive_mode_;
    ros::Time last_drive_mode_time_;
};

// 遥控接管时保存快照并发送急停
class SaveRemoteControlSnapshot : public BT::SyncActionNode
{
public:
    SaveRemoteControlSnapshot(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
private:
    ros::NodeHandle nh_;
    ros::Publisher estop_pub_;
};

// 遥控接管恢复检测
class CheckRemoteControlRecovery : public BT::ConditionNode
{
public:
    CheckRemoteControlRecovery(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts() { return {}; }
    BT::NodeStatus tick() override;
};

// 遥控接管恢复动作
class RecoverFromRemoteControl : public BT::StatefulActionNode
{
public:
    RecoverFromRemoteControl(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout", 30.0, "恢复超时时间（秒）") };
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    ros::NodeHandle nh_;
    ros::Publisher estop_pub_;
    ros::Publisher task_ctrl_pub_;
    ros::Subscriber task_status_sub_;
    ros::Time start_time_;
    double timeout_s_ = 30.0;
    bool position_ok_ = false;
    bool vehicle_idle_ = false;
    bool task_canceled_ = false;
    bool estop_released_ = false;

    void taskStatusCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
};

// ========== 注册函数 ==========
void RegisterNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh);

} // namespace autonomous_loader_bt

#endif // AUTONOMOUS_LOADER_BT_NODES_H
