#include "autonomous_loader_bt/loader_bt_nodes.h"
#include "autonomous_loader_bt/ros_topic_manager.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <cmath>

namespace autonomous_loader_bt
{

// ========== 配置管理器实现 ==========

bool ConfigManager::loadConfig(const std::string& config_file)
{
    try {
        config_ = YAML::LoadFile(config_file);
        config_loaded_ = true;
        ROS_INFO("Successfully loaded config file: %s", config_file.c_str());
        return true;
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load config file: %s", e.what());
        return false;
    }
}

bool ConfigManager::parsePoseEntry(const YAML::Node& entry,
                                   geometry_msgs::PoseStamped& pose,
                                   const std::string& context)
{
    if (!entry) {
        return false;
    }

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = entry["frame_id"] ? entry["frame_id"].as<std::string>() : "map";

    auto fill_position = [&](const YAML::Node& node) -> bool {
        if (!node || !node["x"] || !node["y"]) {
            return false;
        }
        pose.pose.position.x = node["x"].as<double>();
        pose.pose.position.y = node["y"].as<double>();
        pose.pose.position.z = node["z"] ? node["z"].as<double>() : 0.0;
        return true;
    };

    auto fill_orientation_from_quat = [&](const YAML::Node& node) -> bool {
        if (!node || !node["x"] || !node["y"] || !node["z"] || !node["w"]) {
            return false;
        }
        pose.pose.orientation.x = node["x"].as<double>();
        pose.pose.orientation.y = node["y"].as<double>();
        pose.pose.orientation.z = node["z"].as<double>();
        pose.pose.orientation.w = node["w"].as<double>();
        return true;
    };

    auto fill_orientation_from_yaw = [&](const YAML::Node& node) -> bool {
        if (!node || !node["yaw"]) {
            return false;
        }
        double yaw = node["yaw"].as<double>();
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
        return true;
    };

    // Preferred: ROS PoseStamped layout
    const YAML::Node pose_node = entry["pose"];
    bool position_ok = false;
    bool orientation_ok = false;
    if (pose_node) {
        if (pose_node["position"] || pose_node["orientation"]) {
            position_ok = fill_position(pose_node["position"] ? pose_node["position"] : pose_node);
            orientation_ok = fill_orientation_from_quat(pose_node["orientation"])
                             || fill_orientation_from_yaw(pose_node);
        } else {
            position_ok = fill_position(pose_node);
            orientation_ok = fill_orientation_from_yaw(pose_node);
        }
    }

    // Legacy flat format fallback
    if (!position_ok) {
        position_ok = fill_position(entry);
    }
    if (!orientation_ok) {
        orientation_ok = fill_orientation_from_quat(entry)
                         || fill_orientation_from_yaw(entry);
    }

    if (!position_ok || !orientation_ok) {
        ROS_WARN("Failed to parse pose for %s (position_ok=%d, orientation_ok=%d)",
                 context.c_str(), static_cast<int>(position_ok), static_cast<int>(orientation_ok));
    }

    return position_ok && orientation_ok;
}

geometry_msgs::PoseStamped ConfigManager::getBinCenter(int bin_id)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    
    if (!config_loaded_) {
        ROS_WARN("Config file not loaded, using default bin location");
        pose.pose.position.x = 10.0;
        pose.pose.position.y = 10.0;
        tf2::Quaternion q; q.setRPY(0, 0, 0);
        pose.pose.orientation = tf2::toMsg(q);
        return pose;
    }
    
    std::string cang_key = std::to_string(bin_id);
    if (config_["cang"][cang_key]) {
        if (!parsePoseEntry(config_["cang"][cang_key], pose, "cang[" + cang_key + "]")) {
            ROS_WARN("Bin %d pose parse failed, falling back to default", bin_id);
            pose.pose.position.x = 10.0;
            pose.pose.position.y = 10.0;
            tf2::Quaternion q; q.setRPY(0, 0, 0);
            pose.pose.orientation = tf2::toMsg(q);
        }
    } else {
        ROS_WARN("Bin %d config not found, using default location", bin_id);
        pose.pose.position.x = 10.0;
        pose.pose.position.y = 10.0;
        tf2::Quaternion q; q.setRPY(0, 0, 0);
        pose.pose.orientation = tf2::toMsg(q);
    }
    return pose;
}

geometry_msgs::PoseStamped ConfigManager::getHopperPose(int hopper_id)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    
    if (!config_loaded_) {
        ROS_WARN("Config file not loaded, using default hopper location");
        pose.pose.position.x = 5.0;
        pose.pose.position.y = 5.0;
        tf2::Quaternion q; q.setRPY(0, 0, 0);
        pose.pose.orientation = tf2::toMsg(q);
        return pose;
    }
    
    std::string dou_key = std::to_string(hopper_id);
    if (config_["dou"][dou_key]) {
        if (!parsePoseEntry(config_["dou"][dou_key], pose, "dou[" + dou_key + "]")) {
            ROS_WARN("Hopper %d pose parse failed, falling back to default", hopper_id);
            pose.pose.position.x = 5.0;
            pose.pose.position.y = 5.0;
            tf2::Quaternion q; q.setRPY(0, 0, 0);
            pose.pose.orientation = tf2::toMsg(q);
        }
    } else {
        ROS_WARN("Hopper %d config not found, using default location", hopper_id);
        pose.pose.position.x = 5.0;
        pose.pose.position.y = 5.0;
        tf2::Quaternion q; q.setRPY(0, 0, 0);
        pose.pose.orientation = tf2::toMsg(q);
    }
    return pose;
}

geometry_msgs::PoseStamped ConfigManager::getParkingPose(const std::string& parking_name)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    
    if (!config_loaded_) {
        ROS_WARN("Config file not loaded, using default parking location");
        tf2::Quaternion q; q.setRPY(0, 0, 0);
        pose.pose.orientation = tf2::toMsg(q);
        return pose;
    }
    
    if (config_["parking"][parking_name]) {
        if (!parsePoseEntry(config_["parking"][parking_name], pose, "parking[" + parking_name + "]")) {
            ROS_WARN("Parking %s pose parse failed, using default", parking_name.c_str());
            tf2::Quaternion q; q.setRPY(0, 0, 0);
            pose.pose.orientation = tf2::toMsg(q);
        }
    } else {
        ROS_WARN("Parking point %s config not found, using default location", parking_name.c_str());
        tf2::Quaternion q; q.setRPY(0, 0, 0);
        pose.pose.orientation = tf2::toMsg(q);
    }
    return pose;
}

double ConfigManager::getDistanceThreshold(const std::string& threshold_name)
{
    if (config_loaded_ && config_["work_parameters"]["distance_thresholds"][threshold_name]) {
        return config_["work_parameters"]["distance_thresholds"][threshold_name].as<double>();
    }
    ROS_WARN("Distance threshold '%s' not found, using default 6.0m", threshold_name.c_str());
    return 6.0;
}

double ConfigManager::getTimeout(const std::string& timeout_name)
{
    if (config_loaded_ && config_["work_parameters"]["timeouts"][timeout_name]) {
        return config_["work_parameters"]["timeouts"][timeout_name].as<double>();
    }
    ROS_WARN("Timeout '%s' not found, using default 60.0s", timeout_name.c_str());
    return 60.0;
}

// ========== 全局状态管理实现 ==========

void GlobalState::addTask(const Task& task)
{
    std::lock_guard<std::mutex> lock(mutex_);
    task_queue_.push(task);
}

void GlobalState::incrementTasksStarted()
{
    std::lock_guard<std::mutex> lock(mutex_);
    tasks_started_++;
}

int GlobalState::getTasksStarted() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return tasks_started_;
}

bool GlobalState::hasNewTask() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return !task_queue_.empty();
}

Task GlobalState::getNextTask()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (task_queue_.empty()) { return Task(); }
    Task task = task_queue_.front();
    task_queue_.pop();
    return task;
}

void GlobalState::clearTasks()
{
    std::lock_guard<std::mutex> lock(mutex_);
    std::queue<Task> empty;
    task_queue_.swap(empty);
}

void GlobalState::setCurrentTask(const Task& task)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_task_ = task;
}

void GlobalState::setPreviousTask(const Task& task)
{
    std::lock_guard<std::mutex> lock(mutex_);
    previous_task_ = task;
}

void GlobalState::clearCurrentTask()
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_task_ = Task(); // Reset to default task
}

Task GlobalState::getCurrentTask() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return current_task_;
}

Task GlobalState::getPreviousTask() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return previous_task_;
}

void GlobalState::setPauseTask(bool pause)
{
    std::lock_guard<std::mutex> lock(mutex_);
    pause_task_ = pause;
}

void GlobalState::setEndTask(bool end)
{
    std::lock_guard<std::mutex> lock(mutex_);
    end_task_ = end;
}

void GlobalState::setCancelTask(bool cancel)
{
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_task_ = cancel;
}

bool GlobalState::isPauseTask() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return pause_task_;
}

bool GlobalState::isEndTask() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return end_task_;
}

bool GlobalState::isCancelTask() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return cancel_task_;
}

void GlobalState::setNavigationArrived(bool arrived)
{
    std::lock_guard<std::mutex> lock(mutex_);
    navigation_arrived_ = arrived;
}

void GlobalState::setDistanceToGoal(double distance)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (distance >= 0.0) { distance_to_goal_ = distance; }
}

void GlobalState::setCurrentPose(const geometry_msgs::PoseStamped& pose)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = pose;
}

bool GlobalState::isNavigationArrived() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return navigation_arrived_;
}

double GlobalState::getDistanceToGoal() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return distance_to_goal_;
}

geometry_msgs::PoseStamped GlobalState::getCurrentPose() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return current_pose_;
}

void GlobalState::setScoopPoint(const geometry_msgs::PoseStamped& point)
{
    std::lock_guard<std::mutex> lock(mutex_);
    scoop_point_ = point;
}

geometry_msgs::PoseStamped GlobalState::getScoopPoint() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return scoop_point_;
}

bool GlobalState::isScoopPointZero() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return (std::abs(scoop_point_.pose.position.x) < 1e-3 && 
            std::abs(scoop_point_.pose.position.y) < 1e-3);
}

void GlobalState::updateTaskProgress(const std::string& phase, float progress, const std::string& message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_phase_ = phase;
    current_progress_ = progress;
    status_message_ = message;
}

void GlobalState::resetNavigationArrived()
{
    std::lock_guard<std::mutex> lock(mutex_);
    navigation_arrived_ = false;
}

void GlobalState::resetDistanceToGoal()
{
    std::lock_guard<std::mutex> lock(mutex_);
    distance_to_goal_ = 1000.0;
    ROS_INFO("Reset distance_to_goal to 1000.0");
}

void GlobalState::resetDistanceReduction()
{
    std::lock_guard<std::mutex> lock(mutex_);
    segment_start_distance_ = -1.0;
    segment_max_reduction_ = 0.0;
    ROS_INFO("[DistReduce] Reset segment distance reduction trackers");
}

void GlobalState::updateDistanceReduction(double current_distance_to_goal)
{
    // Called whenever a new /distance_to_goal message is received

    std::lock_guard<std::mutex> lock(mutex_);
    if (current_distance_to_goal < 0.0 || !std::isfinite(current_distance_to_goal)) {
        return;
    }
    // initialize only when we receive a sane first reading (0 < d < 900)
    if (segment_start_distance_ < 0.0 &&
        planning_feedback_received_ && planning_successful_ &&
        current_distance_to_goal > 1.0 && current_distance_to_goal < 900.0) {
        segment_start_distance_ = current_distance_to_goal;
        segment_max_reduction_  = 0.0;
        ROS_INFO("[DistReduce] Segment start distance set: %.2f m", segment_start_distance_);
    }
    if (segment_start_distance_ < 0.0) {
        // still not initialized, skip
        return;
    }
    double reduction = segment_start_distance_ - current_distance_to_goal;
    if(reduction < 0) reduction = 0;
    if (reduction > segment_max_reduction_) {
        segment_max_reduction_ = reduction;
    }
    ROS_INFO_THROTTLE(1.0, "[DistReduce] delta=%.2f m (max=%.2f)", reduction, segment_max_reduction_);
}

double GlobalState::getMaxDistanceReduction() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return segment_max_reduction_;
}

void GlobalState::setNavigationResult(const autonomous_loader_msgs::NavigateResult& result)
{
    std::lock_guard<std::mutex> lock(mutex_);
    nav_result_ = result;
}

autonomous_loader_msgs::NavigateResult GlobalState::getNavigationResult() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return nav_result_;
}

// Planning feedback APIs
void GlobalState::setPlanningFeedback(bool success)
{
    std::lock_guard<std::mutex> lock(mutex_);
    planning_feedback_received_ = true;
    planning_successful_ = success;
}

bool GlobalState::hasReceivedPlanningFeedback() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return planning_feedback_received_;
}

bool GlobalState::wasPlanningSuccessful() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return planning_successful_;
}

void GlobalState::resetPlanningFeedback()
{
    std::lock_guard<std::mutex> lock(mutex_);
    planning_feedback_received_ = false;
    planning_successful_ = false;
    ROS_INFO("[BT] Planning feedback state has been reset.");
}

void GlobalState::setArmBucketCode(int code)
{
    std::lock_guard<std::mutex> lock(mutex_);
    arm_bucket_code_ = code;
}

int GlobalState::getArmBucketCode() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_bucket_code_;
}


// ========== 条件节点实现 ==========

BT::PortsList CheckErrorCode::providedPorts()
{
    return { BT::InputPort<uint8_t>("error_code"),
             BT::InputPort<std::string>("recoverable_codes", "", "Semicolon-separated list of error codes that allow automatic retry") };
}

BT::NodeStatus CheckErrorCode::tick()
{
    uint8_t error_code;
    if (!getInput("error_code", error_code)) {
        return BT::NodeStatus::FAILURE; // Default to intervention if code is not readable
    }

    if (error_code == 0) {
        return BT::NodeStatus::SUCCESS; // Success case
    }

    std::string recoverable_codes_str;
    getInput("recoverable_codes", recoverable_codes_str);

    std::stringstream ss(recoverable_codes_str);
    std::string item;
    bool is_recoverable = false;
    while (std::getline(ss, item, ';')) {
        try {
            if (!item.empty() && std::stoi(item) == error_code) {
                is_recoverable = true;
                break;
            }
        }
        catch (const std::exception& e) {
            ROS_ERROR("Invalid value in recoverable_codes string: '%s'", item.c_str());
        }
    }

    if (is_recoverable) {
        ROS_INFO("[BT] Recoverable error code received: %d. Attempting automatic retry.", error_code);
        return BT::NodeStatus::SUCCESS; // Allow retry
    } else {
        ROS_WARN("[BT] Non-recoverable error code received: %d. Manual intervention is required.", error_code);
        return BT::NodeStatus::FAILURE; // Request manual intervention
    }
}

BT::NodeStatus CheckNavStatus::tick()
{
    int status = 0, expected = 0;
    if (!getInput("status", status) || !getInput("expected", expected)) {
        return BT::NodeStatus::FAILURE;
    }
    return (status == expected) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}


BT::NodeStatus CheckEndTask::tick()
{
    return GlobalState::getInstance().isEndTask() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckPauseTask::tick()
{
    return GlobalState::getInstance().isPauseTask() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckCancelTask::tick()
{
    if (GlobalState::getInstance().isCancelTask())
    {
        // Once the cancellation is triggered, consume the flag.
        GlobalState::getInstance().setCancelTask(false);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckEndTaskAfterCycle::tick()
{
    // This node should only succeed if an end task is requested AND there are no more tasks in the queue.
    auto& gs = GlobalState::getInstance();
    if (gs.isEndTask() && !gs.hasNewTask())
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckNewTask::tick()
{
    return GlobalState::getInstance().hasNewTask() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckNavigationArrived::tick()
{
    std::string msg;
    getInput("message", msg);
    if (GlobalState::getInstance().isNavigationArrived()) {
        ROS_INFO("[NAV] Arrival confirmed for: %s", msg.c_str());
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckDistanceThreshold::tick()
{
    std::string threshold_name;
    if (!getInput("threshold_name", threshold_name)) { return BT::NodeStatus::FAILURE; }
    
    auto& state = GlobalState::getInstance();
    auto& config = ConfigManager::getInstance();
    
    double threshold = config.getDistanceThreshold(threshold_name);
    double current_distance = state.getDistanceToGoal();
    ROS_INFO("[CheckDist] Current distance: %.2f m, threshold: %.2f m", current_distance, threshold);
    return current_distance <= threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckScoopPointZero::tick()
{
    return GlobalState::getInstance().isScoopPointZero() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckForNextCycle::tick()
{
    return GlobalState::getInstance().hasNewTask() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckDistanceSinceScoop::tick()
{
    double distance_gt;
    std::string threshold_name;

    if (getInput("threshold_name", threshold_name) && !threshold_name.empty()) {
        auto& config = ConfigManager::getInstance();
        distance_gt = config.getDistanceThreshold(threshold_name);
        ROS_INFO("[CheckDistanceSinceScoop] Using config threshold '%s': %.2f", threshold_name.c_str(), distance_gt);
    } else if (!getInput("distance_gt", distance_gt)) {
        ROS_ERROR("[CheckDistanceSinceScoop] Missing required input 'distance_gt' or 'threshold_name'");
        return BT::NodeStatus::FAILURE;
    }

    auto& state = GlobalState::getInstance();
    double reduced = state.getMaxDistanceReduction();

    if (reduced >= distance_gt) {
        ROS_INFO("[CheckDist] Distance since scoop: %.2f m >= threshold %.2f m. Success.", reduced, distance_gt);
        return BT::NodeStatus::SUCCESS;
    }

    ROS_INFO_THROTTLE(1.0, "[CheckDist] Distance since scoop: %.2f m (threshold=%.2f m)", reduced, distance_gt);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckDistanceFromLastHopper::tick()
{
    double distance_gt;
    std::string threshold_name;

    if (getInput("threshold_name", threshold_name) && !threshold_name.empty()) {
        auto& config = ConfigManager::getInstance();
        distance_gt = config.getDistanceThreshold(threshold_name);
        ROS_INFO("[CheckDistanceFromLastHopper] Using config threshold '%s': %.2f", threshold_name.c_str(), distance_gt);
    } else if (!getInput("distance_gt", distance_gt)) {
        ROS_ERROR("[CheckDistanceFromLastHopper] Missing required input 'distance_gt' or 'threshold_name'");
        return BT::NodeStatus::FAILURE;
    }

    auto& state = GlobalState::getInstance();
    double reduced = state.getMaxDistanceReduction();

    static bool first_tick = true;
    if(first_tick)
    {
        ROS_INFO("[CheckDist delta] Node is active. distance_gt=%.2f", distance_gt);
        first_tick = false;
    }

    if (reduced >= distance_gt) {
        ROS_INFO("[CheckDist] Reduced distance since segment start: %.2f m >= threshold %.2f m. Success.", reduced, distance_gt);
        return BT::NodeStatus::SUCCESS;
    }
    ROS_INFO_THROTTLE(1.0, "[CheckDist delta] Reduced: %.2f m (th=%.2f)", reduced, distance_gt);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckArmBucketDone::tick()
{
    ros::NodeHandle nh("~");
    int last_cmd = 0;
    int abs_state = 0;
    int expect_code = 3;
    getInput("expect_code", expect_code);
    nh.getParam("LastABSCommand", last_cmd);
    nh.getParam("ArmBucketState", abs_state);
    if (last_cmd != expect_code) {
        return BT::NodeStatus::FAILURE;
    }
    bool done = (abs_state == 10);
    ROS_INFO_THROTTLE(1.0, "[ABS] CheckArmBucketDone: expect=%d, last_cmd=%d, state=%d -> %s", expect_code, last_cmd, abs_state, done?"DONE":"NOT DONE");
    return done ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckArmBucketCommandSent::tick()
{
    ros::NodeHandle nh("~");
    int last_cmd = 0;
    int expect_code = 3;
    getInput("expect_code", expect_code);
    nh.getParam("LastABSCommand", last_cmd);
    bool sent = (last_cmd == expect_code);
    ROS_INFO_THROTTLE(1.0, "[ABS] CheckArmBucketCommandSent: expect=%d, last_cmd=%d -> %s", expect_code, last_cmd, sent?"SENT":"NOT SENT");
    return sent ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsFirstTask::tick()
{
    int n = GlobalState::getInstance().getTasksStarted();
    return (n == 1) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}


// ========== 动作节点实现 ==========

BT::NodeStatus SetWorkState::tick()
{
    int value;
    if (!getInput("value", value)) {
        ROS_ERROR("[SetWorkState] Missing required input 'value'");
        return BT::NodeStatus::FAILURE;
    }
    ros::param::set("/workstate", value);
    ROS_INFO("[BT] Set /workstate parameter to %d", value);
    return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus SetArmBucketState::tick()
{
    int code;
    if (!getInput("code", code)) { return BT::NodeStatus::FAILURE; }
    ros::NodeHandle nh("~");
    nh.setParam("ArmBucketState", code);
    nh.setParam("LastABSCommand", code);
    if (code == 3) {
        nh.setParam("BucketLoweredReady", false);
    }
    ROS_INFO("[PARAM] Set ArmBucketState=%d", code);
    
    // 保存当前的 code 到 GlobalState
    GlobalState::getInstance().setArmBucketCode(code);
    
    // 发布"进行中"消息
    std::string status_msg;
    switch(code) {
        case 1: status_msg = "铲斗放平到地面进行中"; break;
        case 2: status_msg = "铲斗抬到运输位置进行中"; break;
        case 3: status_msg = "铲斗放平进行中"; break;
        case 4: status_msg = "举升大臂进行中"; break;
        case 5: status_msg = "降大臂进行中"; break;
        case 6: status_msg = "卸料进行中"; break;
        case 7: status_msg = "铲料进行中"; break;
        case 8: status_msg = "抖料进行中"; break;
        default: status_msg = "臂铲动作进行中"; break;
    }
    ROSTopicManager::getInstance().publishActionStatus(status_msg);
    
    return BT::NodeStatus::SUCCESS;
}

WaitForManualIntervention::WaitForManualIntervention(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), intervention_complete_(false)
{
    // 初始化 ROS 订阅者和发布者
    intervention_sub_ = nh_.subscribe("/autonomous_loader/manual_intervention_complete", 1, &WaitForManualIntervention::interventionCallback, this);
    // estop_pub_ = nh_.advertise<std_msgs::Bool>("/ACU_EStop", 1);
}

void WaitForManualIntervention::interventionCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) {
        ROS_INFO("[BT] Manual intervention complete signal received.");
        intervention_complete_ = true;
    }
}

BT::NodeStatus WaitForManualIntervention::onStart()
{
    intervention_complete_ = false;
    ROS_INFO("[BT] Waiting for manual intervention to complete...");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForManualIntervention::onRunning()
{
    if (intervention_complete_) {
        // 1. 重置参数
        ros::param::set("/autonomous_loader/manual_intervention_required", false);

        // 2. 解除急停
        std_msgs::Bool estop_msg;
        estop_msg.data = false;
        // estop_pub_.publish(estop_msg);
        ROS_INFO("[BT] Manual intervention finished. E-Stop released.");

        intervention_complete_ = false; // Reset the state for the next time
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void WaitForManualIntervention::onHalted()
{
    // 如果节点被中断，确保状态被重置
    intervention_complete_ = false;
}

BT::NodeStatus WaitForArmBucketCompletion::onStart()
{
    if (!getInput("timeout_s", timeout_s_) || timeout_s_ <= 0.0) {
        timeout_s_ = ConfigManager::getInstance().getTimeout("work_device_timeout");
    }
    start_time_ = ros::Time::now();
    
    std::string msg;
    getInput("message", msg);

    ROS_INFO("[WAIT] %s. Waiting for 'ArmBucketState' to be set to 10 (by Work Device). Timeout: %.1fs", msg.c_str(), timeout_s_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForArmBucketCompletion::onRunning()
{
    int value = 0;
    ros::NodeHandle nh("~");
    if (nh.getParam("ArmBucketState", value)) {
        if (value == 10) {
            std::string msg;
            getInput("message", msg);
            if (msg.empty()) { msg = "ArmBucket operation"; }
            ROS_INFO("[DONE] %s. ArmBucketState reached 10 (Action Completed)", msg.c_str());
            
            // 根据 GlobalState 中保存的 code 发送"完成"消息
            int code = GlobalState::getInstance().getArmBucketCode();
            std::string status_msg;
            switch(code) {
                case 1: status_msg = "铲斗放平到地面完成"; break;
                case 2: status_msg = "铲斗抬到运输位置完成"; break;
                case 3: status_msg = "铲斗放平完成"; break;
                case 4: status_msg = "举升大臂完成"; break;
                case 5: status_msg = "降大臂完成"; break;
                case 6: status_msg = "卸料完成"; break;
                case 7: status_msg = "铲料完成"; break;
                case 8: status_msg = "抖料完成"; break;
                default: status_msg = "臂铲动作完成"; break;
            }
            ROSTopicManager::getInstance().publishActionStatus(status_msg);
            
            return BT::NodeStatus::SUCCESS;
        }
        if (value < 0) {
            ROS_ERROR("WaitForArmBucketCompletion detected negative ArmBucketState=%d. Cancelling navigation and failing.", value);
            // Cancel any ongoing navigation goal to avoid continuing to move after device error
            // ROSTopicManager::getInstance().cancelMoveBaseGoal();
            return BT::NodeStatus::FAILURE;
        }
    }
    if ((ros::Time::now() - start_time_).toSec() > timeout_s_) {
        ROS_ERROR("WaitForArmBucketCompletion timeout after %.1fs", timeout_s_);
        // On timeout, also cancel navigation to keep system safe
        ROSTopicManager::getInstance().cancelMoveBaseGoal();
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void WaitForArmBucketCompletion::onHalted() {}

BT::NodeStatus StartPointAccumulation::tick()
{
    ros::NodeHandle nh;
    nh.setParam("/PointAccumulation", 1);
    ROS_INFO("BT: Point accumulation STARTED.");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StopPointAccumulation::tick()
{
    ros::NodeHandle nh;
    nh.setParam("/PointAccumulation", 0);
    ROS_INFO("BT: Point accumulation STOPPED.");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetPointAccumulationStart::tick()
{
    ros::NodeHandle nh("~");
    nh.setParam("PointAccumulation", 1);
    ROS_INFO("BT: SetPointAccumulationStart triggered. Setting PointAccumulation to 1.");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus WaitForPointAccumulationDone::onStart()
{
    if (!getInput("timeout_s", timeout_s_) || timeout_s_ <= 0.0) {
        timeout_s_ = ConfigManager::getInstance().getTimeout("accumulation_timeout");
    }
    start_time_ = ros::Time::now();

    std::string msg;
    getInput("message", msg);

    ROS_INFO("[WAIT] %s. Waiting for 'PointAccumulation' to be set to 10 (by Calculation Module). Timeout: %.1fs", msg.c_str(), timeout_s_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForPointAccumulationDone::onRunning()
{
    int value = 0;
    ros::NodeHandle nh("~");
    if (nh.getParam("PointAccumulation", value)) {
        if (value == 10) {
            ROS_INFO("[DONE] Point accumulation completed. PointAccumulation reached 10");
            ROSTopicManager::getInstance().publishActionStatus("点云累积完成");
            return BT::NodeStatus::SUCCESS;
        }
        if (value < 0) { return BT::NodeStatus::FAILURE; }
    }
    if ((ros::Time::now() - start_time_).toSec() > timeout_s_) {
        ROS_ERROR("WaitForPointAccumulationDone timeout after %.1fs", timeout_s_);
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void WaitForPointAccumulationDone::onHalted() {}

BT::NodeStatus RequestScoopPoint::tick()
{
    auto& state = GlobalState::getInstance();
    uint8_t pileID = static_cast<uint8_t>(state.getCurrentTask().bin_id);
    uint8_t hopperID = static_cast<uint8_t>(state.getCurrentTask().hopper_id);
    geometry_msgs::PoseStamped scoop_point;
    auto& topic_manager = ROSTopicManager::getInstance();
    
    if (topic_manager.requestScoopPoint(pileID, scoop_point)) {
        if (std::abs(scoop_point.pose.position.x) > 1e-3 || std::abs(scoop_point.pose.position.y) > 1e-3) {
            scoop_point.pose.position.x -= 1.0;
        }
        state.setScoopPoint(scoop_point);
        ROS_INFO("[SCOOP] Successfully got scoop point for pile %d", static_cast<int>(pileID));
        return BT::NodeStatus::SUCCESS;
    } else {
        // state.setScoopPoint(ConfigManager::getInstance().getBinCenter(pileID));
        ROS_INFO_THROTTLE(1.0, "[SCOOP] Waiting for scoop point for pile %d...", static_cast<int>(pileID));
        return BT::NodeStatus::RUNNING;
    }
}

BT::NodeStatus SetGoalToBinCenter::tick()
{
    auto& state = GlobalState::getInstance();
    auto& config = ConfigManager::getInstance();
    geometry_msgs::PoseStamped goal = config.getBinCenter(state.getCurrentTask().bin_id);
    setOutput("goal", goal);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SendNavigationGoal::tick()
{
    auto goal = getInput<geometry_msgs::PoseStamped>("goal");
    if (!goal) {
        return BT::NodeStatus::FAILURE;
    }

    int status_in = 2; // Default to OTHER
    getInput("status", status_in);
    auto status = static_cast<NavigationStatus>(status_in);

    auto& gs = GlobalState::getInstance();
    Task current_task = gs.getCurrentTask();
    Task previous_task = gs.getPreviousTask();

    uint8_t current_id = 0;
    uint8_t last_id = 0;

    switch (status)
    {
        case NavigationStatus::SCOOP: // Going to scoop
            current_id = current_task.bin_id;
            // If there was a previous task, last location was a hopper
            if (previous_task.task_id != 0) {
                last_id = previous_task.hopper_id;
            }
            break;
        case NavigationStatus::DUMP: // Going to dump
            current_id = current_task.hopper_id;
            last_id = current_task.bin_id; // Last location was a bin
            break;
        case NavigationStatus::OTHER: // Going to park, etc.
            // If there was a previous task, last location was likely a hopper
             current_id = 255;
            if (previous_task.task_id != 0) {
                last_id = previous_task.hopper_id;
            }
            break;
        default:
            break;
    }

    auto& topic_manager = ROSTopicManager::getInstance();
    topic_manager.sendMoveBaseGoal(goal.value(), status, current_id, last_id);
    
    // 发布"导航进行中"消息
    topic_manager.publishActionStatus("导航进行中");
    
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetGoalToScoopPoint::tick()
{
    geometry_msgs::PoseStamped goal = GlobalState::getInstance().getScoopPoint();
    setOutput("goal", goal);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetGoalToHopper::tick()
{
    auto& state = GlobalState::getInstance();
    auto& config = ConfigManager::getInstance();
    geometry_msgs::PoseStamped goal = config.getHopperPose(state.getCurrentTask().hopper_id);
    setOutput("goal", goal);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SetGoalToParking::tick()
{
    auto& config = ConfigManager::getInstance();
    geometry_msgs::PoseStamped goal = config.getParkingPose("main_parking");
    setOutput("goal", goal);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CancelNavigation::tick()
{
    ROSTopicManager::getInstance().cancelMoveBaseGoal();
    ROS_INFO("BT: Navigation cancelled.");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CancelArmBucketAction::tick()
{
    // This is a placeholder. You should implement the actual logic to stop the arm and bucket.
    // For example, by publishing a specific command or calling a service.
    ros::NodeHandle nh;
    nh.setParam("ArmBucketState", 0); // Assuming 0 is a safe/stop state
    ROS_INFO("BT: Arm/Bucket action cancelled (sent state 0).");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CommitToEndMission::tick()
{
    config().blackboard->set("mission_ending", true);
    // Consume the external flag to prevent re-triggering
    GlobalState::getInstance().setEndTask(false);
    ROS_INFO("BT: Mission end committed to blackboard. External flag consumed.");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ClearCurrentTask::tick()
{
    GlobalState::getInstance().clearCurrentTask();
    ROS_INFO("BT: Current task cleared.");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PauseNavigation::tick()
{
    ROSTopicManager::getInstance().cancelMoveBaseGoal();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UpdateTaskProgress::tick()
{
    std::string phase, message;
    float progress;
    if (!getInput("phase", phase) || !getInput("progress", progress) || !getInput("message", message)) {
        return BT::NodeStatus::FAILURE;
    }
    GlobalState::getInstance().updateTaskProgress(phase, progress, message);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus WaitForNavigationArrival::onStart()
{
    std::string msg;
    getInput("message", msg);
    ROS_INFO("[WAIT] %s. Waiting for navigation arrival.", msg.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForNavigationArrival::onRunning()
{
    if (GlobalState::getInstance().isNavigationArrived()) { return BT::NodeStatus::SUCCESS; }

    // 可配置的任务抢占：默认不允许在执行过程中被新任务打断
    bool allow_preempt = false;
    {
        ros::NodeHandle nh("~");
        nh.param("allow_task_preemption", allow_preempt, false);
        // 打印一次allow_preempt的值
        ROS_INFO_ONCE("[TASK] Allow task preemption: %s", allow_preempt ? "yes" : "no");
    }
    if (allow_preempt && GlobalState::getInstance().hasNewTask()) {
        return BT::NodeStatus::FAILURE; // 允许打断
    }

    // 周期打印导航剩余距离（0.5s 节流）
    std::string msg;
    getInput("message", msg);
    double dist = GlobalState::getInstance().getDistanceToGoal();
    ROS_INFO_THROTTLE(0.5, "[CheckDist] %s | Current distance: %.2f m", msg.c_str(), dist);

    return BT::NodeStatus::RUNNING;
}

void WaitForNavigationArrival::onHalted() {}

BT::NodeStatus GetAndSetCurrentTask::tick()
{
    auto& state = GlobalState::getInstance();
    if (state.hasNewTask()) {
        // Store the old current_task as previous_task
        Task current_t = state.getCurrentTask();
        state.setPreviousTask(current_t);

        // Get the new task and set it as current
        Task new_t = state.getNextTask();
        state.setCurrentTask(new_t);
        state.incrementTasksStarted();

        // 在真正开始执行该任务时，更新全局参数 /pileID 为当前任务的 bin_id
        ros::param::set("/pileID", static_cast<int>(new_t.bin_id));
        ros::param::set("/hopperID", static_cast<int>(new_t.hopper_id));
        ROS_INFO("[TASK] Start Task #%d (bin=%d, hopper=%d). Set /pileID=%d",
                 new_t.task_id, new_t.bin_id, new_t.hopper_id, new_t.bin_id);

        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ResetNavigationArrived::tick()
{
    GlobalState::getInstance().resetNavigationArrived();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ResetDistanceToGoal::tick()
{
    GlobalState::getInstance().resetDistanceToGoal();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ResetDistanceReduction::tick()
{
    GlobalState::getInstance().resetDistanceReduction();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ResetPlanningFeedback::tick()
{
    GlobalState::getInstance().resetPlanningFeedback();
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList WaitForPlanningFeedback::providedPorts()
{
    return { BT::InputPort<double>("timeout", 2.0, "Timeout in seconds to wait for feedback") };
}

BT::NodeStatus WaitForPlanningFeedback::onStart()
{
    if (!getInput("timeout", timeout_s_)) {
        timeout_s_ = 2.0; // Default value if not provided
    }
    start_time_ = ros::Time::now();
    ROS_INFO("[BT] Waiting for planning feedback... (timeout: %.1fs)", timeout_s_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForPlanningFeedback::onRunning()
{
    auto& state = GlobalState::getInstance();

    if (state.hasReceivedPlanningFeedback()) {
        if (state.wasPlanningSuccessful()) {
            ROS_INFO("[BT] Planning feedback received: SUCCESS.");
            return BT::NodeStatus::SUCCESS;
        } else {
            ROS_ERROR("[BT] Planning feedback received: FAILURE.");
            return BT::NodeStatus::FAILURE;
        }
    }

    if ((ros::Time::now() - start_time_).toSec() > timeout_s_) {
        ROS_ERROR("[BT] Timeout waiting for planning feedback after %.1fs.", timeout_s_);
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
}

void WaitForPlanningFeedback::onHalted() {}


// ========== 注册函数实现 ==========

// ========== 新增节点实现 ==========

BT::NodeStatus WaitForNavigationResult::onStart()
{
    // 节点开始时，我们假设导航还未到达
    // 真正的状态由 GlobalState 控制
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForNavigationResult::onRunning()
{
    // 持续检查 GlobalState 中的导航到达标志
    // 周期打印导航剩余距离（0.5s 节流）
    double dist = GlobalState::getInstance().getDistanceToGoal();
    ROS_INFO_THROTTLE(0.5, "[CheckDist] Navigation | Current distance: %.2f m", dist);

    if (GlobalState::getInstance().isNavigationArrived()) {
        ROS_INFO("[BT] Navigation result received.");
        auto result = GlobalState::getInstance().getNavigationResult();
        
        // 将结果写入黑板
        setOutput("success", static_cast<bool>(result.success));
        setOutput("final_distance_error", result.final_distance_error);
        setOutput("final_angle_error", result.final_angle_error);
        setOutput("error_code", result.error_code);
        setOutput("error_msg", result.error_msg);
        
        // 导航成功时发送"导航到达"消息
        if (result.success) {
            ROSTopicManager::getInstance().publishActionStatus("导航到达");
        }
        
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void WaitForNavigationResult::onHalted()
{
    // 节点被挂起时的处理逻辑，这里暂时不需要
}


BT::NodeStatus BlackboardCheck::tick()
{
    std::string key; bool expected=true;
    if(!getInput("key", key)) return BT::NodeStatus::FAILURE;
    getInput("expected", expected);
    bool val=false;
    if(!config().blackboard->get(key,val)) return BT::NodeStatus::FAILURE;
    return (val==expected)? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckPlanningSuccess::tick()
{
    bool plan_succeeded = false;
    if (getInput("plan_succeeded", plan_succeeded)) {
        ROS_INFO("[BT] Planning Success Check: %s", plan_succeeded ? "SUCCESS" : "FAILURE");
        return plan_succeeded ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    ROS_WARN("[BT] Could not get 'plan_succeeded' from blackboard.");
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckArrivalQuality::tick()
{
    double dist_error, angle_error;
    if (!getInput("distance_error", dist_error) || !getInput("angle_error", angle_error)) {
        ROS_WARN("[BT] Could not get arrival errors from blackboard.");
        setOutput("quality_ok", false);
        return BT::NodeStatus::FAILURE;
    }

    auto& config = ConfigManager::getInstance();
    double dist_threshold = config.getDistanceThreshold("arrival_dist_error_threshold");
    double angle_threshold = config.getDistanceThreshold("arrival_angle_error_threshold");

    bool is_ok = (dist_error <= dist_threshold) && (angle_error <= angle_threshold);

    ROS_INFO("[BT] Arrival Quality Check: DistError=%.2f (Th=%.2f), AngleError=%.2f (Th=%.2f) -> %s",
             dist_error, dist_threshold, angle_error, angle_threshold, is_ok ? "OK" : "NOT OK");

    setOutput("quality_ok", is_ok);
    return is_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

RequestManualIntervention::RequestManualIntervention(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    // 初始化 ROS 发布者
    // estop_pub_ = nh_.advertise<std_msgs::Bool>("/ACU_EStop", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("state_machine/status", 10);
}

BT::NodeStatus RequestManualIntervention::tick()
{
    int current_status = -1;
    getInput("current_status", current_status);

    // If intervening for a dumping task (status=1), reset the last status to SCOOP
    // so that the retry navigation has the correct 0->1 transition.
    if (current_status == 1) {
        ROSTopicManager::getInstance().setLastStatus(NavigationStatus::SCOOP);
        ROS_INFO("[BT] Last navigation status reset to SCOOP for retry.");
    }

    // 0. 取消当前导航目标，避免在人工介入期间 move_base 仍在尝试规划
    ROSTopicManager::getInstance().cancelMoveBaseGoal();

    // 1. 发布急停指令
    std_msgs::Bool estop_msg;
    estop_msg.data = true;
    // estop_pub_.publish(estop_msg);
    ROS_WARN("[BT] Emergency Stop Published!");

    // 2. 设置需要人工介入的全局参数
    ros::param::set("/autonomous_loader/manual_intervention_required", true);
    ROS_WARN("[BT] Manual intervention required parameter set.");

    // 3. 发布状态提示信息
    std::string message;
    getInput("message", message);
    if (message.empty()) {
        message = "Manual intervention required. Please move the vehicle to a suitable location.";
    }
    std_msgs::String status_msg;
    status_msg.data = message;
    status_pub_.publish(status_msg);
    ROS_WARN("[BT] Published status: %s", message.c_str());

    return BT::NodeStatus::SUCCESS;
}

void RegisterNodes(BT::BehaviorTreeFactory& factory, ros::NodeHandle& nh)
{
    // Conditions
    factory.registerNodeType<CheckCancelTask>("CheckCancelTask");
    factory.registerNodeType<CheckEndTaskAfterCycle>("CheckEndTaskAfterCycle");
    factory.registerNodeType<CheckEndTask>("CheckEndTask");
    factory.registerNodeType<CheckPauseTask>("CheckPauseTask");
    factory.registerNodeType<CheckNewTask>("CheckNewTask");
    factory.registerNodeType<CheckNavigationArrived>("CheckNavigationArrived");
    factory.registerNodeType<CheckDistanceThreshold>("CheckDistanceThreshold");
    factory.registerNodeType<CheckScoopPointZero>("CheckScoopPointZero");
    factory.registerNodeType<CheckForNextCycle>("CheckForNextCycle");
    factory.registerNodeType<CheckDistanceFromLastHopper>("CheckDistanceFromLastHopper");
    factory.registerNodeType<CheckDistanceSinceScoop>("CheckDistanceSinceScoop");
    factory.registerNodeType<IsFirstTask>("IsFirstTask");
    factory.registerNodeType<CheckArmBucketDone>("CheckArmBucketDone");
    factory.registerNodeType<CheckArmBucketCommandSent>("CheckArmBucketCommandSent");
    factory.registerNodeType<BlackboardCheck>("BlackboardCheck");
    factory.registerNodeType<CheckPlanningSuccess>("CheckPlanningSuccess");
    factory.registerNodeType<CheckArrivalQuality>("CheckArrivalQuality");
    factory.registerNodeType<CheckErrorCode>("CheckErrorCode");
    factory.registerNodeType<CheckNavStatus>("CheckNavStatus");
    
    // Actions
    factory.registerNodeType<SetArmBucketState>("SetArmBucketState");
    factory.registerNodeType<WaitForArmBucketCompletion>("WaitForArmBucketCompletion");
    factory.registerNodeType<SetPointAccumulationStart>("SetPointAccumulationStart");
    factory.registerNodeType<StartPointAccumulation>("StartPointAccumulation");
    factory.registerNodeType<StopPointAccumulation>("StopPointAccumulation");
    factory.registerNodeType<WaitForPointAccumulationDone>("WaitForPointAccumulationDone");
    factory.registerNodeType<RequestScoopPoint>("RequestScoopPoint");
    factory.registerNodeType<SetGoalToBinCenter>("SetGoalToBinCenter");
    factory.registerNodeType<SetGoalToScoopPoint>("SetGoalToScoopPoint");
    factory.registerNodeType<SetGoalToHopper>("SetGoalToHopper");
    factory.registerNodeType<SetGoalToParking>("SetGoalToParking");
    factory.registerNodeType<SendNavigationGoal>("SendNavigationGoal");
    factory.registerNodeType<CancelNavigation>("CancelNavigation");
    factory.registerNodeType<CancelArmBucketAction>("CancelArmBucketAction");
    factory.registerNodeType<ClearCurrentTask>("ClearCurrentTask");
    factory.registerNodeType<CommitToEndMission>("CommitToEndMission");
    factory.registerNodeType<PauseNavigation>("PauseNavigation");
    factory.registerNodeType<UpdateTaskProgress>("UpdateTaskProgress");
    factory.registerNodeType<WaitForNavigationArrival>("WaitForNavigationArrival");
    factory.registerNodeType<GetAndSetCurrentTask>("GetAndSetCurrentTask");
    factory.registerNodeType<ResetNavigationArrived>("ResetNavigationArrived");
    factory.registerNodeType<ResetDistanceReduction>("ResetDistanceReduction");
    factory.registerNodeType<ResetDistanceToGoal>("ResetDistanceToGoal");
    factory.registerNodeType<ResetPlanningFeedback>("ResetPlanningFeedback");
    factory.registerNodeType<WaitForPlanningFeedback>("WaitForPlanningFeedback");
    factory.registerNodeType<RequestManualIntervention>("RequestManualIntervention");
    factory.registerNodeType<WaitForManualIntervention>("WaitForManualIntervention");
    factory.registerNodeType<WaitForNavigationResult>("WaitForNavigationResult");
    factory.registerNodeType<SetWorkState>("SetWorkState");

    ROS_INFO("Registered all autonomous loader robot behavior tree nodes");
}

} // namespace autonomous_loader_bt
