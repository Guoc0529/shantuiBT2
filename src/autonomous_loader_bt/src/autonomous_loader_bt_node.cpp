#include <ros/ros.h>
#include <ros/package.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include "autonomous_loader_msgs/TaskCommand.h"
#include "autonomous_loader_bt/loader_bt_nodes.h"
#include "autonomous_loader_bt/ros_topic_manager.h"
#include "shuju/cangdou.h"
#include "shuju/TaskCtrl.h"
#include <stdexcept> // 用于 std::runtime_error

using namespace autonomous_loader_bt;

// ---------------------------------------------------------------
//  用于将 CMake 注入的宏转换为 C++ 字符串的辅助宏
// ---------------------------------------------------------------
#define XSTR(s) STR(s)
#define STR(s) #s

class AutonomousLoaderBTNode
{
public:
    AutonomousLoaderBTNode(ros::NodeHandle& nh) : nh_(nh)
    {
        // 获取参数
        std::string tree_file;
        std::string config_file;
        double loop_rate;
        bool enable_logging;
        bool enable_console_logging;
        std::string log_file;
        
        nh_.param<std::string>("tree_file", tree_file, "autonomous_loader_tree.xml");
        nh_.param<std::string>("config_file", config_file, "loader_config.yaml");
        nh_.param<double>("tree_loop_rate", loop_rate, 10.0);
        nh_.param<bool>("enable_logging", enable_logging, true);
        nh_.param<bool>("enable_console_logging", enable_console_logging, false);
        nh_.param<std::string>("log_file", log_file, "autonomous_loader_logs.btlog");
        
        bool enable_groot2;
        int groot2_port;
        nh_.param<bool>("enable_groot2", enable_groot2, false);
        nh_.param<int>("groot2_port", groot2_port, 1666);

        ROS_INFO("\033[36m[BT]\033[0m Starting autonomous loader robot behavior tree node");
        ROS_INFO("\033[36m[BT]\033[0m Tree file: %s", tree_file.c_str());
        ROS_INFO("\033[36m[BT]\033[0m Config file: %s", config_file.c_str());
        ROS_INFO("\033[36m[BT]\033[0m Loop rate: %.1f Hz", loop_rate);

        // 加载配置文件
        loadConfigFile(config_file);

        // 创建ROS发布者和订阅者
        setupROSTopics();
        
        // 初始化ROS话题管理器
        auto& topic_manager = autonomous_loader_bt::ROSTopicManager::getInstance();
        topic_manager.initialize(nh_);

        // 初始化任务状态上报器
        autonomous_loader_bt::TaskStatusReporter::instance().init(nh_);

        // 将Action的反馈连接到GlobalState
        topic_manager.setDistanceUpdateCallback([](double dist){
            auto& gs = autonomous_loader_bt::GlobalState::getInstance();
            gs.setDistanceToGoal(dist);
            gs.updateDistanceReduction(dist);
        });
        topic_manager.setArrivalCallback([](bool arrived){
            autonomous_loader_bt::GlobalState::getInstance().setNavigationArrived(arrived);
        });
        topic_manager.setResultCallback([](const autonomous_loader_msgs::NavigateResult& result){
            autonomous_loader_bt::GlobalState::getInstance().setNavigationResult(result);
        });
        topic_manager.setNavCancelConfirmedCallback([](){
            autonomous_loader_bt::GlobalState::getInstance().setNavCancelConfirmed(true);
        });
        topic_manager.setPlanningFeedbackCallback([](bool success){
            // Only set feedback if it hasn't been received yet for this goal
            if (!autonomous_loader_bt::GlobalState::getInstance().hasReceivedPlanningFeedback()) {
                autonomous_loader_bt::GlobalState::getInstance().setPlanningFeedback(success);
            }
        });

        // 创建BehaviorTree工厂
        BT::BehaviorTreeFactory factory;

        // 注册自定义节点
        autonomous_loader_bt::RegisterNodes(factory, nh_);

        try {
            // 构建行为树文件路径
            std::string tree_path = getPackagePath() + "/trees/" + tree_file;
            
            // 注册通用人工接管子树（必须在创建主树之前）
            std::string subtree_path = getPackagePath() + "/trees/manual_override_subtree.xml";
            factory.registerBehaviorTreeFromFile(subtree_path);
            ROS_INFO("\033[36m[BT]\033[0m Registered manual override subtree from: %s", subtree_path.c_str());
            
            // 注册主行为树（包含 SubTree 引用）
            factory.registerBehaviorTreeFromFile(tree_path);
            
            // 使用 createTree 而非 createTreeFromFile（避免警告）
            std::string main_tree_name = "AutonomousLoaderTree";
            tree_ = factory.createTree(main_tree_name);
            ROS_INFO("\033[36m[BT]\033[0m Successfully loaded behavior tree: %s", main_tree_name.c_str());

            // 设置日志记录器
            if (enable_logging) {
                setupLoggers(log_file, enable_console_logging);
            }
            
            // 设置 Groot2 监控（如果启用）
            if (enable_groot2) {
                setupGroot2(groot2_port);
            }

            // 创建定时器
            timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate),
                                   &AutonomousLoaderBTNode::executeTree, this);

            ROS_INFO("\033[36m[BT]\033[0m Waiting for tasks from scheduler (service /liaodou or topic /scheduler/start_task)");

        } catch (const std::exception& e) {
            ROS_ERROR("\033[31m[BT]\033[0m Failed to create behavior tree: %s", e.what());
            ros::shutdown();
        }
    }

private:
    std::string getPackagePath()
    {
        auto exists = [](const std::string& p){ return !p.empty() && access(p.c_str(), F_OK) == 0; };

        std::string param_path;
        if (nh_.getParam("package_root", param_path) && exists(param_path)) {
            ROS_INFO("\033[36m[PATH]\033[0m Using param(~package_root): %s", param_path.c_str());
            return param_path;
        }

        std::string macro_path = XSTR(AUTONOMOUS_LOADER_BT_PATH);
        if (!macro_path.empty() && macro_path != "AUTONOMOUS_LOADER_BT_PATH" && exists(macro_path)) {
            return macro_path;
        }

        std::string rospack_path = ros::package::getPath("autonomous_loader_bt");
        if (exists(rospack_path)) {
            ROS_INFO("\033[36m[PATH]\033[0m Using ros::package::getPath(): %s", rospack_path.c_str());
            return rospack_path;
        }

        char exe_buf[4096] = {0};
        ssize_t n = readlink("/proc/self/exe", exe_buf, sizeof(exe_buf)-1);
        if (n > 0) {
            std::string exe_path(exe_buf);
            auto dir = exe_path.substr(0, exe_path.find_last_of('/'));
            for (int i = 0; i < 8 && !dir.empty(); ++i) {
                std::string candidate = dir;
                if (exists(candidate + "/src/autonomous_loader_bt")) {
                    ROS_INFO("\033[36m[PATH]\033[0m Fallback to discovered src path: %s", (candidate + "/src/autonomous_loader_bt").c_str());
                    return candidate + "/src/autonomous_loader_bt";
                }
                if (exists(candidate + "/trees") && exists(candidate + "/config")) {
                    ROS_INFO("\033[36m[PATH]\033[0m Fallback to discovered package-like path: %s", candidate.c_str());
                    return candidate;
                }
                auto pos = dir.find_last_of('/');
                if (pos == std::string::npos) break;
                dir = dir.substr(0, pos);
            }
        }

        char cwd[4096] = {0};
        if (getcwd(cwd, sizeof(cwd))) {
            std::string guess = std::string(cwd) + "/src/autonomous_loader_bt";
            if (exists(guess)) {
                ROS_INFO("\033[36m[PATH]\033[0m Fallback to CWD/src path: %s", guess.c_str());
                return guess;
            }
            ROS_INFO("\033[36m[PATH]\033[0m Fallback to CWD: %s", cwd);
            return std::string(cwd);
        }

        ROS_ERROR("\033[31m[PATH]\033[0m Unable to resolve package path; using '.'");
        return std::string(".");
    }
    
    void loadConfigFile(const std::string& config_file)
    {
        std::string config_path = getPackagePath() + "/config/" + config_file;
        
        auto& config_manager = autonomous_loader_bt::ConfigManager::getInstance();
        if (!config_manager.loadConfig(config_path)) {
            ROS_WARN("\033[33m[BT]\033[0m Config file loading failed, will use default configuration");
        }
    }

    void setupROSTopics()
    {
        nh_.setParam("/workstate", 4);
        // 发布者
        status_pub_ = nh_.advertise<std_msgs::String>("state_machine/status", 10);
        workstate_pub_ = nh_.advertise<std_msgs::Int8>("/workstate", 10);
        task_id_pub_ = nh_.advertise<std_msgs::Int32>("/current_task_id", 1, true); // Latching publisher
        
        // 订阅者
        start_task_sub_ = nh_.subscribe("scheduler/start_task", 10, &AutonomousLoaderBTNode::startTaskCallback, this);
        pause_task_sub_ = nh_.subscribe("scheduler/pause_task", 10, &AutonomousLoaderBTNode::pauseTaskCallback, this);
        end_task_sub_ = nh_.subscribe("scheduler/end_task", 10, &AutonomousLoaderBTNode::endTaskCallback, this);

        // TaskCtrl 话题订阅 (ctrlCmd: 1=开始任务, 2=远程接管, 3=避障停车, 4=急停, 5=继续, 6=结束)
        taskctrl_sub_ = nh_.subscribe("/task_ctrl_command", 10, &AutonomousLoaderBTNode::taskCtrlCallback, this);
        ROS_INFO("\033[36m[BT]\033[0m Subscribed to /task_ctrl_command for task control commands");

        // 避障相关话题订阅
        obstacle_1_sub_ = nh_.subscribe("/obstacle_1", 10, &AutonomousLoaderBTNode::obstacle1Callback, this);
        obstacle_2_sub_ = nh_.subscribe("/obstacle_2", 10, &AutonomousLoaderBTNode::obstacle2Callback, this);
        backstart_sub_ = nh_.subscribe("/backstart", 10, &AutonomousLoaderBTNode::backstartCallback, this);
        restore_sub_ = nh_.subscribe("/restore", 10, &AutonomousLoaderBTNode::restoreCallback, this);
        ROS_INFO("\033[36m[BT]\033[0m Subscribed to obstacle topics (/obstacle_1, /obstacle_2, /backstart, /restore)");

        // 初始化鸣笛和双闪控制
        ROSTopicManager::getInstance().initializeObstacleControls(nh_);
        
        // 服务端：接收调度下发任务
        task_service_ = nh_.advertiseService("liaodou", &AutonomousLoaderBTNode::taskServiceCallback, this);
        
        ROS_INFO("\033[36m[BT]\033[0m Service /liaodou advertised (private namespace: %s/liaodou)", nh_.getNamespace().c_str());
        ROS_INFO("\033[36m[BT]\033[0m Waiting for task requests from scheduler...");
        ROS_INFO("\033[36m[BT]\033[0m All ROS topics set up");
    }

    void setupLoggers(const std::string& log_file, bool enable_console)
    {
        if (enable_console) {
            console_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
            ROS_INFO("\033[36m[BT]\033[0m Console logging enabled");
        } else {
            ROS_INFO("\033[36m[BT]\033[0m Console logging disabled (use enable_console_logging:=true to enable)");
        }
        
        try {
            file_logger_ = std::make_unique<BT::FileLogger2>(tree_, log_file);
            ROS_INFO("\033[36m[BT]\033[0m File logging enabled: %s", log_file.c_str());
        } catch (const std::exception& e) {
            ROS_WARN("\033[33m[BT]\033[0m Failed to create file logger: %s", e.what());
        }
    }
    
    void setupGroot2(int port)
    {
        try {
            groot2_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, port);
            ROS_INFO("\033[36m[BT]\033[0m Groot2 monitoring enabled on port %d", port);
            ROS_INFO("\033[36m[BT]\033[0m Connect Groot2 to: tcp://localhost:%d", port);
        } catch (const std::exception& e) {
            ROS_WARN("\033[33m[BT]\033[0m Failed to create Groot2 publisher: %s", e.what());
            ROS_WARN("\033[33m[BT]\033[0m Make sure BehaviorTree.CPP was compiled with ZMQ support");
        }
    }

    void executeTree(const ros::TimerEvent&)
    {
        static int tick_count = 0;
        tick_count++;
        try {
            auto& gs = GlobalState::getInstance();

            // Check if halt is requested (emergency stop, obstacle stop, or remote control)
            if (gs.isHaltRequested()) {
                tree_.haltTree();
                gs.clearHaltRequested();
                idle_timer_started_ = false;
                ROS_INFO("\033[36m[BT_ROOT]\033[0m Halt requested - tree halted");
            }

            BT::NodeStatus status = tree_.tickOnce();
            if (tick_count % 100 == 0) {
                ROS_INFO_THROTTLE(10.0, "\033[36m[BT_ROOT]\033[0m Still running, tick #%d", tick_count);
            }
            publishStatus(status);

            // Handle ending state: when task completes and is_ending_ is true, halt tree and set state=8
            if (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::SUCCESS) {
                if (gs.isEnding()) {
                    tree_.haltTree();
                    gs.setIsEnding(false);
                    TaskStatusReporter::instance().setState(TaskStatusReporter::ENDED);
                    idle_timer_started_ = false;
                    ROS_INFO("\033[36m[BT_ROOT]\033[0m Task ended - tree halted, state=8");
                }
            }

            // Handle normal task completion: set state=1 (IDLE) for 2 seconds
            if (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::SUCCESS) {
                // Check if we just completed a task (was RUNNING before)
                if (!idle_timer_started_ && !gs.isEnding() && !gs.isPauseTask()) {
                    idle_start_time_ = ros::Time::now();
                    idle_timer_started_ = true;
                    TaskStatusReporter::instance().setState(TaskStatusReporter::IDLE);
                    ROS_INFO("\033[36m[BT_ROOT]\033[0m Task completed - state=1 (IDLE), 2s timer started");
                }

                // After 2 seconds, if new task exists, set state=2/3; otherwise keep state=1
                if (idle_timer_started_) {
                    ros::Duration elapsed = ros::Time::now() - idle_start_time_;
                    if (elapsed.toSec() >= 2.0) {
                        if (gs.hasNewTask()) {
                            // New task arrived, will be set to 2/3 by cangdou callback
                            idle_timer_started_ = false;
                        }
                        // else: keep state=1 (IDLE), waiting for new task
                    }
                }
            } else if (status == BT::NodeStatus::RUNNING) {
                // Tree is running, reset idle timer
                idle_timer_started_ = false;
            }

        } catch (const std::exception& e) {
            ROS_ERROR("\033[31m[BT]\033[0m Error executing behavior tree: %s", e.what());
        }
    }

    void publishStatus(BT::NodeStatus status)
    {
        std_msgs::String status_msg;
        switch (status) {
            case BT::NodeStatus::SUCCESS: status_msg.data = "SUCCESS"; break;
            case BT::NodeStatus::FAILURE: status_msg.data = "FAILURE"; break;
            case BT::NodeStatus::RUNNING: status_msg.data = "RUNNING"; break;
            case BT::NodeStatus::IDLE:    status_msg.data = "IDLE";    break;
            case BT::NodeStatus::SKIPPED: status_msg.data = "SKIPPED"; break;
        }
        status_pub_.publish(status_msg);
    }

    BT::NodeStatus prev_bt_status_ = BT::NodeStatus::IDLE;
    ros::Time last_task_complete_time_;
    ros::Time idle_start_time_;
    bool idle_timer_started_ = false;

    // ========== ROS回调函数 ==========

    void startTaskCallback(const autonomous_loader_msgs::TaskCommand::ConstPtr& msg)
    {
        // Publish the received task ID, whether it's a new task or a cancellation
        std_msgs::Int32 task_id_msg;
        task_id_msg.data = msg->task_id;
        task_id_pub_.publish(task_id_msg);

        if (msg->task_id == 0)
        {
            ROS_WARN("\033[33m[BT]\033[0m Received task cancellation request (task_id=0) via topic. Sending vehicle to park.");
            auto& config = autonomous_loader_bt::ConfigManager::getInstance();
            geometry_msgs::PoseStamped parking_goal = config.getParkingPose("main_parking");

            auto& gs = autonomous_loader_bt::GlobalState::getInstance();
            auto previous_task = gs.getPreviousTask();
            uint8_t last_id = (previous_task.task_id != 0) ? previous_task.hopper_id : 0;
            uint8_t current_id = 0;
            auto& topic_manager = autonomous_loader_bt::ROSTopicManager::getInstance();
            topic_manager.sendMoveBaseGoal(parking_goal, autonomous_loader_bt::NavigationStatus::OTHER, current_id, last_id);
            tree_.haltTree();
            gs.clearTasks();
            gs.setTaskId(-1);
            autonomous_loader_bt::TaskStatusReporter::instance().setTaskId(-1);
            ROS_INFO("\033[36m[BT]\033[0m Sent new navigation goal to parking spot and halted BT (topic).");
            return;
        }

        ROS_INFO("\033[36m[BT]\033[0m Received start task command - Task ID: %d, Bin: %d, Hopper: %d", msg->task_id, msg->bin_id, msg->hopper_id);
        int task_type = (msg->task_type == "auto" || msg->task_type == "scoop") ? 0 : 1;
        int work_state = (task_type == 0) ? 2 : 3;  // 2=auto, 3=temp
        ROS_INFO("\033[36m[BT]\033[0m DEBUG: task_type=%d, setting work_state=%d", task_type, work_state);
        autonomous_loader_bt::GlobalState::getInstance().setWorkState(work_state);
        autonomous_loader_bt::Task task(msg->task_id, msg->bin_id, msg->hopper_id, task_type, msg->task_type);
        autonomous_loader_bt::GlobalState::getInstance().addTask(task);
        autonomous_loader_bt::GlobalState::getInstance().setTaskId(msg->task_id);
        autonomous_loader_bt::TaskStatusReporter::instance().setTaskId(msg->task_id);

        nh_.setParam("ArmBucketState", 2);
        ROS_INFO("\033[36m[BT]\033[0m Set param ArmBucketState=2 (prepare raise arm)");
    }

    void pauseTaskCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        autonomous_loader_bt::GlobalState::getInstance().setPauseTask(msg->data);
        ROS_INFO("\033[36m[BT]\033[0m Received pause task command: %s", msg->data ? "yes" : "no");
    }

    void endTaskCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        autonomous_loader_bt::GlobalState::getInstance().setEndTask(msg->data);
        ROS_INFO("\033[36m[BT]\033[0m Received end task command: %s", msg->data ? "yes" : "no");
    }

    void taskCtrlCallback(const shuju::TaskCtrl::ConstPtr& msg)
    {
        ROS_INFO("\033[33m[TaskCtrl]\033[0m Received: taskID=%d, ctrlCmd=%d, timestamp=%.1f",
                 msg->taskID, msg->ctrlCmd, msg->timestamp);

        auto& gs = autonomous_loader_bt::GlobalState::getInstance();

        switch (msg->ctrlCmd) {
            case 1:  // 开始任务
                gs.setStartTask(true);
                ROS_INFO("\033[33m[TaskCtrl]\033[0m ctrlCmd=1: Start task flag set");
                break;

            case 2:  // 远程接管
                gs.setPauseTask(true);
                gs.setHaltRequested(true);
                TaskStatusReporter::instance().setState(TaskStatusReporter::REMOTE_CONTROL);
                ROS_INFO("\033[33m[TaskCtrl]\033[0m ctrlCmd=2: Remote takeover - halt BT, set state=6");
                break;

            case 3:  // 避障停车（带坐标）- 先完成当前任务，再去避障点
                {
                    geometry_msgs::PoseStamped obstacle_target;
                    obstacle_target.header.stamp = ros::Time::now();
                    obstacle_target.header.frame_id = "map";
                    obstacle_target.pose.position.x = msg->target_x;
                    obstacle_target.pose.position.y = msg->target_y;
                    obstacle_target.pose.position.z = 0.0;
                    // Convert yaw to quaternion
                    double cy = cos(msg->target_yaw * 0.5);
                    double sy = sin(msg->target_yaw * 0.5);
                    obstacle_target.pose.orientation.w = cy;
                    obstacle_target.pose.orientation.x = 0;
                    obstacle_target.pose.orientation.y = 0;
                    obstacle_target.pose.orientation.z = sy;

                    // 保存避障目标，不立即触发，等待当前任务完成
                    gs.setObstacleTarget(obstacle_target);
                    gs.setObstacleType(0);  // 0表示从TaskCtrl来的坐标
                    gs.setObstaclePending(true);  // 标记为待执行避障
                    gs.setObstacleFromCtrl3(true);  // 标记来源为 ctrlCmd=3
                    // 立即设置工作状态为5（人工干预）
                    TaskStatusReporter::instance().setState(TaskStatusReporter::OBSTACLE_STOP);
                    ROS_INFO("\033[33m[TaskCtrl]\033[0m ctrlCmd=3: Obstacle pending, target (%.2f, %.2f, yaw=%.2f), state=5. Will avoid after current task completes",
                             msg->target_x, msg->target_y, msg->target_yaw);
                }
                break;

            case 4:  // 紧急停车
                gs.setEmergencyStop(true);
                gs.setPauseTask(true);
                gs.setHaltRequested(true);
                TaskStatusReporter::instance().setState(TaskStatusReporter::EMERGENCY_STOP);
                ROS_INFO("\033[33m[TaskCtrl]\033[0m ctrlCmd=4: Emergency stop - halt BT, set state=4");
                break;

            case 5:  // 继续任务 - 根据来源执行不同恢复动作
                {
                    bool from_ctrl3 = gs.isObstacleFromCtrl3();
                    gs.setObstacleFromCtrl3(false);  // 清除来源标记

                    if (from_ctrl3) {
                        // ctrlCmd=3 触发的避障：只需清除标志、关闭双闪、状态=1
                        gs.setObstacleTriggered(false);
                        gs.setObstaclePending(false);
                        gs.setObstacleType(0);
                        gs.setRestoreRequested(false);  // 不需要等待恢复，直接清除
                        // 关闭双闪
                        ROSTopicManager::getInstance().publishHazardLights(false);
                        TaskStatusReporter::instance().setState(TaskStatusReporter::IDLE);
                        ROS_INFO("\033[33m[TaskCtrl]\033[0m ctrlCmd=5: Resume from ctrlCmd=3 obstacle - clear flags, hazard off, state=1");
                    } else {
                        // 其他来源（ctrlCmd=2/4）触发的避障：完整恢复流程
                        gs.setPauseTask(false);
                        gs.setEmergencyStop(false);
                        gs.clearHaltRequested();
                        gs.setRestoreRequested(true);  // 触发 WaitForRestore
                        gs.setObstacleTriggered(false);
                        gs.setObstaclePending(false);
                        gs.setObstacleType(0);
                        TaskStatusReporter::instance().setState(TaskStatusReporter::IDLE);
                        ROS_INFO("\033[33m[TaskCtrl]\033[0m ctrlCmd=5: Resume task - clear flags, restore obstacle state, set state=1");
                    }
                }
                break;

            case 6:  // 结束任务
                gs.setEndTask(true);
                gs.setIsEnding(true);
                TaskStatusReporter::instance().setState(TaskStatusReporter::ENDING);
                ROS_INFO("\033[33m[TaskCtrl]\033[0m ctrlCmd=6: End task - set state=7, will halt after current task completes");
                break;

            default:
                ROS_WARN("\033[33m[TaskCtrl]\033[0m Unknown ctrlCmd: %d", msg->ctrlCmd);
                break;
        }
    }

    // ===== 避障相关回调函数 =====
    // obstacle_1/obstacle_2: 设置 obstacle_pending=true，等待任务完成后执行避障
    void obstacle1Callback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data) {
            auto& gs = autonomous_loader_bt::GlobalState::getInstance();
            if (!gs.isObstaclePending()) {
                gs.setObstaclePending(true);     // 改为待执行模式
                gs.setObstacleFromCtrl3(false);  // 标记来源不是ctrlCmd=3
                gs.setObstacleType(1);           // obstacle_1
                TaskStatusReporter::instance().setState(TaskStatusReporter::OBSTACLE_STOP);
                ROS_INFO("\033[36m[Obstacle]\033[0m /obstacle_1 triggered, obstacle_pending=true, obstacle_type=1, state=5");
            }
        }
    }

    void obstacle2Callback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data) {
            auto& gs = autonomous_loader_bt::GlobalState::getInstance();
            if (!gs.isObstaclePending()) {
                gs.setObstaclePending(true);     // 改为待执行模式
                gs.setObstacleFromCtrl3(false);  // 标记来源不是ctrlCmd=3
                gs.setObstacleType(2);           // obstacle_2
                TaskStatusReporter::instance().setState(TaskStatusReporter::OBSTACLE_STOP);
                ROS_INFO("\033[36m[Obstacle]\033[0m /obstacle_2 triggered, obstacle_pending=true, obstacle_type=2, state=5");
            }
        }
    }

    void backstartCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data) {
            auto& gs = autonomous_loader_bt::GlobalState::getInstance();
            if (!gs.isObstacleTriggered()) {
                gs.setObstacleTriggered(true);
                gs.setObstacleType(3);  // backstart
                ROS_INFO("\033[36m[Obstacle]\033[0m /backstart triggered, obstacle_type=3");
            }
        }
    }

    void restoreCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data) {
            auto& gs = autonomous_loader_bt::GlobalState::getInstance();
            
            // /restore 只处理 obstacle_1/obstacle_2 来源的恢复
            // ctrlCmd=3 来源的恢复由 ctrlCmd=5 处理
            if (gs.isObstaclePending() && !gs.isObstacleFromCtrl3()) {
                gs.setObstacleTriggered(false);
                gs.setObstaclePending(false);
                gs.setObstacleType(0);
                gs.setRestoreRequested(false);
                ROSTopicManager::getInstance().publishHazardLights(false);
                TaskStatusReporter::instance().setState(TaskStatusReporter::IDLE);
                ROS_INFO("\033[36m[Obstacle]\033[0m /restore: Resume from obstacle_1/2, hazard off, state=1");
            } else {
                ROS_WARN("\033[33m[Obstacle]\033[0m /restore: Ignored (no pending obstacle or ctrlCmd=3 source)");
            }
        }
    }

    bool taskServiceCallback(shuju::cangdou::Request& req, shuju::cangdou::Response& res)
    {
        // Publish the received task ID, whether it's a new task or a cancellation
        std_msgs::Int32 task_id_msg;
        task_id_msg.data = req.taskID;
        task_id_pub_.publish(task_id_msg);

        if (req.taskID == 0) {
            ROS_WARN("Received task cancellation request (taskID=0). Sending vehicle to park.");

            // 1. Get the parking goal
            auto& config = autonomous_loader_bt::ConfigManager::getInstance();
            geometry_msgs::PoseStamped parking_goal = config.getParkingPose("main_parking");

            // 2. Determine IDs for the goal message
            auto& gs = autonomous_loader_bt::GlobalState::getInstance();
            auto previous_task = gs.getPreviousTask();
            uint8_t last_id = (previous_task.task_id != 0) ? previous_task.hopper_id : 0;
            uint8_t current_id = 0; // Special ID for parking

            // 3. Send the new action goal
            auto& topic_manager = autonomous_loader_bt::ROSTopicManager::getInstance();
            topic_manager.sendMoveBaseGoal(parking_goal, autonomous_loader_bt::NavigationStatus::OTHER, current_id, last_id);

            // Also halt the behavior tree to prevent conflicts
            tree_.haltTree();
            gs.clearTasks();
            gs.setTaskId(-1);
            autonomous_loader_bt::TaskStatusReporter::instance().setTaskId(-1);

            ROS_INFO("\033[36m[BT]\033[0m Sent new navigation goal to parking spot and halted BT.");
            res.huiying = true;
            return true;
        }

        ROS_INFO("\033[36m[BT]\033[0m Received service task: taskID=%d, type=%d, bin=%d, hopper=%d", req.taskID, req.type, req.cang, req.dou);
        ROS_INFO("\033[36m[BT]\033[0m DEBUG: req.type=%d, setting state=%s", req.type, req.type == 0 ? "2(AUTO_WORKING)" : "3(TEMP_WORKING)");
        std::string type_str = (req.type == 0) ? "auto" : "temp";
        Task task(req.taskID, req.cang, req.dou, req.type, type_str);
        GlobalState::getInstance().addTask(task);
        GlobalState::getInstance().setTaskId(req.taskID);
        TaskStatusReporter::instance().setTaskId(req.taskID);

        // 根据任务类型设置工作状态
        int work_state = (req.type == 0) ? TaskStatusReporter::AUTO_WORKING : TaskStatusReporter::TEMP_WORKING;
        TaskStatusReporter::instance().setState(work_state);
        nh_.setParam("canggoal", static_cast<int>(req.cang));
        res.huiying = true;
        return true;
    }

    ros::NodeHandle& nh_;
    BT::Tree tree_;
    ros::Timer timer_;
    
    ros::Publisher status_pub_;
    ros::Publisher workstate_pub_;
    ros::Publisher task_id_pub_;
    
    ros::Subscriber start_task_sub_;
    ros::Subscriber pause_task_sub_;
    ros::Subscriber end_task_sub_;
    ros::Subscriber taskctrl_sub_;
    ros::Subscriber obstacle_1_sub_;
    ros::Subscriber obstacle_2_sub_;
    ros::Subscriber backstart_sub_;
    ros::Subscriber restore_sub_;
    
    ros::ServiceServer task_service_;
    
    std::unique_ptr<BT::StdCoutLogger> console_logger_;
    std::unique_ptr<BT::FileLogger2> file_logger_;
    std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomous_loader_bt_node");
    ros::NodeHandle nh("~");
    
    AutonomousLoaderBTNode node(nh);
    
    ROS_INFO("\033[36m[BT]\033[0m Autonomous loader robot behavior tree node started");
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}