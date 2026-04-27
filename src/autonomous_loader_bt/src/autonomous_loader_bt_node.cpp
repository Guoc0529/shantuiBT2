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
#include <stdexcept> // 用于 std::runtime_error

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

        ROS_INFO("Starting autonomous loader robot behavior tree node");
        ROS_INFO("Tree file: %s", tree_file.c_str());
        ROS_INFO("Config file: %s", config_file.c_str());
        ROS_INFO("Loop rate: %.1f Hz", loop_rate);

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
            
            // 创建行为树
            tree_ = factory.createTreeFromFile(tree_path);
            ROS_INFO("Successfully loaded behavior tree: %s", tree_path.c_str());

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

            ROS_INFO("Waiting for tasks from scheduler (service /liaodou or topic /scheduler/start_task)");

        } catch (const std::exception& e) {
            ROS_ERROR("Failed to create behavior tree: %s", e.what());
            ros::shutdown();
        }
    }

private:
    std::string getPackagePath()
    {
        auto exists = [](const std::string& p){ return !p.empty() && access(p.c_str(), F_OK) == 0; };

        std::string param_path;
        if (nh_.getParam("package_root", param_path) && exists(param_path)) {
            ROS_INFO("[PATH] Using param(~package_root): %s", param_path.c_str());
            return param_path;
        }

        std::string macro_path = XSTR(AUTONOMOUS_LOADER_BT_PATH);
        if (!macro_path.empty() && macro_path != "AUTONOMOUS_LOADER_BT_PATH" && exists(macro_path)) {
            return macro_path;
        }

        std::string rospack_path = ros::package::getPath("autonomous_loader_bt");
        if (exists(rospack_path)) {
            ROS_INFO("[PATH] Using ros::package::getPath(): %s", rospack_path.c_str());
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
                    ROS_INFO("[PATH] Fallback to discovered src path: %s", (candidate + "/src/autonomous_loader_bt").c_str());
                    return candidate + "/src/autonomous_loader_bt";
                }
                if (exists(candidate + "/trees") && exists(candidate + "/config")) {
                    ROS_INFO("[PATH] Fallback to discovered package-like path: %s", candidate.c_str());
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
                ROS_INFO("[PATH] Fallback to CWD/src path: %s", guess.c_str());
                return guess;
            }
            ROS_INFO("[PATH] Fallback to CWD: %s", cwd);
            return std::string(cwd);
        }

        ROS_ERROR("[PATH] Unable to resolve package path; using '.'");
        return std::string(".");
    }
    
    void loadConfigFile(const std::string& config_file)
    {
        std::string config_path = getPackagePath() + "/config/" + config_file;
        
        auto& config_manager = autonomous_loader_bt::ConfigManager::getInstance();
        if (!config_manager.loadConfig(config_path)) {
            ROS_WARN("Config file loading failed, will use default configuration");
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
        
        // 服务端：接收调度下发任务
        task_service_ = nh_.advertiseService("liaodou", &AutonomousLoaderBTNode::taskServiceCallback, this);
        
        ROS_INFO("Service /liaodou advertised (private namespace: %s/liaodou)", nh_.getNamespace().c_str());
        ROS_INFO("Waiting for task requests from scheduler...");
        ROS_INFO("All ROS topics set up");
    }

    void setupLoggers(const std::string& log_file, bool enable_console)
    {
        if (enable_console) {
            console_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
            ROS_INFO("Console logging enabled");
        } else {
            ROS_INFO("Console logging disabled (use enable_console_logging:=true to enable)");
        }
        
        try {
            file_logger_ = std::make_unique<BT::FileLogger2>(tree_, log_file);
            ROS_INFO("File logging enabled: %s", log_file.c_str());
        } catch (const std::exception& e) {
            ROS_WARN("Failed to create file logger: %s", e.what());
        }
    }
    
    void setupGroot2(int port)
    {
        try {
            groot2_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, port);
            ROS_INFO("Groot2 monitoring enabled on port %d", port);
            ROS_INFO("Connect Groot2 to: tcp://localhost:%d", port);
        } catch (const std::exception& e) {
            ROS_WARN("Failed to create Groot2 publisher: %s", e.what());
            ROS_WARN("Make sure BehaviorTree.CPP was compiled with ZMQ support");
        }
    }

    void executeTree(const ros::TimerEvent&)
    {
        static int tick_count = 0;
        tick_count++;
        try {
            BT::NodeStatus status = tree_.tickOnce();
            if (tick_count % 100 == 0) {  // Every 10 seconds (100 ticks at 10Hz)
                ROS_INFO_THROTTLE(10.0, "[BT_ROOT] Still running, tick #%d", tick_count);
            }
            publishStatus(status);
            // publishWorkState(status);

            // This logic is now handled in the Behavior Tree XML
            // if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
            //     tree_.haltTree();
            // }

        } catch (const std::exception& e) {
            ROS_ERROR("Error executing behavior tree: %s", e.what());
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

    void publishWorkState(BT::NodeStatus bt_status)
    {
        auto& state = autonomous_loader_bt::GlobalState::getInstance();

        // Detect transition from a running state to an idle state, which signifies a task cycle has just finished.
        if ((prev_bt_status_ == BT::NodeStatus::RUNNING) && (bt_status == BT::NodeStatus::IDLE || bt_status == BT::NodeStatus::SUCCESS))
        {
            // Only update the timestamp if we are not in a pause/end state.
            if (!state.isPauseTask() && !state.isEndTask()) {
                last_task_complete_time_ = ros::Time::now();
                ROS_INFO("Task cycle finished. Entering 5-second cooldown (workstate=4).");
            }
        }
        prev_bt_status_ = bt_status;

        // Determine if we are inside the 5-second window after task completion
        bool in_cooldown_window = false;
        if(last_task_complete_time_.is_zero() == false)
        {
            ros::Duration dt = ros::Time::now() - last_task_complete_time_;
            if (dt.toSec() < 5.0) {
                in_cooldown_window = true;
            }
        }

        int8_t work_state = 0; // Default to Idle

        if (state.isPauseTask()) {
            work_state = 2; // Paused
        } else if (state.isEndTask()) {
            work_state = (bt_status == BT::NodeStatus::IDLE || bt_status == BT::NodeStatus::SUCCESS) ? 4 : 3; // Ended or Ending
        } else if (state.hasNewTask() || bt_status == BT::NodeStatus::RUNNING) {
            work_state = 1; // Executing
            // If we start a new task, cancel the cooldown window
            last_task_complete_time_ = ros::Time(0);
        } else if (in_cooldown_window) {
            work_state = 4; // Just finished (cooldown)
        } else {
            work_state = 0; // Idle
        }

        std_msgs::Int8 msg; msg.data = work_state;
        workstate_pub_.publish(msg);
        nh_.setParam("/workstate", static_cast<int>(work_state));
    }

    // ========== ROS回调函数 ==========

    void startTaskCallback(const autonomous_loader_msgs::TaskCommand::ConstPtr& msg)
    {
        // Publish the received task ID, whether it's a new task or a cancellation
        std_msgs::Int32 task_id_msg;
        task_id_msg.data = msg->task_id;
        task_id_pub_.publish(task_id_msg);

        if (msg->task_id == 0)
        {
            ROS_WARN("Received task cancellation request (task_id=0) via topic. Sending vehicle to park.");
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
            ROS_INFO("Sent new navigation goal to parking spot and halted BT (topic).");
            return;
        }

        ROS_INFO("Received start task command - Task ID: %d, Bin: %d, Hopper: %d", msg->task_id, msg->bin_id, msg->hopper_id);
        autonomous_loader_bt::Task task(msg->task_id, msg->bin_id, msg->hopper_id, msg->task_type);
        autonomous_loader_bt::GlobalState::getInstance().addTask(task);
        autonomous_loader_bt::GlobalState::getInstance().setTaskId(msg->task_id);
        autonomous_loader_bt::TaskStatusReporter::instance().setTaskId(msg->task_id);

        nh_.setParam("ArmBucketState", 2);
        ROS_INFO("Set param ArmBucketState=2 (prepare raise arm)");
    }

    void pauseTaskCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        autonomous_loader_bt::GlobalState::getInstance().setPauseTask(msg->data);
        ROS_INFO("Received pause task command: %s", msg->data ? "yes" : "no");
    }

    void endTaskCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        autonomous_loader_bt::GlobalState::getInstance().setEndTask(msg->data);
        ROS_INFO("Received end task command: %s", msg->data ? "yes" : "no");
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

            ROS_INFO("Sent new navigation goal to parking spot and halted BT.");
            res.huiying = true;
            return true;
        }

        ROS_INFO("Received service task: taskID=%d, bin=%d, hopper=%d", req.taskID, req.cang, req.dou);
        autonomous_loader_bt::Task task(req.taskID, req.cang, req.dou, "scoop");
        autonomous_loader_bt::GlobalState::getInstance().addTask(task);
        autonomous_loader_bt::GlobalState::getInstance().setTaskId(req.taskID);
        autonomous_loader_bt::TaskStatusReporter::instance().setTaskId(req.taskID);

        nh_.setParam("canggoal", static_cast<int>(req.cang));
        res.huiying = true;
        // The workstate will be set to 1 by the publishWorkState function on the next tick
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
    
    ROS_INFO("Autonomous loader robot behavior tree node started");
    
    ros::spin();
    
    return 0;
}