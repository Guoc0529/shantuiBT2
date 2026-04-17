#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Path.h>
#include <lidar_camera_fusion/PerceptInfo.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>

// ## 结构体定义 ##

// 用于存储路径入侵规则的参数
struct PathIntrusionRule {
    double corridor_width; // 安全走廊宽度
    struct HumanAlert { // 针对行人的特殊警报规则
        bool enabled;
        double max_distance;
    } human_alert;
};

// 车辆尺寸模型
struct VehicleModel {
    double length;
    double width;
    double safety_margin;
};

class ObstacleAvoider
{
public:
    ObstacleAvoider() : nh_("~") // 使用私有节点句柄 "~"，允许从launch文件加载参数
    {
        loadParams(); // 初始化时加载所有参数

        // 初始化发布者
        estop_pub_ = nh_.advertise<std_msgs::UInt8>(topics_["estop"], 1);
        horn_pub_ = nh_.advertise<std_msgs::UInt8>(topics_["horn"], 1);

        // 初始化订阅者
        path_sub_ = nh_.subscribe(topics_["path"], 1, &ObstacleAvoider::pathCallback, this);
        perception_sub_ = nh_.subscribe(topics_["perception"], 1, &ObstacleAvoider::perceptionCallback, this);
        pointcloud_sub_ = nh_.subscribe(topics_["pointcloud"], 1, &ObstacleAvoider::pointcloudCallback, this);

        ROS_INFO("Obstacle Avoider node started with RECTANGULAR vehicle model and dual coordinate system handling.");
    }

    // 节点主循环
    void run()
    {
        ros::Rate rate(10); // 设置循环频率为 10 Hz
        while (ros::ok())
        {
            checkObstacles(); // 每个循环执行一次障碍物检查
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // 从参数服务器加载配置
    void loadParams()
    {
        ros::NodeHandle public_nh; // 创建一个公共节点句柄以从全局命名空间加载参数

        public_nh.getParam("topics", topics_);
        public_nh.param("stockyard_x_threshold", stockyard_x_threshold_, -3.0);

        public_nh.param("vehicle_model/length", vehicle_model_.length, 10.0);
        public_nh.param("vehicle_model/width", vehicle_model_.width, 3.0);
        public_nh.param("vehicle_model/safety_margin", vehicle_model_.safety_margin, 0.5);

        public_nh.param("path_intrusion_rule/corridor_width", path_intrusion_rule_.corridor_width, 6.0);
        public_nh.param("path_intrusion_rule/human_alert/enabled", path_intrusion_rule_.human_alert.enabled, true);
        public_nh.param("path_intrusion_rule/human_alert/max_distance", path_intrusion_rule_.human_alert.max_distance, 20.0);

        public_nh.getParam("stockyard_pointcloud_disable_states", stockyard_pointcloud_disable_states_);
        public_nh.param("arm_raising_state", arm_raising_state_, 4);
    }

    // ## 回调函数 ##
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        ROS_INFO_ONCE("Received first path with %zu poses.", msg->poses.size());
        std::lock_guard<std::mutex> lock(data_mutex_);
        current_path_ = *msg;
    }
    void perceptionCallback(const lidar_camera_fusion::PerceptInfo::ConstPtr& msg) { std::lock_guard<std::mutex> lock(data_mutex_); percept_info_ = *msg; }
    void pointcloudCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg) { std::lock_guard<std::mutex> lock(data_mutex_); bbox_array_ = *msg; }

    // --- #################### 核心逻辑 #################### ---
    void checkObstacles()
    {
        data_mutex_.lock();
        nav_msgs::Path path = current_path_;
        lidar_camera_fusion::PerceptInfo self_info = percept_info_;
        jsk_recognition_msgs::BoundingBoxArray bbox_obstacles = bbox_array_;
        self_info.loc.x = (self_info.loc.x / 20.0) - 20.924917;
        self_info.loc.y = (self_info.loc.y / 20.0) - 90.169417;
        
        data_mutex_.unlock();

        int arm_bucket_state = 0;
        ros::param::get("/ArmBucketState", arm_bucket_state);

        bool is_in_stockyard = self_info.loc.x > stockyard_x_threshold_;
        bool arm_is_raising = (arm_bucket_state == arm_raising_state_);

        bool estop = false;
        bool horn = false;

        // 1. 处理全局坐标系下的障碍物 (PerceptInfo.obstacles)
        for (const auto& obs : self_info.obstacles) {
            processObstacle(obs, path, self_info, is_in_stockyard, arm_is_raising, estop, horn);
            if (estop) break;
        }
        // 2. 处理车身坐标系下的障碍物 (PerceptInfo.originObss)
        if (!estop) {
            for (const auto& obs : self_info.originObss) {
                processObstacle(obs, path, self_info, is_in_stockyard, arm_is_raising, estop, horn);
                if (estop) break;
            }
        }
        // 3. 处理车身坐标系下的点云障碍物 (jsk_bboxes)
        if (!estop) {
            for (const auto& bbox : bbox_obstacles.boxes) {
                processObstacle(bbox, path, self_info, is_in_stockyard, arm_is_raising, estop, horn);
                if (estop) break;
            }
        }

        std_msgs::UInt8 estop_msg; estop_msg.data = estop ? 1 : 0;
        std_msgs::UInt8 horn_msg; horn_msg.data = horn ? 1 : 0;
        estop_pub_.publish(estop_msg);
        horn_pub_.publish(horn_msg);
    }

    template<typename T>
    void processObstacle(const T& obstacle, const nav_msgs::Path& path, const lidar_camera_fusion::PerceptInfo& self_info, 
                         bool is_in_stockyard, bool arm_is_raising, bool& estop, bool& horn)
    {
        if (arm_is_raising && isObstacleInFront(obstacle, self_info)) {
            return;
        }

        if (isObstacleOnPath(obstacle, path, self_info, path_intrusion_rule_.corridor_width)) {
            double distance = getDistanceToObstacle(obstacle, self_info);
            if (getObstacleClass(obstacle) == 0 && path_intrusion_rule_.human_alert.enabled && 
                distance < path_intrusion_rule_.human_alert.max_distance) {
                ROS_WARN("[Rule A Triggered] Human on path! Class: %d, Distance: %.1fm. Horn and E-Stop!", getObstacleClass(obstacle), distance);
                horn = true;
            } else {
                ROS_WARN("[Rule A Triggered] Obstacle on path corridor! Class: %d, Distance: %.1fm. E-Stop!", getObstacleClass(obstacle), distance);
            }
            estop = true;
            return;
        }

        if (getObstacleClass(obstacle) == 0 || getObstacleClass(obstacle) == 1) { // Human or Vehicle
            if (isObstacleNearBody(obstacle, self_info)) {
                ROS_WARN("[Rule B Triggered] Key target near body! Class: %d, Distance: %.1fm. E-Stop!", getObstacleClass(obstacle), getDistanceToObstacle(obstacle, self_info));
                estop = true;
                return;
            }
        }

        if (std::is_same<T, jsk_recognition_msgs::BoundingBox>::value) {
            if (is_in_stockyard) {
                return;
            }
            if (!is_in_stockyard) {
                if (isObstacleNearBody(obstacle, self_info)) {
                    ROS_WARN("[Rule C Triggered] Pointcloud obstacle near body in corridor! Distance: %.1fm. E-Stop!", getDistanceToObstacle(obstacle, self_info));
                    estop = true;
                    return;
                }
            }
        }
    }

    // --- ## 辅助函数 ## ---

    // ## 核心算法 1: 判断障碍物是否在全局路径上 ##
    template<typename T>
    bool isObstacleOnPath(const T& obstacle, const nav_msgs::Path& path, const lidar_camera_fusion::PerceptInfo& self_info, double corridor_width) {
        if (path.poses.empty()) return false;

        double global_x, global_y;
        getGlobalObstacleCoords(obstacle, self_info, global_x, global_y);

        double half_width_sq = std::pow(corridor_width / 2.0, 2);
        for (const auto& pose : path.poses) {
            double dist_sq = std::pow(global_x - pose.pose.position.x, 2) + std::pow(global_y - pose.pose.position.y, 2);
            ROS_INFO_THROTTLE(5.0, "[Path Check Debug] Obs Global(%.2f, %.2f) vs Path Point(%.2f, %.2f). DistSq: %.2f, ThreshSq: %.2f",
                global_x, global_y, pose.pose.position.x, pose.pose.position.y, dist_sq, half_width_sq);
            if (dist_sq < half_width_sq) return true;
        }
        return false;
    }

    // ## 核心算法 2: 判断障碍物是否在车身矩形内 ##
    template<typename T>
    bool isObstacleNearBody(const T& obstacle, const lidar_camera_fusion::PerceptInfo& self_info) {
        double local_x, local_y;
        getLocalObstacleCoords(obstacle, self_info, local_x, local_y);

        double half_length = (vehicle_model_.length / 2.0) + vehicle_model_.safety_margin;
        double half_width = (vehicle_model_.width / 2.0) + vehicle_model_.safety_margin;

        return (std::abs(local_x) < half_length) && (std::abs(local_y) < half_width);
    }

    template<typename T>
    double getDistanceToObstacle(const T& obstacle, const lidar_camera_fusion::PerceptInfo& self_info) {
        double local_x, local_y;
        getLocalObstacleCoords(obstacle, self_info, local_x, local_y);
        return std::sqrt(local_x * local_x + local_y * local_y);
    }

    template<typename T>
    bool isObstacleInFront(const T& obstacle, const lidar_camera_fusion::PerceptInfo& self_info) {
        double local_x, local_y;
        getLocalObstacleCoords(obstacle, self_info, local_x, local_y);
        return local_x > 0;
    }

    // --- ## 坐标转换与取值函数 ## ---

    // 获取障碍物在车身坐标系下的坐标
    void getLocalObstacleCoords(const lidar_camera_fusion::Obstacle& obs, const lidar_camera_fusion::PerceptInfo& self, double& out_x, double& out_y) {
        double dx = obs.x - self.loc.x;
        double dy = obs.y - self.loc.y;
        double cos_yaw = cos(-self.yaw);
        double sin_yaw = sin(-self.yaw);
        out_x = dx * cos_yaw - dy * sin_yaw;
        out_y = dx * sin_yaw + dy * cos_yaw;
    }
    void getLocalObstacleCoords(const lidar_camera_fusion::originObs& obs, const lidar_camera_fusion::PerceptInfo&, double& out_x, double& out_y) {
        out_x = obs.x;
        out_y = obs.y;
    }
    void getLocalObstacleCoords(const jsk_recognition_msgs::BoundingBox& obs, const lidar_camera_fusion::PerceptInfo&, double& out_x, double& out_y) {
        out_x = obs.pose.position.x;
        out_y = obs.pose.position.y;
    }

    // 获取障碍物在全局坐标系下的坐标
    void getGlobalObstacleCoords(const lidar_camera_fusion::Obstacle& obs, const lidar_camera_fusion::PerceptInfo&, double& out_x, double& out_y) {
        out_x = obs.x;
        out_y = obs.y;
    }
    void getGlobalObstacleCoords(const lidar_camera_fusion::originObs& obs, const lidar_camera_fusion::PerceptInfo& self, double& out_x, double& out_y) {
        double cos_yaw = cos(self.yaw);
        double sin_yaw = sin(self.yaw);
        out_x = self.loc.x + (obs.x * cos_yaw - obs.y * sin_yaw);
        out_y = self.loc.y + (obs.x * sin_yaw + obs.y * cos_yaw);
    }
    void getGlobalObstacleCoords(const jsk_recognition_msgs::BoundingBox& obs, const lidar_camera_fusion::PerceptInfo& self, double& out_x, double& out_y) {
        double cos_yaw = cos(self.yaw);
        double sin_yaw = sin(self.yaw);
        out_x = self.loc.x + (obs.pose.position.x * cos_yaw - obs.pose.position.y * sin_yaw);
        out_y = self.loc.y + (obs.pose.position.y * sin_yaw + obs.pose.position.y * cos_yaw);
    }

    // 获取障碍物类别
    int getObstacleClass(const lidar_camera_fusion::Obstacle&) { return -1; } // Obstacle 消息没有类别ID
    int getObstacleClass(const lidar_camera_fusion::originObs& obs) { return obs.classid; }
    int getObstacleClass(const jsk_recognition_msgs::BoundingBox&) { return -1; } // BBox 消息没有类别ID

    // --- ## 成员变量 ## ---
    ros::NodeHandle nh_;
    ros::Publisher estop_pub_, horn_pub_;
    ros::Subscriber path_sub_, perception_sub_, pointcloud_sub_;

    std::map<std::string, std::string> topics_;
    double stockyard_x_threshold_;
    VehicleModel vehicle_model_;
    PathIntrusionRule path_intrusion_rule_;
    std::vector<int> stockyard_pointcloud_disable_states_;
    int arm_raising_state_;

    std::mutex data_mutex_;
    nav_msgs::Path current_path_;
    lidar_camera_fusion::PerceptInfo percept_info_;
    jsk_recognition_msgs::BoundingBoxArray bbox_array_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoider_node");
    ObstacleAvoider avoider;
    avoider.run();
    return 0;
}
