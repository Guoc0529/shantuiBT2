#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Path.h>
#include <lidar_camera_fusion/PerceptInfo.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/Quaternion.h>
#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <limits>

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

// 2D 点结构
struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double _x, double _y) : x(_x), y(_y) {}
};

// 旋转矩形结构（有朝向的矩形）
struct OrientedRect {
    Point2D center;   // 中心点（车身坐标系下）
    double yaw;       // 朝向角（弧度）
    double length;   // 长
    double width;    // 宽
    std::vector<Point2D> corners;  // 四个角点

    OrientedRect() : yaw(0), length(0), width(0) {}
    OrientedRect(double cx, double cy, double _yaw, double _length, double _width)
        : center(cx, cy), yaw(_yaw), length(_length), width(_width) {
        computeCorners();
    }

    void computeCorners() {
        corners.clear();
        double half_l = length / 2.0;
        double half_w = width / 2.0;
        // 矩形的四个角点（局部坐标系下）
        double local_corners[4][2] = {
            {-half_l, -half_w},  // 左后
            { half_l, -half_w},  // 右后
            { half_l,  half_w},  // 右前
            {-half_l,  half_w}   // 左前
        };
        double cos_yaw = std::cos(yaw);
        double sin_yaw = std::sin(yaw);
        for (int i = 0; i < 4; ++i) {
            double lx = local_corners[i][0];
            double ly = local_corners[i][1];
            Point2D corner;
            corner.x = center.x + lx * cos_yaw - ly * sin_yaw;
            corner.y = center.y + lx * sin_yaw + ly * cos_yaw;
            corners.push_back(corner);
        }
    }
};

// 计算点到线段的最小距离（点到线段两端点距离，或到线段的垂直距离）
static double pointToSegmentDistance(const Point2D& p, const Point2D& a, const Point2D& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double len_sq = dx * dx + dy * dy;
    if (len_sq < 1e-9) {
        // 线段退化为点
        return std::sqrt((p.x - a.x) * (p.x - a.x) + (p.y - a.y) * (p.y - a.y));
    }
    // 计算投影比例 t
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq;
    t = std::max(0.0, std::min(1.0, t));
    // 最近点
    Point2D proj;
    proj.x = a.x + t * dx;
    proj.y = a.y + t * dy;
    return std::sqrt((p.x - proj.x) * (p.x - proj.x) + (p.y - proj.y) * (p.y - proj.y));
}

// 计算两个有朝向矩形之间的最小距离
static double getMinDistanceBetweenRects(const OrientedRect& rect1, const OrientedRect& rect2) {
    double min_dist = std::numeric_limits<double>::max();

    // 遍历 rect1 的四个角点，计算到 rect2 四条边的最小距离
    for (size_t i = 0; i < rect1.corners.size(); ++i) {
        const Point2D& corner = rect1.corners[i];
        const Point2D& edge_start = rect2.corners[i];
        const Point2D& edge_end = rect2.corners[(i + 1) % rect2.corners.size()];
        double d = pointToSegmentDistance(corner, edge_start, edge_end);
        min_dist = std::min(min_dist, d);
    }

    // 遍历 rect2 的四个角点，计算到 rect1 四条边的最小距离
    for (size_t i = 0; i < rect2.corners.size(); ++i) {
        const Point2D& corner = rect2.corners[i];
        const Point2D& edge_start = rect1.corners[i];
        const Point2D& edge_end = rect1.corners[(i + 1) % rect1.corners.size()];
        double d = pointToSegmentDistance(corner, edge_start, edge_end);
        min_dist = std::min(min_dist, d);
    }

    return min_dist;
}

// 计算两矩形是否相交（简化为轴对齐包围盒检测）
static bool rectsOverlap(const OrientedRect& rect1, const OrientedRect& rect2) {
    // 使用旋转矩形角点计算 AABB
    double min1_x = rect1.corners[0].x, max1_x = rect1.corners[0].x;
    double min1_y = rect1.corners[0].y, max1_y = rect1.corners[0].y;
    double min2_x = rect2.corners[0].x, max2_x = rect2.corners[0].x;
    double min2_y = rect2.corners[0].y, max2_y = rect2.corners[0].y;

    for (const auto& c : rect1.corners) {
        min1_x = std::min(min1_x, c.x); max1_x = std::max(max1_x, c.x);
        min1_y = std::min(min1_y, c.y); max1_y = std::max(max1_y, c.y);
    }
    for (const auto& c : rect2.corners) {
        min2_x = std::min(min2_x, c.x); max2_x = std::max(max2_x, c.x);
        min2_y = std::min(min2_y, c.y); max2_y = std::max(max2_y, c.y);
    }

    // AABB 检测
    return !(max1_x < min2_x || max2_x < min1_x || max1_y < min2_y || max2_y < min1_y);
}

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

    // 从四元数提取 yaw 角
    static double quaternionToYaw(const geometry_msgs::Quaternion& q) {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // ## 核心算法 2: 判断障碍物是否在车身矩形内 ##
    // 对 BoundingBox 使用矩形碰撞检测，对其他类型使用质心判断
    bool isObstacleNearBody(const jsk_recognition_msgs::BoundingBox& bbox, const lidar_camera_fusion::PerceptInfo& self_info) {
        // 构建车辆矩形（车身坐标系，原点在中心，朝向为 self.yaw）
        OrientedRect vehicle_rect(0, 0, self_info.yaw, vehicle_model_.length, vehicle_model_.width);
        vehicle_rect.computeCorners();

        // 构建障碍物矩形（已是在车身坐标系下）
        double bbox_yaw = quaternionToYaw(bbox.pose.orientation);
        OrientedRect obstacle_rect(bbox.pose.position.x, bbox.pose.position.y, bbox_yaw,
                                    bbox.dimensions.x, bbox.dimensions.y);
        obstacle_rect.computeCorners();

        // 碰撞检测：使用 AABB 近似检测
        return rectsOverlap(vehicle_rect, obstacle_rect);
    }

    template<typename T>
    bool isObstacleNearBody(const T& obstacle, const lidar_camera_fusion::PerceptInfo& self_info) {
        double local_x, local_y;
        getLocalObstacleCoords(obstacle, self_info, local_x, local_y);

        double half_length = (vehicle_model_.length / 2.0) + vehicle_model_.safety_margin;
        double half_width = (vehicle_model_.width / 2.0) + vehicle_model_.safety_margin;

        return (std::abs(local_x) < half_length) && (std::abs(local_y) < half_width);
    }

    // ## 计算障碍物到车辆最近点距离
    // 对 BoundingBox 使用矩形间最小距离，对其他类型使用质心距离
    double getDistanceToObstacle(const jsk_recognition_msgs::BoundingBox& bbox, const lidar_camera_fusion::PerceptInfo& self_info) {
        // 构建车辆矩形
        OrientedRect vehicle_rect(0, 0, self_info.yaw, vehicle_model_.length, vehicle_model_.width);
        vehicle_rect.computeCorners();

        // 构建障碍物矩形
        double bbox_yaw = quaternionToYaw(bbox.pose.orientation);
        OrientedRect obstacle_rect(bbox.pose.position.x, bbox.pose.position.y, bbox_yaw,
                                   bbox.dimensions.x, bbox.dimensions.y);
        obstacle_rect.computeCorners();

        // 检查是否碰撞
        if (rectsOverlap(vehicle_rect, obstacle_rect)) {
            return 0.0;
        }

        // 计算最小距离
        return getMinDistanceBetweenRects(vehicle_rect, obstacle_rect);
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
        out_y = self.loc.y + (obs.pose.position.x * sin_yaw + obs.pose.position.y * cos_yaw);
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
