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

struct PathIntrusionRule {
    double corridor_width;
    struct HumanAlert {
        bool enabled;
        double max_distance;
    } human_alert;
};

struct VehicleModel {
    double length;
    double width;
    double safety_margin;
};

struct SafetyThresholds {
    double body_proximity;
    double path_corridor;
    double human_alert_distance;
};

struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double _x, double _y) : x(_x), y(_y) {}
};

struct OrientedRect {
    Point2D center;
    double yaw;
    double length;
    double width;
    std::vector<Point2D> corners;

    OrientedRect() : yaw(0), length(0), width(0) {}
    OrientedRect(double cx, double cy, double _yaw, double _length, double _width)
        : center(cx, cy), yaw(_yaw), length(_length), width(_width) {
        computeCorners();
    }

    void computeCorners() {
        corners.clear();
        double half_l = length / 2.0;
        double half_w = width / 2.0;
        double local_corners[4][2] = {
            {-half_l, -half_w},
            { half_l, -half_w},
            { half_l,  half_w},
            {-half_l,  half_w}
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

// 点到线段的最小距离
static double pointToSegmentDistance(const Point2D& p, const Point2D& a, const Point2D& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double len_sq = dx * dx + dy * dy;
    if (len_sq < 1e-9) {
        return std::sqrt((p.x - a.x) * (p.x - a.x) + (p.y - a.y) * (p.y - a.y));
    }
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq;
    t = std::max(0.0, std::min(1.0, t));
    Point2D proj;
    proj.x = a.x + t * dx;
    proj.y = a.y + t * dy;
    return std::sqrt((p.x - proj.x) * (p.x - proj.x) + (p.y - proj.y) * (p.y - proj.y));
}

// 轴对齐包围盒检测
static bool rectsAABBOverlap(const OrientedRect& r1, const OrientedRect& r2) {
    double min1_x = r1.corners[0].x, max1_x = r1.corners[0].x;
    double min1_y = r1.corners[0].y, max1_y = r1.corners[0].y;
    double min2_x = r2.corners[0].x, max2_x = r2.corners[0].x;
    double min2_y = r2.corners[0].y, max2_y = r2.corners[0].y;
    for (const auto& c : r1.corners) {
        min1_x = std::min(min1_x, c.x); max1_x = std::max(max1_x, c.x);
        min1_y = std::min(min1_y, c.y); max1_y = std::max(max1_y, c.y);
    }
    for (const auto& c : r2.corners) {
        min2_x = std::min(min2_x, c.x); max2_x = std::max(max2_x, c.x);
        min2_y = std::min(min2_y, c.y); max2_y = std::max(max2_y, c.y);
    }
    return !(max1_x < min2_x || max2_x < min1_x || max1_y < min2_y || max2_y < min1_y);
}

// 旋转矩形间的最小距离（统一接口）
static double minDistanceBetweenRects(const OrientedRect& rect1, const OrientedRect& rect2) {
    if (rectsAABBOverlap(rect1, rect2)) {
        return 0.0;
    }
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < rect1.corners.size(); ++i) {
        const Point2D& edge_start = rect2.corners[i];
        const Point2D& edge_end = rect2.corners[(i + 1) % rect2.corners.size()];
        double d = pointToSegmentDistance(rect1.corners[i], edge_start, edge_end);
        min_dist = std::min(min_dist, d);
    }
    for (size_t i = 0; i < rect2.corners.size(); ++i) {
        const Point2D& edge_start = rect1.corners[i];
        const Point2D& edge_end = rect1.corners[(i + 1) % rect1.corners.size()];
        double d = pointToSegmentDistance(rect2.corners[i], edge_start, edge_end);
        min_dist = std::min(min_dist, d);
    }
    return min_dist;
}

// 点到旋转矩形最小距离（用于质心点障碍物）
static double minDistancePointToRect(const Point2D& point, const OrientedRect& rect) {
    OrientedRect point_rect(point.x, point.y, 0, 0, 0);
    return minDistanceBetweenRects(point_rect, rect);
}

// 从四元数提取 yaw 角
static double quaternionToYaw(const geometry_msgs::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

class ObstacleAvoider
{
public:
    ObstacleAvoider() : nh_("~")
    {
        loadParams();

        estop_pub_ = nh_.advertise<std_msgs::UInt8>(topics_["estop"], 1);
        horn_pub_ = nh_.advertise<std_msgs::UInt8>(topics_["horn"], 1);

        path_sub_ = nh_.subscribe(topics_["path"], 1, &ObstacleAvoider::pathCallback, this);
        perception_sub_ = nh_.subscribe(topics_["perception"], 1, &ObstacleAvoider::perceptionCallback, this);
        pointcloud_sub_ = nh_.subscribe(topics_["pointcloud"], 1, &ObstacleAvoider::pointcloudCallback, this);

        ROS_INFO("Obstacle Avoider started with unified min-distance collision detection.");
    }

    void run()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            checkObstacles();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void loadParams()
    {
        ros::NodeHandle public_nh;

        public_nh.getParam("topics", topics_);
        public_nh.param("stockyard_x_threshold", stockyard_x_threshold_, -3.0);

        public_nh.param("vehicle_model/length", vehicle_model_.length, 10.0);
        public_nh.param("vehicle_model/width", vehicle_model_.width, 3.0);
        public_nh.param("vehicle_model/safety_margin", vehicle_model_.safety_margin, 0.5);

        public_nh.param("safety_thresholds/body_proximity", safety_thresholds_.body_proximity, 0.0);
        public_nh.param("safety_thresholds/path_corridor", safety_thresholds_.path_corridor, 1.0);
        public_nh.param("safety_thresholds/human_alert_distance", safety_thresholds_.human_alert_distance, 20.0);

        public_nh.param("path_intrusion_rule/corridor_width", path_intrusion_rule_.corridor_width, 2.0);
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
    void perceptionCallback(const lidar_camera_fusion::PerceptInfo::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        percept_info_ = *msg;
    }
    void pointcloudCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        bbox_array_ = *msg;
    }

    // ## 坐标转换与取值函数 ##
    void getLocalCoordsFromObstacle(const lidar_camera_fusion::Obstacle& obs, const lidar_camera_fusion::PerceptInfo& self, double& out_x, double& out_y) {
        double dx = obs.x - self.loc.x;
        double dy = obs.y - self.loc.y;
        double cos_yaw = std::cos(-self.yaw);
        double sin_yaw = std::sin(-self.yaw);
        out_x = dx * cos_yaw - dy * sin_yaw;
        out_y = dx * sin_yaw + dy * cos_yaw;
    }
    void getLocalCoordsFromObstacle(const lidar_camera_fusion::originObs& obs, const lidar_camera_fusion::PerceptInfo&, double& out_x, double& out_y) {
        out_x = obs.x;
        out_y = obs.y;
    }
    void getLocalCoordsFromObstacle(const jsk_recognition_msgs::BoundingBox& obs, const lidar_camera_fusion::PerceptInfo&, double& out_x, double& out_y) {
        out_x = obs.pose.position.x;
        out_y = obs.pose.position.y;
    }

    void getGlobalCoordsFromObstacle(const lidar_camera_fusion::Obstacle& obs, const lidar_camera_fusion::PerceptInfo&, double& out_x, double& out_y) {
        out_x = obs.x;
        out_y = obs.y;
    }
    void getGlobalCoordsFromObstacle(const lidar_camera_fusion::originObs& obs, const lidar_camera_fusion::PerceptInfo& self, double& out_x, double& out_y) {
        double cos_yaw = std::cos(self.yaw);
        double sin_yaw = std::sin(self.yaw);
        out_x = self.loc.x + (obs.x * cos_yaw - obs.y * sin_yaw);
        out_y = self.loc.y + (obs.x * sin_yaw + obs.y * cos_yaw);
    }
    void getGlobalCoordsFromObstacle(const jsk_recognition_msgs::BoundingBox& obs, const lidar_camera_fusion::PerceptInfo& self, double& out_x, double& out_y) {
        double cos_yaw = std::cos(self.yaw);
        double sin_yaw = std::sin(self.yaw);
        out_x = self.loc.x + (obs.pose.position.x * cos_yaw - obs.pose.position.y * sin_yaw);
        out_y = self.loc.y + (obs.pose.position.x * sin_yaw + obs.pose.position.y * cos_yaw);
    }

    int getObstacleClass(const lidar_camera_fusion::Obstacle&) { return -1; }
    int getObstacleClass(const lidar_camera_fusion::originObs& obs) { return obs.classid; }
    int getObstacleClass(const jsk_recognition_msgs::BoundingBox&) { return -1; }

    // ## 路径入侵检测：障碍物全局坐标是否在路径走廊内
    template<typename T>
    bool isOnPath(const T& obstacle, const nav_msgs::Path& path, const lidar_camera_fusion::PerceptInfo& self_info) {
        if (path.poses.empty()) return false;
        double global_x, global_y;
        getGlobalCoordsFromObstacle(obstacle, self_info, global_x, global_y);
        double half_width = path_intrusion_rule_.corridor_width / 2.0;
        double half_width_sq = half_width * half_width;
        for (const auto& pose : path.poses) {
            double dist_sq = std::pow(global_x - pose.pose.position.x, 2) + std::pow(global_y - pose.pose.position.y, 2);
            if (dist_sq < half_width_sq) return true;
        }
        return false;
    }

    // ## 核心：统一最小距离判断
    template<typename T>
    double computeMinDistance(const T& obstacle, const lidar_camera_fusion::PerceptInfo& self_info, const OrientedRect& vehicle_rect) {
        double local_x, local_y;
        getLocalCoordsFromObstacle(obstacle, self_info, local_x, local_y);

        if (std::is_same<T, jsk_recognition_msgs::BoundingBox>::value) {
            const auto& bbox = obstacle;
            double bbox_yaw = quaternionToYaw(bbox.pose.orientation);
            OrientedRect obs_rect(bbox.pose.position.x, bbox.pose.position.y, bbox_yaw,
                                   bbox.dimensions.x, bbox.dimensions.y);
            obs_rect.computeCorners();
            return minDistanceBetweenRects(vehicle_rect, obs_rect);
        } else {
            Point2D obs_point(local_x, local_y);
            return minDistancePointToRect(obs_point, vehicle_rect);
        }
    }

    // ## 判断障碍物是否在车辆正前方
    template<typename T>
    bool isInFront(const T& obstacle, const lidar_camera_fusion::PerceptInfo& self_info) {
        double local_x, local_y;
        getLocalCoordsFromObstacle(obstacle, self_info, local_x, local_y);
        return local_x > 0;
    }

    // ## 主检测函数：统一逻辑处理所有障碍物
    void checkObstacles()
    {
        // 1. 获取数据
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

        // 2. 构建车辆旋转矩形
        OrientedRect vehicle_rect(0, 0, self_info.yaw, vehicle_model_.length, vehicle_model_.width);
        vehicle_rect.computeCorners();

        bool estop = false;
        bool horn = false;

        // 3. 处理 PerceptInfo.obstacles
        for (const auto& obs : self_info.obstacles) {
            processObstacle(obs, path, self_info, vehicle_rect, is_in_stockyard, arm_is_raising, estop, horn);
            if (estop) break;
        }

        // 4. 处理 PerceptInfo.originObss
        if (!estop) {
            for (const auto& obs : self_info.originObss) {
                processObstacle(obs, path, self_info, vehicle_rect, is_in_stockyard, arm_is_raising, estop, horn);
                if (estop) break;
            }
        }

        // 5. 处理 jsk_bboxes
        if (!estop) {
            for (const auto& bbox : bbox_obstacles.boxes) {
                processObstacle(bbox, path, self_info, vehicle_rect, is_in_stockyard, arm_is_raising, estop, horn);
                if (estop) break;
            }
        }

        // 6. 发布结果
        std_msgs::UInt8 estop_msg;
        estop_msg.data = estop ? 1 : 0;
        std_msgs::UInt8 horn_msg;
        horn_msg.data = horn ? 1 : 0;
        estop_pub_.publish(estop_msg);
        horn_pub_.publish(horn_msg);
    }

    // ## 统一障碍物处理逻辑
    template<typename T>
    void processObstacle(const T& obstacle, const nav_msgs::Path& path,
                        const lidar_camera_fusion::PerceptInfo& self_info,
                        const OrientedRect& vehicle_rect,
                        bool is_in_stockyard, bool arm_is_raising,
                        bool& estop, bool& horn)
    {
        // --- 跳过条件 ---
        // Rule D: 臂举升时忽略正前方障碍物
        if (arm_is_raising && isInFront(obstacle, self_info)) {
            return;
        }

        // Rule C: 点云障碍物在料场内时跳过
        if (std::is_same<T, jsk_recognition_msgs::BoundingBox>::value && is_in_stockyard) {
            return;
        }

        // --- 计算最小距离 ---
        double min_dist = computeMinDistance(obstacle, self_info, vehicle_rect);
        int classid = getObstacleClass(obstacle);

        // --- Rule B: 车身边界判断（最小距离 <= 0 表示重叠）---
        if (min_dist <= safety_thresholds_.body_proximity) {
            if (classid == 0) {
                ROS_WARN("[Rule B] 行人距离车辆过近！最小距离: %.2fm. 急停.", min_dist);
            } else if (classid == 1) {
                ROS_WARN("[Rule B] 车辆距离车辆过近！最小距离: %.2fm. 急停.", min_dist);
            } else {
                ROS_WARN("[Rule B] 障碍物距离车辆过近！最小距离: %.2fm. 急停.", min_dist);
            }
            estop = true;
            return;
        }

        // --- Rule A: 路径入侵判断 ---
        if (isOnPath(obstacle, path, self_info)) {
            if (classid == 0 && path_intrusion_rule_.human_alert.enabled &&
                min_dist < path_intrusion_rule_.human_alert.max_distance) {
                ROS_WARN("[Rule A] 行人在路径上！最小距离: %.2fm. 鸣笛+急停.", min_dist);
                horn = true;
            } else {
                ROS_WARN("[Rule A] 障碍物入侵路径走廊！最小距离: %.2fm. 急停.", min_dist);
            }
            estop = true;
            return;
        }
    }

    // ## 成员变量 ##
    ros::NodeHandle nh_;
    ros::Publisher estop_pub_, horn_pub_;
    ros::Subscriber path_sub_, perception_sub_, pointcloud_sub_;

    std::map<std::string, std::string> topics_;
    double stockyard_x_threshold_;
    VehicleModel vehicle_model_;
    SafetyThresholds safety_thresholds_;
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
