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
#include <fstream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <map>
#include <csignal>

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

// 鸣笛参数结构体
struct HornParams {
    bool enabled;
    double min_distance;
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

// 获取当前时间戳字符串（用于文件名）
static std::string getTimestampForFilename() {
    time_t now = time(nullptr);
    struct tm* tm_info = localtime(&now);
    char buffer[32];
    strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", tm_info);
    return std::string(buffer);
}

// 获取带毫秒的时间戳字符串（用于日志内容）
static std::string getTimestampWithMs() {
    ros::Time now = ros::Time::now();
    time_t sec = now.sec;
    struct tm* tm_info = localtime(&sec);
    char date_buffer[32];
    strftime(date_buffer, sizeof(date_buffer), "%Y-%m-%d %H:%M:%S", tm_info);
    int ms = now.nsec / 1000000;
    char full_buffer[64];
    snprintf(full_buffer, sizeof(full_buffer), "%s.%03d", date_buffer, ms);
    return std::string(full_buffer);
}

// 创建目录（递归创建）
static bool createDirectory(const std::string& path) {
    std::string cmd = "mkdir -p " + path;
    return system(cmd.c_str()) == 0;
}

// 障碍物类型枚举
enum class ObstacleType : int {
    NONE = 0,
    PEDESTRIAN = 1,
    VEHICLE = 2,
    OTHER = 3,
    PATH_INTRUSION = 4
};

// 触发的障碍物信息结构体（用于日志记录）
struct TriggeredObstacleInfo {
    bool valid;
    ObstacleType type;
    double min_dist;
    double obs_x, obs_y;
    double veh_x, veh_y, veh_yaw;
    std::string rule_type;
    int confirm_count;
};

// 单条日志记录
struct LogRecord {
    std::string timestamp;
    double min_dist;
    double obs_x, obs_y;
    double veh_x, veh_y, veh_yaw;
};

// 单个规则的日志数据
struct RuleLogData {
    std::vector<LogRecord> records;
    double global_min = 9999.0;      // 初始化为大值，C/D跳过规则使用-1
    ros::Time last_save_time;
};

// 前向声明
class ObstacleAvoider;

// 全局实例指针，用于信号处理
static ObstacleAvoider* g_avoider_instance = nullptr;

// 前向声明 signalHandler（定义在类之后）
void signalHandler(int sig);

class ObstacleAvoider;

class ObstacleAvoider
{
public:
    ObstacleAvoider() : nh_("~"),
        emergency_stop_hold_duration_(5.0),
        emergency_stop_hold_time_(ros::Time())
    {
        loadParams();
        initLogging();

        estop_pub_ = nh_.advertise<std_msgs::UInt8>(topics_["estop"], 1);
        horn_pub_ = nh_.advertise<std_msgs::UInt8>(topics_["horn"], 1);

        path_sub_ = nh_.subscribe(topics_["path"], 1, &ObstacleAvoider::pathCallback, this);
        perception_sub_ = nh_.subscribe(topics_["perception"], 1, &ObstacleAvoider::perceptionCallback, this);
        pointcloud_sub_ = nh_.subscribe(topics_["pointcloud"], 1, &ObstacleAvoider::pointcloudCallback, this);

        ROS_INFO("Obstacle Avoider started with unified min-distance collision detection.");

        // 注册全局实例和信号处理
        g_avoider_instance = this;
        signal(SIGINT, signalHandler);
    }

    ~ObstacleAvoider() {
        finalizeLogging();
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

        // 急停保持机制参数
        public_nh.param("emergency_stop_hold_duration", emergency_stop_hold_duration_, 5.0);

        // 鸣笛参数
        public_nh.param("horn/enabled", horn_params_.enabled, true);
        public_nh.param("horn/min_distance", horn_params_.min_distance, 20.0);
        public_nh.param("log_save_interval", log_save_interval_, 1.0);
        public_nh.param("skip_log_console_interval", skip_log_console_interval_, 1.0);
        public_nh.param("rule_a_confirm_frames", rule_a_confirm_frames_, 2);
        public_nh.param("rule_b_confirm_frames", rule_b_confirm_frames_, 2);

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
        // ======= 【新增这一行：只要Bag在放，这里就会疯狂更新】 =======
        last_perception_recv_time_ = ros::Time::now(); 
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

    // ## 核心：统一最小距离判断（重载版本）
    double computeMinDistance(const lidar_camera_fusion::Obstacle& obstacle,
                            const lidar_camera_fusion::PerceptInfo& self_info,
                            const OrientedRect& vehicle_rect) {
        double local_x, local_y;
        getLocalCoordsFromObstacle(obstacle, self_info, local_x, local_y);
        Point2D obs_point(local_x, local_y);
        return minDistancePointToRect(obs_point, vehicle_rect);
    }

    double computeMinDistance(const lidar_camera_fusion::originObs& obstacle,
                            const lidar_camera_fusion::PerceptInfo& self_info,
                            const OrientedRect& vehicle_rect) {
        double local_x, local_y;
        getLocalCoordsFromObstacle(obstacle, self_info, local_x, local_y);

        if (obstacle.length > 0.0 && obstacle.width > 0.0) {
            OrientedRect obs_rect(local_x, local_y, obstacle.yaw,
                                  obstacle.length, obstacle.width);
            obs_rect.computeCorners();
            return minDistanceBetweenRects(vehicle_rect, obs_rect);
        }

        Point2D obs_point(local_x, local_y);
        return minDistancePointToRect(obs_point, vehicle_rect);
    }

    double computeMinDistance(const jsk_recognition_msgs::BoundingBox& bbox,
                            const lidar_camera_fusion::PerceptInfo& self_info,
                            const OrientedRect& vehicle_rect) {
        double bbox_yaw = quaternionToYaw(bbox.pose.orientation);
        OrientedRect obs_rect(bbox.pose.position.x, bbox.pose.position.y, bbox_yaw,
                              bbox.dimensions.x, bbox.dimensions.y);
        obs_rect.computeCorners();
        return minDistanceBetweenRects(vehicle_rect, obs_rect);
    }

    // ## 判断障碍物是否在车辆正前方
    template<typename T>
    bool isInFront(const T& obstacle, const lidar_camera_fusion::PerceptInfo& self_info) {
        double local_x, local_y;
        getLocalCoordsFromObstacle(obstacle, self_info, local_x, local_y);
        return local_x > 0;
    }

    // ## 主检测函数：统一逻辑处理所有障碍物
// ## 主检测函数：统一逻辑处理所有障碍物
// ## 主检测函数：统一逻辑处理所有障碍物
    void checkObstacles()
    {
        // 0. 读取 /ArmBucketState，判断是否需要清空路径（在加锁前读取，避免死锁）
        int arm_bucket_state = 0;
        ros::param::get("/ArmBucketState", arm_bucket_state);
        if (last_workstate_ != -1 && (arm_bucket_state == 6 || arm_bucket_state == 7)) {
            ROS_WARN("[ObstacleAvoider] workstate changed to %d, will clear path.", arm_bucket_state);
            need_clear_path_ = true;
        }
        last_workstate_ = arm_bucket_state;

        // 1. 获取数据
        data_mutex_.lock();
        if (need_clear_path_) {
            if (!current_path_.poses.empty()) {
                ROS_WARN("[ObstacleAvoider] Clearing path (was %zu poses).", current_path_.poses.size());
            }
            current_path_.poses.clear();
            current_path_.header.stamp = ros::Time::now();
            need_clear_path_ = false;
        }
        nav_msgs::Path path = current_path_;
        lidar_camera_fusion::PerceptInfo self_info = percept_info_;
        jsk_recognition_msgs::BoundingBoxArray bbox_obstacles = bbox_array_;
        self_info.loc.x = (self_info.loc.x / 20.0) - 20.924917;
        self_info.loc.y = (self_info.loc.y / 20.0) - 90.169417;
        data_mutex_.unlock();

        // ======= 【修改后的超时保护：使用接收时间差判断】 =======
        ros::Time now = ros::Time::now(); // 这里的 now 保留
        if (!last_perception_recv_time_.isZero() && (now - last_perception_recv_time_).toSec() > 0.5) {
            ROS_WARN_THROTTLE(2.0, "[Timeout] Bag paused or topic stopped! Skipping obstacle check.");
            resetConfirmationCounter();
            return;
        }
        // ========================================================

        bool is_in_stockyard = self_info.loc.x > stockyard_x_threshold_;
        bool arm_is_raising = (arm_bucket_state == arm_raising_state_);
        bool disable_pointcloud_in_current_state =
            is_in_stockyard &&
            std::find(stockyard_pointcloud_disable_states_.begin(),
                      stockyard_pointcloud_disable_states_.end(),
                      arm_bucket_state) != stockyard_pointcloud_disable_states_.end();

        // 3. 构建车辆局部坐标系下的车体矩形（车体坐标系中车身朝向恒为 0）
        OrientedRect vehicle_rect(0, 0, 0.0, vehicle_model_.length, vehicle_model_.width);
        vehicle_rect.computeCorners();

        bool estop_triggered = false;  // 本次循环是否触发了急停
        bool horn_triggered = false;   // 本次循环是否触发了鸣笛
        TriggeredObstacleInfo obs_info;    // 本次触发的障碍物信息
        obs_info.valid = false;
        obs_info.confirm_count = 0;

        // 4. 处理 PerceptInfo.obstacles
        for (const auto& obs : self_info.obstacles) {
            if (processObstacle(obs, path, self_info, vehicle_rect, is_in_stockyard, arm_is_raising, horn_triggered, &obs_info)) {
                estop_triggered = true;
                break;
            }
        }

        // 5. 处理 PerceptInfo.originObss
        if (!estop_triggered) {
            for (const auto& obs : self_info.originObss) {
                if (processObstacle(obs, path, self_info, vehicle_rect, is_in_stockyard, arm_is_raising, horn_triggered, &obs_info)) {
                    estop_triggered = true;
                    break;
                }
            }
        }

        // 6. 处理 jsk_bboxes
        if (!estop_triggered) {
            for (const auto& bbox : bbox_obstacles.boxes) {
                if (processObstacle(bbox, path, self_info, vehicle_rect, disable_pointcloud_in_current_state, arm_is_raising, horn_triggered, &obs_info)) {
                    estop_triggered = true;
                    break;
                }
            }
        }

        // 6.5 如果没有障碍物命中，清除确认状态
        // 如果本轮循环没有触发 Rule_B，则单独清空 Rule_B 的连续帧计数
        if (rule_counters_.find("Rule_B") != rule_counters_.end()) {
            // 检查当前这次大循环里，Rule_B 是否被 processObstacle 真正访问并加过计数
            // 如果这一帧的计数并没有随着循环增长，说明这一帧没有障碍物威胁到车体边界，清除它
            static int last_b_count = 0;
            if (rule_counters_["Rule_B"] == last_b_count) {
                rule_counters_["Rule_B"] = 0;
            }
            last_b_count = rule_counters_["Rule_B"];
        }

        // 同理，如果本轮循环没有触发 Rule_A，则单独清空 Rule_A 的连续帧计数
        if (rule_counters_.find("Rule_A") != rule_counters_.end()) {
            static int last_a_count = 0;
            if (rule_counters_["Rule_A"] == last_a_count) {
                rule_counters_["Rule_A"] = 0;
            }
            last_a_count = rule_counters_["Rule_A"];
        }
        
        if (rule_counters_.find("Rule_A_PEDESTRIAN") != rule_counters_.end()) {
            static int last_a_ped_count = 0;
            if (rule_counters_["Rule_A_PEDESTRIAN"] == last_a_ped_count) {
                rule_counters_["Rule_A_PEDESTRIAN"] = 0;
            }
            last_a_ped_count = rule_counters_["Rule_A_PEDESTRIAN"];
        }

        // 6.6 如果触发了急停，按规则限频记录日志
        if (estop_triggered && obs_info.valid) {
            LogRecord record;
            record.timestamp = getTimestampWithMs();
            record.min_dist = obs_info.min_dist;
            record.obs_x = obs_info.obs_x;
            record.obs_y = obs_info.obs_y;
            record.veh_x = obs_info.veh_x;
            record.veh_y = obs_info.veh_y;
            record.veh_yaw = obs_info.veh_yaw;

            saveRuleLogWithThrottle(obs_info.rule_type, record, obs_info.min_dist);
        }

        // 6. 急停保持机制：当前时刻急停且保持时间有效
        bool estop_final = false;
        // ros::Time now = ros::Time::now();

        if (estop_triggered) {
            // 立即触发急停，更新保持时间起点
            emergency_stop_hold_time_ = now;
            estop_final = true;
        } else if (emergency_stop_hold_duration_ > 0.0 && !emergency_stop_hold_time_.isZero()) {
            // Check if still within hold duration
            double elapsed = (now - emergency_stop_hold_time_).toSec();
            if (elapsed < emergency_stop_hold_duration_) {
                estop_final = true;
                if (elapsed < emergency_stop_hold_duration_ * 0.5) {
                    ROS_INFO_THROTTLE(1.0, "[E-Stop Hold] Obstacle cleared, holding stop (%.1f/%.1fs)",
                                      elapsed, emergency_stop_hold_duration_);
                }
            } else {
                emergency_stop_hold_time_ = ros::Time();
                ROS_INFO("[E-Stop Hold] Duration exceeded, releasing stop");
            }
        }

        // 7. 鸣笛逻辑：本次触发了鸣笛或者在急停保持期间鸣笛
        bool horn_final = horn_triggered;
        if (!horn_final && horn_params_.enabled && estop_final && !emergency_stop_hold_time_.isZero()) {
            // 在急停保持期间，检查是否需要持续鸣笛（检查最近一次鸣笛触发障碍物的距离）
            // 简化为：急停保持期间持续鸣笛
            horn_final = true;
        }

        // 8. 发布结果
        std_msgs::UInt8 estop_msg;
        estop_msg.data = estop_final ? 1 : 0;
        std_msgs::UInt8 horn_msg;
        horn_msg.data = horn_final ? 1 : 0;
        estop_pub_.publish(estop_msg);
        horn_pub_.publish(horn_msg);
    }

    bool updateConfirmationCounter(const std::string& rule_type, int required_frames, int& counter,
        double obs_x, double obs_y) {
    if (required_frames <= 1) {
    counter = 1;
    return true;
    }
    // 针对当前触发的规则类型独立累加
    rule_counters_[rule_type]++; 
    counter = rule_counters_[rule_type];
    return counter >= required_frames;
    }

    void resetConfirmationCounter() {
    rule_counters_.clear(); // 清空所有规则的计数
    }

    void saveRuleLogWithThrottle(const std::string& rule_type, const LogRecord& record, double min_dist_for_global_min) {
        auto it = rule_logs_.find(rule_type);
        if (it == rule_logs_.end()) {
            RuleLogData data;
            data.global_min = min_dist_for_global_min;
            data.last_save_time = ros::Time(0);
            rule_logs_[rule_type] = data;
            it = rule_logs_.find(rule_type);
        }

        if (min_dist_for_global_min < it->second.global_min) {
            it->second.global_min = min_dist_for_global_min;
        }

        ros::Time now_for_log = ros::Time::now();
        bool should_save_log = it->second.last_save_time.isZero() ||
                               (now_for_log - it->second.last_save_time).toSec() >= log_save_interval_;
        if (!should_save_log) {
            return;
        }

        it->second.records.push_back(record);
        it->second.last_save_time = now_for_log;
        ROS_INFO("[Log] Saved to %s.log (count: %zu)", rule_type.c_str(), it->second.records.size());
    }

    // ## 统一障碍物处理逻辑
    // 返回 true 表示触发了急停，false 表示未触发
    // 通过引用参数 horn_triggered 返回是否触发鸣笛
    // 通过 obs_info 输出障碍物信息用于日志记录
    template<typename T>
    bool processObstacle(const T& obstacle, const nav_msgs::Path& path,
                        const lidar_camera_fusion::PerceptInfo& self_info,
                        const OrientedRect& vehicle_rect,
                        bool skip_pointcloud_for_current_state, bool arm_is_raising,
                        bool& horn_triggered,
                        TriggeredObstacleInfo* obs_info = nullptr)
    {
        // --- Skip conditions ---
        // Rule D: Ignore front obstacles when arm is raising
        if (arm_is_raising && isInFront(obstacle, self_info)) {
            double local_x, local_y;
            getLocalCoordsFromObstacle(obstacle, self_info, local_x, local_y);
            ROS_INFO_THROTTLE(skip_log_console_interval_, "[Rule D] Skip front obstacle while arm raising: local=(%.2f, %.2f)",
                              local_x, local_y);

            double obs_x, obs_y;
            getGlobalCoordsFromObstacle(obstacle, self_info, obs_x, obs_y);
            LogRecord record;
            record.timestamp = getTimestampWithMs();
            record.min_dist = -1.0;
            record.obs_x = obs_x;
            record.obs_y = obs_y;
            record.veh_x = self_info.loc.x;
            record.veh_y = self_info.loc.y;
            record.veh_yaw = self_info.yaw;
            saveRuleLogWithThrottle("Rule_D", record, -1.0);
            return false;
        }

        // Rule C: Skip pointcloud obstacles in configured stockyard states
        if (std::is_same<T, jsk_recognition_msgs::BoundingBox>::value && skip_pointcloud_for_current_state) {
            double local_x, local_y;
            getLocalCoordsFromObstacle(obstacle, self_info, local_x, local_y);
            ROS_INFO_THROTTLE(skip_log_console_interval_,
                              "[Rule C] Skip pointcloud obstacle in stockyard state: local=(%.2f, %.2f)",
                              local_x, local_y);

            double obs_x, obs_y;
            getGlobalCoordsFromObstacle(obstacle, self_info, obs_x, obs_y);
            LogRecord record;
            record.timestamp = getTimestampWithMs();
            record.min_dist = -1.0;
            record.obs_x = obs_x;
            record.obs_y = obs_y;
            record.veh_x = self_info.loc.x;
            record.veh_y = self_info.loc.y;
            record.veh_yaw = self_info.yaw;
            saveRuleLogWithThrottle("Rule_C", record, -1.0);
            return false;
        }

        // --- Compute min distance ---
        double min_dist = computeMinDistance(obstacle, self_info, vehicle_rect);
        int classid = getObstacleClass(obstacle);
        double local_x, local_y;
        getLocalCoordsFromObstacle(obstacle, self_info, local_x, local_y);

        // --- Rule B: Body boundary check (min_dist <= 0 means overlap) ---
        if (false) {
        // if (min_dist <= safety_thresholds_.body_proximity) {
            int confirm_count = 0;
            bool confirmed = updateConfirmationCounter("Rule_B", rule_b_confirm_frames_, confirm_count,
                                                      local_x, local_y);
            ROS_WARN_THROTTLE(1.0,
                              "[Rule B] Candidate hit %d/%d, min_dist=%.2fm local=(%.2f, %.2f)",
                              confirm_count, std::max(1, rule_b_confirm_frames_), min_dist, local_x, local_y);

            if (confirmed && classid == 0) {
                ROS_WARN("[Rule B] Pedestrian too close! min_dist=%.2fm local=(%.2f, %.2f). E-Stop.",
                         min_dist, local_x, local_y);
            } else if (confirmed && classid == 1) {
                ROS_WARN("[Rule B] Vehicle too close! min_dist=%.2fm local=(%.2f, %.2f). E-Stop.",
                         min_dist, local_x, local_y);
            } else if (confirmed) {
                ROS_WARN("[Rule B] Obstacle too close! min_dist=%.2fm local=(%.2f, %.2f). E-Stop.",
                         min_dist, local_x, local_y);
            }

            // Fill obstacle info for logging
            if (obs_info && confirmed) {
                obs_info->valid = true;
                obs_info->min_dist = min_dist;
                obs_info->rule_type = "Rule_B";
                obs_info->confirm_count = confirm_count;
                obs_info->type = (classid == 0) ? ObstacleType::PEDESTRIAN :
                                 (classid == 1) ? ObstacleType::VEHICLE : ObstacleType::OTHER;
                double obs_x, obs_y;
                getGlobalCoordsFromObstacle(obstacle, self_info, obs_x, obs_y);
                obs_info->obs_x = obs_x;
                obs_info->obs_y = obs_y;
                obs_info->veh_x = self_info.loc.x;
                obs_info->veh_y = self_info.loc.y;
                obs_info->veh_yaw = self_info.yaw;
            }

            return confirmed;
        }

        // --- Rule A: Path intrusion check ---
        if (isOnPath(obstacle, path, self_info)) {
            bool should_horn = horn_params_.enabled &&
                              classid == 0 &&
                              min_dist < horn_params_.min_distance;
            const std::string confirmed_rule_type = should_horn ? "Rule_A_PEDESTRIAN" : "Rule_A";
            int confirm_count = 0;
            bool confirmed = updateConfirmationCounter(confirmed_rule_type, rule_a_confirm_frames_, confirm_count,
                                                    local_x, local_y);
            ROS_WARN_THROTTLE(1.0,
                              "[Rule A] Candidate hit %d/%d, min_dist=%.2fm local=(%.2f, %.2f)",
                              confirm_count, std::max(1, rule_a_confirm_frames_), min_dist, local_x, local_y);

            if (confirmed && should_horn) {
                ROS_WARN("[Rule A] Pedestrian on path! min_dist=%.2fm local=(%.2f, %.2f). Horn+Stop.",
                         min_dist, local_x, local_y);
                horn_triggered = true;
            } else if (confirmed) {
                ROS_WARN("[Rule A] Obstacle in path corridor! min_dist=%.2fm local=(%.2f, %.2f). E-Stop.",
                         min_dist, local_x, local_y);
            }

            // Fill obstacle info for logging
            if (obs_info && confirmed) {
                obs_info->valid = true;
                obs_info->min_dist = min_dist;
                obs_info->rule_type = confirmed_rule_type;
                obs_info->confirm_count = confirm_count;
                obs_info->type = (classid == 0) ? ObstacleType::PATH_INTRUSION : ObstacleType::OTHER;
                double obs_x, obs_y;
                getGlobalCoordsFromObstacle(obstacle, self_info, obs_x, obs_y);
                obs_info->obs_x = obs_x;
                obs_info->obs_y = obs_y;
                obs_info->veh_x = self_info.loc.x;
                obs_info->veh_y = self_info.loc.y;
                obs_info->veh_yaw = self_info.yaw;
            }

            return confirmed;
        }

        return false;
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
    HornParams horn_params_;
    std::vector<int> stockyard_pointcloud_disable_states_;
    int arm_raising_state_;
    double log_save_interval_;
    double skip_log_console_interval_;
    int rule_a_confirm_frames_;
    int rule_b_confirm_frames_;
    std::map<std::string, int> rule_counters_;
    ros::Time last_perception_recv_time_; 

    // 急停保持机制
    double emergency_stop_hold_duration_;  // E-stop hold duration (seconds)

    // 急停保持机制
    ros::Time emergency_stop_hold_time_;   // Last e-stop trigger time

    std::mutex data_mutex_;
    nav_msgs::Path current_path_;
    lidar_camera_fusion::PerceptInfo percept_info_;
    jsk_recognition_msgs::BoundingBoxArray bbox_array_;

    // 状态切换检测：workstate 变为 6 或 7 时标记清空路径
    int last_workstate_ = -1;
    bool need_clear_path_ = false;

    // ===== Logging related =====
public:
    void finalizeLogging();  // Called by signal handler

private:
    void initLogging() {
        if (!createDirectory(log_directory_)) {
            ROS_WARN("[Log] Failed to create base directory: %s", log_directory_.c_str());
            return;
        }

        session_dir_ = log_directory_ + "/" + getTimestampForFilename();
        if (!createDirectory(session_dir_)) {
            ROS_WARN("[Log] Failed to create session directory: %s", session_dir_.c_str());
            return;
        }

        ROS_INFO("[Log] Log directory initialized: %s", session_dir_.c_str());
    }

    std::string session_dir_;                              // Session log directory
    std::string log_directory_;                           // Base log directory
    std::map<std::string, RuleLogData> rule_logs_;         // Logs per rule
};

// Implementation of finalizeLogging (needs class definition first)
void ObstacleAvoider::finalizeLogging() {
    if (session_dir_.empty()) return;

    ROS_INFO("[Log] Writing log summaries to session: %s", session_dir_.c_str());

    for (const auto& pair : rule_logs_) {
        const std::string& rule_type = pair.first;
        const RuleLogData& data = pair.second;

        if (data.records.empty()) continue;

        std::string filepath = session_dir_ + "/" + rule_type + ".log";
        std::ofstream out_file;
        out_file.open(filepath, std::ios::out);

        if (!out_file.is_open()) {
            ROS_WARN("[Log] Failed to write log file: %s", filepath.c_str());
            continue;
        }

        // Write header
        out_file << "========================================" << std::endl;
        out_file << "Rule Type: " << rule_type << std::endl;
        out_file << "Total Records: " << data.records.size() << std::endl;
        out_file << "========================================" << std::endl;
        out_file << std::endl;

        // Write each record
        for (size_t i = 0; i < data.records.size(); ++i) {
            const LogRecord& rec = data.records[i];
            out_file << "--- Record " << (i + 1) << " ---" << std::endl;
            out_file << "Timestamp: " << rec.timestamp << std::endl;
            out_file << "Min Distance: " << std::fixed << std::setprecision(2) << rec.min_dist << " m" << std::endl;
            out_file << "Obstacle Pos: (" << std::fixed << std::setprecision(3) << rec.obs_x << ", " << rec.obs_y << ") m" << std::endl;
            out_file << "Vehicle Pos: (" << std::fixed << std::setprecision(3) << rec.veh_x << ", " << rec.veh_y << ", " << rec.veh_yaw << ") m/rad" << std::endl;
            out_file << std::endl;
        }

        // Write minimum summary
        out_file << "========================================" << std::endl;
        out_file << "MIN: " << std::fixed << std::setprecision(2) << data.global_min << " m" << std::endl;
        out_file << "========================================" << std::endl;

        out_file.close();
        ROS_INFO("[Log] Wrote %s.log with %zu records, MIN=%.2fm",
                 rule_type.c_str(), data.records.size(), data.global_min);
    }

    if (rule_logs_.empty()) {
        ROS_INFO("[Log] No obstacle events recorded during this session.");
    }
}

// Ctrl+C signal handler
void signalHandler(int sig) {
    ROS_INFO("[ObstacleAvoider] Received shutdown signal, saving log summaries...");
    if (g_avoider_instance != nullptr) {
        g_avoider_instance->finalizeLogging();
    }
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoider_node");
    ObstacleAvoider avoider;
    avoider.run();
    return 0;
}
