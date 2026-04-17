# PerceptInfo.msg
geometry_msgs/Point loc  # 位置信息，包含 x, y, z 坐标
float64 yaw               # 偏航角
Obstacle[] obstacles      # 全局坐标系
originObs[] originObss    # 新加：障碍物信息在车辆坐标系
uint8 obsWarningLevel     # 障碍物警告等级

