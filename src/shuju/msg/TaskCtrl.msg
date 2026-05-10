int32 taskID          # 任务编号
int32 ctrlCmd         # 控制指令码 (1-6)
                      #   1: 开始任务
                      #   2: 远程接管
                      #   3: 避障停车（需要位置信息）
                      #   4: 紧急停车
                      #   5: 继续任务
                      #   6: 结束任务
float64 target_x      # 目标X坐标（米），仅当 ctrlCmd=3 时有效
float64 target_y      # 目标Y坐标（米），仅当 ctrlCmd=3 时有效
float64 target_yaw    # 目标航向角（弧度），仅当 ctrlCmd=3 时有效
bool has_location     # 位置信息有效性标志
                      #   true: 坐标数据有效（ctrlCmd=3 时必须为 true）
                      #   false: 坐标数据无效或已重置为默认值
float64 timestamp     # Unix 时间戳（秒），用于检测新指令和验证数据时效性
