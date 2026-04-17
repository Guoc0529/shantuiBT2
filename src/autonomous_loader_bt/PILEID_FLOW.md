# pileID 请求铲料点流程详解

## 📊 完整数据流

```
任务对象（Task）
    bin_id: 1, 2, 3... (料仓编号)
         ↓
RequestScoopPoint::tick() (第626行)
         ↓
pileID = bin_id (第634行)
    uint8_t pileID = static_cast<uint8_t>(current_task.bin_id);
         ↓
调用 ROS Service (第640行)
    topic_manager.requestScoopPoint(pileID, scoop_point)
         ↓
发送到服务端 /spadingPose
    请求: {pileID: 1, 2, 3...}
         ↓
服务端根据 pileID 计算铲料点
    pileID=1 → 料仓1的铲料点
    pileID=2 → 料仓2的铲料点
         ↓
返回铲料点位置
    响应: {pose_output: geometry_msgs/Pose}
         ↓
存储到 GlobalState
    state.setScoopPoint(scoop_point)
```

## 🔍 关键代码片段

### 1. 获取任务信息并转换
```cpp
// 文件: loader_bt_nodes.cpp 第630-634行
Task current_task = state.getCurrentTask();
uint8_t pileID = static_cast<uint8_t>(current_task.bin_id);
```
**说明**: 从当前任务获取 `bin_id`（料仓编号），转换为 `pileID`

### 2. 调用服务
```cpp
// 文件: loader_bt_nodes.cpp 第640行
topic_manager.requestScoopPoint(pileID, scoop_point)
```

### 3. 服务端接收
```cpp
// 文件: ros_topic_manager.h 第91-110行
bool requestScoopPoint(uint8_t pileID, geometry_msgs::PoseStamped& result)
{
    autonomous_loader_msgs::spadingPose srv;
    srv.request.pileID = pileID;  // ← 这里传入pileID
    
    if (spading_pose_client_.call(srv))
    {
        result.pose = srv.response.pose_output;
        // 返回铲料点位置
    }
}
```

## 💡 关键概念

### pileID = bin_id (料仓编号)

| 变量名 | 含义 | 类型 |
|--------|------|------|
| `bin_id` | 料仓编号（任务中的） | int |
| `pileID` | 料仓编号（服务请求参数） | uint8_t |
| **功能**: 两者表示同一个东西，只是在不同场景下的命名 |

## 📋 示例

假设收到任务：`{task_id: 1, bin_id: 2, hopper_id: 3}`

```
1. GetAndSetCurrentTask 设置当前任务
   bin_id = 2
   
2. RequestScoopPoint 执行
   pileID = bin_id = 2
   
3. 调用 /spadingPose 服务
   请求: {pileID: 2}
   
4. 服务端处理
   根据 pileID=2 找到料仓2的位置
   计算该料仓的铲料点
   
5. 返回结果
   响应: {pose_output: {x, y, z, orientation}}
   x, y, z 是铲料点的坐标
```

## 🎯 服务端需要实现的功能

```cpp
bool spadingPoseCallback(autonomous_loader_msgs::spadingPose::Request& req,
                         autonomous_loader_msgs::spadingPose::Response& res)
{
    uint8_t pileID = req.pileID;  // 接收 pileID（料仓编号）
    
    // 根据 pileID 获取料仓信息
    // 例如：pileID=1 → 料仓1，pileID=2 → 料仓2
    
    geometry_msgs::Pose scoop_pose;
    
    switch(pileID) {
        case 1:
            // 计算料仓1的铲料点
            scoop_pose.position.x = 10.0;
            scoop_pose.position.y = 10.0;
            break;
        case 2:
            // 计算料仓2的铲料点
            scoop_pose.position.x = 15.0;
            scoop_pose.position.y = 15.0;
            break;
        // ...
    }
    
    res.pose_output = scoop_pose;
    return true;
}
```

## ✅ 总结

**是的，通过 pileID 去请求对应的料仓位置！**

- `pileID` = `bin_id`（料仓编号）
- `pileID` 用于标识要铲料的料仓
- 服务端根据 `pileID` 计算并返回对应的铲料点位置
- 铲料点是该料仓内的一个具体位置坐标

