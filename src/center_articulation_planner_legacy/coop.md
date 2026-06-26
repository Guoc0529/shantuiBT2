# 一键启动系统 - 模块配合规范

## 一、系统概述

### 1.1 架构说明

```
┌─────────────────────────────────────────────────────────────────┐
│                     一键启动系统架构                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌────────────────────┐                                        │
│   │  diagnostic_system  │                                        │
│   │  (系统监控器)        │                                        │
│   │                     │                                        │
│   │  • fork 启动模块     │    roslaunch                          │
│   │  • 监控进程状态       │ ──────────────────────────────────────┼─> CAN to ROS
│   │  • 汇总诊断信息       │                                      │
│   │  • 显示状态表格       │ ──────────────────────────────────────┼─> Localization
│   │                     │                                      │─> Lidar Driver
│   │  • 监听 /diagnostics │                                      │
│   │  • 心跳超时检测       │ <─────────────────────────────────────┼─> Perception
│   │                     │        (1Hz DiagnosticArray)          │─> Camera Driver
│   └────────────────────┘                                      │─> Radar Driver
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 数据流

```
业务模块                          system_monitor
──────────────────────────────────────────────────────────────
                                    ▲
                                    │ /diagnostics (1Hz)
                                    │
┌───────────────────────┐          │
│   业务回调 (任意频率)    │          │
│   只写 health_ 状态     │──────────┘
│   不发 Diagnostic      │
└───────────────────────┘

┌───────────────────────┐          │
│   1Hz Timer           │──────────▶  汇总 → 显示 → 故障处理
│   发布 DiagnosticArray │
└───────────────────────┘
```

---

## 二、Diagnostic 数据规范

### 2.1 Topic 规范

| 属性 | 值 |
|------|-----|
| Topic | `/diagnostics` |
| 消息类型 | `diagnostic_msgs/DiagnosticArray` |
| 发布频率 | **1Hz**（（暂定）全系统统一）|

### 2.2 消息字段

```cpp
diagnostic_msgs::DiagnosticStatus st;
st.name       = "模块名";           // 必须与 modules.yaml 一致
st.level      = 0;                  // 0=OK, 1=WARN, 2=ERROR
st.message    = "状态描述";         // 界面可读
st.values     = [{key, value}];     // 结构化信息（可选）
```

### 2.3 状态级别定义

| level | 名称 | 含义 |
|-------|------|------|
| 0 | OK | 正常运行 |
| 1 | WARN | 警告（有异常但可工作）|
| 2 | ERROR | 错误（功能受损）|

### 2.4 错误代码规范

```
格式: XYY
  X  - 模块类别（顶层功能）
  YY - 具体错误编号


错误编号约定:
路径规划失败	7101	全局规划算法求解失败
规划服务不可用	7102	全局规划算法不在线
无距离反馈	7103	跟踪过程中，等待距离反馈信息超时
```

---

## 三、模块改造指南

### 3.1 改造流程图

```
开始改造
    │
    ▼
┌───────────────────────────────────────┐
│ Step 1: 添加健康状态缓存               │
│   struct ModuleHealth {               │
│     atomic<int> level;                │
│     string message;                   │
│     string error_code;                │
│     ros::Time last_update;            │
│   } health_;                          │
└───────────────────────────────────────┘
    │
    ▼
┌───────────────────────────────────────┐
│ Step 2: 初始化 Diagnostic             │
│   • advertise DiagnosticArray         │
│   • createTimer 1Hz                   │
│   • init last_update                   │
└───────────────────────────────────────┘
    │
    ▼
┌───────────────────────────────────────┐
│ Step 3: 修改业务回调                   │
│   • 原有逻辑保持不变                   │
│   • 末尾只写 health_                    │
│   • 不发 Diagnostic                    │
└───────────────────────────────────────┘
    │
    ▼
┌───────────────────────────────────────┐
│ Step 4: 实现 publishDiagnostic         │
│   • 读取 health_                       │
│   • 超时检测                           │
│   • 发布 DiagnosticArray               │
└───────────────────────────────────────┘
    │
    ▼
┌───────────────────────────────────────┐
│ Step 5: 更新 modules.yaml              │
│   • 添加 name                         │
│   • 添加 launch_cmd                    │
│   • 添加 missing_error_code            │
└───────────────────────────────────────┘
    │
    ▼
   完成
```

### 3.2 代码模板

#### 3.2.1 头文件声明

```cpp
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <atomic>

class YourModule {
private:
    // ========== 健康状态缓存 ==========
    // 唯一允许业务回调修改的结构体
    struct ModuleHealth {
        std::atomic<int> level{0};          // 0=OK, 1=WARN, 2=ERROR
        std::string message{"OK"};          // 状态描述
        std::string error_code{"0"};        // 错误代码
        ros::Time last_update;              // 心跳时间戳
    } health_;

    // ROS 成员
    ros::NodeHandle nh_;
    ros::Subscriber sub_;                   // 业务订阅
    ros::Publisher diag_pub_;              // Diagnostic 发布
    ros::Timer diag_timer_;                // 1Hz Timer

    // 回调函数
    void callback(const YourMsgType::ConstPtr& msg);
    void publishDiagnostic(const ros::TimerEvent&);

public:
    YourModule();
};
```

#### 3.2.2 构造函数

```cpp
YourModule::YourModule() {
    // 1. 订阅业务 topic（频率任意）
    sub_ = nh_.subscribe("/your_topic", 10, &YourModule::callback, this);

    // 2. 发布 DiagnosticArray（1Hz 由 Timer 控制）
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

    // 3. 创建 1Hz Timer
    diag_timer_ = nh_.createTimer(
        ros::Duration(1.0),
        &YourModule::publishDiagnostic,
        this
    );

    // 4. 初始化 last_update（防止启动时误报超时）
    health_.last_update = ros::Time::now();
}
```

#### 3.2.3 业务回调（核心改动）

```cpp
void YourModule::callback(const YourMsgType::ConstPtr& msg) {
    // ========== 原有业务逻辑（保持不动）==========
    bool has_error = false;

    // ... 你的原有判断逻辑 ...
    if (msg->ranges.empty()) {
        has_error = true;
    }
    // ... 更多业务判断 ...

    // ========== 增量改动：只写健康状态 ==========
    if (has_error) {
        health_.level.store(2);              // ERROR 级别
        health_.message = "Data invalid";   // 给用户看的描述
        health_.error_code = "3XXX";         // 按规范填写（见错误代码规范）
    } else {
        health_.level.store(0);               // OK
        health_.message = "OK";
        health_.error_code = "0";
    }
    health_.last_update = ros::Time::now();

    // ⚠️ 禁止在这里发布 Diagnostic！
    // ⚠️ 禁止有任何诊断相关的发布！
}
```

#### 3.2.4 诊断发布（Timer 回调）

```cpp
void YourModule::publishDiagnostic(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticStatus st;

    // 必须与 modules.yaml 中的 name 完全一致！
    st.name = "YourModuleName";

    // ===== 心跳超时检测（核心功能）=====
    ros::Duration dt = ros::Time::now() - health_.last_update;

    if (dt.toSec() > 2.0) {
        // 超时 → 模块可能崩溃或卡死
        st.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        st.message = "No data timeout (>2s)";
        st.values.clear();
        st.values.push_back({make_pair("error_code", "TIMEOUT")});
    } else {
        // 正常 → 使用业务回调写入的状态
        st.level = health_.level.load();
        st.message = health_.message;
        st.values.clear();
        st.values.push_back({make_pair("error_code", health_.error_code)});
    }

    // 发布 DiagnosticArray
    diagnostic_msgs::DiagnosticArray arr;
    arr.header.stamp = ros::Time::now();
    arr.status.push_back(st);
    diag_pub_.publish(arr);
}
```

### 3.3 modules.yaml 配置

在 `src/diagnostic_system/config/modules.yaml` 中添加：

```yaml
modules:
  # ============ 你的模块 ============
  - name: "YourModuleName"           # 必须与 st.name 完全一致
    launch_cmd: "roslaunch your_package your_launch.launch"
    missing_error_code: "XYYY"       # 错误代码（按归属分类填写，如 3010）
```

---

## 四、完整示例 - Lidar 模块

```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <atomic>

class LidarModule {
private:
    struct ModuleHealth {
        std::atomic<int> level{0};
        std::string message{"OK"};
        std::string error_code{"0"};
        ros::Time last_update;
    } health_;

    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher diag_pub_;
    ros::Timer diag_timer_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        bool has_error = false;

        if (msg->ranges.empty()) {
            has_error = true;
        }

        if (msg->angle_increment <= 0) {
            has_error = true;
        }

        if (has_error) {
            health_.level.store(2);
            health_.message = "Invalid scan data";
            health_.error_code = "2021";        // 定位层 (2XXX) + 1Y = 驱动错误
        } else {
            health_.level.store(0);
            health_.message = "OK";
            health_.error_code = "0";
        }
        health_.last_update = ros::Time::now();
    }

    void publishDiagnostic(const ros::TimerEvent&) {
        diagnostic_msgs::DiagnosticStatus st;
        st.name = "Lidar Driver";               // 与 modules.yaml 一致

        ros::Duration dt = ros::Time::now() - health_.last_update;

        if (dt.toSec() > 2.0) {
            st.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            st.message = "No data timeout (>2s)";
            st.values.clear();
            st.values.push_back({make_pair("error_code", "TIMEOUT")});
        } else {
            st.level = health_.level.load();
            st.message = health_.message;
            st.values.clear();
            st.values.push_back({make_pair("error_code", health_.error_code)});
        }

        diagnostic_msgs::DiagnosticArray arr;
        arr.header.stamp = ros::Time::now();
        arr.status.push_back(st);
        diag_pub_.publish(arr);
    }

public:
    LidarModule() {
        scan_sub_ = nh_.subscribe("/scan", 10, &LidarModule::scanCallback, this);
        diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
        diag_timer_ = nh_.createTimer(ros::Duration(1.0), &LidarModule::publishDiagnostic, this);
        health_.last_update = ros::Time::now();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_node");
    LidarModule node;
    ros::spin();
    return 0;
}
```

---

## 五、错误代码速查

> **提示**：错误代码的具体定义应根据实际项目需求制定。以下是建议的编码规则。

### 5.1 编码规则

```
格式: XYYY
  X   - 模块类别（顶层功能，对应错误代码规范）
  YYY - 具体错误编号

编号建议:
  X0YY  - 进程缺失/启动失败
  X1YY  - 驱动或硬件错误
  X2YY  - 数据异常
  X3YY  - 通信超时
  X9YY  - 其他错误
```

### 5.2 示例

| 错误代码 | 含义 |
|---------|------|
| `TIMEOUT` | 心跳超时（所有模块通用）|
| `2XXX` | 定位层模块通用前缀 |
| `3XXX` | 感知层模块通用前缀 |
| `3XYY` | 感知层具体错误 |

### 5.3 命名建议

错误代码的命名应遵循：
- **简短**：便于显示和记录
- **清晰**：能快速定位问题类型
- **一致**：同类型错误使用相同编号规则

实际项目中的具体错误代码，建议在模块开发时根据真实场景补充。

---

## 六、进程检测机制

系统监控器采用 **双重检测** 机制确保进程异常能及时发现：

```
┌─────────────────────────────────────────────────────────────┐
│                     进程检测双重保障                          │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────────┐      ┌─────────────────────────┐   │
│  │   检测方式一         │      │   检测方式二            │   │
│  │   waitpid 检测       │      │   心跳超时检测           │   │
│  │   (fork 子进程)      │      │   (/diagnostics)        │   │
│  ├─────────────────────┤      ├─────────────────────────┤   │
│  │ • fork 启动模块      │      │ • 监听 /diagnostics     │   │
│  │ • 记录 PID          │      │ • 记录最后心跳时间       │   │
│  │ • 子进程退出立即感知  │      │ • 超过阈值判定为崩溃     │   │
│  ├─────────────────────┤      ├─────────────────────────┤   │
│  │ 检测速度: < 1秒      │      │ 检测速度: 2-3秒         │   │
│  │ 适用: fork 启动     │      │ 适用: 所有模块           │   │
│  └─────────────────────┘      └─────────────────────────┘   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 6.1 检测方式一：fork + waitpid

fork 启动的模块通过 `waitpid` 检测子进程退出：

| 场景 | 检测结果 | 处理 |
|------|---------|------|
| 子进程正常退出 | `waitpid` 返回 PID | 标记 `has_crashed = true` |
| 子进程崩溃 | `waitpid` 返回 PID + status | 标记 `has_crashed = true` |
| 子进程未退出 | `waitpid` 返回 0 | 正常，继续监控 |

```cpp
void checkProcessStatus(const ros::TimerEvent&) {
    for (auto& [name, info] : module_infos_) {
        if (info.pid > 0 && !info.has_crashed) {
            int status;
            pid_t result = waitpid(info.pid, &status, WNOHANG);
            if (result == info.pid) {
                info.has_crashed = true;
                ROS_ERROR("!!! Module '%s' (PID: %d) has crashed !!!",
                          name.c_str(), info.pid);
            }
        }
    }
}
```

### 6.2 检测方式二：心跳超时

即使进程突然消失（来不及发送停止消息），心跳超时也能检测到：

| 超时阈值 | 说明 |
|---------|------|
| 3 秒 | 若模块超过 3 秒未发布 `/diagnostics`，判定为异常 |

```cpp
void checkHeartbeatTimeout(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();
    for (auto& [name, info] : module_infos_) {
        if (info.has_crashed) continue;  // 已标记崩溃，跳过
        if (info.pid <= 0) continue;     // 没有 PID，跳过

        if (!info.last_heartbeat.isZero()) {
            double elapsed = (now - info.last_heartbeat).toSec();
            if (elapsed > heartbeat_timeout_sec_) {
                info.has_crashed = true;
                ROS_ERROR("!!! Module '%s' heartbeat timeout (%.1f sec) !!!",
                          name.c_str(), elapsed);
            }
        }
    }
}
```

### 6.3 检测流程图

```
模块启动
    │
    ├── fork() ──▶ 记录 PID ────────────────────┐
    │                                        │
    ▼                                        ▼
┌─────────────────────┐            ┌─────────────────────┐
│ 定时器 1: checkProcessStatus  │   │ 定时器 2: checkHeartbeatTimeout
│   waitpid 检测                   │   │   心跳超时检测
│   检测子进程退出                 │   │   检测 /diagnostics 停止
└─────────┬───────────┘            └─────────┬───────────┘
          │                                  │
          └──────────┬───────────────────────┘
                     │
                     ▼
              has_crashed = true
                     │
                     ▼
              发布 ERROR 状态
              上报错误代码
```

---

## 七、禁止事项

```
┌────────────────────────────────────────────────────────┐
│                      禁止行为清单                         │
├────────────────────────────────────────────────────────┤
│                                                        │
│  ❌ 在业务回调中直接发布 Diagnostic                      │
│     → 回调只能写 health_，不能 publish                   │
│                                                        │
│  ❌ 在多个地方发布同一个模块的 Diagnostic                 │
│     → 只能在一个 Timer 回调中发布                       │
│                                                        │
│  ❌ Diagnostic 发布频率超过 1Hz                         │
│     → 必须使用 1Hz Timer 控制                           │
│                                                        │
│  ❌ 模块 name 与 modules.yaml 不一致                     │
│     → 必须严格匹配，否则 system_monitor 无法识别          │
│                                                        │
│  ❌ 错误代码不符合规范                                   │
│     → 必须按归属分类填写                                 │
│                                                        │
└────────────────────────────────────────────────────────┘
```

---

