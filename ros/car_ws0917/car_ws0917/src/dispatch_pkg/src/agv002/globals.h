#ifndef GLOBALS_H
#define GLOBALS_H

#include <ros/ros.h>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <std_msgs/Int32.h>
#include <set>
#include <std_msgs/UInt8MultiArray.h>
#include <utility>
#include <vector>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32MultiArray.h> //用于获取AGV实时坐标
using json = nlohmann::json;

/* ---------- 自定义数据结构 ---------- */
struct Task
{
    std::string startPositionCode;
    std::string startPositionName;
    std::string endPositionCode;
    std::string endPositionName;
    int taskType;
    int taskStatus;
    std::string type;
    std::string lineNumber;
};

struct AgvOrder
{
    std::string orderCode; // 订单编号
    std::string updateTime;
    std::vector<Task> taskList;
};
/* ---------- 新增：AGV状态机 ---------- */
enum class AgvSystemState
{
    IDLE,            // 空闲，可以接受新订单
    EXECUTING,       // 正在执行任务
    PAUSED,          // 暂停
    EMERGENCY_STOP,  // 急停
    MANUAL_TAKEOVER, // 人工接管
    CHARGING         // 【新增】正在充电
};
extern AgvSystemState g_agv_state; // AGV的当前状态，初始为空闲

/* ---------- 全局变量 ---------- */
extern std::string g_auth_token;
extern std::string g_api_base_url;
extern std::string g_api_username;
extern std::string g_api_password;
extern ros::Publisher g_dispatch_pub;
extern ros::Subscriber g_task_complete_sub;
extern ros::Subscriber status_flags_sub;
extern ros::Subscriber agv_current_postion_sub; // 车辆当前位置订阅节点
// --- 新增：全局变量存储实时坐标 ---
extern double g_realtime_x;
extern double g_realtime_y;
extern ros::Subscriber manual_takeover_sub;

extern ros::Subscriber manual_takeover_cancel_sub; //  新增：取消接管订阅
// 新增：用于标记取消接管后 10 秒延迟是否激活的标志位
extern std::atomic<bool> g_is_cancel_delay_active;
extern ros::Timer post_cancel_timer;               //  新增：10秒检测任务定时器
extern bool cancel_pending;                //  新增：延迟检测标志
/* ================= 【新增】AGV等待接口相关变量 ================= */
extern ros::Publisher g_wait_signal_pub;      // 发布停车/继续行驶信号 (1:停, 0:行)
extern ros::Timer g_wait_timer;               // 10秒轮询定时器
extern Task g_current_wait_task;              // 缓存当前正在检测的任务信息
extern bool g_is_waiting_signal;      // 标记是否正处于等待服务器信号的状态
/* ================= 【新增】缓冲区等待相关变量 ================= */
extern ros::Timer g_buffer_wait_timer;        // 缓冲区等待轮询定时器
extern bool g_is_buffer_waiting;     // 标记是否正处于缓冲区等待状态
/* ================= 区域等待相关变量 ================= */
extern ros::Timer g_area_wait_timer;          // 区域等待轮询定时器
extern bool g_is_area_waiting;        // 标记是否正处于区域等待状态
/* ============================================================= */
extern std::unordered_map<std::string, std::string> g_lastUpdateTimes; // 上一次更新的时间
extern std::set<std::string> g_orderCodes;                        // 用于存储所有的agv订单编号用于更新订单 //暂时没用到 后续是否要改成set？升序？
extern std::vector<struct AgvOrder> g_agvOrders;                       // 用于存储所有AGV订单
extern AgvOrder currentOrder;                                          // 当前的订单，用于存储现在正在执行的订单
extern Task currentTask;                                               // 用于存储当前的任务
extern bool if_one;                                             // 是否是第一次加载任务
extern bool if_order_ok;
// bool if_task_ok = false;

extern bool if_exeing;         // 用于标记是否正在执行任务
extern std::string agvStatus;   // 默认值为10 为待机状态
extern std::string taskStatus;  // （10-未下发、20-排队中、30-正在执行、40-等待、50-暂停、60-挂起、70-已完成、80-已关闭）
extern std::string order_staus; // （10-未下发、20-排队中、30-正在执行、40-等待、50-暂停、60-挂起、70-已完成、80-已关闭）
extern std::string agvpostion;
extern std::string last_agv_status;
extern int agv_dianliang; // 0926改
extern Task last_task;
extern int g_current_dispatch_code; // 初始化为一个无效值 下发给下位机的任务编号

// 使用嵌套地图，查找方式：MAP[起点][终点]
extern const std::unordered_map<std::string, std::unordered_map<std::string, int>> ENDPOINT_CODE_MAP;
/* ================= 【新增】定点停车请求配置 ================= */
struct RequestPoint {
    double x;
    double y;
    std::string name;
};

// 定义需要检测的坐标点列表 (单位: 米)
extern const std::vector<RequestPoint> g_request_points;
// 定义区域等待点列表 (使用postBufferEntryRequest)
extern const std::vector<RequestPoint> g_area_points;
enum WaitCheckResult {
    SERVER_ALLOW_GO,      // 服务器放行
    SERVER_REQUIRE_WAIT,  // 服务器要求等待
    NETWORK_FAILURE,      // 网络请求失败
    PARSE_ERROR           // 解析错误
};
// 记录上一次触发的点名称，防止在1米范围内重复频繁触发请求
extern std::string g_last_triggered_point;
// 用于控制请求频率，避免高频请求
extern ros::Time g_last_request_time;
// 记录上一次触发的区域等待点名称
extern std::string g_last_triggered_area_point;
// 用于控制区域等待点请求频率
extern ros::Time g_last_area_request_time;
// 跟踪AGV是否在封闭区域内
extern bool g_is_in_area;

/* ========================================================== */
/* ---------- 函数声明 ---------- */
void manualTakeoverCallback(const std_msgs::UInt8::ConstPtr &msg);

void publishOneTask(const Task &t, const std::string &orderCode);
void printGreen(const std::string &msg);
void printYellow(const std::string &msg);
bool httpPost(const std::string &url, const json &payload, std::string &resp_out, long &http_code_out, bool use_auth);
void taskCompleteCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
void postCheckAgvSchedulingOrder();
void postReportAgvTaskDetail(bool if_order_ok);
WaitCheckResult postTaskEventTrigger(int eventType, const Task& t);
bool loginAndFetchToken();

/* ---------- 新增函数声明 ---------- */
std::string nowStr();
void postRebackAgvStatus();
void postRebackAgvHeartbeat();
void updateAgvPosition(const std::string &positionCode);
void statusFlagsCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
void manualTakeoverCancelCallback(const std_msgs::UInt8::ConstPtr &msg);
void postCancelTimerCallback(const ros::TimerEvent &event);
void timerCallback(const ros::TimerEvent &event);
void dispatchNextOrderCallback(const ros::TimerEvent &event);
void waitTimerCallback(const ros::TimerEvent &event);
void bufferWaitTimerCallback(const ros::TimerEvent &event);
void areaWaitTimerCallback(const ros::TimerEvent &event);
void agvPoseCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);
/* ---------- 内联函数定义 ---------- */
inline void printGreen(const std::string &msg) { ROS_INFO_STREAM("\033[1;32m" << msg << "\033[0m"); }
inline void printYellow(const std::string &msg) { ROS_INFO_STREAM("\033[1;33m" << msg << "\033[0m"); }

#endif // GLOBALS_H
