#include "globals.h"

#include <clocale>

/* ---------- 新增：AGV状态机 ---------- */
AgvSystemState g_agv_state = AgvSystemState::IDLE; // AGV的当前状态，初始为空闲
/* ---------- 全局变量 ---------- */
std::string g_auth_token = "";
std::string g_api_base_url;
std::string g_api_username;
std::string g_api_password;
ros::Publisher g_dispatch_pub;
ros::Subscriber g_task_complete_sub;
ros::Subscriber status_flags_sub;
ros::Subscriber agv_current_postion_sub; // 车辆当前位置订阅节点
// --- 新增：全局变量存储实时坐标 ---
double g_realtime_x = 0.0;
double g_realtime_y = 0.0;
ros::Subscriber manual_takeover_sub;

ros::Subscriber manual_takeover_cancel_sub; //  新增：取消接管订阅
// 新增：用于标记取消接管后 10 秒延迟是否激活的标志位
std::atomic<bool> g_is_cancel_delay_active{false};
ros::Timer post_cancel_timer;               //  新增：10秒检测任务定时器
bool cancel_pending = false;                //  新增：延迟检测标志
/* ================= 【新增】AGV等待接口相关变量 ================= */
ros::Publisher g_wait_signal_pub;      // 发布停车/继续行驶信号 (1:停, 0:行)
ros::Timer g_wait_timer;               // 10秒轮询定时器
Task g_current_wait_task;              // 缓存当前正在检测的任务信息
bool g_is_waiting_signal = false;      // 标记是否正处于等待服务器信号的状态
/* ================= 【新增】缓冲区等待接口相关变量 ================= */
ros::Timer g_buffer_wait_timer;        // 缓冲区等待10秒轮询定时器
bool g_is_buffer_waiting = false;      // 标记是否正处于缓冲区等待服务器信号的状态
ros::Timer g_area_wait_timer;          // 区域等待10秒轮询定时器
bool g_is_area_waiting = false;        // 标记是否正处于区域等待服务器信号的状态
/* ============================================================= */
std::unordered_map<std::string, std::string> g_lastUpdateTimes; // 上一次更新的时间
std::set<std::string> g_orderCodes = {};                        // 用于存储所有的agv订单编号用于更新订单 //暂时没用到 后续是否要改成set？升序？
std::vector<struct AgvOrder> g_agvOrders;                       // 用于存储所有AGV订单
AgvOrder currentOrder;                                          // 当前的订单，用于存储现在正在执行的订单
Task currentTask;                                               // 用于存储当前的任务
bool if_one = true;                                             // 是否是第一次加载任务
bool if_order_ok = false;
// bool if_task_ok = false;

bool if_exeing = false;         // 用于标记是否正在执行任务
std::string agvStatus = "10";   // 默认值为10 为待机状态
std::string taskStatus = "20";  // （10-未下发、20-排队中、30-正在执行、40-等待、50-暂停、60-挂起、70-已完成、80-已关闭）
std::string order_staus = "30"; // （10-未下发、20-排队中、30-正在执行、40-等待、50-暂停、60-挂起、70-已完成、80-已关闭）
std::string agvpostion = "chargingPark001";
std::string last_agv_status = "suibian";
int agv_dianliang = 100; // 0926改
Task last_task;
int g_current_dispatch_code = -1; // 初始化为一个无效值 下发给下位机的任务编号

// 使用嵌套地图，查找方式：MAP[起点][终点]-agv002地图路线对应任务编号
const std::unordered_map<std::string, std::unordered_map<std::string, int>> ENDPOINT_CODE_MAP = {
    {"chargingPark001",{
        {"warehousePark001", 101},
        {"stationPark1-1",134},{"stationPark1-2",135},{"stationPark1-3",136},{"stationPark1-4",137},{"stationPark1-5",138},{"stationPark1-6",139},{"stationPark1-7",140},{"stationPark1-8",141},{"stationPark1-9",142},{"stationPark1-10",143},
        {"stationPark4-1",201},{"stationPark4-2",202},{"stationPark4-3",203},{"stationPark4-4",204},
        {"stationPark5-1",220},{"stationPark5-2",221}
    }},
    {"warehousePark001",{
        {"stationPark1-1",102},{"stationPark1-2",103},{"stationPark1-3",104},{"stationPark1-4",105},{"stationPark1-5",106},{"stationPark1-6",107},{"stationPark1-7",108},{"stationPark1-8",109},{"stationPark1-9",110},{"stationPark1-10",111},
        {"stationPark5-1",224},{"stationPark5-2",225},
        {"chargingPark001",226}
    }},
    {"stationPark1-1",{{"warehousePark002",112},{"chargingPark001",144},{"stationPark1-2",154},{"stationPark1-3",155},{"stationPark1-4",156},{"stationPark1-5",157},{"stationPark1-6",158},{"stationPark1-7",159},{"stationPark1-8",160},{"stationPark1-9",161},{"stationPark1-10",162}}},
    {"stationPark1-2",{{"warehousePark002",113},{"chargingPark001",145},{"stationPark1-3",163},{"stationPark1-4",164},{"stationPark1-5",165},{"stationPark1-6",166},{"stationPark1-7",167},{"stationPark1-8",168},{"stationPark1-9",169},{"stationPark1-10",170}}},
    {"stationPark1-3",{{"warehousePark002",114},{"chargingPark001",146},{"stationPark1-4",171},{"stationPark1-5",172},{"stationPark1-6",173},{"stationPark1-7",174},{"stationPark1-8",175},{"stationPark1-9",176},{"stationPark1-10",177}}},
    {"stationPark1-4",{{"warehousePark002",115},{"chargingPark001",147},{"stationPark1-5",178},{"stationPark1-6",179},{"stationPark1-7",180},{"stationPark1-8",181},{"stationPark1-9",182},{"stationPark1-10",183}}},
    {"stationPark1-5",{{"warehousePark002",116},{"chargingPark001",148},{"stationPark1-6",184},{"stationPark1-7",185},{"stationPark1-8",186},{"stationPark1-9",187},{"stationPark1-10",188}}},
    {"stationPark1-6",{{"warehousePark002",117},{"chargingPark001",149},{"stationPark1-7",189},{"stationPark1-8",190},{"stationPark1-9",191},{"stationPark1-10",192}}},
    {"stationPark1-7",{{"warehousePark002",118},{"chargingPark001",150},{"stationPark1-8",193},{"stationPark1-9",194},{"stationPark1-10",195}}},
    {"stationPark1-8",{{"warehousePark002",119},{"chargingPark001",151},{"stationPark1-9",196},{"stationPark1-10",197}}},
    {"stationPark1-9",{{"warehousePark002",120},{"chargingPark001",152},{"stationPark1-10",198}}},
    {"stationPark1-10",{{"warehousePark002",121},{"chargingPark001",153}}},
    {"warehousePark002",{{"chargingPark001",122},{"warehousePark001",123},{"stationPark1-1",124},{"stationPark1-2",125},{"stationPark1-3",126},{"stationPark1-4",127},{"stationPark1-5",128},{"stationPark1-6",129},{"stationPark1-7",130},{"stationPark1-8",131},{"stationPark1-9",132},{"stationPark1-10",133}}},
    {"stationPark4-1",{
        {"stationPark5-1",205},
        {"stationPark4-2",214},{"stationPark4-3",215},{"stationPark4-4",216}
    }},
    {"stationPark4-2",{
        {"stationPark5-1",206},
        {"stationPark4-3",217},{"stationPark4-4",218}
    }},
    {"stationPark4-3",{
        {"stationPark5-1",207},
        {"stationPark4-4",219}
    }},
    {"stationPark4-4",{
        {"stationPark5-1",208}
    }},
    {"stationPark5-1",{
        {"stationPark4-1",209},{"stationPark4-2",210},{"stationPark4-3",211},{"stationPark4-4",212},
        {"chargingPark001",213},
        {"warehousePark001",222}
    }},
    {"stationPark5-2",{
        {"warehousePark001",223}
    }},
};
/* ================= 【新增】定点停车请求配置 ================= */

// 定义需要检测的坐标点列表 (单位: 米)
const std::vector<RequestPoint> g_request_points = {
    // {-7.012074, -34.77586, "缓冲区等待请求点"},
    // {24.370000, -28.48000, "交通枢纽请求点"},
    // {44.750816, -22.481133, "半成品下线出口等待点请求点"},
    // {21.406800, -22.238800, "热前加工出口等待点请求点"},
    // {45.001366, 34.52000,  "清洗2等待请求点"},
    // {31.820000, -82.04000, "清洗1（左边的点）请求点"},
    // {1.113419, 0.893041, "充电点请求等待点"}
};

// 定义区域等待点列表 (使用postBufferEntryRequest)
const std::vector<RequestPoint> g_area_points = {
    {0.1, -0.80, "agv002等待点"},
    {0.1, 0.90, "agv003等待点"},
    {21.63, -22.95, "路口1"},
    {31.15, -33.05, "路口2"},
    {44.80, -23.35, "路口3"}
};

// 记录上一次触发的点名称，防止在1米范围内重复频繁触发请求
std::string g_last_triggered_point = "";
// 用于控制请求频率，避免高频请求
ros::Time g_last_request_time(0);

// 记录上一次触发的区域等待点名称
std::string g_last_triggered_area_point = "";
// 用于控制区域等待点请求频率
ros::Time g_last_area_request_time(0);

/* ====================================================== */
/* main                            */
/* ====================================================== */
int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "http_agv_client");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("api_base_url", g_api_base_url, "");
    pnh.param<std::string>("api_username", g_api_username, "");
    pnh.param<std::string>("api_password", g_api_password, "");
    while (!g_api_base_url.empty() && g_api_base_url.back() == '/')
        g_api_base_url.pop_back();

    if (g_api_base_url.empty() || g_api_username.empty() || g_api_password.empty())
    {
        ROS_FATAL("Dispatch configuration is missing. Set api_base_url, api_username and api_password.");
        return 2;
    }

    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK)
    {
        ROS_FATAL("Failed to initialize libcurl.");
        return 3;
    }

    /* ---------- 登录阶段：若失败则循环重试 ---------- */
    while (ros::ok())
    {
        if (loginAndFetchToken())
            break; // 成功 -> 跳出
        ROS_WARN("[init] 网络不可用或登录失败，5 秒后重试…");
        ros::Duration(5.0).sleep(); // 等待再来
    }

    if (!ros::ok())
    {
        curl_global_cleanup();
        return 0;
    }


    /* ---------- 以下保持原有逻辑 ---------- */
    postRebackAgvStatus();
    g_dispatch_pub = nh.advertise<std_msgs::Int32>("task_sequence_request", 10);
    g_task_complete_sub = nh.subscribe("task_sequence_complete", 10, taskCompleteCallback);
    status_flags_sub = nh.subscribe("dispatch_and_pause_flags", 10, statusFlagsCallback);
    manual_takeover_sub = nh.subscribe("manual_takeover", 10, manualTakeoverCallback);

    manual_takeover_cancel_sub = nh.subscribe("manual_takeover_cancel", 10, manualTakeoverCancelCallback); // 订阅取消接管
    // 定时器：10秒延时检测调度端任务
    post_cancel_timer = nh.createTimer(ros::Duration(10.0), postCancelTimerCallback, true, false); // <-- true 只使用一次，初始不启用
    ros::Timer timer = nh.createTimer(ros::Duration(5.0), timerCallback);
    // 新增1秒的定时器，用于调度AGV的下一个任务
    ros::Timer dispatch_timer = nh.createTimer(ros::Duration(5.0), dispatchNextOrderCallback);
    /* ================= 【新增】初始化发布者和轮询定时器 ================= */
    // 1. 定义发布话题 "agv_wait_flag" ，队列长度10
    g_wait_signal_pub = nh.advertise<std_msgs::Int32>("agv_wait_flag", 10);
    // 2. 定义10秒定时器，绑定回调 waitTimerCallback
    // false, false 表示：oneshot=false(循环执行), autostart=false(初始不启动，由代码逻辑控制启动)
    g_wait_timer = nh.createTimer(ros::Duration(10.0), waitTimerCallback, false, false);
    // 3. 定义10秒区域缓冲区等待定时器，绑定回调 bufferWaitTimerCallback
    g_buffer_wait_timer = nh.createTimer(ros::Duration(10.0), bufferWaitTimerCallback, false, false);
    // 4. 路线区域占用请求返回的等待请求定时器
    g_area_wait_timer = nh.createTimer(ros::Duration(9.0), areaWaitTimerCallback, false, false);
    // 订阅 ndt_maching.cpp 中的定位话题
    agv_current_postion_sub = nh.subscribe<std_msgs::Int32MultiArray>("/ndt_pose_m", 10, agvPoseCallback);
    /* ================================================================ */
    ROS_INFO("HTTP AGV Client node started.");
    ros::spin();
    curl_global_cleanup();
    return 0;
}
