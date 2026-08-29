#include "globals.h"

// 定义封闭区域的多边形坐标点 (A->B->C->D->E->F->A)
const std::vector<std::pair<double, double>> g_area_polygon = {
    {-5.7, -30.35},   // A
    {-5.7, 4.5},      // B
    {-0.6, 4.5},      // C
    {-0.6, -27.35},   // D
    {49.35, -27.35},  // E
    {49.35, -30.35}   // F
};

// 跟踪AGV是否在区域内
bool g_is_in_area = false;

// 进入路线冲突区域事件，默认5(进入区域)
int g_current_area_event_type = 5;

void postAreaEventRequest(int eventType); // 区域进入/离开请求
WaitCheckResult postBufferEntryRequest();

/**
 * @brief 【新增】等待接口的定时轮询回调 (每10秒触发)
 */
void waitTimerCallback(const ros::TimerEvent &)
{
    if (g_is_waiting_signal)
    {
        ROS_INFO("[WaitPoll] 车辆等待中，正在轮询服务器等待状态 (isWait?)...");
        // 定时轮询时，eventType 传入 0 或其他值均可，因为接口主要关注位置信息
        postTaskEventTrigger(0, g_current_wait_task); 
    }
}

/**
 * @brief 缓冲区等待定时器回调函数
 * 每10秒轮询一次缓冲区进入请求，直到服务器允许通行
 */
void bufferWaitTimerCallback(const ros::TimerEvent &)
{
    if (g_is_buffer_waiting)
    {
        ROS_INFO("[BufferWaitPoll] 缓冲区等待中，正在轮询服务器缓冲区进入状态...");
        // 重新调用缓冲区进入请求进行轮询
        postBufferEntryRequest();
    }
}

/**
 * @brief 区域等待定时器回调函数
 * 每10秒轮询一次区域进入/离开请求，直到服务器允许通行
 * 确保在程序初始化时创建 g_area_wait_timer
 */
void areaWaitTimerCallback(const ros::TimerEvent &)
{
    if (g_is_area_waiting)
    {
        ROS_INFO("[AreaWaitPoll] 区域等待中，正在轮询服务器区域状态...");
        // 重新调用区域进入/离开请求进行轮询
        postAreaEventRequest(g_current_area_event_type);
    }
}

/**
 * @brief 射线法检测点是否在多边形内
 * @param px 点的x坐标
 * @param py 点的y坐标
 * @param polygon 多边形顶点列表
 * @return true 在内部，false 在外部
 */
bool isPointInPolygon(double px, double py, const std::vector<std::pair<double, double>>& polygon) {
    int n = polygon.size();
    bool inside = false;
    
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = polygon[i].first, yi = polygon[i].second;
        double xj = polygon[j].first, yj = polygon[j].second;
        
        if (((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    
    return inside;
}

/**
 * @brief 发送缓冲区即将进入区域请求给调度，请求是否允许通行
 * @return WaitCheckResult 返回等待检查结果
 */
WaitCheckResult postBufferEntryRequest() {
    const std::string url = g_api_base_url + "/prod-api/interfaces/receive-data-handle/receiveData?bizCode=getLaneOccupancy";
    json payload;
    payload["agvCode"] = "AGV-qianyin-002";
    payload["laneCode"] = "LANE001";
    ROS_INFO_STREAM("[post-BufferEntryRequest] 发送请求到: " << url);
    std::string resp;
    long httpCode = 0;
    if (!httpPost(url, payload, resp, httpCode, true)) {
        ROS_ERROR_STREAM("[post-BufferEntryRequest] HTTP 请求失败: " << resp);
        return NETWORK_FAILURE;
    }
    
    if (httpCode != 200) {
        ROS_WARN_STREAM("[post-BufferEntryRequest] HTTP " << httpCode << ", 响应: " << resp);
        return NETWORK_FAILURE;
    }
    
    // 解析响应
    try {
        auto j = json::parse(resp);
        std::string isAllowed = "Y"; // 默认为允许通行
        
        // 解析 isAllowed 字段，兼容直接返回或在 data 字段下的情况
        if (j.contains("data") && j["data"].contains("isAllowed")) {
            isAllowed = j["data"]["isAllowed"].get<std::string>();
        } else if (j.contains("isAllowed")) {
            isAllowed = j["isAllowed"].get<std::string>();
        }
        
        std_msgs::Int32 msg;
        
        // 根据 isAllowed 判断是否需要等待
        if (isAllowed == "Y") {
            // 允许通行，不停车
            ROS_INFO("[post-BufferEntryRequest] 缓冲区请求响应: 允许通行 (isAllowed=Y)");
            msg.data = 0; // 继续行驶
            g_wait_signal_pub.publish(msg);
            
            // 如果正在缓冲区等待，停止等待
            if (g_is_buffer_waiting) {
                ROS_INFO("[pos-BufferEntryRequest] 服务器允许通行，停止缓冲区等待轮询");
                g_is_buffer_waiting = false;
                g_buffer_wait_timer.stop();
            }
            
            return SERVER_ALLOW_GO;
        } else if (isAllowed == "N") {
            // 不允许通行，需要等待，停车
            ROS_WARN("[post-BufferEntryRequest] 缓冲区请求响应: 不允许通行 (isAllowed=N)，发布停车信号");
            msg.data = 1; // 停车
            g_wait_signal_pub.publish(msg);
            
            // 如果不在缓冲区等待状态，进入缓冲区等待状态
            if (!g_is_buffer_waiting) {
                ROS_WARN("[post-BufferEntryRequest] 进入缓冲区等待状态，启动轮询...");
                g_is_buffer_waiting = true;
                g_buffer_wait_timer.start();
            }
            
            return SERVER_REQUIRE_WAIT;
        } else {
            ROS_WARN_STREAM("[post-BufferEntryRequest] 未知的 isAllowed 值: " << isAllowed << "，默认允许通行");
            msg.data = 0;
            g_wait_signal_pub.publish(msg);
            return SERVER_ALLOW_GO;
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("[post-BufferEntryRequest] JSON 解析异常: " << e.what() << " | Resp: " << resp);
        return PARSE_ERROR;
    }
}

/**
 * @brief 发送区域进入/离开请求给调度
 * @param eventType 事件类型：5=进入区域，6=离开区域
 */
void postAreaEventRequest(int eventType) {
    const std::string url = g_api_base_url + "/prod-api/interfaces/receive-data-handle/receiveData?bizCode=addLaneOccupancy";
    const int max_retries = 3;  // 最多重试3次
    const int retry_delay_ms = 500;  // 重试间隔500毫秒
    
    std::string isOccupied = (eventType == 6) ? "N" : "Y"; // 6=离开时为N，5=进入时为Y
    
    json payload;
    payload["agvCode"] = "AGV-qianyin-002";
    payload["laneCode"] = "LANE001";
    payload["isOccupied"] = isOccupied;
    
    ROS_INFO_STREAM("[postAreaEventRequest] 发送请求到: " << url);
    
    std::string resp;
    long httpCode = 0;
    bool success = false;
    
    // 重试逻辑：最多尝试 max_retries 次
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        if (!httpPost(url, payload, resp, httpCode, true)) {
            ROS_ERROR_STREAM("[postAreaEventRequest] 第 " << attempt << " 次请求失败: " << resp);
            
            // 如果还有重试机会，延迟后重试
            if (attempt < max_retries) {
                ROS_WARN_STREAM("[postAreaEventRequest] 等待 " << retry_delay_ms << "ms 后进行第 " << (attempt + 1) << " 次重试...");
                ros::Duration(retry_delay_ms / 1000.0).sleep();
                continue;
            }
            continue;
        }
        
        if (httpCode == 200) {
            ROS_INFO_STREAM("[postAreaEventRequest] 区域事件请求成功发送，类型: " << eventType << " (isOccupied: " << isOccupied << ")");
            
            // 解析响应以检查 isStopNow
            try {
                auto j = json::parse(resp);
                std::string isStopNow = "N"; // 默认为允许通行
                
                // 解析 isStopNow 字段，兼容直接返回或在 data 字段下的情况
                if (j.contains("data") && j["data"].contains("isStopNow")) {
                    isStopNow = j["data"]["isStopNow"].get<std::string>();
                } else if (j.contains("isStopNow")) {
                    isStopNow = j["isStopNow"].get<std::string>();
                }
                
                std_msgs::Int32 msg;
                
                // 根据 isStopNow 判断是否需要等待
                if (isStopNow == "Y") {
                    // 需要停车，停车
                    ROS_WARN("[postAreaEventRequest] 区域请求响应: 需要停车 (isStopNow=Y)，发布停车信号");
                    msg.data = 1; // 停车
                    g_wait_signal_pub.publish(msg);
                    
                    // 如果不在区域等待状态，进入区域等待状态
                    if (!g_is_area_waiting) {
                        ROS_WARN("[postAreaEventRequest] 进入区域等待状态，启动轮询...");
                        g_is_area_waiting = true;
                        g_current_area_event_type = eventType;
                        g_area_wait_timer.start();
                    }
                } else if (isStopNow == "N") {
                    // 允许通行，不停车
                    ROS_INFO("[postAreaEventRequest] 区域请求响应: 允许通行 (isStopNow=N)");
                    msg.data = 0; // 继续行驶
                    g_wait_signal_pub.publish(msg);
                    
                    // 如果正在区域等待，停止等待
                    if (g_is_area_waiting) {
                        ROS_INFO("[postAreaEventRequest] 服务器允许通行，停止区域等待轮询");
                        g_is_area_waiting = false;
                        g_area_wait_timer.stop();
                    }
                } else {
                    ROS_WARN_STREAM("[postAreaEventRequest] 未知的 isStopNow 值: " << isStopNow << "，默认允许通行");
                    msg.data = 0;
                    g_wait_signal_pub.publish(msg);
                }
                
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("[postAreaEventRequest] JSON 解析异常: " << e.what() << " | Resp: " << resp);
                // 解析失败时，默认允许通行
                std_msgs::Int32 msg;
                msg.data = 0;
                g_wait_signal_pub.publish(msg);
            }
            
            success = true;
            break;
        } else {
            ROS_WARN_STREAM("[postAreaEventRequest] 第 " << attempt << " 次请求返回 HTTP " << httpCode);
            
            // 如果还有重试机会，延迟后重试
            if (attempt < max_retries) {
                ROS_WARN_STREAM("[postAreaEventRequest] 等待 " << retry_delay_ms << "ms 后进行第 " << (attempt + 1) << " 次重试...");
                ros::Duration(retry_delay_ms / 1000.0).sleep();
            }
        }
    }
    
    if (!success) {
        ROS_ERROR_STREAM("[postAreaEventRequest] 经过 " << max_retries << " 次重试后，请求仍未成功");
    }
}
/* ====================================================== */
/* 任务完成回调函数 (已更新位置处理逻辑) */
/* ====================================================== */
void taskCompleteCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
    if (msg->data.size() >= 5) // 1015改，检查长度确保索引4有效，避免越界
    {
        agv_dianliang = msg->data[4];
        //ROS_INFO_STREAM("[taskComplete] Battery level updated to: " << agv_dianliang);
    }

    // 协议至少需要2个字节来获取完成状态
    if (msg->data.size() < 2)
    {
        ROS_ERROR_STREAM("[taskComplete] Received incomplete message! Size: " << msg->data.size() << ", Expected >= 2");
        return;
    }

    uint8_t completion_status_byte = msg->data[1];
    int completed_task_id = completion_status_byte;

    if (g_agv_state == AgvSystemState::EXECUTING &&
        g_current_dispatch_code != -1 &&
        completed_task_id == g_current_dispatch_code)
    {
        ROS_INFO_STREAM("[taskComplete] Confirmation received! Vehicle completed task ID: " << completed_task_id);
        g_current_dispatch_code = -1;
        if_order_ok = false;

        // --- 核心修改：在任务完成后，直接用任务的终点来更新位置 ---
        if (currentOrder.taskList.empty())
        {
            ROS_ERROR("[taskComplete] TaskList is empty, but received a valid completion signal. State is inconsistent.");
            return;
        }

        // 记录刚刚完成的任务，并从中获取终点位置
        last_task = currentOrder.taskList.front();

        updateAgvPosition(last_task.endPositionCode);
        // ================= 【新增修改 1】触发任务结束事件 =================
        // 此时 last_task 刚刚执行完毕
        //postTaskEventTrigger(2, last_task); // Type 2 = End
        // -------------------------------------------------------------

        taskStatus = "70";
        order_staus = "30";
        postReportAgvTaskDetail(if_order_ok);

        currentOrder.taskList.erase(currentOrder.taskList.begin());
        ROS_INFO_STREAM("Sub-task completed and removed: " << last_task.endPositionCode);

        if (!currentOrder.taskList.empty())
        {
            currentTask = currentOrder.taskList.front();
            
            // ================= 【新增修改 2】触发新任务开始事件 =================
            // 队列不为空，即将开始下一个子任务
            // if (currentTask.startPositionCode == "chargingPark001") {
            //     postTaskEventTrigger(1, currentTask); // Type 1 = Start
            // }

            taskStatus = "20";
            order_staus = "30";
            if_order_ok = false;
            postReportAgvTaskDetail(if_order_ok);
            ROS_INFO_STREAM("Preparing for next sub-task: " << currentTask.endPositionCode);
        }
        else
        {
            ROS_INFO_STREAM("Order completed: " << currentOrder.orderCode);
            if_order_ok = true;
            order_staus = "70";
            taskStatus = "70";
            postReportAgvTaskDetail(if_order_ok);

            if (!g_agvOrders.empty() && g_agvOrders.front().orderCode == currentOrder.orderCode)
            {
                g_orderCodes.erase(currentOrder.orderCode);
                g_agvOrders.erase(g_agvOrders.begin());
            }

            currentOrder = AgvOrder();
            currentTask = Task();
            g_agv_state = AgvSystemState::IDLE;
            agvStatus = "10";
            postRebackAgvStatus();
        }
    }
}
/* ====================================================== */
/* 新：定时器回调，用于调度新订单 (新增)           */
/* ====================================================== */
void dispatchNextOrderCallback(const ros::TimerEvent &)
{
    // 只有在AGV处于空闲状态，并且全局订单队列里有订单时，才开始新订单
    if (g_agv_state == AgvSystemState::IDLE && !g_agvOrders.empty())
    {
        ROS_INFO("AGV is IDLE, dispatching next order from the queue.");

        // 1. 获取新订单和新任务
        currentOrder = g_agvOrders.front();
        if (currentOrder.taskList.empty())
        {
            ROS_ERROR_STREAM("Order " << currentOrder.orderCode << " has no tasks. Removing it.");
            g_orderCodes.erase(currentOrder.orderCode);
            g_agvOrders.erase(g_agvOrders.begin());
            return;
        }
        currentTask = currentOrder.taskList.front();

        // 新增：查询第一个任务是否停车（仅当起始点为chargingPark001时）
        // WaitCheckResult result = SERVER_ALLOW_GO; // 默认放行
        // if (currentTask.startPositionCode == "chargingPark001") {
        //     result = postTaskEventTrigger(1, currentTask);  // eventType=1 表示任务开始
        // }
        // if (result == SERVER_REQUIRE_WAIT) {
        //     ROS_WARN("[dispatch] 初始任务被要求等待");
        // } else if (result == SERVER_ALLOW_GO) {
        //     ROS_INFO("[dispatch] 初始任务放行。");
        // } else {
        //     ROS_WARN("[dispatch] 查询失败，默认放行。");
        // }

        // 2. 转换状态为“执行中”
        g_agv_state = AgvSystemState::EXECUTING;

        // 3. 更新协议状态并上报
        if_order_ok = false;
        order_staus = "30";
        taskStatus = "20";
        agvStatus = "30";
        postReportAgvTaskDetail(if_order_ok);
        postRebackAgvStatus();
    }
}
/* ====================================================== */
/* 接口1：调度订单查询（核心接口）                 
    车AGV-qianyin-002*/
/* ====================================================== */
void postCheckAgvSchedulingOrder()
{
    const std::string url = g_api_base_url + "/prod-api/orderSendAgv/checkAgvSchedulingOrder";
    json payload = {
        {"packageId", "PSDD" + nowStr()},
        {"reqTime", nowStr()},
        {"clientCode", "CMES"},
        {"agvCode", "AGV-qianyin-002"}};

    // 填充上次已知的订单列表
    json orderList = json::array();
    for (const auto &code : g_orderCodes)
    {
        orderList.push_back({{"orderCode", code}, {"updateTime", g_lastUpdateTimes[code]}});
    }
    payload["agvOrderList"] = orderList;

    std::string resp;
    long httpCode = 0;
    if (!httpPost(url, payload, resp, httpCode, true))
    {
        ROS_ERROR_STREAM("[postCheckAgvSchedulingOrder] HTTP 请求失败: " << resp);
        return;
    }

    if (httpCode != 200)
        return;

    try
    {
        auto j = json::parse(resp);
        if (j.contains("message") &&
            j["message"] == "没有新增或更新的订单需要处理")
        {
            ROS_INFO("No new or updated orders to process. Skipping this round.");
            ROS_INFO_STREAM(nowStr());
            return;
        }
        ROS_INFO_STREAM("开始解析返回的订单数据..." << resp);
        ROS_INFO_STREAM(nowStr());
        // 逐条处理返回的订单数据
        for (const auto &item : j["data"])
        {
            const auto &agvOrderJson = item.at("agvOrder");
            AgvOrder order;
            order.orderCode = agvOrderJson.at("orderCode").get<std::string>();
            order.updateTime = agvOrderJson.at("updateTime").get<std::string>();

            // 判断是否已存在
            if (g_orderCodes.find(order.orderCode) == g_orderCodes.end())
            {
                g_orderCodes.insert(order.orderCode);
                ROS_INFO_STREAM("新增订单编号 " << order.orderCode);

                // 解析并加入任务
                for (const auto &t : agvOrderJson.at("taskList"))
                {
                    Task task;
                    // task.lineNumber        = t.at("lineNumber").get<std::string>();
                    task.startPositionCode = t.at("startPositionCode").get<std::string>();
                    task.startPositionName = t.at("startPositionName").get<std::string>();
                    task.endPositionCode = t.at("endPositionCode").get<std::string>();
                    task.endPositionName = t.at("endPositionName").get<std::string>();
                    task.taskType = t.at("taskType").get<int>();
                    task.taskStatus = t.at("taskStatus").get<int>();
                    task.type = t.at("type").get<std::string>();
                    order.taskList.push_back(std::move(task));
                }

                // —— 根据 lineNumber 数值大小排序 —— //
                // std::sort(order.taskList.begin(), order.taskList.end(),
                //     [](const Task& a, const Task& b) {
                //         return std::stoi(a.lineNumber) < std::stoi(b.lineNumber);
                //     }
                // );
                ROS_INFO("已完成解析并加入任务");
                ROS_INFO_STREAM(nowStr());
                // // 排序后再加入全局队列
                g_agvOrders.push_back(std::move(order));
                ROS_INFO("已加入队列");
                ROS_INFO_STREAM(nowStr());
                // 含义：接到新任务的时候触发停车查询（此时任务尚未开始执行，还在排队中）
                // 取新订单的第一个任务作为代表进行上报
                // if (!g_agvOrders.back().taskList.empty()) 
                // {
                //     const Task& firstTask = g_agvOrders.back().taskList.front();
                //     // 参数 3 代表 "Task Received"
                //     postTaskEventTrigger(3, firstTask);
                // }
            }
            else
            {
                // 已存在的订单，跳过
                ROS_INFO_STREAM("已存在订单编号 " << order.orderCode);
                ROS_INFO_STREAM(nowStr());
            }
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("解析 JSON 出错: " << e.what());
    }
}

/* ================= 接口 2：AGV心跳反馈 发 这个无所谓================= */
void postRebackAgvHeartbeat()
{
    const std::string url = g_api_base_url +
        "/prod-api/interfaces/receive-data-handle/receiveData?bizCode=RebackAgvHeartbeatService";

    json payload;
    payload["packageId"] = "PSDD" + nowStr();
    payload["reqTime"] = nowStr();
    payload["clientCode"] = "CMES";
    payload["agvCode"] = "AGV-qianyin-002";

    std::string resp;
    long code = 0;
    if (httpPost(url, payload, resp, code, true))
    {
        // ROS_INFO_STREAM("[rebackAgvHeartbeat] HTTP " << code << "\n" << resp); 暂时不打印，已经调试没有问题了
    }
}

/* ================= 接口 3：AGV状态反馈 发 ================= */
void postRebackAgvStatus()
{
    const std::string url = g_api_base_url +
        "/prod-api/interfaces/receive-data-handle/receiveData?bizCode=RebackAgvStatusService";

    json payload;
    payload["packageId"] = "STATUS_" + nowStr();
    payload["reqTime"] = nowStr();
    payload["clientCode"] = "CMES";
    payload["agvCode"] = "AGV-qianyin-002";
    payload["agvBatteryLevel"] = agv_dianliang; // nlohmann/json 会自动处理 int 类型
    payload["agvCurrentPosition"] = agvpostion;
    payload["orderCode"] = currentOrder.orderCode;
    payload["agvErrorMessage"] = "";
    payload["agvWarningMessage"] = "";
    payload["agvStatus"] = agvStatus;
    payload["agvSpeed"] = "1.2"; // m/s

    std::string resp;
    long code = 0;
    printGreen("Sending data: " + payload.dump());
    if (httpPost(url, payload, resp, code, true))
    {
        ROS_INFO_STREAM("[rebackAgvStatus] HTTP " << code << "\n"
                                                  << resp);
    }
}

// ================= 接口 4：AGV 任务明细上报 =================
void postReportAgvTaskDetail(bool order_d_ok) // 是否完成的是最后一个任务
{
    const std::string url = g_api_base_url +
        "/prod-api/interfaces/receive-data-handle/receiveData?bizCode=RebackAgvSchedulingOrder";
    if (currentTask.endPositionCode.empty())
    {
        return;
    }

    // 1. 构造请求 JSON
    json payload;
    payload["packageId"] = "PSDD" + nowStr(); // 唯一包号：业务单号+时间
    payload["reqTime"] = nowStr();            // 请求时间戳
    payload["clientCode"] = "CMES";           // 客户端编号
    payload["agvCode"] = "AGV-qianyin-002";   // AGV 编码

    payload["orderCode"] = currentOrder.orderCode; // 订单编码
    payload["workingStatus"] = order_staus;        // 订单状态
    if (order_d_ok)                                // 如果是最后一个任务
    {
        payload["taskStartPositionCode"] = last_task.startPositionCode; // 任务起点编码
        payload["taskStartPositionName"] = last_task.startPositionName; // 任务起点名称（可选）
        payload["taskEndPositionCode"] = last_task.endPositionCode;     // 任务终点编码
        payload["taskEndPositionName"] = last_task.endPositionName;     // 任务终点名称（可选）
        payload["taskStatus"] = taskStatus;                             // 任务状态
    }
    else
    {
        payload["taskStartPositionCode"] = currentTask.startPositionCode; // 任务起点编码
        payload["taskStartPositionName"] = currentTask.startPositionName; // 任务起点名称（可选）
        payload["taskEndPositionCode"] = currentTask.endPositionCode;     // 任务终点编码
        payload["taskEndPositionName"] = currentTask.endPositionName;     // 任务终点名称（可选）
        payload["taskStatus"] = taskStatus;                               // 任务状态
    }

    // 2. 打印并发送）
    printGreen("发送任务明细成功，内容：" + payload.dump());

    std::string resp;
    long http_code = 0;
    bool ok = httpPost(url, payload, resp, http_code, true);

    if (!ok)
    {
        ROS_ERROR("[postReportAgvTaskDetail] HTTP 请求失败");
        return;
    }

    // 3. 打印接收
    // printYellow("[postReportAgvTaskDetail] HTTP "
    //             + std::to_string(http_code) + "\n" + resp);
}

/* ================= 接口5 (新增)：任务开始/结束及到暂停点前触发-是否停车查询接口 ================= */
/**
 * @brief 停车等待触发，上报任务事件触发（开始或结束）
 * @param eventType  0: 轮询检查 1: 子任务开始 (Task Start), 2: 任务结束 (Task End) 3：任务接收 (Task Received) 4：进入等待点查询
 * @param t 相关的任务信息
 */
WaitCheckResult postTaskEventTrigger(int eventType, const Task& t)
{
    const std::string url = g_api_base_url + "/prod-api/interfaces/receive-data-handle/receiveData?bizCode=checkAgvWait";

    json payload;
    payload["agvCode"] = "AGV-qianyin-002";
    //附带当前任务信息
    payload["taskStartPositionCode"] = t.startPositionCode;
    payload["taskStartPositionName"] = t.startPositionName;
    payload["taskEndPositionCode"] = t.endPositionCode;
    payload["taskEndPositionName"] = t.endPositionName;

    if (eventType != 0) {
        printGreen("[CheckWait] 触发等待检查, 类型: " + std::to_string(eventType));
    }

    std::string resp;
    long http_code = 0;
    // 发送请求
    bool ok = httpPost(url, payload, resp, http_code, true);

    if (!ok) {
        ROS_ERROR("[postTaskEventTrigger] HTTP 请求失败");
        return NETWORK_FAILURE;
    } else {
        ROS_INFO_STREAM("[postTaskEventTrigger] Response: " << resp);
        try {
            auto j = json::parse(resp);
            std::string isWait = "N"; // 默认为 N

            // 解析 isWait 字段，兼容直接返回或在 data 字段下的情况
            if (j.contains("data") && j["data"].contains("isWait")) {
                isWait = j["data"]["isWait"].get<std::string>();
            } else if (j.contains("isWait")) {
                isWait = j["isWait"].get<std::string>();
            }

            std_msgs::Int32 msg;

            // --- 情况 1: 服务器要求等待 (Y) ---
            if (isWait == "Y") 
            {
                msg.data = 1; 
                ROS_WARN("[HTTP] Detected Wait Signal (isWait=Y). Publishing agv_wait_flag = 1");
                g_wait_signal_pub.publish(msg); // 发布 1 通知停车
                
                if (!g_is_waiting_signal) {
                    ROS_WARN("[CheckWait] 收到等待指令 (isWait=Y)，车辆暂停。开启10秒轮询...");
                    g_is_waiting_signal = true;
                    g_current_wait_task = t; // 缓存当前任务用于轮询
                    g_wait_timer.start();    // 启动定时器
                } else {
                    ROS_WARN("[CheckWait] 持续等待中 (isWait=Y)...");
                     msg.data = 1; 
                    ROS_WARN("[HTTP] Detected Wait Signal (isWait=Y). Publishing agv_wait_flag = 1");
                    g_wait_signal_pub.publish(msg); // 发布 1 通知停车
                }
                return SERVER_REQUIRE_WAIT; // 【新增】返回 false，表示需要等待
            }
            // --- 情况 2: 服务器允许继续行驶 (N) ---
            else if (isWait == "N")
            {
                // 如果之前是在等待状态，现在变 N 了，打印日志提示放行
                if (g_is_waiting_signal) {
                    ROS_INFO("[CheckWait] 从等待中收到继续行驶指令 (isWait=N)，等待结束。");
                    g_is_waiting_signal = false;
                    // 停止轮询定时器
                    g_wait_timer.stop();
                } else if(eventType != 0){
                    // eventType != 0 表示这是触发时的第一次检查
                    ROS_INFO("[CheckWait] 检测结果: 无需等待 (N)。");
                }
                msg.data = 0;
                ROS_INFO("[CheckWait] 发布继续行驶信号。");
                g_wait_signal_pub.publish(msg); // 发布 0 通知放行

                return SERVER_ALLOW_GO; // 【新增】返回 true，表示放行
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("[postTaskEventTrigger] JSON 解析异常: " << e.what() << " | Resp: " << resp);
            return PARSE_ERROR;
        }
    }
    return SERVER_ALLOW_GO; //默认返回 go，可以通行
}

/* ================= 新增：坐标回调函数 ================= */
/**
    * @brief 实时AGV位置回调函数
 */
void agvPoseCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 3)
    {
        return;
    }

    int32_t x_cm = msg->data[1];
    int32_t y_cm = msg->data[2];

    g_realtime_x = x_cm / 100.0;
    g_realtime_y = y_cm / 100.0;

    // 调试打印 (可选)
    //ROS_INFO_STREAM_THROTTLE(1.0, "Real-time Pose: (" << g_realtime_x << ", " << g_realtime_y << ")");
    // 遍历所有定义的停车等待请求点
    ros::Time current_time = ros::Time::now();
    for (const auto& point : g_request_points) 
    {
        // 计算当前位置与目标点的欧几里得距离
        double dx = g_realtime_x - point.x;
        double dy = g_realtime_y - point.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        std_msgs::Int32 check_msg;
        // 判定距离小于 1 米
        if (dist <= 1.0) 
        {
            // 如果这个点已经被标记为“放行过”(Locked)，则不再请求，直接通过
            if (g_last_triggered_point == point.name) {
                continue; 
            }
            ROS_INFO_STREAM("[PositionCheck] 进入请求点范围: " << point.name << "，发起查询...");
            // 【核心修改】频率控制：每隔 1.0 秒发送一次请求
            if ((current_time - g_last_request_time).toSec() > 1.0) 
            {
                if (!g_is_waiting_signal) {
                    Task checkTask = currentTask;
                    // 调用接口，并根据返回值判断是否通过
                    // 注意：如果网络断了或服务器回Y，这里返回 false，g_last_triggered_point 不会更新
                    // 这样下一次循环（1秒后）会再次请求，直到成功为止
                    WaitCheckResult is_passed = postTaskEventTrigger(4, checkTask);
                    if (is_passed == SERVER_ALLOW_GO) {
                        // 只有服务器明确返回 "N" (Go)，我们才锁定这个点，不再请求
                        ROS_INFO_STREAM("[PositionCheck] " << point.name << " 已放行，锁定触发器。");
                        g_last_triggered_point = point.name;
                        check_msg.data = 0;
                        ROS_INFO("[PositionCheck] 发布继续行驶信号。");
                        g_wait_signal_pub.publish(check_msg); // 发布 0 通知放行
                    } else if (is_passed == SERVER_REQUIRE_WAIT){
                        ROS_WARN_STREAM("[PositionCheck] " << point.name << " 请求被要求等待，继续停车。");
                        check_msg.data = 1;
                        g_wait_signal_pub.publish(check_msg);
                    }
                    else if (is_passed == NETWORK_FAILURE){
                        ROS_WARN_STREAM("[PositionCheck] " << point.name << " 网络请求失败，发布一个停车信号确保安全");
                        // 可以在这里强制发布一个停车信号，确保安全
                        std_msgs::Int32 stop_msg;
                        stop_msg.data = 1;
                        g_wait_signal_pub.publish(stop_msg);
                        // 【新增】网络失败时，进入等待状态并启动轮询，直到网络恢复
                        if (!g_is_waiting_signal) {
                            ROS_WARN("[PositionCheck] 网络失败，进入等待状态并启动轮询...");
                            g_is_waiting_signal = true;
                            g_current_wait_task = checkTask;
                            g_wait_timer.start();
                        }
                    }
                } else {
                    // 如果已经在等待状态，只记录日志，不重复请求
                    ROS_INFO_THROTTLE(5.0, "[PositionCheck] 已在等待状态，等待轮询结果...");
                }
                // 更新请求时间
                g_last_request_time = current_time;
            }
        }
    }
    // 【离开复位逻辑】
    // 如果当前记录了在某个点，但计算发现已经远离该点（> 1.5米，设置迟滞区间防止临界跳变），则重置
    if (!g_last_triggered_point.empty()) 
    {
        bool still_in_range = false;
        // 查找当前锁定的点坐标
        for (const auto& point : g_request_points) {
            if (point.name == g_last_triggered_point) {
                double dx = g_realtime_x - point.x;
                double dy = g_realtime_y - point.y;
                double dist = std::sqrt(dx * dx + dy * dy);
                
                // 迟滞比较：进入用 1.0，离开用 1.5，避免在边界处反复触发
                if (dist <= 1.5) { 
                    still_in_range = true;
                }
                break;
            }
        }

        if (!still_in_range) {
            ROS_INFO_STREAM("[PositionCheck] AGV驶离请求点: " << g_last_triggered_point << "，重置触发锁。");
            g_last_triggered_point = ""; // 允许下一次再次触发
        }
    }

    // 【新增】区域等待点检测（使用postBufferEntryRequest）
    for (const auto& point : g_area_points) 
    {
        // 计算当前位置与目标点的欧几里得距离
        double dx = g_realtime_x - point.x;
        double dy = g_realtime_y - point.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        // 判定距离小于 1 米
        if (dist <= 0.6) 
        {
            // 如果这个点已经被标记为"放行过"(Locked)，则不再请求，直接通过
            if (g_last_triggered_area_point == point.name) {
                continue; 
            }
            ROS_INFO_STREAM("[AreaPositionCheck] 进入区域等待点范围: " << point.name << "，发起查询...");
            // 【核心修改】频率控制：每隔 1.0 秒发送一次请求
            if ((current_time - g_last_area_request_time).toSec() > 1.0) 
            {
                if (!g_is_buffer_waiting) {
                    // 调用postBufferEntryRequest进行请求
                    WaitCheckResult is_passed = postBufferEntryRequest();
                    if (is_passed == SERVER_ALLOW_GO) {
                        // 只有服务器明确返回 "N" (Go)，我们才锁定这个点，不再请求
                        ROS_INFO_STREAM("[AreaPositionCheck] " << point.name << " 已放行，锁定触发器。");
                        g_last_triggered_area_point = point.name;
                    } else if (is_passed == SERVER_REQUIRE_WAIT){
                        ROS_WARN_STREAM("[AreaPositionCheck] " << point.name << " 请求被要求等待，继续停车。");
                    }
                    else if (is_passed == NETWORK_FAILURE){
                        ROS_WARN_STREAM("[AreaPositionCheck] " << point.name << " 网络请求失败，发布一个停车信号确保安全");
                        // 可以在这里强制发布一个停车信号，确保安全
                        std_msgs::Int32 stop_msg;
                        stop_msg.data = 1;
                        g_wait_signal_pub.publish(stop_msg);
                        // 网络失败时，进入等待状态并启动轮询
                        if (!g_is_buffer_waiting) {
                            ROS_WARN("[AreaPositionCheck] 网络失败，进入缓冲区等待状态并启动轮询...");
                            g_is_buffer_waiting = true;
                            g_buffer_wait_timer.start();
                        }
                    }
                } else {
                    // 如果已经在等待状态，只记录日志，不重复请求
                    ROS_INFO_THROTTLE(5.0, "[AreaPositionCheck] 已在缓冲区等待状态，等待轮询结果...");
                }
                // 更新请求时间
                g_last_area_request_time = current_time;
            }
        }
    }
    // 【离开复位逻辑】区域等待点
    // 如果当前记录了在某个区域等待点，但计算发现已经远离该点（> 1.5米），则重置
    if (!g_last_triggered_area_point.empty()) 
    {
        bool still_in_range = false;
        // 查找当前锁定的点坐标
        for (const auto& point : g_area_points) {
            if (point.name == g_last_triggered_area_point) {
                double dx = g_realtime_x - point.x;
                double dy = g_realtime_y - point.y;
                double dist = std::sqrt(dx * dx + dy * dy);
                
                // 迟滞比较：进入用 1.0，离开用 1.5，避免在边界处反复触发
                if (dist <= 0.8) { 
                    still_in_range = true;
                }
                break;
            }
        }

        if (!still_in_range) {
            ROS_INFO_STREAM("[AreaPositionCheck] AGV驶离区域等待点: " << g_last_triggered_area_point << "，重置触发锁。");
            g_last_triggered_area_point = ""; // 允许下一次再次触发
        }
    }

    // 【新增】区域进入/离开检测
    bool currently_in_area = isPointInPolygon(g_realtime_x, g_realtime_y, g_area_polygon);
    
    // 区域检测：进入/离开区域时，发送相应请求
    if (currently_in_area != g_is_in_area) {
        if (currently_in_area) {
            ROS_INFO_STREAM("[AreaCheck] AGV进入封闭区域，发送进入请求...");
            postAreaEventRequest(5); // 5 = 进入区域
        } else {
            ROS_INFO_STREAM("[AreaCheck] AGV离开封闭区域，发送离开请求...");
            postAreaEventRequest(6); // 6 = 离开区域
        }
        g_is_in_area = currently_in_area;
    }
}
/* ====================================================== */
/* 发布任务 (已修改为使用嵌套地图查找) */
/* ====================================================== */
void publishOneTask(const Task &t, const std::string &orderCode)
{
    int dispatchCode = 0;

    // 【新逻辑】直接使用两个字符串进行查找，并用try-catch处理查找失败的情况
    try
    {
        dispatchCode = ENDPOINT_CODE_MAP.at(t.startPositionCode).at(t.endPositionCode);
    }
    catch (const std::out_of_range &oor)
    {
        ROS_ERROR_STREAM("Unknown task combination: FROM [" << t.startPositionCode
                                                            << "] TO [" << t.endPositionCode << "]");
        g_current_dispatch_code = -1;
        return;
    }

    // 发布消息 (这部分不变)
    std_msgs::Int32 msg;
    msg.data = dispatchCode;

    g_current_dispatch_code = dispatchCode;
    g_dispatch_pub.publish(msg);

    ROS_INFO_STREAM("Dispatched task for order " << orderCode
                                                 << ": Path \"" << t.startPositionCode << " -> " << t.endPositionCode
                                                 << "\" -> code " << dispatchCode << ". Waiting for completion confirmation.");
}
