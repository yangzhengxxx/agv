#include "globals.h"

/* ====================================================== */
/* 人工接管回调：收到1则把当前订单和队列里所有订单逐个完成   */
/* ====================================================== */
void manualTakeoverCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    if (!msg || msg->data != 1)
        return; // 仅当 data==1 才触发

    ROS_WARN("[manual_takeover] 收到人工接管信号，开始将当前及队列中所有订单的任务逐个上报为已完成...");

    // 定义一个工具 lambda，用于清空任意订单的任务 (此函数内部逻辑不变)
    auto completeOrder = [&](AgvOrder &order)
    {
        if (order.orderCode.empty())
            return;

        while (!order.taskList.empty())
        {
            bool is_last = (order.taskList.size() == 1);
            last_task = order.taskList.front();

            taskStatus = "70"; // 任务已完成
            order_staus = is_last ? "70" : "30";

            postReportAgvTaskDetail(/*order_d_ok=*/is_last);

            order.taskList.erase(order.taskList.begin());
        }

        ROS_WARN_STREAM("[manual_takeover] 订单 " << order.orderCode << " 已全部任务完成并标记完成。");
    };

    // --- START: 核心逻辑修改 ---

    // 1) 统一处理所有订单：将当前订单(currentOrder)也放入总队列g_agvOrders的首位进行统一处理
    //    (更稳妥的做法是确保currentOrder就是g_agvOrders.front()的拷贝，所以我们直接遍历g_agvOrders即可)
    //    移除了对 currentOrder 的单独处理，以防止重复。
    //    if (!currentOrder.orderCode.empty()) {
    //      completeOrder(currentOrder);
    //    }

    // 2) 统一、且仅有一次地遍历并清空所有在队列中的订单
    for (auto &order : g_agvOrders)
    {
        completeOrder(order);
    }

    // 3) 清空全局订单队列和索引
    g_agvOrders.clear();
    g_orderCodes.clear();

    // 4) 【已修正】彻底清空本地所有相关的执行状态
    g_agv_state = AgvSystemState::IDLE;
    currentOrder = AgvOrder();
    currentTask = Task();
    last_task = Task(); // <-- 新增：必须重置 last_task 防止发送脏数据
    if_exeing = false;
    if_one = true;
    // agvStatus      = "10";   // 待机
    agvStatus = "80"; // 新增为80人工接管状态
    // order_staus    = "30";   // 默认执行中
    order_staus = "70";
    // agvpostion     ="chargingPark001";
    postRebackAgvStatus();

    // --- END: 核心逻辑修改 ---

    // ROS_WARN("[manual_takeover] 所有订单已清空，车辆进入待机状态。");
    ROS_WARN("[manual_takeover] 所有订单已清空，车辆进入人工接管状态。");
        // ================= 【新增】在人工接管时关闭等待定时器 =================
    if (g_is_waiting_signal) {
        ROS_WARN("[manual_takeover] 检测到车辆正处于等待状态，立即关闭等待轮询定时器");
        g_is_waiting_signal = false;
        g_wait_timer.stop();
        
        // 发布继续行驶信号，确保车辆不会卡在等待状态
        std_msgs::Int32 go_msg;
        go_msg.data = 0;
        g_wait_signal_pub.publish(go_msg);
        ROS_INFO("[manual_takeover] 已发布继续行驶信号并关闭等待定时器");
    }
}
/* ====================================================== */
/*    取消遥控接管逻辑（遥控→默认，状态=10）再检查是否有任务               */
/* ====================================================== */
void manualTakeoverCancelCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    if (!msg || msg->data != 1)
        return;
    ROS_WARN("[manual_takeover_cancel] 收到取消遥控接管信号，遥控→默认待机");
        // 1. 检查是否有任务需要执行
    bool hasTasks = !g_agvOrders.empty() || !currentOrder.orderCode.empty();
    
    if (hasTasks) {
        // 如果有任务，设置状态为执行中
        agvStatus = "30";
        order_staus = "30";
        g_agv_state = AgvSystemState::IDLE;
        ROS_INFO("[manual_takeover_cancel] 检测到有任务，AGV状态设置为执行中(30)");
    } else {
        // 如果没有任务，设置状态为待机
        agvStatus = "10";
        order_staus = "30";
        g_agv_state = AgvSystemState::IDLE;
        ROS_INFO("[manual_takeover_cancel] 无任务，AGV状态设置为待机(10)");
    }

    // 设置 AGV 状态为 10（待机）
    // agvStatus = "10"; 
    // order_staus = "30"; // 默认执行中
    postRebackAgvStatus();

    // 2. 将任务起点改为当前位置
    if (!currentTask.endPositionCode.empty()) {
        currentTask.startPositionCode = agvpostion;
        currentTask.startPositionName = agvpostion;
        ROS_INFO_STREAM("[manual_takeover_cancel] 更新任务起点为当前位置: " << agvpostion);
    }

    // 3. 10秒后任务检测
    // 设置全局标志位：进入 10 秒等待期
    g_is_cancel_delay_active.store(true);
    cancel_pending = true;
    post_cancel_timer.stop(); // 停止任何可能在运行的实例 (安全起见)
    post_cancel_timer.setPeriod(ros::Duration(10.0)); // 确保周期是 10s
    post_cancel_timer.start(); // 启动
    ROS_INFO("[manual_takeover_cancel] 已启动 10 秒任务检测延迟。在此期间主调度器将跳过任务查询。");
}
/* ====================================================== */
/*       10秒后检测任务逻辑（调用调度端接口）              */
/* ====================================================== */
void postCancelTimerCallback(const ros::TimerEvent &)
{
    // 解除全局延迟检测标志位：结束等待期
    g_is_cancel_delay_active.store(false);
    if (!cancel_pending)
        return;
    cancel_pending = false;

    ROS_INFO("[manual_takeover_cancel] 10秒延迟检测结束,调用调度端任务查询接口...");
    postCheckAgvSchedulingOrder();

    if (agvpostion.find("chargingPark") != std::string::npos)
    {
        ROS_INFO("[manual_takeover_cancel] 当前位置为充电点，（调度端）不生成充电任务。");
    }
    else
    {
        ROS_INFO("[manual_takeover_cancel] 当前非充电点，调度端计时生成充电任务。");
    }
}

// 订阅者回调函数
void statusFlagsCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
    // 检查接收到的数据，并根据条件更新 agvStatus
    if (msg->data.size() >= 2)
    {
        // 急停
        if (msg->data[0] == 1)
        {
            agvStatus = "10"; // 如果第一个数据为 1，设置 agvStatus 为 10
            order_staus = "80";
            taskStatus = "20";
            last_agv_status = agvStatus;
            postRebackAgvStatus();
            postReportAgvTaskDetail(if_order_ok);
            ROS_INFO_STREAM("AGV状态与订单状态反馈回去" << agvStatus);
        }
        // 暂停
        else if (msg->data[1] == 1)
        {
            agvStatus = "40"; // 如果第二个数据为 1，设置 agvStatus 为 40
            order_staus = "50";
            taskStatus = "20";
            last_agv_status = agvStatus;
            postRebackAgvStatus();
            postReportAgvTaskDetail(if_order_ok);
            ROS_INFO_STREAM("AGV状态与订单状态反馈回去" << agvStatus);
        }
        // if (msg->data[2] == 1) {
        //   agvStatus = "30";  // 如果第二个数据为 1，设置 agvStatus 为 40
        //   order_staus = "30";
        //   taskStatus = "20";
        //   postRebackAgvStatus();
        //   postReportAgvTaskDetail(if_order_ok);
        //   ROS_INFO_STREAM("AGV状态与订单状态反馈回去"<<agvStatus);
        // }
        // jibuzhantingyebujiting
        else
        {
            if (agvStatus == "10" && g_agvOrders.empty())
            {
                return;
            }
            // 暂时先忽略
            if (last_agv_status != "30")
            {
                agvStatus = "30"; // 否则，保持默认值 30
                order_staus = "30";
                taskStatus = "20";
                postRebackAgvStatus();
                postReportAgvTaskDetail(if_order_ok);
                last_agv_status = "30";
            }
        }

        // 调用接口将更新后的状态发送到服务器
    }
}

/* ====================================================== */
/* 定时器回调   定时查询和发布任务                        */
/* ====================================================== */
void timerCallback(const ros::TimerEvent &)
{
    // 登录和网络检查逻辑保持不变
    if (g_auth_token.empty())
    {
        if (!loginAndFetchToken())
        {
            ROS_WARN_THROTTLE(30, "超时登陆，请检查网络或者服务器");
            return;
        }
        ROS_INFO("登陆成功");
    }

    
    // 检查“取消接管”的 10 秒延迟标志位
    if (g_is_cancel_delay_active.load())
    {
        ROS_INFO_THROTTLE(2.0, "[timer] 正在等待“取消接管”延迟，暂时跳过订单查询...");
    }
    else
    {
        // 只有在延迟未激活时，才执行常规的订单查询
        postCheckAgvSchedulingOrder();
    }
    // 定期查询新订单
    //postCheckAgvSchedulingOrder();

    // 定期发送心跳
    postRebackAgvHeartbeat();

    // 只有在执行任务的状态下，才需要持续向下位机发布当前任务指令
    if (g_agv_state == AgvSystemState::EXECUTING)
    {
        if (currentTask.endPositionCode.empty())
        {
            ROS_WARN_THROTTLE(5.0, "[timer] In EXECUTING state but currentTask is empty. Check logic.");
        }
        else
        {
            publishOneTask(currentTask, currentOrder.orderCode);
        }
    }
}
