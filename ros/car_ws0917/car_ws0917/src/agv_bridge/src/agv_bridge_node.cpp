#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <algorithm>
#include <chrono>
#include <cstdint>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <serial/serial.h> // 串口通信库
#include <std_msgs/String.h>

/*************** 几何与超时 ***************/
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
const double LIDAR_TIMEOUT = 0.5;

/*************** 全局状态（雷达） ***************/
ros::Time last_scan1_time(0.0), last_scan2_time(0.0);
uint8_t scan1_fault = 1;
uint8_t scan2_fault = 1;
float minDistance1 = 0.0;
float minDistance2 = 0.0;
float minDistancey1 = 0.0;
float minDistancey2 = 0.0;
float minDistancey = 0.0;

uint8_t rp_errorFlag = 0; // 0：雷达正常    1：前雷达数据丢失    2：后雷达数据丢失   3：前后雷达数据丢失

/*************** 全局串口与并发 ***************/
serial::Serial g_ser;
std::mutex g_tx_mtx;
int g_min_tx_gap_ms = 2;
std::chrono::steady_clock::time_point g_last_tx_tp;
std::chrono::steady_clock::time_point last_takeover_cancel_pub_tp_;
/*************** 任务帧状态与接收帧参数 ***************/
uint8_t g_last_task_id = 0; // 'd'：记录上次下发任务编号，决定切换标志
int g_frame_len = 6;        // 'a' 帧总长度
int g_debounce_ms = 500;    // 2->3 防抖
int debounces_ms = 400;     // 1->(3)2防抖
bool g_log_hex = true;
uint8_t g_last_agv_status = 3; // 上一次 'a' 帧的任务状态
std::chrono::steady_clock::time_point g_last_takeover_tp;

/*************** 多线协议参数 ***************/
int g_ndt_maching_default = 0; // 't' 帧的 maching 字段（0 正常 1 失败）

/*************** 实用函数 ***************/
inline bool isInRectangle(float x, float y)
{
  return (x >= -0.6 && x <= 0.6 && y >= 0 && y < 2.0);
}
inline bool isInRectangleRear(float x, float y)
{
  const float offset = 0.6f;
  x -= offset;
  return (x >= -0.6f && x <= 0.6f && y >= 0.0f && y < 2.0f);
}
inline float toRadians(float degree) { return degree * M_PI / 180.0; }

float calculateCoordinates(float degree, float distance, float &x, float &y)
{
  float radians = 0.0;
  if (degree >= 90)
  {
    radians = toRadians(180 - degree); // 第一象限: 90° 到 180°
    x = distance * sin(radians);
    y = distance * cos(radians);
  }
  else if (degree <= -90)
  {
    radians = toRadians(degree + 180); // 第二象限: -90° 到 -180°
    x = -(distance * sin(radians));
    y = distance * cos(radians);
  }
  return y;
}

// 统一发送：带互斥与最小帧间隔
static void sendFrame(const std::vector<uint8_t> &frame)
{
  std::lock_guard<std::mutex> lk(g_tx_mtx);
  auto now = std::chrono::steady_clock::now();
  auto gap = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_tx_tp).count();
  if (gap < g_min_tx_gap_ms)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(g_min_tx_gap_ms - gap));
  }
  g_ser.write(reinterpret_cast<const uint8_t *>(frame.data()), frame.size());
  g_last_tx_tp = std::chrono::steady_clock::now();
}

// 24位/16位大端封装（带符号）
static uint8_t checksum_sum1to10(const std::vector<uint8_t> &f)
{
  uint32_t s = 0;
  for (int i = 1; i <= 10; ++i)
    s += f[i];
  return static_cast<uint8_t>(s & 0xFF);
}

/*************** /scan1 与 /scan2 回调 ***************/
void scanCallback1(const sensor_msgs::LaserScan::ConstPtr &scan)
{

  int count = scan->scan_time / scan->time_increment;
  minDistance1 = std::numeric_limits<float>::infinity();
  last_scan1_time = ros::Time::now();
  scan1_fault = 0;

  for (int i = 0; i < count; i++)
  {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    float distance = scan->ranges[i];
    if (degree >= 90 || degree <= -90)
    {
      float x = 0, y = 0;
      minDistancey = calculateCoordinates(degree, distance, x, y);
      if (isInRectangle(x, y))
      {
        if (distance < minDistance1)
        {
          minDistance1 = distance;
          minDistancey1 = minDistancey;
        }
      }
      // else { minDistance1 = 6; minDistancey1 = 6; }
    }
  }
  if (minDistance1 == std::numeric_limits<float>::infinity())
  {
    minDistance1 = 6;
    minDistancey1 = 6;
  }
  ROS_INFO("frontLidarDistance=%.2f", minDistance1);
}
void scanCallback2(const sensor_msgs::LaserScan::ConstPtr &scan)
{

  int count = scan->scan_time / scan->time_increment;
  minDistance2 = std::numeric_limits<float>::infinity();
  last_scan2_time = ros::Time::now();
  scan2_fault = 0;

  for (int i = 0; i < count; i++)
  {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    float distance = scan->ranges[i];
    if (degree >= 90 || degree <= -90)
    {
      float x = 0, y = 0;
      minDistancey = calculateCoordinates(degree, distance, x, y);
      if (isInRectangleRear(x, y))
      {
        if (distance < minDistance2)
        {
          minDistance2 = distance;
          minDistancey2 = minDistancey;
        }
      }
      // else { minDistance2 = 6; minDistancey2 = 6; }
    }
  }
  if (minDistance2 == std::numeric_limits<float>::infinity())
  {
    minDistance2 = 6;
    minDistancey2 = 6;
  }
  ROS_INFO("rearLidarDistance=%.2f", minDistance2);
}

/*************** 't'：/ndt_pose_m -> 多线雷达协议 12B 下发 ***************/
void ndtPoseMCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  if (!g_ser.isOpen() || !msg || msg->data.size() < 4)
    return;

  int32_t x100 = msg->data[1];
  int32_t y100 = msg->data[2];
  int32_t yaw100 = msg->data[3];
  static ros::Publisher debug_pub;
  if (!debug_pub)
  {
    ros::NodeHandle nh;
    debug_pub = nh.advertise<std_msgs::String>("debug_xy", 10);
  }
  std_msgs::String debug_msg;
  debug_msg.data = "x100:" + std::to_string(x100) + ",y100:" + std::to_string(y100);
  debug_pub.publish(debug_msg);

  std::vector<uint8_t> f(12, 0x00);
  f[0] = static_cast<uint8_t>('t');
  f[1] = static_cast<uint8_t>(g_ndt_maching_default & 0xFF);
  int32_t x_abs = abs(x100);
  uint8_t x_sign = (x100 < 0) ? 0x80 : 0x00;
  f[2] = static_cast<uint8_t>((x_abs >> 16) & 0x7F) | x_sign;
  f[3] = static_cast<uint8_t>((x_abs >> 8)) & 0xFF;
  f[4] = static_cast<uint8_t>(x_abs & 0xFF);

  int32_t y_abs = abs(y100);
  uint8_t y_sign = (y100 < 0) ? 0x80 : 0x00;
  f[5] = static_cast<uint8_t>((y_abs >> 16) & 0x7F) | y_sign;
  f[6] = static_cast<uint8_t>((y_abs >> 8)) & 0xFF;
  f[7] = static_cast<uint8_t>(y_abs & 0xFF);

  uint16_t yaw_unsigned = static_cast<uint16_t>(yaw100);
  f[8] = (yaw_unsigned >> 8) & 0xFF;
  f[9] = yaw_unsigned & 0xFF;
  f[10] = 0x00;
  f[11] = checksum_sum1to10(f);

  try
  {
    sendFrame(f);
    if (g_log_hex)
    {
      // 太多先不打印
      // ROS_INFO_STREAM("[serial] TX 't' 12B (mach=" << int(f[1]) << ")");
    }
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("[serial] send 't' failed: " << e.what());
  }
}

/*************** 'd'：/task_sequence_request -> 任务协议 12B 下发 ***************/
void taskCb(const std_msgs::Int32::ConstPtr &msg)
{
  if (!g_ser.isOpen())
    return;
  const uint8_t task_id = static_cast<uint8_t>(msg->data & 0xFF);
  const uint8_t switch_flag = (task_id != g_last_task_id) ? 1 : 0;

  std::vector<uint8_t> frame(12, 0x00);
  frame[0] = static_cast<uint8_t>('d');
  frame[1] = task_id;
  frame[2] = switch_flag;
  frame[11] = 0xFC;

  try
  {
    sendFrame(frame);
    g_last_task_id = task_id;
    if (g_log_hex)
    {
      ROS_INFO_STREAM("[serial] TX 'd' 12B (id=" << int(task_id) << ", sw=" << int(switch_flag) << ")");
    }
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("[serial] send 'd' failed: " << e.what());
  }
}

/*************** 'w'：/agv_wait_flag -> 停车等待/继续行驶信号 12B 下发 ***************/
void waitFlagCallback(const std_msgs::Int32::ConstPtr &msg)
{
  if (!g_ser.isOpen())
    return;

  // 构造 12 字节帧
  // 协议定义：Header='w' (0x77), Data1=wait_flag (0或1), Footer=0xFC (参考 'd' 帧格式)
  std::vector<uint8_t> frame(12, 0x00);
  frame[0] = static_cast<uint8_t>('w');       // 帧头
  frame[1] = static_cast<uint8_t>(msg->data); // 第一位数据字节：停车标志 (1=停, 0=行)
  frame[11] = checksum_sum1to10(frame);       // 帧尾

  ROS_INFO("agv_wait_flag: %d", msg->data); // 2026/03/02

  try
  {
    sendFrame(frame);
    // 【日志修改】移除 if(g_log_hex) 限制，强制打印，并显示校验值
    ROS_INFO_STREAM("[client] SERIAL TX 'w': wait_flag=" << msg->data
                                                         << " | Checksum=" << (int)frame[11]
                                                         << " (Hex: 0x" << std::hex << (int)frame[11] << std::dec << ")");
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("[serial] send 'w' failed: " << e.what());
  }
}
/*************** 'a'：接收线程，解析回传状态 ***************/
ros::Publisher g_pub_feedback;       // std_msgs/UInt8MultiArray  [id, done, task]
ros::Publisher g_pub_takeover;       // std_msgs/UInt8            data=1 (2->3)
ros::Publisher pub_takeover_cancel_; // 取消接管话题发布1->3
ros::Publisher g_pub_ob;

/*************** 'a'：接收线程，解析回传状态 (根据新协议更新) ***************/
void rxThread()
{
  // 根据新协议定义常量，总长度9字节
  const uint8_t FRAME_HEADER = static_cast<uint8_t>('a');
  const size_t EXPECTED_FRAME_LENGTH = 9; // 1B header + 7B data + 1B checksum
  std::vector<uint8_t> frame;
  std::vector<uint8_t> buf;
  buf.reserve(512);

  while (ros::ok())
  {
    if (!g_ser.isOpen())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    // 从串口读取数据并放入缓冲区 (此部分逻辑不变)
    try
    {
      size_t avail = g_ser.available();
      if (avail == 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      std::string s = g_ser.read(avail);
      if (!s.empty())
      {
        buf.insert(buf.end(), s.begin(), s.end());
      }
    }
    catch (const std::exception &e)
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "[serial] read error: " << e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // 循环解析缓冲区中的数据帧
    while (true)
    {
      // 1. 寻找帧头 'a'
      auto it = std::find(buf.begin(), buf.end(), FRAME_HEADER);
      if (it == buf.end())
      {
        if (buf.size() > 2048)
          buf.clear();
        break;
      }

      // 2. 清理帧头前的所有无效数据
      size_t header_idx = std::distance(buf.begin(), it);
      if (header_idx > 0)
      {
        buf.erase(buf.begin(), buf.begin() + header_idx);
      }

      // 3. 检查是否有足够的数据构成一个完整的帧
      if (buf.size() < EXPECTED_FRAME_LENGTH)
      {
        break;
      }

      std::vector<uint8_t> frame(buf.begin(), buf.begin() + EXPECTED_FRAME_LENGTH);

      // 4. 【新增】验证校验和 (sum(1..7) at index 8)
      uint8_t received_checksum = frame[8];
      uint8_t calculated_checksum = 0;
      for (size_t i = 1; i <= 7; ++i)
      { // 累加所有7个数据字节
        calculated_checksum += frame[i];
      }

      if (received_checksum != calculated_checksum)
      {
        ROS_WARN_STREAM("[serial] Checksum error! Received: " << (int)received_checksum
                                                              << ", Calculated: " << (int)calculated_checksum << ". Discarding frame.");
        buf.erase(buf.begin()); // 校验失败，只丢弃错误的帧头，继续寻找下一个
        continue;
      }

      // 校验成功，从缓冲区正式移除整个帧
      buf.erase(buf.begin(), buf.begin() + EXPECTED_FRAME_LENGTH);

      // 5. 【已更新】解析所有7个数据字节
      uint8_t executing_id = frame[1];       // 正在执行的编号 (不变)
      uint8_t done_flag = frame[2];          // 是否完成 (不变)
      uint8_t agv_mode_status = frame[4];    // 任务/遥控状态 (不变)
      uint8_t agv_vehicle_status = frame[3]; // 车辆状态 (不变)
      uint8_t agv_battery = frame[5];        // 电量 (不变, 但现在后面有新数据)
      uint8_t agv_brush_status = frame[6];   // 【新】电刷状态
      uint8_t agv_charge_status = frame[7];  // 【新】充电状态

      // 【已更新】发布包含7个数据字段的ROS消息
      std_msgs::UInt8MultiArray fb;
      fb.data = {executing_id, done_flag, agv_mode_status, agv_vehicle_status, agv_battery, agv_brush_status, agv_charge_status};
      g_pub_feedback.publish(fb);

      // 6. 【已修正】打印完整、正确的日志
      if (g_log_hex)
      {
        // ROS_INFO_STREAM("[serial] RX 'a': id=" << int(executing_id)
        //                                        << " done=" << int(done_flag)
        //                                        << " mode=" << int(agv_mode_status)
        //                                        << " status=" << int(agv_vehicle_status)
        //                                        << " battery=" << int(agv_battery)
        //                                        << " brush=" << int(agv_brush_status)
        //                                        << " charge=" << int(agv_charge_status));
      }

      // 7. 处理人工接管逻辑 (使用 frame[3] 任务状态，此逻辑不变)
      if (g_last_agv_status == 2 && agv_mode_status == 3)
      {
        // 使用正确的 steady_clock
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_takeover_tp).count();
        if (elapsed >= g_debounce_ms)
        {
          std_msgs::UInt8 t;
          t.data = 1;
          g_pub_takeover.publish(t);
          g_last_takeover_tp = now;
          ROS_WARN("[serial] takeover event: 2->3");
        }
      }
      // ===8. 遥控(1)→（经过3）->自驾(2)：取消接管（修改后，复用带防抖） ===
      if ((g_last_agv_status == 1 && agv_mode_status == 2) || (g_last_agv_status == 3 && agv_mode_status == 2))
      {
        ROS_WARN_STREAM("[状态切换3] 触发事件 g_last_agv_status:"
                        << int(g_last_agv_status) << " agv_status:" << int(agv_mode_status));
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_takeover_cancel_pub_tp_).count();
        if (elapsed_ms >= debounces_ms)
        {
          std_msgs::UInt8 cancel;
          cancel.data = 1;
          pub_takeover_cancel_.publish(cancel);
          last_takeover_cancel_pub_tp_ = now; // 更新取消接管事件的发布时间
          ROS_WARN("[task_serial_bridge] 触发事件：遥控->自驾(1->3->2)，已发布 /manual_takeover_cancel");
        }
        else
        {
          ROS_DEBUG("[task_serial_bridge] 取消接管事件防抖中，未再次发布");
        }
      }
      g_last_agv_status = agv_mode_status;
    }
  }
}

/*************** 定时器：发送 'l' 12B 帧（你原来的协议） ***************/
void timerCallback(const ros::TimerEvent &) // 2025/10/10改
{
  ros::Time now = ros::Time::now();

  const ros::Duration timeout_duration(LIDAR_TIMEOUT);
  bool front_ok = (last_scan1_time != ros::Time(0.0)) && ((now - last_scan1_time) < timeout_duration);
  bool rear_ok = (last_scan2_time != ros::Time(0.0)) && ((now - last_scan2_time) < timeout_duration);

  if (front_ok && rear_ok)
  {
    rp_errorFlag = 0;
  }
  else if (!front_ok && rear_ok)
  {
    rp_errorFlag = 1;
  }
  else if (front_ok && !rear_ok)
  {
    rp_errorFlag = 2;
  }
  else
  {
    rp_errorFlag = 3;
  }

  int minDisty1_dm = static_cast<int>(minDistance1 * 10);
  int minDisty2_dm = static_cast<int>(minDistance2 * 10);

  std::vector<uint8_t> data(12, 0x00);
  data[0] = 'l';
  data[1] = minDisty1_dm & 0xFF;
  data[7] = minDisty2_dm & 0xFF;
  // data[2..6]、[8..9] 维持 0
  uint8_t sum = 0;
  for (int i = 1; i < 10; i++)
    sum += data[i];
  data[10] = sum;
  data[11] = 0xFB;

  // if (g_ser.isOpen()) {
  //   try { sendFrame(data); }
  //   catch (const std::exception& e) { ROS_ERROR_STREAM("[serial] send 'l' failed: "<<e.what()); }
  // }

  std_msgs::UInt8MultiArray ob_msg;

  ob_msg.data = {rp_errorFlag, (uint8_t)minDisty1_dm, (uint8_t)minDisty2_dm, 0x00, 0x00, 0x00, 0x00, 0x00};
  g_pub_ob.publish(ob_msg);
}

/*************** main ***************/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "agv_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 参数
  std::string port = "/dev/ttyS7";
  int baud = 115200, timeout_ms = 1000;
  pnh.param<std::string>("serial_port", port, port);
  pnh.param("baudrate", baud, baud);
  pnh.param("timeout_ms", timeout_ms, timeout_ms);
  pnh.param("frame_len", g_frame_len, g_frame_len); // 'a' 帧长度
  pnh.param("debounce_ms", g_debounce_ms, g_debounce_ms);
  pnh.param("min_tx_gap_ms", g_min_tx_gap_ms, g_min_tx_gap_ms);
  pnh.param("ndt_maching_default", g_ndt_maching_default, g_ndt_maching_default);
  pnh.param("log_hex", g_log_hex, g_log_hex);

  // 订阅雷达
  ros::Subscriber sub1 = nh.subscribe<sensor_msgs::LaserScan>("/scan1", 10, scanCallback1); // 1015改 1000->10
  ros::Subscriber sub2 = nh.subscribe<sensor_msgs::LaserScan>("/scan2", 10, scanCallback2); // 1015改 1000->10

  // 用于下发 't' 协议
  ros::Subscriber ndt_pose_m_tx = nh.subscribe<std_msgs::Int32MultiArray>("/ndt_pose_m", 10, ndtPoseMCallback);

  // 任务下发：'d'
  ros::Subscriber sub_task = nh.subscribe<std_msgs::Int32>("task_sequence_request", 10, taskCb);

  // 【新增】订阅停车/放行标志：agv_wait_flag -> 发送 'w' 帧
  ros::Subscriber sub_wait_flag = nh.subscribe<std_msgs::Int32>("agv_wait_flag", 10, waitFlagCallback);

  // 回传与接管事件话题
  g_pub_feedback = nh.advertise<std_msgs::UInt8MultiArray>("task_sequence_complete", 10);
  g_pub_takeover = nh.advertise<std_msgs::UInt8>("manual_takeover", 10);
  pub_takeover_cancel_ = nh.advertise<std_msgs::UInt8>("manual_takeover_cancel", 10); // 取消接管发布
  g_pub_ob = nh.advertise<std_msgs::UInt8MultiArray>("ob_pub", 10);

  // 定时器：周期性发送 'l'
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);

  // 打开串口
  try
  {
    g_ser.setPort(port);
    g_ser.setBaudrate(static_cast<unsigned long>(baud));
    // 兼容性好的 5参超时
    g_ser.setTimeout(0, timeout_ms, 0, timeout_ms, 0);
    g_ser.setParity(serial::parity_none);
    g_ser.setBytesize(serial::eightbits);
    g_ser.setStopbits(serial::stopbits_one);
    g_ser.open();
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Unable to open port: " << e.what());
    return -1;
  }
  if (g_ser.isOpen())
    ROS_INFO_STREAM("Serial Port opened: " << port << " @" << baud);
  else
    return -1;

  // 初始化时间点
  g_last_takeover_tp = std::chrono::steady_clock::now() - std::chrono::milliseconds(g_debounce_ms);
  g_last_tx_tp = std::chrono::steady_clock::now() - std::chrono::milliseconds(g_min_tx_gap_ms);
  last_takeover_cancel_pub_tp_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(debounces_ms);
  // 接收线程
  std::thread rx_thread(rxThread);

  ros::spin();

  if (rx_thread.joinable())
    rx_thread.join();
  return 0;
}
