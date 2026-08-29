#include <pthread.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <iomanip>
/*YXZ add*/
#include <std_msgs/Int32.h>

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

static pose initial_pose, predict_pose, previous_pose, ndt_pose, current_pose, localizer_pose;

static double offset_x, offset_y, offset_z, offset_yaw; // current_pos - previous_pose

// Can't load if typed "pcl::PointCloud<pcl::PointXYZRGB> map, add;"
static pcl::PointCloud<pcl::PointXYZ> map, add;

// If the map is loaded, map_loaded will be 1.
static int map_loaded = 0;
static int init_pos_set = 1;

static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

// Default values
static int max_iter = 10;       // Maximum iteration
static float ndt_res = 2.5;     // Resolution//0.44
static double step_size = 0.1;  // Step size0.1
static double trans_eps = 0.01; // Transformation epsilon

static ros::Publisher ndt_pose_pub;
static ros::Publisher ndt_pose_pub_m;
static geometry_msgs::PoseStamped ndt_pose_msg;
std_msgs::Int32MultiArray ndt_pose_msg_m_buf;
std::vector<int32_t> ndt_pose_msg_m;

static double exe_time = 0.0;
static int iteration = 0;
static double trans_probability = 0.0;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;

static double current_velocity = 0.0, previous_velocity = 0.0, previous_previous_velocity = 0.0; // [m/s]
static double current_velocity_x = 0.0, previous_velocity_x = 0.0;
static double current_velocity_y = 0.0, previous_velocity_y = 0.0;
static double current_velocity_z = 0.0, previous_velocity_z = 0.0;
// static double current_velocity_yaw = 0.0, previous_velocity_yaw = 0.0;
static double current_velocity_smooth = 0.0;

static double angular_velocity = 0.0;
/*YXZ add*/
static double previous_angular_velocity = 0.0;

static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;

static ros::Publisher time_ndt_matching_pub;
static std_msgs::Float32 time_ndt_matching;

static int _queue_size = 1;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol;

static ros::Publisher ndt_reliability_pub;
static std_msgs::Float32 ndt_reliability;

static unsigned int points_map_num = 0;

pthread_mutex_t mutex;
static void map_callback(const sensor_msgs::PointCloud2::ConstPtr &input) //回調加載地圖
{
  if (map_loaded == 0)
  {
    if (points_map_num != input->width)
    {
      points_map_num = input->width;

      // Convert the data type(from sensor_msgs to pcl).
      pcl::fromROSMsg(*input, map);
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map)); // map---map_ptr

      // Setting point cloud to be aligned to.
      pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      new_ndt.setResolution(ndt_res);
      new_ndt.setInputTarget(map_ptr);
      new_ndt.setMaximumIterations(max_iter);
      new_ndt.setStepSize(step_size);
      new_ndt.setTransformationEpsilon(trans_eps);

      new_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&mutex);
      ndt = new_ndt;
      pthread_mutex_unlock(&mutex);
      std::cout << "Update points_map." << std::endl;

      map_loaded = 1;
    }
  }
}

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

/*YXZ add*/  
static ros::Time last_pointcloud_time(0); // 定时器检测点云接收超时
static bool pointcloud_timeout = false;
static ros::Publisher wait_flag_pub;
static bool pointcloud_timeout_triggered = false;
static double timeout_threshold = 0.3;  // 0.3秒超时阈值
/*YXZ add*/  
void checkTimeoutCallback(const ros::TimerEvent& e)
{
  ros::Time now = ros::Time::now();
  
  if ((now - last_pointcloud_time).toSec() > timeout_threshold && 
      last_pointcloud_time != ros::Time(0))
  {
    if (!pointcloud_timeout)
    {
      ROS_WARN("Pointcloud timeout! Last received: %.3f seconds ago", 
               (now - last_pointcloud_time).toSec());
      pointcloud_timeout = true;
      
      // 发送停车指令
      if (!pointcloud_timeout_triggered)
      {
        std_msgs::Int32 stop_msg;
        stop_msg.data = 1;  // 1表示停车
        wait_flag_pub.publish(stop_msg);
        pointcloud_timeout_triggered = true;
        ROS_ERROR("Sent stop command due to pointcloud timeout!");
      }
    }
  }
}

void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
  /*YXZ add*/
  last_pointcloud_time = ros::Time::now();  // 更新最后接收时间
  pointcloud_timeout = false;
  /*YXZ add*/
  if (pointcloud_timeout_triggered) // 如果之前触发了超时停车，现在恢复行驶
  {
    std_msgs::Int32 resume_msg;
    resume_msg.data = 0;  // 0表示恢复行驶
    wait_flag_pub.publish(resume_msg);
    ROS_WARN("Pointcloud recovered, resuming movement");
    pointcloud_timeout_triggered = false;

    current_velocity = 0.0;
    current_velocity_x = 0.0;
    current_velocity_y = 0.0;
    current_velocity_z = 0.0;
    angular_velocity = 0.0;
    previous_velocity = 0.0;
    previous_previous_velocity = 0.0;
    current_velocity_smooth = 0.0;
  }

  // ROS_INFO("111");
  if (map_loaded == 1 && init_pos_set == 1)
  {
    // std::cout << "start location." << std::endl;
    matching_start = std::chrono::system_clock::now(); // system time

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion predict_q, ndt_q, current_q, localizer_q;

    pcl::PointXYZ p;
    pcl::PointCloud<pcl::PointXYZ> filtered_scan;

    ros::Time current_scan_time = input->header.stamp;
    static ros::Time previous_scan_time = current_scan_time;

    pcl::fromROSMsg(*input, filtered_scan);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
    int scan_points_num = filtered_scan_ptr->size(); // point number

    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());  // base_link
    Eigen::Matrix4f t2(Eigen::Matrix4f::Identity()); // localizer

    pthread_mutex_lock(&mutex); //上鎖
    ndt.setInputSource(filtered_scan_ptr);

    // Guess the initial gross estimation of the transformation
    double diff_time = (current_scan_time - previous_scan_time).toSec();

    /*YXZ add*/
    if (current_velocity_smooth == 0.0) {
      offset_x = 0.0;
      offset_y = 0.0;
      offset_z = 0.0;
      offset_yaw = 0.0;
    } else {
      offset_x = current_velocity_x * diff_time;
      offset_y = current_velocity_y * diff_time;
      offset_z = current_velocity_z * diff_time;
      offset_yaw = angular_velocity * diff_time;
    }

    predict_pose.x = previous_pose.x + offset_x;
    predict_pose.y = previous_pose.y + offset_y;
    predict_pose.z = previous_pose.z + offset_z;
    predict_pose.roll = previous_pose.roll;
    predict_pose.pitch = previous_pose.pitch;
    predict_pose.yaw = previous_pose.yaw + offset_yaw;

    /*YXZ add*/
    ROS_INFO("\033[1;31mdiff_time: %.3f, current_velocity_x: %.3f, current_velocity_y: %.3f\033[0m", diff_time, current_velocity_x, current_velocity_y);
    ROS_INFO("\033[1;32mcurrent_velocity_smooth: %.3f, previous_pose.x: %.3f, previous_pose.y: %.3f\033[0m", current_velocity_smooth, previous_pose.x, previous_pose.y);

    pose predict_pose_for_ndt;
    predict_pose_for_ndt = predict_pose;

    Eigen::Translation3f init_translation(predict_pose_for_ndt.x, predict_pose_for_ndt.y, predict_pose_for_ndt.z);
    Eigen::AngleAxisf init_rotation_x(predict_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(predict_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(predict_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    ndt.align(*output_cloud, init_guess);

    t = ndt.getFinalTransformation();
    iteration = ndt.getFinalNumIteration(); //得到最終的迭代次數

    trans_probability = ndt.getTransformationProbability();

    t2 = t * tf_btol.inverse();

    pthread_mutex_unlock(&mutex);
    tf::Matrix3x3 mat_l; // localizer
    mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                   static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                   static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

    // Update localizer_pose
    localizer_pose.x = t(0, 3);
    localizer_pose.y = t(1, 3);
    localizer_pose.z = t(2, 3);
    mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

    tf::Matrix3x3 mat_b; // base_link
    mat_b.setValue(static_cast<double>(t2(0, 0)), static_cast<double>(t2(0, 1)), static_cast<double>(t2(0, 2)),
                   static_cast<double>(t2(1, 0)), static_cast<double>(t2(1, 1)), static_cast<double>(t2(1, 2)),
                   static_cast<double>(t2(2, 0)), static_cast<double>(t2(2, 1)), static_cast<double>(t2(2, 2)));

    // Update ndt_pose
    ndt_pose.x = t2(0, 3);
    ndt_pose.y = t2(1, 3);
    ndt_pose.z = t2(2, 3);
    mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

    current_pose.x = ndt_pose.x;
    current_pose.y = ndt_pose.y;
    current_pose.z = ndt_pose.z;
    current_pose.roll = ndt_pose.roll;
    current_pose.pitch = ndt_pose.pitch;
    current_pose.yaw = ndt_pose.yaw;

    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
    diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    /*YXZ add*/
    if (diff_time > timeout_threshold) {  // 型号中断判断器
      current_velocity = 0.0;
      current_velocity_x = 0.0;
      current_velocity_y = 0.0;
      current_velocity_z = 0.0;
      angular_velocity = 0.0;
      diff_time = 0.1;  // 设定安全时间差
    } else if (diff_time < 0.1) {  // 时间差噪声信号阈值滤波器
      current_velocity = previous_velocity;
      current_velocity_x = previous_velocity_x;
      current_velocity_y = previous_velocity_y;
      current_velocity_z = previous_velocity_z;
      angular_velocity = previous_angular_velocity;
      diff_time = 0.1;
    } else {
      current_velocity = (diff_time > 0) ? (diff / diff_time) : 0;
      current_velocity_x = (diff_time > 0) ? (diff_x / diff_time) : 0;
      current_velocity_y = (diff_time > 0) ? (diff_y / diff_time) : 0;
      current_velocity_z = (diff_time > 0) ? (diff_z / diff_time) : 0;
      angular_velocity = (diff_time > 0) ? (diff_yaw / diff_time) : 0;
    }

    current_velocity_smooth = (current_velocity + previous_velocity + previous_previous_velocity) / 3.0;
    if (current_velocity_smooth < 0.4)
    {
      current_velocity_smooth = 0.0;
      /*YXZ add*/
      current_velocity = 0.0;
      current_velocity_x = 0.0;
      current_velocity_y = 0.0;
      current_velocity_z = 0.0;
      angular_velocity = 0.0;
    }

    matching_end = std::chrono::system_clock::now();
    exe_time = std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;
    time_ndt_matching.data = exe_time;
    time_ndt_matching_pub.publish(time_ndt_matching);

    /* Compute NDT_Reliability */
    ndt_reliability.data = Wa * (exe_time / 100.0) * 100.0 + Wb * (iteration / 10.0) * 100.0 +
                           Wc * ((2.0 - trans_probability) / 2.0) * 100.0;
    ndt_reliability_pub.publish(ndt_reliability);



    ndt_q.setRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw);
    ndt_pose_msg.header.frame_id = "/map";
    ndt_pose_msg.header.stamp = current_scan_time;
    ndt_pose_msg.pose.position.x = ndt_pose.x;
    ndt_pose_msg.pose.position.y = ndt_pose.y;
    ndt_pose_msg.pose.position.z = ndt_pose.z;
    ndt_pose_msg.pose.orientation.x = ndt_q.x();
    ndt_pose_msg.pose.orientation.y = ndt_q.y();
    ndt_pose_msg.pose.orientation.z = ndt_q.z();
    ndt_pose_msg.pose.orientation.w = ndt_q.w();

    ndt_pose_pub.publish(ndt_pose_msg);
    double x = current_pose.x;
    double y = current_pose.y;
    double yaw = current_pose.yaw * 180 / 3.14;
    if (yaw < 0)
      yaw += 360;

    ndt_pose_msg_m = {0x11, (int32_t)(x*100), (int32_t)(y*100), (int32_t)(yaw*100), 0x00, 0x00, 0x00, 0x12};
    ndt_pose_msg_m_buf.data = ndt_pose_msg_m;
    ndt_pose_pub_m.publish(ndt_pose_msg_m_buf);

    std::cout << " ndt_pose : "
              << "x = " << std::fixed << std::setprecision(6) << x << ",\t"
              << "y = " << std::fixed << std::setprecision(6) << y << ",\t"
              << "z = " << std::fixed << std::setprecision(6) << current_pose.z << ",\t"
              << "yaw = " << std::fixed << std::setprecision(6) << yaw << std::endl;

// Update previous_***
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    previous_scan_time = current_scan_time;

    previous_previous_velocity = previous_velocity;
    previous_velocity = current_velocity;
    previous_velocity_x = current_velocity_x;
    previous_velocity_y = current_velocity_y;
    previous_velocity_z = current_velocity_z;
    /*YXZ add*/
    previous_angular_velocity = angular_velocity;
  }
}

void *thread_func(void *args)
{
  ros::NodeHandle nh_map;
  ros::CallbackQueue map_callback_queue;
  nh_map.setCallbackQueue(&map_callback_queue);

  ros::Subscriber map_sub = nh_map.subscribe("points_map", 10, map_callback);
  ros::Rate ros_rate(10);
  while (nh_map.ok())
  {
    map_callback_queue.callAvailable(ros::WallDuration()); //調用隊列裏所有
    ros_rate.sleep();
  }

  return nullptr;
}

int main(int argc, char **argv)

{
  ros::init(argc, argv, "ndt_matching");
  ROS_INFO("Started ndt_matching node");
  pthread_mutex_init(&mutex, NULL); //互斥鎖初始化
  ros::NodeHandle nh;
  //初始化节点
  _tf_y = 0;
  _tf_z = 0;
  _tf_roll = 0;
  _tf_pitch = 0;
  _tf_yaw = 0;
  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX()); // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  initial_pose.x = 0.0;
  initial_pose.y = 0.0;
  initial_pose.z = 0.0;
  initial_pose.roll = 0.0;
  initial_pose.pitch = 0.0;
  initial_pose.yaw = 0.0;

  // init set
  localizer_pose.x = initial_pose.x;
  localizer_pose.y = initial_pose.y;
  localizer_pose.z = initial_pose.z;
  localizer_pose.roll = initial_pose.roll;
  localizer_pose.pitch = initial_pose.pitch;
  localizer_pose.yaw = initial_pose.yaw;

  previous_pose.x = initial_pose.x;
  previous_pose.y = initial_pose.y;
  previous_pose.z = initial_pose.z;
  previous_pose.roll = initial_pose.roll;
  previous_pose.pitch = initial_pose.pitch;
  previous_pose.yaw = initial_pose.yaw;

  current_pose.x = initial_pose.x;
  current_pose.y = initial_pose.y;
  current_pose.z = initial_pose.z;
  current_pose.roll = initial_pose.roll;
  current_pose.pitch = initial_pose.pitch;
  current_pose.yaw = initial_pose.yaw;
  ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 10);
  ndt_pose_pub_m = nh.advertise<std_msgs::Int32MultiArray>("/ndt_pose_m", 100);
  time_ndt_matching_pub = nh.advertise<std_msgs::Float32>("/time_ndt_matching", 10);
  /*YXZ add*/
  wait_flag_pub = nh.advertise<std_msgs::Int32>("agv_wait_flag", 10); // 初始化等待标志发布器
	
  ros::Subscriber points_sub = nh.subscribe("filtered_points", _queue_size, points_callback);
  pthread_t thread;
  pthread_create(&thread, NULL, thread_func, NULL);

  /*YXZ add*/
  ros::Timer timeout_timer = nh.createTimer(ros::Duration(0.1), checkTimeoutCallback); // 添加定时器（每0.1秒检查一次）
  last_pointcloud_time = ros::Time::now(); // 初始化最后接收时间

  ros::spin(); //节点进入循环状态，有消息到达时调用回调函数完成处理

  return 0;
}
