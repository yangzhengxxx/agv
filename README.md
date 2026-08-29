# AGV 车辆控制与定位系统

基于 STM32F407ZG、ROS Noetic 与 Linux 工控机的 AGV 控制项目，包含底盘控制固件、静态地图 NDT 定位、HTTP 调度、串口控制桥和 SocketCAN 障碍物状态转发。

仓库只保留项目实际使用的业务代码和 STM32 外设模块；Livox、RPLIDAR 等厂商 SDK 与 ROS 雷达驱动需在运行设备上单独安装。

## 功能

- PCD 静态地图加载、点云体素降采样与 NDT 定位
- 前后二维激光雷达障碍物距离计算及掉线检测
- ROS 与 STM32 之间的串口任务、定位和状态协议
- 障碍物状态通过 SocketCAN 周期发送至车辆控制器
- HTTP 调度登录、任务接收、状态反馈、区域占用及等待控制
- STM32 CAN1/CAN2、USART1/UART5、TIM2/TIM5 和车辆控制逻辑

## 目录

```text
ros/
  run.sh                         ROS 系统一键启动脚本
  car_ws0917/car_ws0917/        ROS Noetic Catkin 工作空间
    src/agv_bridge/              串口、定位与雷达数据桥
    src/can_pkg/                 SocketCAN 障碍物状态发送
    src/dispatch_pkg/            HTTP 调度客户端
    src/ndt_mapping/             地图加载、点云过滤与 NDT 定位

stm32f4/
  Project/RVMDK（uv5）/          Keil MDK-ARM 工程
  User/                          AGV 业务与板级驱动代码
  Libraries/                     精简后的 CMSIS 和 STM32 标准外设库
```

## 编译 STM32 固件

目标芯片为 **STM32F407ZG**。使用 Keil MDK-ARM 打开并 Rebuild：

```text
stm32f4/Project/RVMDK（uv5）/BH-F407.uvprojx
```

工程仅保留当前固件实际使用的 CAN、Flash、GPIO、RCC、TIM 和 USART 外设驱动。生成的 BIN、HEX、AXF、Listing 和 IDE 用户配置不会提交到仓库。

## 编译 ROS

推荐环境：Ubuntu 20.04、ROS Noetic、PCL 和支持 SocketCAN 的 Linux 主机。

安装项目额外依赖：

```bash
sudo apt update
sudo apt install \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-serial \
  libcurl4-openssl-dev \
  nlohmann-json3-dev
```

编译 Catkin 工作空间：

```bash
cd ros/car_ws0917/car_ws0917
catkin_make
```

## 运行 ROS

调度服务器地址和凭据必须通过环境变量提供，不保存在源码中：

```bash
export AGV_API_BASE_URL='https://dispatch.example.com'
export AGV_API_USERNAME='your-user'
export AGV_API_PASSWORD='your-password'

bash ros/run.sh
```

也可以在工作空间编译后直接启动：

```bash
source ros/car_ws0917/car_ws0917/devel/setup.bash
roslaunch agv_bridge core.launch
```

主要启动参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `points_topic` | `/livox/lidar` | 三维点云输入 |
| `scan_front_topic` | `/scan1` | 前二维雷达输入 |
| `scan_rear_topic` | `/scan2` | 后二维雷达输入 |
| `controller_port` | `/dev/ttyS7` | STM32 串口设备 |
| `can_interface` | `can1` | SocketCAN 接口 |

覆盖参数示例：

```bash
bash ros/run.sh controller_port:=/dev/ttyUSB0 can_interface:=can0
```

## 雷达与地图

仓库不包含雷达驱动。运行前需要由外部驱动发布：

| 话题 | 消息类型 | 用途 |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | NDT 定位点云 |
| `/scan1` | `sensor_msgs/LaserScan` | 前方障碍物检测 |
| `/scan2` | `sensor_msgs/LaserScan` | 后方障碍物检测 |

默认地图位于：

```text
ros/car_ws0917/car_ws0917/src/ndt_mapping/maps/map0226.pcd
```

替换地图时可覆盖 `map_path` 参数：

```bash
roslaunch ndt_mapping localization.launch map_path:=/absolute/path/to/map.pcd
```

## 通信

- ROS 控制桥默认使用 `115200` 波特率连接 STM32。
- `can_pkg` 以标准帧 CAN ID `0x010` 周期发送障碍物状态。
- SocketCAN 接口的波特率和启停需根据车辆 CAN 总线配置在系统中预先完成。

## 许可证

当前项目未附加统一的开源许可证。STM32 CMSIS、标准外设库及其他第三方组件遵循各自源码中声明的许可证；其余项目代码保留所有权利。
