#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <mutex>
#include <string>

static const uint32_t RP_CAN_ID = 0x010;
static const uint8_t DLC = 8;

static int socket_fd = -1;
static uint8_t rp_buf[DLC] = {0};
static std::mutex rp_mtx;

static int open_can_socket(const std::string &ifname)
{
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
    {
        ROS_ERROR("socket(PF_CAN, SOCK_RAW, CAN_RAW) failed: errno=%d (%s)", errno, strerror(errno));
        return -1;
    }
    ROS_INFO("CAN socket created: fd=%d", s);

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
    {
        ROS_ERROR("ioctl(SIOCGIFINDEX) failed for ifname=%s: errno=%d (%s)", ifname.c_str(), errno, strerror(errno));
        close(s);
        return -1;
    }
    ROS_INFO("CAN ifindex for %s is %d", ifname.c_str(), ifr.ifr_ifindex);

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        ROS_ERROR("bind(AF_CAN) failed on %s: errno=%d (%s)", ifname.c_str(), errno, strerror(errno));
        close(s);
        return -1;
    }

    ROS_INFO("CAN bind success on %s (fd=%d)", ifname.c_str(), s);
    return s;
}

static void sendCanMessage(int fd, bool extended_id, uint32_t canID, const uint8_t *canbuf)
{
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = canID;
    if (extended_id)
        frame.can_id |= CAN_EFF_FLAG;

    frame.can_dlc = DLC;
    std::memcpy(frame.data, canbuf, DLC);

    ROS_INFO("[CAN SEND] fd=%d id=0x%X dlc=%d data=%02X %02X %02X %02X %02X %02X %02X %02X",
             fd, frame.can_id, frame.can_dlc,
             frame.data[0], frame.data[1], frame.data[2], frame.data[3],
             frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

    int n = write(fd, &frame, sizeof(frame));
    if (n != (int)sizeof(frame))
    {
        ROS_ERROR("[CAN ERROR] write failed: n=%d expected=%zu errno=%d (%s)",
                  n, sizeof(frame), errno, strerror(errno));
    }
    else
    {
        ROS_INFO("[CAN OK] write success");
    }
}

static void obCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
    const size_t n = msg->data.size();
    if (n < 3)
    {
        ROS_WARN("[obCallback] ob_pub too short: size=%zu (need >=3)", n);
        return;
    }

    uint8_t flag = msg->data[0];
    uint8_t front = msg->data[1];
    uint8_t rear = msg->data[2];

    {
        std::lock_guard<std::mutex> lk(rp_mtx);
        std::memset(rp_buf, 0, DLC);
        rp_buf[0] = flag;
        rp_buf[1] = front;
        rp_buf[2] = rear;
        rp_buf[7] = (uint8_t)(rp_buf[0] + rp_buf[1] + rp_buf[2]);
    }

    ROS_INFO("[obCallback] size=%zu flag=%u front=%u rear=%u checksum=0x%02X",
             n, flag, front, rear, (uint8_t)(flag + front + rear));
}

static void obSend_TimerCallback(const ros::TimerEvent &)
{
    uint8_t local[DLC];
    {
        std::lock_guard<std::mutex> lk(rp_mtx);
        std::memcpy(local, rp_buf, DLC);
    }

    ROS_INFO("[TIMER] try send CAN id=0x%X", RP_CAN_ID);

    if (socket_fd < 0)
    {
        ROS_ERROR("[TIMER] socket_fd invalid (%d), not sending", socket_fd);
        return;
    }

    sendCanMessage(socket_fd, false, RP_CAN_ID, local);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string can_if = "can1";
    std::string ob_topic = "ob_pub";
    double send_rate_hz = 100.0;

    pnh.param<std::string>("can_interface", can_if, can_if);
    pnh.param<std::string>("ob_topic", ob_topic, ob_topic);
    pnh.param<double>("send_rate_hz", send_rate_hz, send_rate_hz);

    if (send_rate_hz <= 0.0)
        send_rate_hz = 100.0;
    double period = 1.0 / send_rate_hz;

    ROS_INFO("Params: can_interface=%s ob_topic=%s send_rate_hz=%.2f period=%.4fs",
             can_if.c_str(), ob_topic.c_str(), send_rate_hz, period);

    socket_fd = open_can_socket(can_if);
    ROS_INFO("socket_fd=%d", socket_fd);

    ros::Subscriber ob_sub = nh.subscribe<std_msgs::UInt8MultiArray>(ob_topic, 10, obCallback);
    ros::Timer send_timer = nh.createTimer(ros::Duration(period), obSend_TimerCallback);

    ros::spin();

    if (socket_fd >= 0)
        close(socket_fd);
    return 0;
}
