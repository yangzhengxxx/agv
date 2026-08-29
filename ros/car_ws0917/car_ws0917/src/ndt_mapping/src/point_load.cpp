#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <string>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_read");
 
    ROS_INFO("Started PCL read node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("points_map", 1);
    
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    std::string map_path;
    private_nh.param<std::string>("map_path", map_path, std::string());
    if (map_path.empty())
    {
        ROS_FATAL("~map_path is required");
        return 1;
    }
    if (pcl::io::loadPCDFile(map_path, cloud) != 0)
    {
        ROS_FATAL_STREAM("Failed to load PCD map: " << map_path);
        return 1;
    }

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;   
}

   
