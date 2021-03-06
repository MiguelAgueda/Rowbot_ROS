#include <thread>
#include <mutex>
#include <ros/ros.h>

// PCL specific includes
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class ICP
{
private:
    bool first_scan = true;
    geometry_msgs::Vector3Stamped icp_msg;
    Eigen::Vector3f icp_y = Eigen::Vector3f::Zero();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_km;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    ros::Publisher icp_pub;
    ros::Subscriber lidar_sub;
    void update_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr, ros::Time);

public:
    ICP(ros::NodeHandle *nh);
    void point_cloud_update(const sensor_msgs::PointCloud2ConstPtr &);
};
