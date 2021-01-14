#include <ros/ros.h>
#include <ros/console.h>
// PCL specific includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <laser_geometry/laser_geometry.h>

ros::Publisher pub;
laser_geometry::LaserProjection projector_;

/*
 * Process LiDAR output into PCL point cloud.
 * 
 * Returns
 * -------
 *      pcl::PointCloud::Ptd : Pointer to latest point cloud data.
 * 
 * Assumptions
 * -----------
 *      The rover's environment is flat.
 * 
 *      Since the RPLiDAR A2 is a 2D range scanner, the z-coordinate for every
 *          point is set to zero.
 *      This effect is not accounted for in any way by the ICP algorithm,
 *          leading to a zeroing of the EKF Z-coordinate via corrective update.
 */
// pcl::PointCloud<pcl::PointXYZ>::Ptr get_lidar_data(bool logging, float t_from_start)
void scan_to_pc(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud2 cloud2;

    projector_.projectLaser(*scan, cloud2);
    ROS_DEBUG("Projected Scan to Cloud");

    // sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(n_rays, 1));
    // Publish cloud.
    pub.publish(cloud2);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "scan_to_pc");
    ros::NodeHandle nh;

    ROS_DEBUG("Scan to Point Cloud Generator: Started");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("scan", 1, scan_to_pc);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

    // Spin
    ros::spin();
}
