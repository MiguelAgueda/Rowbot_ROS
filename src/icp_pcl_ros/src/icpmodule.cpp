// #include <icpmodule.hpp>
#include <icp_pcl_ros/icpmodule.hpp>

/*
ICP Object Constructor.

Parameters
----------
    *nh: Pointer to node handler.
        Used for subscribing and publishing to ROS topics.
*/
ICP::ICP(ros::NodeHandle *nh)
{
    icp_msg.header.frame_id = "base_link";
    icp_pub = nh->advertise<geometry_msgs::Vector3Stamped>("icp", 10);
    lidar_sub = nh->subscribe("/pointcloud", 10, &ICP::point_cloud_update, this);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-9);
    icp.setMaxCorrespondenceDistance(0.5e-1);
    icp.setEuclideanFitnessEpsilon(1);
    icp.setRANSACOutlierRejectionThreshold(.5e-3);


    std::cout << "ICP Instantiated" << std::endl;
}

/*
Callback function to handle incoming point cloud data from /pointcloud topic.
Updates state using ICP, publishes output to /icp topic.

Parameters
----------
    cloud_k_ros: Point cloud specified in ROS' sensor_msgs::PointCloud2 format.
*/
void ICP::point_cloud_update(const sensor_msgs::PointCloud2ConstPtr &cloud_k_ros)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k_pcl; // Container for converted PCL point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k_pcl (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*cloud_k_ros, *cloud_k_pcl); // Convert ROS-PC2 to PCL-PC.
    // std_msgs::Header header = cloud_k_ros->header;
    if (!first_scan)  // Cannot perform ICP without using a previously saved scan.
    {
        // Perform ICP.
        icp.setInputSource(cloud_km);  // Update source cloud to last cloud.
        icp.setInputTarget(cloud_k_pcl);  // Update destination cloud to current cloud.

        pcl::PointCloud<pcl::PointXYZ> Final;  // Create cloud to store result of icp alignment.
        icp.align(Final);  // Align the point clouds defined above.
        Eigen::Matrix<float, 4, 4> R_p;  // Create Eigen matrix to hold rotation and translation results.
        R_p = icp.getFinalTransformation();  // Obtain results from ICP.
        Eigen::Matrix3f R;  // Create container for rotation component of R_p.
        Eigen::Vector3f t;  // Create container for translation component of R_p.
        R = R_p(Eigen::seq(0,2), Eigen::seq(0, 2));
        t = R_p(Eigen::seq(0,2), 3);

        icp_y = R * (icp_y - t);  // Apply rotation and translation to existing pose point [x y z].

        icp_msg.header.stamp = cloud_k_ros->header.stamp;
        icp_msg.vector.x = icp_y(0);
        icp_msg.vector.y = icp_y(1);
        icp_msg.vector.z = icp_y(2);
        // icp_msg.vector.z = 0.0;  // Lock icp output to 2D plane.

        icp_pub.publish(icp_msg);
    }

    // Set last cloud to current cloud. This is done upon completing all computations involving last cloud.
    cloud_km = cloud_k_pcl;

    if (first_scan)
        first_scan = false;
}
