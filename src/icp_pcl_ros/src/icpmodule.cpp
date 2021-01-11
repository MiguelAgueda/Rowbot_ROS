// #include <icpmodule.hpp>
#include <icp_pcl_ros/icpmodule.hpp>


ICP::ICP(ros::NodeHandle *nh)
{
    icp_msg.header.frame_id = "base_link";
    icp_pub = nh -> advertise<geometry_msgs::Vector3>("icp", 10);
    lidar_sub = nh -> subscribe("pointcloud", 10, &ICP::point_cloud_update, this); 
    // Set the max correspondence distance to 25 cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.25);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);

    std::cout << "ICP Instantiated" << std::endl;
}

void ICP::update_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k, ros::Time t_k)
{
    icp.setInputSource(cloud_km);  // Update source cloud to last cloud.
    icp.setInputTarget(cloud_k);  // Update destination cloud to current cloud.

    pcl::PointCloud<pcl::PointXYZ> Final;  // Create cloud to store result of icp alignment.
    icp.align(Final);  // Align the point clouds defined above.
    // pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 R_p;  // Create Eigen matrix to hold results.
    Eigen::Matrix<float, 4, 4> R_p;  // Create Eigen matrix to hold results.
    R_p = icp.getFinalTransformation();  // Obtain results from ICP.
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
    R = R_p(Eigen::seq(0,2), Eigen::seq(0, 2));
    t = R_p(Eigen::seq(0,2), 3);
    icp_y = R * (icp_y - t);
    icp_msg.header.stamp = t_k;
    icp_msg.vector.x = icp_y(0);
    icp_msg.vector.y = icp_y(1);
    icp_msg.vector.z = icp_y(2);
    
    icp_pub.publish(icp_msg);
}

// const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud< T > &pcl_cloud
// void ICP::point_cloud_update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k)
void ICP::point_cloud_update(const sensor_msgs::PointCloud2ConstPtr &cloud_k_ros)
{ 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_k_pcl;  // Container for converted PCL point cloud.
    pcl::fromROSMsg(*cloud_k_ros, *cloud_k_pcl);  // Convert ROS-PC2 to PCL-PC.
    // Perform ICP.
    update_icp(cloud_k_pcl, cloud_k_ros->header.stamp);
    // Set last cloud to current cloud. This is done upon completing all computations involving last cloud.
    *cloud_km = *cloud_k_pcl;
    // }
}
