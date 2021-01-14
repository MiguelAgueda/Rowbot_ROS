#include <icp_pcl_ros/icpmodule.hpp>

/*
Main function to start ICP node. 
*/
int main (int argc, char **argv)
{
    ros::init(argc, argv, "icp_module");
    ros::NodeHandle nh;

    ICP *myICP = new ICP(&nh);

    ros::spin();
    return 0;
}