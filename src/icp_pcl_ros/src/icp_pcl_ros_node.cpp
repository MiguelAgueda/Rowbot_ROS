#include <icp_pcl_ros/icpmodule.hpp>


int main (int argc, char **argv)
{
    ros::init(argc, argv, "icp_module");
    ros::NodeHandle nh;

    ICP myICP = ICP(&nh);

    ros::spin();
    return 0;
}