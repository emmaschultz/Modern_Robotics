#include <ros/ros.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "coplanar_points");
	ros::NodeHandle nh;
    CwruPclUtils pcl_utils(&nh);


	return 0;
}