#include <ros/ros.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "coplanar_points");
    ros::NodeHandle nh;
    
    // create instance of cwru_pcl_utils library
    // when this is instantiated, it initializes necessary subscribers (ex: subscriber to selected rviz points)
    CwruPclUtils pcl_utils(&nh);

    Eigen::Affine3f meow; // TODO rename this

    while(ros::ok()) {
        if(pcl_utils.got_selected_points()) {
            pcl_utils.transform_selected_points_cloud(meow);
            pcl_utils.reset_got_selected_points();
        }
    }

    // subscribe to published points topic
    // use these selected points to find coplanar points (by using cwru_pcl_utils function)
    // select all coplanar points in rviz (do this by publishing those points?)


    // see cwru_pcl_utils_example_main.cpp for how to do the above steps!!!

	return 0;
}