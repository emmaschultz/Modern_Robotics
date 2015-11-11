#include <ros/ros.h>
//#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <my_pcl_utils/my_pcl_utils.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coplanar_points");
    ros::NodeHandle nh;
    
    // create instance of cwru_pcl_utils library
    // when this is instantiated, it initializes necessary subscribers (ex: subscriber to selected rviz points)
    //CwruPclUtils pcl_utils(&nh);
    MyPclUtils pcl_utils(&nh);
    
    // wait for a point cloud
    while(!pcl_utils.got_kinect_cloud()){
    	ROS_INFO("did not receive point cloud yet.");
    	ros::spinOnce();
    	ros::Duration(1.0).sleep();
    }

    ROS_INFO("Got a point cloud. Now saving point cloud.");
    pcl_utils.save_kinect_snapshot();
    pcl_utils.save_kinect_clr_snapshot();
    
    // create a publisher of point clouds
    // this will publish all coplanar points so they can be seen in rviz
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_cloud_display", 1);

    pcl::PointCloud<pcl::PointXYZ> display_cloud; // this is the data type that will be published
    sensor_msgs::PointCloud2 pcl2_display_cloud;  // this will be used in the ROS message

    tf::StampedTransform tf_sensor_frame_to_torso_frame;  // use this to transform from sensor to torso frame
    tf::TransformListener tf_listener;

    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while(tferr){
    	tferr = false;
    	try {
    		tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
    	} catch(tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
    		tferr = true;
    		ros::Duration(0.5).sleep();
    		ros::spinOnce();
    	}
    }

    ROS_INFO("tf is good.");

    // convert the tf to an Eigen matrix
    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);

    // transform data to be relative to the torso frame and then save this data
    pcl_utils.transform_kinect_cloud(A_sensor_wrt_torso);
    pcl_utils.save_transformed_kinect_snapshot();
    ROS_INFO("saved transform");

    Eigen::Vector3f plane_normal;
    double plane_dist;
    ROS_INFO("entering while loop...");
    while(ros::ok()) {
        ROS_INFO("you have entered the loop");
        if(pcl_utils.got_selected_points()) {
            ROS_INFO("received selected points");
            pcl_utils.transform_selected_points_cloud(A_sensor_wrt_torso);
            ROS_INFO("transformed selected point cloud");
            pcl_utils.reset_got_selected_points();
            ROS_INFO("reset the selected points");

            // finds what plane the selected points are in
            pcl_utils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
            ROS_INFO_STREAM("normal: " << plane_normal.transpose() << "dist: " << plane_dist);

            pcl_utils.find_coplanar_points();

            pcl_utils.get_gen_purpose_cloud(display_cloud);
        }

        // convert point cloud to be compatible for ros msg and update the time stamp
        pcl::toROSMsg(display_cloud, pcl2_display_cloud);
        pcl2_display_cloud.header.stamp = ros::Time::now();

        // publish the point cloud containing all coplanar points
        pubCloud.publish(pcl2_display_cloud);

        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    ROS_INFO("All done!");

    return 0;
}