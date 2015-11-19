#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/cwru_baxter_cart_moveAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cwru_pcl_utils/cwru_pcl_utils.h>

//start at pre pose?
//needs to subscribe to rviz topic of selected points
//when receives info from rviz topic:
	//calculate centroid of selected points
	//add some number to the z-axis number
	//compute trajectory to have hand of baxter move to this point
	//have baxter move hand back and forth around this point

// bring in class definition, verbatim
// this is a work around until header file for this class is created
#include "arm_motion_commander_class.cpp"



int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_guided_motion_action_client"); // name this node
    ros::NodeHandle nh;

    // create an instance of arm commander class and cwru_pcl_utils class
    ArmMotionCommander arm_motion_commander(&nh);
    CwruPclUtils cwru_pcl_utils(&nh);

    // wait to receive point cloud
    while (!cwru_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");

    tf::StampedTransform tf_sensor_frame_to_torso_frame; // use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; // start a transform listener

    // warm up the tf_listener (this is to make sure it get's all the transforms it needs to avoid crashing)
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); // tf_listener found a complete chain from sensor to world



    Eigen::Affine3f A_sensor_wrt_torso;
    Eigen::Affine3d Affine_des_gripper;
    Eigen::Vector3d xvec_des,yvec_des,zvec_des,origin_des;
    geometry_msgs::PoseStamped rt_tool_pose;
    Eigen::Vector3f plane_normal, major_axis, centroid;
    Eigen::Matrix3d Rmat;
    int rtn_val;    
    double plane_dist;

    // convert the tf to an Eigen::Affine
    A_sensor_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    
    // send a command to plan and then execute a joint-space move to pre-defined pose
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
    
    // repeatedly scan for a new patch of selected points
    while (ros::ok()) {
        if (cwru_pcl_utils.got_selected_points()) {
            ROS_INFO("transforming selected points");
            cwru_pcl_utils.transform_selected_points_cloud(A_sensor_wrt_torso);

            //fit a plane to these selected points:
            cwru_pcl_utils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
            ROS_INFO_STREAM("normal: " << plane_normal.transpose() << "\ndist = " << plane_dist);

            major_axis= cwru_pcl_utils.get_major_axis();
            centroid = cwru_pcl_utils.get_centroid();
            cwru_pcl_utils.reset_got_selected_points();  // reset the selected-points trigger
            
            //construct a goal affine pose:
            for (int i = 0; i < 3; i++) {
                origin_des[i] = centroid[i]; // convert to double precision
                zvec_des[i] = -plane_normal[i]; //want tool z pointing OPPOSITE surface normal
                xvec_des[i] = major_axis[i];
            }

            origin_des[2] += 0.02; //raise up 2cm
            yvec_des = zvec_des.cross(xvec_des); //construct consistent right-hand triad
            Rmat.col(0)= xvec_des;
            Rmat.col(1)= yvec_des;
            Rmat.col(2)= zvec_des;
            Affine_des_gripper.linear()=Rmat;
            Affine_des_gripper.translation()=origin_des;
            ROS_INFO_STREAM("des origin: " << Affine_des_gripper.translation().transpose() << endl);
            ROS_INFO_STREAM("orientation: " << endl);
            ROS_INFO_STREAM(Rmat << endl);
            //cout<<"des origin: "<<Affine_des_gripper.translation().transpose()<<endl;
            //cout<<"orientation: "<<endl;
            //cout<<Rmat<<endl;

            //convert des pose from Eigen::Affine to geometry_msgs::PoseStamped
            rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
            //could fill out the header as well...
            
            // send move plan request
            rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
            if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS){ 
                //send command to execute planned motion
                rtn_val=arm_motion_commander.rt_arm_execute_planned_path();

                /* New code added to enable wiping motion */
                geometry_msgs::PoseStamped rt_tool_wipe_table;
                origin_des[1] += 0.1;
                Affine_des_gripper.translation() = origin_des;
                rt_tool_wipe_table.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);

                rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_wipe_table);
                if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS){
                	rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
                } else {
                	ROS_WARN("Cartesian path to desired pose not achievable");
                }

                origin_des[1] -= 0.2;
                Affine_des_gripper.translation() = origin_des;
                rt_tool_wipe_table.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);

                rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_wipe_table);
                if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS){
                	rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
                } else {
                	ROS_WARN("Cartesian path to desired pose not achievable");
                }

                origin_des[1] += 0.2;
                Affine_des_gripper.translation() = origin_des;
                rt_tool_wipe_table.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);

                rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_wipe_table);
                if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS){
                	rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
                } else {
                	ROS_WARN("Cartesian path to desired pose not achievable");
                }

                origin_des[1] -= 0.1;
                Affine_des_gripper.translation() = origin_des;
                rt_tool_wipe_table.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);

                rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_wipe_table);
                if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS){
                	rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
                } else {
                	ROS_WARN("Cartesian path to desired pose not achievable");
                }
                /* end of new code */
            }
            else {
                ROS_WARN("Cartesian path to desired pose not achievable");
            }
        }
        ros::Duration(0.5).sleep(); // sleep for half a second
        ros::spinOnce();
    }
 
    return 0;
}