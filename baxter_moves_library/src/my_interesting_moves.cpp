
// correct order of joints: right_s0, right_s1, right_e0, right_e1, right_w0, right_w1, right_w2


#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <cwru_action/trajAction.h>
#include <baxter_moves_library/my_interesting_moves.h>
#include <vector>


// constructor
InterestingMoves::InterestingMoves(ros::NodeHandle *nh)
{
    nh_ = *nh;
}

void InterestingMoves::set_goal_extend_arm()
{
    Vectorq7x1 q_extend_arm_pose;
    q_extend_arm_pose << 0, 0, 0, 0, 0, 0, 0;
    find_and_send_trajectory(q_extend_arm_pose);
    ROS_INFO("Arm extension complete.");
}

void InterestingMoves::set_goal_bend_arm()
{
    Vectorq7x1 q_bend_arm_pose;
    q_bend_arm_pose << 0, 0, 3.14, 1.5, 1, 0, 0;
    find_and_send_trajectory(q_bend_arm_pose);
    ROS_INFO("Elbow has been bent. Ready to wave hello.");
}

void InterestingMoves::set_goal_wave_hand()
{
    Vectorq7x1 q_wave_hand_pose;
    q_wave_hand_pose << 0, 0, 3.14, 1.5, 1, -1, 0;  // shoulder0, shoulder1, rotationElbow, bendElbow
    find_and_send_trajectory(q_wave_hand_pose);
    q_wave_hand_pose << 0, 0, 3.14, 1.5, 1, 0, 0;
    find_and_send_trajectory(q_wave_hand_pose);
    q_wave_hand_pose << 0, 0, 3.14, 1.5, 1, -1, 0;
    find_and_send_trajectory(q_wave_hand_pose);
    ROS_INFO("Wave five complete.");
}

void InterestingMoves::find_and_send_trajectory(Vectorq7x1 position)
{
    Vectorq7x1 q_pose;
    q_pose << position;
    Eigen::VectorXd q_in_vecxd;
    Vectorq7x1 q_vec_right_arm;

    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory;
    Baxter_traj_streamer baxter_traj_streamer(&nh_);


    ROS_INFO("warming up callbacks...");
    for (int i = 0; i < 100; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // find the current pose of the robot
    ROS_INFO("getting current right arm pose");
    q_vec_right_arm =  baxter_traj_streamer.get_qvec_right_arm();
    ROS_INFO_STREAM("r_arm state: " << q_vec_right_arm.transpose());

    // push back the current pose of the robot, followed by the desired pose for the robot
    q_in_vecxd = q_vec_right_arm;
    des_path.push_back(q_in_vecxd);
    q_in_vecxd = q_pose;
    des_path.push_back(q_in_vecxd);

    // convert this into a trajectory
    ROS_INFO("stuffing trajectory");
    baxter_traj_streamer.stuff_trajectory(des_path, des_trajectory);

    // copy this trajectory into a goal message
    cwru_action::trajGoal goal;
    goal.trajectory = des_trajectory;

    // initialize this node as an action client
    actionlib::SimpleActionClient<cwru_action::trajAction> action_client("trajActionServer", true);
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0));
    if (!server_exists)
    {
        ROS_WARN("could not connect to server");
        return;
    }
    // server_exists = action_client.waitForServer(); // wait forever
    ROS_INFO("connected to action server");  // if here, then we connected to the server;

    // send the goal to the server
    action_client.sendGoal(goal);

    bool finished_before_timeout = action_client.waitForResult();  // wait forever for result
}
