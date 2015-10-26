#include <ros/ros.h>
#include <Eigen/Eigen>
#include <baxter_moves_library/my_interesting_moves.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "interesting_moves_usage");   //this is the name of this node
	ros::NodeHandle nh;


	//create an instance of my InterestingMoves class
	InterestingMoves interestingMoves;

	//have the robot perform the following actions that are laid out in the class
	interestingMoves.set_goal_wave(/* param goes here */);
	ROS_INFO("Wave complete.");

	interestingMoves.set_goal_do_the_robot(/* param goes here */);
	ROS_INFO("Dance complete.");

	interestingMoves.set_goal_high_five(/* param goes here */);
	ROS_INFO("High five complete.");
}