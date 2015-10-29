#include <ros/ros.h>
#include <baxter_moves_library/my_interesting_moves.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "interesting_moves_usage");   //this is the name of this node
	ros::NodeHandle nh;  //TODO is this necessary?


	//create an instance of my InterestingMoves class
	InterestingMoves im(&nh);

	//have the robot perform the following actions that are laid out in the class
	im.set_goal_wave();

	for(int i = 0; i < 100; i++){
		//just wait for a second before robot moves again
	}

	im.set_goal_extend_arm();

	for(int i = 0; i < 100; i++){
		//just wait for a second before robot moves again
	}
	
	im.set_goal_high_five();

	return 0;
}