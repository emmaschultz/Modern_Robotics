#ifndef MY_INTERESTING_MOVES_H
#define MY_INTERESTING_MOVES_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;

class InterestingMoves {
public:
	ros::NodeHandle nh_;
	int g_count;

	InterestingMoves(ros::NodeHandle *nh);

	/**
	 * This will give the baxter robot the command to wave its right arm
	 */
	void set_goal_wave();

	/**
	 * This will give the baxter robot the command to do the robot dance move with its right arm
	 */
	void set_goal_extend_arm();

	/**
	 * This will give the baxter robot the command to put its right arm out to receive a high five
	 */
	void set_goal_high_five();

private:
	/**
	 * This will calculate the trajectories and send the goal to the action server
	 * @param position: the desired position for the robot to go to
	 */
	void find_and_send_trajectory(Vectorq7x1 position);
};

#endif