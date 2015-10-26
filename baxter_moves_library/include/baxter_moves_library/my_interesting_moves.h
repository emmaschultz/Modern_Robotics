#ifndef MY_INTERESTING_MOVES_H
#define MY_INTERESTING_MOVES_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

class InterestingMoves {
public:
	InterestingMoves();

	/**
	 * This will give the baxter robot the command to wave its right arm
	 * @param des_trajectory: 
	 */
	set_goal_wave(trajectory_msgs::JointTrajectory &des_trajectory);

	/**
	 * This will give the baxter robot the command to do the robot dance move with its right arm
	 * @param des_trajectory: 
	 */
	set_goal_do_the_robot(trajectory_msgs::JointTrajectory &des_trajectory);

	/**
	 * This will give the baxter robot the command to put its right arm out to receive a high five
	 * @param des_trajectory: 
	 */
	set_goal_high_five(trajectory_msgs::JointTrajectory &des_trajectory);

private:
	//private member functions go here!

};

#endif