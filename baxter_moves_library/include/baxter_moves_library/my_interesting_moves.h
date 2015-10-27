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
	 */
	set_goal_wave();

	/**
	 * This will give the baxter robot the command to do the robot dance move with its right arm
	 */
	set_goal_extend_arm();

	/**
	 * This will give the baxter robot the command to put its right arm out to receive a high five
	 */
	set_goal_high_five();

private:
	/**
	 * This will calculate the trajectories and send the goal to the action server
	 * @param position: the desired position for the robot to go to
	 */
	find_trajectory(Vectorq7x1 position);
};

#endif