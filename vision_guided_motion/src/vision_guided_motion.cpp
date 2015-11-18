#include <ros/ros.h>


//start at pre pose?
//needs to subscribe to rviz topic of selected points
//when receives info from rviz topic:
	//calculate centroid of selected points
	//add some number to the z-axis number
	//compute trajectory to have hand of baxter move to this point
	//have baxter move hand back and forth around this point