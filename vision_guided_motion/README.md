# Vision Guided Motion Project

The file vision_guided_motion.cpp commands the baxter robot to complete a series of tasks both in joint space and cartesian space. When Rviz is running, the user can select a patch of points to be published. Baxter will find the centroid of the selected points and move its arm directly above the centroid. Baxter will then move his arm back and forth, as if wiping the table that is in front of him.

## Example usage
In order to run this code, please use the following commands:

`roslaunch cwru_baxter_sim baxter_world.launch`

`roslaunch vision_guided_motion vision_guided_motion.launch`