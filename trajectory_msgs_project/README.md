# Trajectory Messages Project

The robot in this project is described in the .urdf file within this package. This description is loaded into gazebo and rviz in order to simulate the robot. The trajectory action client calculates a goal trajectory (which in this case is in a sinusoidal pattern) and sends it as a goal to the action server. The action server than smoothes out this trajectory goal and publishes the commands. The joint controller subscribes to these commands and adjusts the robot's position accordingly. Once the goal has been achieved, the action server sends a result back to the client telling it that the goal has been completed successfully. 

## Example usage
In order to run this code, first start up a roscore and use catkin_make to compile to code. In a new terminal, type "roslaunch trajectory_msgs_project trajectory_msgs_project.launch". Make sure that gravity in gazebo is set to 0 for best results.
