# Action Server Project

This package manipulates the velocity of an object by controlling the force placed on the object. A action client node (amplitude_frequency_action_client.cpp) get user input to determine an amplitude, a frequency, and a number of cycles (periods). The client then requests that the amplitude, frequency, and number of cycles in the action service node (velocity_commander_action_server.cpp) are changed to match. The action server uses the amplitude and frequency to calculate a sine wave and uses the number of cycles to determine how many periods to run for. The sine wave is then calculated with the given amplitude and frequency. The sine wave represents what the velocity should be. This is the velocity command (vel_cmd). This is handed off to the controller which calculates the force needed in order to increase/decrease the current velocity to match the vel_cmd. The simulator then changes the current velocity based on the calculated force.

## Example usage
In command line:
roscd
roscore

In a second command line prompt:
roslaunch action_server_project action_server_project.launch

In a third command line prompt:
rqt_plot

In a fourth command line prompt:
rosrun action_server_project amplitude_frequency_action_client
    