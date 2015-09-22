# Velocity Project

This package manipulates the velocity of an object by controlling the force placed on the object. A client node (amplitude_frequency_client) get user input to determine an amplitude and a frequency. The client then requests that the amplitude and frequency in the service node (velocity_commander) are changed to match. The velocity_commander uses the amplitude and frequency to calculate a sine wave. The sine wave is then calculated with the given amplitude and frequency. The sine wave represents what the velocity should be. This is the velocity command (vel_cmd). This is handed off to the velocity_controller which calculates the force needed in order to increase/decrease the current velocity to match the vel_cmd. The velocity_simulator then changes the current velocity based on the calculated force.

## Example Usage
In command line:
	roscd
	roscore

In a second command line prompt:
	roscd
	roslaunch velocity_project velocity_project.launch

In a third command line prompt:
	roscd
	rosrun velocity_project amplitude_frequency_client
Then enter in your desired amplitude and frequency.

In a fourth command line prompt:
	roscd
	rqt_plot