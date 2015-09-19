# Velocity Project

Basic example of a controller, commander, and simulator. The commander creates a sin wave with a given amplitude and frequency. The value of the sin graph at a point in time is used as the velocity command for the controller. The controller changes the velocity depending on the velocity command that is received from the commander.



This package manipulates the velocity of an object by controlling the force placed on the object. A service node (amplitude_frequency_server) tells the client node (velocity_commander) what the amplitude and frequency of a sine wave should be. This information is gathered through user input. The sine wave is then calculated with the given amplitude and frequency. The sine wave represents what the velocity should be. This is the velocity command (vel_cmd). This is handed off to the velocity_controller which calculates the force needed in order to increase/decrease the current velocity to match the vel_cmd. The velocity_simulator then changes the current velocity based on the calculated force.
