# Baxter Moves Usage

Your description goes here

## Example usage
In order to run this code, please complete the following commands all in their own terminals:

roslaunch cwru_baxter_sim baxter_world.launch   <-- you need to wait for this one to say "Gravity compensation was turned off" before continuing

rosrun baxter_traj_streamer traj_interpolator_as

rosrun baxter_tools enable_robot.py -e

rosrun baxter_moves_usage my interesting_moves_usage

## Running tests/demos
    