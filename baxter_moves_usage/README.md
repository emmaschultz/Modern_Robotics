# Baxter Moves Usage

The code in this package works in conjuction with the code contained in the baxter_moves_library package. This package contains a file that uses the library in baxter_moves_library to make baxter do a set of pre-defined motions.

## Example usage
In order to run this code, please complete the following commands all in their own terminals:

`roslaunch baxter_moves_usage baxter_world.launch`

^ you need to wait for this one to say "Gravity compensation was turned off" before continuing

`rosrun baxter_traj_streamer traj_interpolator_as`

`rosrun baxter_tools enable_robot.py -e`

`rosrun baxter_moves_usage my_interesting_moves_usage`

    