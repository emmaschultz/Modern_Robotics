# Point Cloud Processing

This package will accept user input of selected points (through rviz). It will take these selected points and find all coplanar points using the cwru_pcl_utils library. It will then publish all coplanar points and display them in rviz.

## Example usage
In order to run this code, you must run the following commands:

`roslaunch cwru_baxter_sim baxter_world.launch`

`roslaunch cwru_baxter_sim kinect_xform.launch`

`rosrun point_cloud_processing find_coplanar_points`

`rosrun rviz rviz` (and set display to see kinect/depth/points and /pcl_cloud_display)
    