#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <velocity_project/amplitude_frequency_msg.h>

#define PI 3.14159265

std_msgs::Float64 g_amplitude = 1.0;
std_msgs::Float64 g_frequency = 1.0;

bool callback(velocity_project::amplitude_frequency_msgRequest& request, velocity_project::amplitude_frequency_msgResponse& response) {
    ROS_INFO("callback activated");
    g_amplitude.data = request.amplitude;
    g_frequency.data = request.frequency;

    response.amplitude_changed = true;
    response.frequency_changed = true;
    
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_commander1");
    ros::init(argc, argv, "amplitude_frequency_service1"); //TODO: is this necessary?
    ros::NodeHandle n;
    ros::Publisher my_commander_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);  //publish to vel_cmd topic
    ros::ServiceServer amplitude_frequency_service = n.advertiseService("change_amplitude_and_frequency", callback);
    
    //ros::spin()   //TODO: this was in the example service. does this need to be anywhere in this node? or maybe this needs to be a ros::spinOnce()?

    std_msgs::Float64 command; //this will contain the value that will be published to vel_cmd
   
    //variables to determine vel_cmd
    //double amplitude = 1.0;
    //double frequency = 1.0;
    double point_in_time = 0.0;

    ros::Rate naptime(100.0);   //increased this to have higher sample rate, which results in a smoother sin wave

    command.data = 0.0; //initialze the vel_cmd to 0

    while (ros::ok()) {
        ros::spinOnce(); //update data from callback function
        command.data = g_amplitude * sin(2 * PI * g_frequency * point_in_time);  //calculate the sin wave values
        point_in_time += 0.01;  //increment the counter; this should execute 100 times in one second
        my_commander_object.publish(command);  //publishes the current vel_cmd
        naptime.sleep();
    }
}