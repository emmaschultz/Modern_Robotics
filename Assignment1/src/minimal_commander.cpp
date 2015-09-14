#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

#define PI 3.14159265

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_commander1"); //name this node
    ros::NodeHandle n;
    ros::Publisher my_commander_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);  //publish to vel_cmd topic
    
    std_msgs::Float64 command; //this will contain the value that will be published to vel_cmd
   
    //variables to determine vel_cmd
    double amplitude = 1.0;
    double frequency = 1.0;
    double point_in_time = 0.0;

    ros::Rate naptime(1.0);   //set the sleep timer for 1Hz repetition rate (arg is in units of Hz)

    command.data = 0.0; //initialize

    while (ros::ok()) {
        command.data = amplitude * sin(2 * PI * frequency * point_in_time);  //calculate the value in the sin wave
        point_in_time += 0.1;
        my_commander_object.publish(command);
        naptime.sleep();
    }
}