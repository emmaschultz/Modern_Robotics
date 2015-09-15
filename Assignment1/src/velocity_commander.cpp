#include <ros/ros.h>
#include <std_msgs/Float64.h>

#define PI 3.14159265

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_commander1");
    ros::NodeHandle n;
    ros::Publisher my_commander_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);  //publish to vel_cmd topic
    
    std_msgs::Float64 command; //this will contain the value that will be published to vel_cmd
   
    //variables to determine vel_cmd
    double amplitude = 1.0;
    double frequency = 1.0;
    double point_in_time = 0.0;

    ros::Rate naptime(100.0);   //increased this to have higher sample rate, which results in a smoother sin wave

    command.data = 0.0; //initialze the vel_cmd to 0

    while (ros::ok()) {
        command.data = amplitude * sin(2 * PI * frequency * point_in_time);  //calculate the sin wave values
        point_in_time += 0.01;  //increment the counter; this should execute 100 times in one second
        my_commander_object.publish(command);  //publishes the current vel_cmd
        naptime.sleep();
    }
}