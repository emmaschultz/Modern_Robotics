#include <ros/ros.h>
#include <velocity_project/amplitude_frequency_msg.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <string>  //TODO: do I need this still on here?
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "amplitude_frequency_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<velocity_project::amplitude_frequency_msg>("change_amplitude_and_frequency");
    velocity_project::amplitude_frequency_msg srv;

    //initialize the amplitude and frequency to zero
    std_msgs::Float64 amplitude;
    amplitude.data = 0.0;
    std_msgs::Float64 frequency;
    frequency.data = 0.0;

    while (ros::ok()) {
        cout << "\n";
        cout << "Enter an amplitude: ";
        cin >> amplitude.data;

        srv.request.amplitude = amplitude.data; //sets the amplitude in the request to the amplitude input by the user

        cout << "\n\n";
        cout << "Enter a frequency: ";
        cin >> frequency.data;

        srv.request.frequency = frequency.data; //sets the frequency in the request to the frequency input by the user

        if (client.call(srv)) {
            //checks whether the amplitude and frequency have been changed or not
            if (srv.response.amplitude_changed && srv.response.frequency_changed){
                ROS_INFO("Changed value of amplitude to %f", srv.request.amplitude);
                ROS_INFO("Changed value of frequency to %f", srv.request.frequency);
            } else if(srv.response.frequency_changed) {
                ROS_INFO("Changed value of frequency to %f", srv.request.frequency);
                ROS_ERROR("Did not change value of amplitude.");
            } else if(srv.response.amplitude_changed) {
                ROS_INFO("Changed value of amplitude to %f", srv.request.amplitude);
                ROS_ERROR("Did nto change value of frequency.");
            } else {
                ROS_ERROR("Did not change value of amplitude or frequency.");
                return 1;
            }
        } else {
            ROS_ERROR("Failed to call service change_amplitude_and_frequency");
            return 1;
        }
    }
    return 0;
}