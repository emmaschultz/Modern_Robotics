#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <action_server_project/amplitude_frequency_msgAction.h>

#define PI 3.14159265

std_msgs::Float64 g_amplitude;
std_msgs::Float64 g_frequency;
std_msgs::Int32 g_numCycles;

class VelocityCommanderActionServer {
private:
    ros::NodeHandle nh_;

    actionlib::SimpleActionServer<action_server_project::amplitude_frequency_msgAction> as_;

    action_server_project::amplitude_frequency_msgGoal goal_;
    action_server_project::amplitude_frequency_msgResult result_;
    action_server_project::amplitude_frequency_msgFeedback feedback_;

public:
    VelocityCommanderActionServer();

    ~VelocityCommanderActionServer(void) {
    }
    void executeCB(const actionlib::SimpleActionServer<action_server_project::amplitude_frequency_msgAction>::GoalConstPtr& goal);
};


VelocityCommanderActionServer::VelocityCommanderActionServer() : as_(nh_, "amplitude_frequency_action", boost::bind(&VelocityCommanderActionServer::executeCB, this, _1), false) {
    ROS_INFO("in constructor of VelocityCommanderActionServer...");
    as_.start();   //start the server
}


void VelocityCommanderActionServer::executeCB(const actionlib::SimpleActionServer<action_server_project::amplitude_frequency_msgAction>::GoalConstPtr& goal) {
    g_amplitude.data = goal_.requested_amplitude;
    g_frequency.data = goal_.requested_frequency;
    g_numCycles.data = goal_.requested_num_cycles;

    result_.goal_succeeded = true;

    as_.setSucceeded(result_);
}

/*
bool callback(velocity_project::amplitude_frequency_msgRequest& request, velocity_project::amplitude_frequency_msgResponse& response) {
    ROS_INFO("callback activated");
    g_amplitude.data = request.amplitude;  //set amplitude equal to value specified by client
    g_frequency.data = request.frequency;  //set frequency equal to value specified by client

    //return true to the client to signal that the task has been completed
    response.amplitude_changed = true;
    response.frequency_changed = true;
    
    return true;
}
*/

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_commander_action_server_node");  //name of this node
    ros::Publisher my_commander_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);  //publish to vel_cmd topic
    //ros::ServiceServer amplitude_frequency_service = n.advertiseService("change_amplitude_and_frequency", callback);

    VelocityCommanderActionServer actionServerObject;   //create an instance of VelocityCommanderActionServer

    std_msgs::Float64 command; //this will contain the value that will be published to vel_cmd
    double point_in_time = 0.0;

    ros::Rate naptime(100.0);   //increased this to have higher sample rate, which results in a smoother sin wave

    command.data = 0.0; //initialze the vel_cmd to 0

    while (ros::ok()) {
        ros::spinOnce(); //update data from callback function
        command.data = g_amplitude.data * sin(2 * PI * g_frequency.data * point_in_time);  //calculate the sin wave values
        point_in_time += 0.01;  //increment the counter; this should execute 100 times in one second
        my_commander_object.publish(command);  //publishes the current vel_cmd
        naptime.sleep();
    }
}

