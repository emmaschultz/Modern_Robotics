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

int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_commander_action_server_node");  //name of this node
    ros::Publisher my_commander_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);  //publish to vel_cmd topic
    //ros::ServiceServer amplitude_frequency_service = n.advertiseService("change_amplitude_and_frequency", callback);

    VelocityCommanderActionServer actionServerObject;   //create an instance of VelocityCommanderActionServer

    std_msgs::Float64 command; //this will contain the value that will be published to vel_cmd
    std_msgs::Float64 period;  //this will house the data containing the length of the period for the sine wave
    double point_in_time = 0.0;
    double totalPeriods = 0.0;
    double counter = 0.0;

    ros::Rate naptime(100.0);   //increased this to have higher sample rate, which results in a smoother sin wave

    command.data = 0.0; //initialze the vel_cmd to 0

    while (ros::ok()) {
        ros::spinOnce();  //update to the most current data

        period.data = (2 * PI)/g_frequency.data;       //determine the length of one period of the sine wave
        totalPeriods = period.data * g_numCycles.data; //determine the length of all of the periods (cycles) based on the number of cycles requested

        //if the number of cycles has not been reached yet, continue calculating the sine wave
        //else command a velocity of zero
        if(totalPeriods > counter){
            counter += 0.01;
            command.data = g_amplitude.data * sin(2 * PI * g_frequency.data * point_in_time);
        } else {
            counter = 0.0;  //reset counter back to zero because last cycle has been reached
            command.data = 0.0; //command a velocity of zero when the requested number of cycles have been completed
        }

        point_in_time += 0.01;
        my_commander_object.publish(command);
        naptime.sleep();
    }

}

