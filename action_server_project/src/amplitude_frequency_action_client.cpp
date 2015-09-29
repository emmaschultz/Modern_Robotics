#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/client/simple_action_client.h>
#include <action_server_project/amplitude_frequency_msgAction.h>

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state, const action_server_project::amplitude_frequency_msgResultConstPtr& result) {
    //TODO what goes here? if anything?
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "amplitude_frequency_action_client_node"); //name of this node
        
        action_server_project::amplitude_frequency_msgGoal goal;
        
        actionlib::SimpleActionClient<action_server_project::amplitude_frequency_msgAction> action_client("amplitude_frequency_action", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        //something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out
        }
        
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        while(true) {
            std_msgs::Float64 amplitude;
            std_msgs::Float64 frequency;
            std_msgs::Int32 numCycles;
            std::cout << "Please enter in a desired amplitude: "
            std::cin >> amplitude.data;

            std::cout << "Please enter in a desired frequency: "
            std::cin >> frequency.data;

            std::cout << "Please enter in a desired number of cycles: "
            std::cin >> numCycles.data;

            //set the amplitude/frequency/number of cycles to be equal to user input and then send the goal
            goal.requested_amplitude = amplitude.data;
            goal.requested_frequency = frequency.data;
            goal.requested_num_cycles = numCycles.data;
            action_client.sendGoal(goal,&doneCb);
            
            bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
            //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
            if (!finished_before_timeout) {
                ROS_WARN("giving up waiting on result for goal number %d",g_count);
                return 0;
            } else {
              //if here, then server returned a result to us
            }
        
        }

    return 0;
}