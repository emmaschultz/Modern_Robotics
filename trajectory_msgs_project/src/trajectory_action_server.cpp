// trajectory_action_server: complementary server for trajectory messages
// accepts trajectory goals, interpolates between points linearly at specified dt
// The velocity commands are ignored by this simple interpolator

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs_project/TrajMsgAction.h>
#include <std_msgs/Float64.h>
#include <vector>

using namespace std;

const double dt= 0.01; // decide what time resolution to use for interpolating the trajectory
const double min_dt = 0.0000001; // require that time steps have a minimum spacing
    
int g_count = 0;
bool g_count_failure = false;

class TrajectoryActionServer {
private:

    ros::NodeHandle nh_;
    ros::Publisher jnt_cmd_publisher_; //this publisher talks to the joint_controller
    actionlib::SimpleActionServer<trajectory_msgs_project::TrajMsgAction> as_;
    
    // here are some message types to communicate with our client(s)
    trajectory_msgs_project::TrajMsgGoal goal_; // goal message, received from client
    trajectory_msgs_project::TrajMsgResult result_; // put results here, to be sent back to the client when done w/ goal
    trajectory_msgs_project::TrajMsgFeedback feedback_; // not used in this example;
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    
    void send_joint_commands_(vector <double> q_cmd_jnts); // helper function to encapsulate details of how to talk to the controller;

public:
    TrajectoryActionServer(ros::NodeHandle* nodehandle); //define the body of the constructor outside of class definition

    ~TrajectoryActionServer(void) {
    }

    void executeCB(const actionlib::SimpleActionServer<trajectory_msgs_project::TrajMsgAction>::GoalConstPtr& goal);
};

//clumsy sytax for initializing objects within the constructor
TrajectoryActionServer::TrajectoryActionServer(ros::NodeHandle* nodehandle):nh_(*nodehandle), as_(nh_, "traj_action_server", boost::bind(&TrajectoryActionServer::executeCB, this, _1),false) {
    ROS_INFO("in constructor of TrajectoryActionServer...");
    ROS_INFO("Initializing Publisher");
    //next line specialized for use with joint_controller:
    jnt_cmd_publisher_ = nh_.advertise<std_msgs::Float64>("pos_cmd", 1, true);
    as_.start(); //start the server running
}

//helper function to encapsulate details of how to talk to the controller; need to specialize this for your controller
void  TrajectoryActionServer::send_joint_commands_(vector <double> q_cmd_jnts) { 
    std_msgs::Float64 q_cmd_msg;

    //publish all position commands
    for(int i = 0; i < q_cmd_jnts.size(); i++){
    	q_cmd_msg.data = q_cmd_jnts[i];
    	jnt_cmd_publisher_.publish(q_cmd_msg);
    	ROS_INFO("commanding: %f",q_cmd_jnts[i]);
    }
}
   
   
//here is where we do the work to act on goal requests
void TrajectoryActionServer::executeCB(const actionlib::SimpleActionServer<trajectory_msgs_project::TrajMsgAction>::GoalConstPtr& goal) {
    trajectory_msgs::JointTrajectory trajectory = goal->trajectory; // goal-> notation is annoying, so copy the important stuff to a shorter named var

    ros::Rate rate_timer(1/dt); //here is a timer, consistent with our chosen time step

    int npts = trajectory.points.size(); // how many poses are contained in the goal request?
    ROS_INFO("received trajectory with %d points",npts); //debug/interpretation output
 
    int njoints = trajectory.joint_names.size(); // determine how many joints we have...though we really already know, for this example robot
    vector <double> q_prev_jnts; // for extension to multiple joints, create a variable-length array to hold joint commands
    vector <double> q_next_jnts; // we will interpolate between coarse joint commands, from q_prev_jnts to q_next_jnts
    vector <double> q_cmd_jnts;  // will contain interpolated joint command values
    q_prev_jnts.resize(njoints); // these arrays need to be this large to be consistent with dimension of input
    q_next_jnts.resize(njoints);
    q_cmd_jnts.resize(njoints);
    
    ROS_INFO("trajectory message commands %d joint(s)",njoints);
    double t_final = trajectory.points[npts - 1].time_from_start.toSec(); // what is the last value of time_from_start specified?  convert to seconds
    ROS_INFO("t_final = %f",t_final);
    double t_previous = 0.0; 
    double t_next = trajectory.points[1].time_from_start.toSec();
    double t_stream=0.0; //start the clock from time 0
    double fraction_of_range=0.0; // this will be a ratio describing fraction of current segment completed
    double q_prev, q_next, q_cmd; 
    //start streaming points...interpolate, as needed.
    int ipt_next = 1;
    double t_range = t_next-t_previous;
    if (t_range< min_dt) {
        ROS_WARN("time step invalid in trajectory!");
        as_.setAborted(result_);
    }
    //put the starting joint commands here.  Really, these NEED to be the robot's actual joint commands!
    // this should be tested (and abort goal if not within some tolerance)
    q_prev_jnts = trajectory.points[ipt_next - 1].positions;
    q_next_jnts = trajectory.points[ipt_next].positions;
    while (t_stream < t_final) {
        //compute desired pose at current time
        //see if t has stepped across current range, t_previous to t_next
        while (t_stream > t_next) { //find the next time range in the sequence
            ipt_next++;
            t_previous = t_next; //shift this down--former "next" is new "previous" in t_previous< t < t_next
            t_next = trajectory.points[ipt_next].time_from_start.toSec();
            t_range = t_next-t_previous;            
            if (t_range < min_dt) {
                ROS_WARN("time step invalid in trajectory!");
                as_.setAborted(result_);
                break;
            }   
            q_prev_jnts = trajectory.points[ipt_next - 1].positions;  //find bounds for joint commands at previous and next time boundaries
            q_next_jnts = trajectory.points[ipt_next].positions;
        }        

        //if here, have valid range t_previous < t_stream < t_next      
        fraction_of_range = (t_stream - t_previous)/(t_next - t_previous);

        //interpolate on the joint commands...linearly;  COULD BE BETTER, e.g. with splines
        for (int ijnt = 0; ijnt < njoints; ijnt++) {
           q_cmd_jnts[ijnt] = q_prev_jnts[ijnt] + fraction_of_range * (q_next_jnts[ijnt] - q_prev_jnts[ijnt]); //here is a vector of new joint commands;
        }
        ROS_INFO("t_prev, t_stream, t_next, fraction = %f, %f, %f, %f",t_previous,t_stream,t_next,fraction_of_range);
        ROS_INFO("q_prev, q_cmd, q_next: %f, %f, %f",q_prev_jnts[0],q_cmd_jnts[0],q_next_jnts[0]);
        send_joint_commands_(q_cmd_jnts);
        
        rate_timer.sleep();
        t_stream+=dt;
    }
    
    q_cmd_jnts = trajectory.points[npts-1].positions;
    send_joint_commands_(q_cmd_jnts);
    as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
}

//the main program instantiates a TrajectoryActionServer, then lets the callbacks do all the work
int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_action_node");

    ros::NodeHandle nh;

    ROS_INFO("main: instantiating an object of type TrajectoryActionServer");
    TrajectoryActionServer as_object(&nh);
    
    
    ROS_INFO("going into spin");

    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    return 0;
}

