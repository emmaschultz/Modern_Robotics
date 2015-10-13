// trajectory_action_client: 
// see complementary server, "trajectory_action_server"
// this simple node populates a trajectory message and sends it to the trajectory action server for execution
// Run this together with minimal robot; start-up minimal robot with:   roslaunch minimal_robot_description minimal_robot.launch 

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <trajectory_msgs_project/TrajMsgAction.h>


// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state, const trajectory_msgs_project::TrajMsgResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_client_node"); // name this node 
    trajectory_msgs_project::TrajMsgGoal goal; //instantiate a goal message compatible with our server, as defined in this package
    // we will command a limited-duration sinusoidal motion; define amplitude, frequency and duration
	double omega = 1.0; //rad/sec
    double amp = 0.5; //0.5; //radians
	double start_angle= amp;
	double final_phase = 4 * 3.1415927; // radians--two periods
        
    //dt: break up trajectory into incremental commands this far apart in time
	double dt = 0.1; 
        
    actionlib::SimpleActionClient<trajectory_msgs_project::TrajMsgAction> action_client("traj_action_server", true);
        
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds

    ros::Duration sleep1s(1);
    if (!server_exists) {
        ROS_WARN("could not connect to server; retrying");
        bool server_exists = action_client.waitForServer(ros::Duration(1.0));
        sleep1s.sleep();
    }
        
    ROS_INFO("connected to action server");  // if here, then we connected to the server;


    // instantiate and populate a goal message:
	trajectory_msgs::JointTrajectory trajectory; //this contains an array (a vector) of trajectory points
	trajectory_msgs::JointTrajectoryPoint trajectory_point; //here is a single trajectory point, which will be populated and included in the trajectory


	trajectory.joint_names.push_back("joint1");
    trajectory.joint_names.push_back("joint2");   //added this in for new joint


	// joint position commands below must be specified in the same order as specified in the joint_names array
	int njnts = trajectory.joint_names.size(); // we specified this many joints; need same size for position and velocity vectors
	trajectory_point.positions.resize(njnts);
	trajectory_point.velocities.resize(njnts);

	ROS_INFO("populating trajectory...");
	double final_time; //seconds
    double phase = 0.0; //radians        
	double time_from_start = 0.0; // seconds
	double q_des, qdot_des; //radians, radians/sec
    double q2_des, q2dot_des; //for joint2
        
    //"phase" is a convenient variable = omega*time
	for (phase = 0.0; phase < final_phase; phase += omega * dt) {
		q_des = start_angle + amp * sin(phase); //here we make up a desired trajectory shape: q_des(t)
		qdot_des = amp * omega * cos(phase); // this is the time derivative of q_des;
		trajectory_point.positions[0] = q_des;
		trajectory_point.velocities[0] = qdot_des; //velocities will get ignored in this case

        //for joint2
        q2_des = start_angle + amp * sin(phase);
        q2dot_des = amp * omega * cos(phase);
        trajectory_point.positions[1] = q2_des;
        trajectory_point.velocities[1] = q2dot_des;

		time_from_start += dt; //cumulative time from start of move


 		ROS_INFO("phase = %f, t = %f",phase,time_from_start);               
		//specify arrival time for this point--in ROS "duration" format
		trajectory_point.time_from_start = ros::Duration(time_from_start); //this converts from seconds to ros::Duration data type
		//append this trajectory point to the vector of points in trajectory:
		trajectory.points.push_back(trajectory_point);	
        //dt = 0.01;
	}

	final_time = time_from_start; // the last assigned time; we should expect "success" back from our server after this long, else something went wrong
	int npts = trajectory.points.size();  //we just created this many points in our trajectory message
	ROS_INFO("populated trajectory with %d points",npts);
	//copy this trajectory into our action goal:	
	goal.trajectory = trajectory;

	//and send out the goal
    action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired

    // wait for expected duration--plus some tolerance (chosen arbitrarily to be 2 seconds)
    bool finished_before_timeout = action_client.waitForResult(ros::Duration(final_time + 2.0));
    //bool finished_before_timeout = action_client.waitForResult(); // alternative: wait forever...
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result");
        return 0;
    }
    else {
        ROS_INFO("main: goal was reported as successfully executed. Bye.");
    }
        

    return 0;
}

