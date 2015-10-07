#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>

#define PI 3.14159265

//a simple saturation function; provide saturation threshold, sat_val, and arg to be saturated, val
double sat(double val, double sat_val) {
    if (val>sat_val)
        return (sat_val);
    if (val< -sat_val)
        return (-sat_val);
    return val;
    
}

double g_pos_cmd=0.0; //position command input-- global var
void posCmdCB(const std_msgs::Float64& pos_cmd_msg) 
{ 
  ROS_INFO("received value of pos_cmd is: %f",pos_cmd_msg.data); 
  g_pos_cmd = pos_cmd_msg.data;
} 



int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_controller");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);
    
    // make sure service is available before attempting to proceed, else node will crash
    bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
      ROS_INFO("waiting for apply_joint_effort service");
      half_sec.sleep();
    }
    ROS_INFO("apply_joint_effort service exists");

    ros::ServiceClient set_trq_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    
    service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/get_joint_properties",true);
      ROS_INFO("waiting for /gazebo/get_joint_properties service");
      half_sec.sleep();
    }
    ROS_INFO("/gazebo/get_joint_properties service exists");
    
    ros::ServiceClient get_jnt_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

    //for joint1
    gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg;
    gazebo_msgs::GetJointProperties get_joint_state_srv_msg;

    //for joint2
    gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg2;
    gazebo_msgs::GetJointProperties get_joint_state_srv_msg2;
    
    ros::Publisher trq_publisher = nh.advertise<std_msgs::Float64>("jnt_trq", 1);   //MAYBE NEED TO HAVE TWO COPIES OF THESE?
    ros::Publisher vel_publisher = nh.advertise<std_msgs::Float64>("jnt_vel", 1);
    ros::Publisher pos_publisher = nh.advertise<std_msgs::Float64>("jnt_pos", 1);
    ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    ros::Subscriber pos_cmd_subscriber = nh.subscribe("pos_cmd",1,posCmdCB);
    
    std_msgs::Float64 trq_msg;
    std_msgs::Float64 q1_msg,q1dot_msg;
    std_msgs::Float64 q2_msg, q2dot_msg;      //added in new q2 messages for joint2
    sensor_msgs::JointState joint_state_msg;  //for joint1
    sensor_msgs::JointState joint_state_msg2; //for joint2

    double q1, q1dot, q2, q2dot;  //added in a new q value to represent the second joint
    double dt = 0.01;
    ros::Duration duration(dt);
    ros::Rate rate_timer(1/dt);
    
    effort_cmd_srv_msg.request.joint_name = "joint1";
    effort_cmd_srv_msg.request.effort = 0.0;
    effort_cmd_srv_msg.request.duration= duration;

    get_joint_state_srv_msg.request.joint_name = "joint1";
    //double q1_des = 1.0;
    double q1_err, q2_err;    //added in error field for second joint
    double Kp = 10.0;
    double Kv = 3;
    double trq_cmd;

    // set up the joint_state_msg fields to define a single joint,
    // called joint1, and initial position and vel values of 0
	joint_state_msg.header.stamp = ros::Time::now();
	joint_state_msg.name.push_back("joint1");
    joint_state_msg.position.push_back(0.0);
    joint_state_msg.velocity.push_back(0.0);
    


    //new code for joint2
    effort_cmd_srv_msg2.request.joint_name = "joint2";
    effort_cmd_srv_msg2.request.effort = 0.0;
    effort_cmd_srv_msg2.request.duration = duration;

    get_joint_state_srv_msg2.request.joint_name = "joint2";

    joint_state_msg2.header.stamp = ros::Time::now();
    joint_state_msg2.name.push_back("joint2");
    joint_state_msg2.position.push_back(0.0);
    joint_state_msg2.velocity.push_back(0.0);




    int point_in_time = 0; //keeps track of the "time" for the sine function (keeps the function moving along the x-axis)
    while(ros::ok()) {
        get_jnt_state_client.call(get_joint_state_srv_msg);
        q1 = get_joint_state_srv_msg.response.position[0];  //this is the line that causes the segfault
        q1_msg.data = q1;
        pos_publisher.publish(q1_msg);

        q1dot = get_joint_state_srv_msg.response.rate[0];
        q1dot_msg.data = q1dot;
        vel_publisher.publish(q1dot_msg);

		joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.position[0] = q1; 
        joint_state_msg.velocity[0] = q1dot;

		joint_state_publisher.publish(joint_state_msg);
        
		g_pos_cmd = sin(2 * PI * 2.0 * point_in_time);     //frequency for joint1 is 2.0

        q1_err = g_pos_cmd - q1;
        if (q1_err > M_PI) {
            q1_err -= 2 * M_PI;
        }
        if (q1_err < -M_PI) {
            q1_err += 2 * M_PI;
        }        
            
        trq_cmd = Kp * (q1_err) - Kv * q1dot;
        trq_msg.data = trq_cmd;
        trq_publisher.publish(trq_msg);

        // send torque command to Gazebo
        effort_cmd_srv_msg.request.effort = trq_cmd;
        set_trq_client.call(effort_cmd_srv_msg);

        //make sure service call was successful
        bool result = effort_cmd_srv_msg.response.success;
        if (!result)
            ROS_WARN("service call to apply_joint_effort failed!");



        //BEGIN CODE FOR JOINT2
        get_jnt_state_client.call(get_joint_state_srv_msg2);
        q2 = get_joint_state_srv_msg2.response.position[0];
        q2_msg.data = q2;
        pos_publisher.publish(q2_msg);

        q2dot = get_joint_state_srv_msg2.response.rate[0];
        q2dot_msg.data = q2dot;
        vel_publisher.publish(q2dot_msg);

        joint_state_msg2.header.stamp = ros::Time::now();
        joint_state_msg2.position[0] = q2;
        joint_state_msg2.velocity[0] = q2dot;

        joint_state_publisher.publish(joint_state_msg2);

        g_pos_cmd = sin(2 * PI * 3.0 * point_in_time);  //frequency for joint2 is 3.0

        q2_err = g_pos_cmd - q2;
        if (q2_err > M_PI) {
        	q2_err -= 2 * M_PI;
        }
        if (q2_err < -M_PI) {
        	q2_err += 2 * M_PI;
        }

        trq_cmd = Kp * (q2_err) - Kv * q2dot;
        trq_msg.data = trq_cmd;
        trq_publisher.publish(trq_msg);

        //send to gazebo
        effort_cmd_srv_msg2.request.effort = trq_cmd;
        set_trq_client.call(effort_cmd_srv_msg2);

        //ensure success
        result = effort_cmd_srv_msg2.response.success;
        if (!result)
        	ROS_WARN("service call to apply_joint_effort failed!");




        point_in_time++; //increment "timer"

        ros::spinOnce();
		rate_timer.sleep();
    }
}
