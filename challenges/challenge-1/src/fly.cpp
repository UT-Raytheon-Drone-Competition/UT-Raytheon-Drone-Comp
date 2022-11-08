#include "lib.hpp"
#include <std_msgs/Float64.h>

mavros_msgs::State current_state;
bool landing = false;
double y_error = 0;

void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void line_coord_callback(const std_msgs::Float64::ConstPtr& msg){
    if(landing){
        y_error = msg->data;
    }
}

void altitude_callback(const sensor_msgs::Range::ConstPtr& msg){
    current_pose_g.pose.pose.position.z = msg->range; 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");

    ros::NodeHandle nh;

    // Establish subscribers, publishers, and service client
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, state_callback);
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom", 1, pos_callback);
    ros::Subscriber line_coord_sub = nh.subscribe<std_msgs::Float64>
            ("/line_coordinates", 1, line_coord_callback);
    ros::Subscriber altitude_sub = nh.subscribe<sensor_msgs::Range>
	    ("/mavros/altitude/local", 1, altitude_callback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
	    ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Land command message
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    // Waypoints
    tf::Quaternion q;
    q.setRPY(0, 0, 1.570796);

    geometry_msgs::PoseStamped pose1;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 5;
    // pose1.header.frame_id = "local_origin";
    pose1.header.stamp = ros::Time::now();
    quaternionTFToMsg(q, pose1.pose.orientation);

    ROS_INFO_STREAM("takeoff altitude: " << pose1.pose.position.z);
    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose1);
        ros::spinOnce();
        rate.sleep();
    }

    // Set drone to Offboard mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // Arm drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int cur_goal_idx = 0;
    double ypos = 0;

    bool missionDone = false;
    bool startDescent = false;
    LandingController lander(nh, 1, 0); // TODO: tune xy gain
    double last_waypoint_time = 0.0;

    while(ros::ok() && !missionDone) {
	if(current_state.mode == "OFFBOARD"){
		ROS_INFO_ONCE("STARTING MISSION");
		if(!landing){
			if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
			    if(arming_client.call(arm_cmd) && arm_cmd.response.success)
				    ROS_INFO("Vehicle armed");
			    last_request = ros::Time::now();
		}

		    geometry_msgs::PoseStamped pose;
			pose.pose.position.x = 0;
			pose.pose.position.y = ypos;
			pose.pose.position.z = 5;
			quaternionTFToMsg(q, pose.pose.orientation);
		    // pose.header.frame_id = "local_origin";
		    pose.header.stamp = ros::Time::now();

		    local_pos_pub.publish(pose);
		    if(check_waypoint_reached(pose)) {
			ROS_INFO_STREAM("Waypoint" << ypos << " reached");
			ypos+=4.572;	
			if(ypos > 28) {
			    ROS_INFO("Landing");
			    landing = true;
			    last_waypoint_time = ros::Time::now().toSec();

			}
		    }
		}
		else{
		    if(ros::Time::now().toSec()-last_waypoint_time > 5){
			lander.update(0,y_error);
			if(lander.landed()){
			    missionDone = true;
			}
		    }
		}
	}
	else{
		local_pos_pub.publish(pose1);
	}

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

