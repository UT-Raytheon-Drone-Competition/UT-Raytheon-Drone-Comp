#include "lib.hpp"

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/global_position/local", 10, pos_callback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
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
    pose1.pose.position.z = 6;
    pose1.orientation = q;
    geometry_msgs::PoseStamped pose2;
    pose2.pose.position.x = 0;
    pose2.pose.position.y = 30;
    pose2.pose.position.z = 6;
    pose2.orientation = q;

    std::vector<geometry_msgs::PoseStamped*> goals;
    goals.push_back(&pose1);
    goals.push_back(&pose2);

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

    //bool missionDone = false;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(*(goals[cur_goal_idx]));
	if(check_waypoint_reached(*(goals[cur_goal_idx]))){
	    ROS_INFO_STREAM("Waypoint" << cur_goal_idx << " reached");
	    cur_goal_idx++;	
	    if(cur_goal_idx >= goals.size()) {
		    ROS_INFO("Landing");
		    //land();
		    while (!(land_client.call(land_cmd) && land_cmd.response.success)) {
		      //local_pos_pub.publish(pose);
		      ROS_INFO("tring to land");
		      ros::spinOnce();
		      rate.sleep();
    		    }
		    //missionDone = true;
	    }
	}

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
