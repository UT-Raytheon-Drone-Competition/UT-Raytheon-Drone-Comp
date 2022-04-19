#include "lib.hpp"

mavros_msgs::State current_state;

void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");

    ros::NodeHandle nh;

    // Establish subscribers, publishers, and service client
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_callback);
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/odometry/in", 10, pos_callback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
	    ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
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

    ROS_INFO("Creating Waypoints");

    tf::Quaternion q;
    q.setRPY(0, 0, 1.570796);

    std::vector<double> x;
    std::vector<double> y;

    double y_len = 5;
    double x_len = 2;
    int numPoints = 10;

    // Interpolate waypoints
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < numPoints; j++) {
            x.push_back(i * x_len + j * x_len/numPoints);
            (i % 2 == 0) ? y.push_back(j * y_len/numPoints) : y.push_back(y_len - j * y_len/numPoints);
        }
    }

    std::vector<geometry_msgs::PoseStamped> goals = generate_waypoints(x, y, 6, q);

    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish((goals[0]));
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

    bool missionDone = false;
    TagTracker tracker(&nh);
    bool landingStarted = false;
    while(ros::ok() && !missionDone) {

	// Switch to offboard mode
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                ROS_INFO("Offboard enabled");
            last_request = ros::Time::now();
	// Arm drone
        } else {
            if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if(arming_client.call(arm_cmd) && arm_cmd.response.success)
                    ROS_INFO("Vehicle armed");
                last_request = ros::Time::now();
            }
        }
        if(tracker.checkTimeToLand()){
            tracker.startLanding(rate);
            missionDone = true;
        }
        else{
            local_pos_pub.publish((goals[cur_goal_idx]));
            if(check_waypoint_reached((goals[cur_goal_idx]))) {
                ROS_INFO_STREAM("Waypoint" << cur_goal_idx << " reached");
                cur_goal_idx++;	
                if(cur_goal_idx >= goals.size()) {
                    ROS_INFO("Landing");
                    land_client.call(land_cmd);
                    missionDone = true;
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

