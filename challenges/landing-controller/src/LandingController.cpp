#include "LandingController.h"

LandingController::LandingController(ros::NodeHandle& nh, double xy_gain, double K_d){

    rcvdFirstAltitudeMsg = false;
    done = false;
    altitude_sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub", 1,
                    &LandingController::altitude_cb, this);
    target_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    land_client.waitForExistence(ros::Duration(0.5));
    this->xy_gain = xy_gain;
    this->K_d = K_d;
    pose_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/local_position/odom", 1, &LandingController::pos_callback, this);
    prev_x = INT_MAX;
    prev_y = INT_MAX;
    start_x = INT_MAX;
    start_y = INT_MAX;
}

void LandingController::altitude_cb(const sensor_msgs::Range::ConstPtr& msg) {
    altitude = msg->range;
    if(!rcvdFirstAltitudeMsg) {
        ROS_INFO("Landing controller initialized");
        rcvdFirstAltitudeMsg = true;
    }
}

void LandingController::update(double x_error, double y_error) {
    if(!rcvdFirstAltitudeMsg)
        return;
    // Initialize start x and y if not first iteration
    if(start_x == INT_MAX){
        start_x = current_pose.position.x;
        start_y = current_pose.position.y;	
    }
    // Find location of line/marker
    double error_norm = sqrt(pow(x_error, 2) + pow(y_error, 2));
    ROS_INFO_STREAM(x_error << " " << y_error << " " << error_norm);
    if(altitude < LAND_HEIGHT && error_norm < ERROR_THRESHOLD){
        ROS_INFO("Calling land client");
        land_client.call(landCmd);
        done = true;
        return;
    }
    // Send velocity command to mavros, where z velocity is DESCENT_RATE, and x/y velocity is a function of the offset of the line/marker from the center of the image
    geometry_msgs::PoseStamped cmd;
    cmd.header.stamp = ros::Time::now();
    // Initialize prev errors if first iteration
    if(prev_x == INT_MAX){
        prev_x = x_error;
        prev_y = y_error;
    }
    
    cmd.pose.position.x = current_pose.position.x + xy_gain*x_error + K_d*(x_error-prev_x);
    cmd.pose.position.y = current_pose.position.y - xy_gain*y_error - K_d*(y_error-prev_y);
    if (error_norm < 0.5) {
    	cmd.pose.position.z = std::max((double)LAND_HEIGHT-0.1, current_pose.position.z - DESCENT_RATE);
    } else {
	cmd.pose.position.z = current_pose.position.z;
    }

    cmd.pose.orientation = current_pose.orientation;
    cmd.header.stamp = ros::Time::now();

    if(cmd.pose.position.x < (start_x + SAFETY_RADIUS) && 
       cmd.pose.position.x > (start_x - SAFETY_RADIUS) &&
       cmd.pose.position.y < (start_y + SAFETY_RADIUS) &&
       cmd.pose.position.y > (start_y - SAFETY_RADIUS)){
	    target_pub.publish(cmd);
	    prev_x = x_error;
	    prev_y = y_error;
    }
    else{
	cmd.pose.position.x = current_pose.position.x;
	cmd.pose.position.y = current_pose.position.y;
	cmd.pose.position.z = current_pose.position.z;
	target_pub.publish(cmd);
    }
}

bool LandingController::landed(){
    return done;
}

void LandingController::pos_callback(const nav_msgs::Odometry::ConstPtr& msg){
    current_pose = msg->pose.pose;
}
