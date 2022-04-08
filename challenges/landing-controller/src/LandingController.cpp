#include "LandingController.h"

LandingController::LandingController(ros::NodeHandle& nh, double xy_gain){

    rcvdFirstAltitudeMsg = false;
    done = false;
    altitude_sub = nh.subscribe<sensor_msgs::Range>("/sonar", 1,
                    &LandingController::altitude_cb, this);
    target_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    land_client.waitForExistence(ros::Duration(0.5));
    this->xy_gain = xy_gain;
    ros::Subscriber pose_sub = nh.subscribe<nav_msgs::Odometry>
            ("mavros/odometry/in", 1, LandingController::pos_callback, this);
}

void LandingController::altitude_cb(const sensor_msgs::Range::ConstPtr& msg) {
    altitude = msg->range;
    if(!rcvdFirstAltitudeMsg)
        rcvdFirstAltitudeMsg = true;
}

void LandingController::update(double x_error, double y_error) {
    if(!rcvdFirstAltitudeMsg)
        return;
    // Find location of line/marker
    double error_norm = sqrt(pow(x_error,2)+pow(y_error,2));
    if(altitude < LAND_HEIGHT && error_norm < ERROR_THRESHOLD){
        ROS_INFO("Calling land client");
        land_client.call(landCmd);
        done = true;
        return;
    }
    // Send velocity command to mavros, where z velocity is DESCENT_RATE, and x/y velocity is a function of the offset of the line/marker from the center of the image
    geometry_msgs::PoseStamped cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.pose.position.x = current_pose.position.x + xy_gain*x_error;
    cmd.pose.position.y = current_pose.position.y + xy_gain*y_error;
    cmd.pose.position.z = current_pose.position.z - DESCENT_RATE;
    target_pub.publish(cmd);
}

bool LandingController::landed(){
    return done;
}

void LandingController::pos_callback(const nav_msgs::Odometry:ConstPtr& msg){
    current_pose = msg->pose.pose;
}