#include "LandingController.h"

LandingController::LandingController(ros::NodeHandle& nh, double xy_gain){

    rcvdFirstAltitudeMsg = false;
    done = false;
    altitude_sub = nh.subscribe<sensor_msgs::Range>("/sonar", 1,
                    &LandingController::altitude_cb, this);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel",1);
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    land_client.waitForExistence(ros::Duration(0.5));
    this->xy_gain = xy_gain;
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
    geometry_msgs::Twist vel_cmd;
    
    vel_cmd.linear.x = xy_gain*x_error;
    vel_cmd.linear.y = xy_gain*y_error;
    vel_cmd.linear.z = DESCENT_RATE;
    velocity_pub.publish(vel_cmd);
}

bool LandingController::landed(){
    return done;
}