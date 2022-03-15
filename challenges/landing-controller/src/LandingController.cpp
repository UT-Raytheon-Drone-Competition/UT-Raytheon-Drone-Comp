#include "LandingController.h"

LandingController::LandingController(ros::NodeHandle& nh, bool line){
    landOnLine = line;
    rcvdFirstAltitudeMsg = false;
    altitude_sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/TODO", 1, 
                    &LandingController::altitude_cb, this);
    image_sub = nh.subscribe<sensor_msgs::Image>("/rrbot/camera1/image_raw",1,
                    &LandingController::image_cb, this);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped",1);
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    land_client.waitForExistence(ros::Duration(0.5));
}

void LandingController::altitude_cb(const sensor_msgs::Range::ConstPtr& msg){
    altitude = msg->range;
    if(!rcvdFirstAltitudeMsg){
        rcvdFirstAltitudeMsg = true;
    }
}

void LandingController::image_cb(const sensor_msgs::Image::ConstPtr& msg){
    if(!rcvdFirstAltitudeMsg){
        return;
    }
    // Find location of line/marker
    // TODO: find error from image
    x_error = 0;
    y_error = 0;
    error_norm = sqrt(x_error, y_error);
    if(altitude < LAND_HEIGHT && error_norm < ERROR_THRESHOLD){
        land_client.call(landCmd);
        return;
    }
    // Send velocity command to mavros, where z velocity is DESCENT_RATE, and x/y velocity is a function of the offset of the line/marker from the center of the image
    geometry_msgs::Twist vel_cmd;
    
    vel_cmd.linear.x = XY_GAIN*x_error;
    vel_cmd.linear.y = XY_GAIN*y_error;
    vel_cmd.linear.z = DESCENT_RATE;
    velocity_pub.publish(vel_cmd);
}
