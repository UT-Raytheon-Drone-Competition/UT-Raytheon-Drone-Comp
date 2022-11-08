#ifndef GNC_FUNCTIONS
#define GNC_FUNCTIONS

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h> 
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h> 
#include <nav_msgs/Odometry.h>
#include <vector>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include "LandingController.h"


nav_msgs::Odometry current_pose_g;

bool check_waypoint_reached(geometry_msgs::PoseStamped goal, float pos_tolerance=1, float heading_tolerance=0.01) {

    // Check for correct position 
    float deltaX = abs(goal.pose.position.x - current_pose_g.pose.pose.position.x);
    float deltaY = abs(goal.pose.position.y - current_pose_g.pose.pose.position.y);
    float deltaZ = abs(goal.pose.position.z - current_pose_g.pose.pose.position.z);
    float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );

    // ROS_INFO("dMag %f", dMag);
    // ROS_INFO("current pose x %F y %f z %f", (current_pose_g.pose.pose.position.x), (current_pose_g.pose.pose.position.y), (current_pose_g.pose.pose.position.z));
    // ROS_INFO("waypoint pose x %F y %f z %f", waypoint_g.pose.position.x, waypoint_g.pose.position.y,waypoint_g.pose.position.z);
    //check orientation
    // float cosErr = cos(current_heading_g*(M_PI/180)) - cos(local_desired_heading_g*(M_PI/180));
    // float sinErr = sin(current_heading_g*(M_PI/180)) - sin(local_desired_heading_g*(M_PI/180));
    
    // float headingErr = sqrt( pow(cosErr, 2) + pow(sinErr, 2) );

    // ROS_INFO("current heading %f", current_heading_g);
    // ROS_INFO("local_desired_heading_g %f", local_desired_heading_g);
    // ROS_INFO("current heading error %f", headingErr);

    if( dMag < pos_tolerance /*&& headingErr < heading_tolerance*/)
	return 1;
    else
	return 0;
}

void pos_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose_g = *msg;
}

#endif //GNC_FUNCTIONS

