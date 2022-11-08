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
#include <apriltag_ros/AprilTagDetectionArray.h>
#include "LandingController.h"

nav_msgs::Odometry current_pose_g;



bool check_waypoint_reached(geometry_msgs::PoseStamped goal, float pos_tolerance=1.5, float heading_tolerance=0.01) {

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

std::vector<geometry_msgs::PoseStamped> generate_waypoints(std::vector<double>& x, std::vector<double>& y, double z, tf::Quaternion q){
    std::vector<geometry_msgs::PoseStamped> goals;
    for(int i = 0; i < x.size(); i++){
        geometry_msgs::PoseStamped goal;
        goal.pose.position.x = x[i];
        goal.pose.position.y = y[i];
        goal.pose.position.z = z;
        quaternionTFToMsg(q, goal.pose.orientation);
        goals.push_back(goal);
    }
    return goals;
}

class TagTracker{

private:
    ros::NodeHandle* n;
    int consecutive_frames;
    bool timeToLand;
    ros::Subscriber downward_april_sub;
    double x_error;
    double y_error;
    bool freshData;
public:
    TagTracker(ros::NodeHandle* nh):
        n(nh), consecutive_frames(0), timeToLand(false)
    {
        downward_april_sub = nh->subscribe<apriltag_ros::AprilTagDetectionArray>
            ("/tag_detections", 1, &TagTracker::downward_april_callback, this);
        x_error = 0;
	y_error = 0;
	freshData = false;
    }
    void downward_april_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg){
        if(timeToLand){
            for(auto& detection : msg->detections){
                x_error = -1*detection.pose.pose.pose.position.x; // TODO: Check coordinate axes
                y_error = -1*detection.pose.pose.pose.position.y; // TODO: scale these errors accordingly
		freshData = true;
                return;
            }
        }
        if(msg->detections.size() > 0){
            consecutive_frames++;
            if(consecutive_frames > 4){
                timeToLand = true;
            }
        }else{
            consecutive_frames = 0;
        }
    }
    bool checkTimeToLand(){
        return timeToLand;
    }
    void startLanding(ros::Rate& rate){
        ROS_INFO("Found tag, starting landing");
        LandingController lander(*n, 0.6, 0.4);// TODO: tune xy_gains
        while(ros::ok() && !lander.landed()) {
            //if(freshData){
		ROS_INFO_STREAM("x: " << x_error << ", y: " << y_error);    
                lander.update(x_error, y_error);
                freshData = false;
            //}
            ros::spinOnce();
            rate.sleep();
        }
    }
};

#endif //GNC_FUNCTIONS

