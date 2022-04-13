#ifndef LAND_CONTROL
#define LAND_CONTROL
#include "ros/ros.h"
#include "mavros_msgs/CommandTOL.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"

#define LAND_HEIGHT 1 // Height at which to send the land command (in meters)
#define ERROR_THRESHOLD 0.05 // where error is how far from the center of the image the line/marker is
#define DESCENT_RATE 0.5

class LandingController{
private:
    float altitude;
    bool rcvdFirstAltitudeMsg;
    mavros_msgs::CommandTOL landCmd;
    ros::Subscriber altitude_sub;
    ros::Subscriber pose_sub;
    ros::Publisher target_pub;
    ros::ServiceClient land_client;
    bool done;
    double xy_gain;
    double K_d;
    geometry_msgs::Pose current_pose;
    double prev_x;
    double prev_y;

public:
    LandingController(ros::NodeHandle& nh, double xy_gain, double K_d);
    void altitude_cb(const sensor_msgs::Range::ConstPtr& msg);
    void update(double x_error, double y_error);
    bool landed();
    void pos_callback(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif