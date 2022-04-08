#ifndef LAND_CONTROL
#define LAND_CONTROL
#include "ros/ros.h"
#include "mavros_msgs/CommandTOL.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"

#define LAND_HEIGHT 3 // Height at which to send the land command (in meters)
#define ERROR_THRESHOLD 0.1 // where error is how far from the center of the image the line/marker is
#define DESCENT_RATE 0.02 // m/s

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
    geometry_msgs::Pose current_pose;

public:
    LandingController(ros::NodeHandle& nh, double xy_gain);
    void altitude_cb(const sensor_msgs::Range::ConstPtr& msg);
    void update(double x_error, double y_error);
    bool landed();
    void pos_callback(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif