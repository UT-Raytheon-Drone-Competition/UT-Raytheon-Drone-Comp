#include <first_flight/gnc_functions.hpp>
//include API 

const double ALTITUDE = 2;

int main(int argc, char** argv)
{
  //initialize ros 
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node;
  
  //initialize control publisher/subscribers
  init_publisher_subscriber(gnc_node);

  // wait for FCU connection
  wait4connect();

  //create local reference frame 
  initialize_local_frame();

  //wait for used to switch to mode OFFBOARD
  wait4start();

  //request takeoff
  takeoff(ALTITUDE);
  ros::Time takeoff_time = (ros::Time::now());
  ros::Rate rate(20);
  bool landing = false;

  while(ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
      if(!landing){
        if (ros::Time::now() - takeoff_time > ros::Duration(10.0)){
          land();
          landing = true;
        } else {
          set_destination(0,0, ALTITUDE, 0);
        }
      }
    }
  return 0;
}
