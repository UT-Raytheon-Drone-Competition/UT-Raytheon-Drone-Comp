#inlcude "gns_functions.hpp"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node;

  init_publisher_subscriber(gnc_node);

  //wait to connect with FCU
  wait4connect();
  //hold program until unyil piloy executes the program by switching the fcu flight mode to guided. Can be done in QGC
  wait4start();
  
  initialize_local_frame();
  takeoff(3);

  //create waypoints

}
