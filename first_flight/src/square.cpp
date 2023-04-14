#include <first_flight/gnc_functions.hpp>
//include API 

int main(int argc, char** argv)
{
  //initialize ros 
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node;

  // Load parameters
  double altitude;
  double square_size;
  gnc_node.param<double>("/altitude", altitude, 2.0);
  gnc_node.param<double>("/square_size", square_size, 2.0);
  
  //initialize control publisher/subscribers
  init_publisher_subscriber(gnc_node);

  // wait for FCU connection
  wait4connect();

  //create local reference frame 
  initialize_local_frame();

  //wait for used to switch to mode OFFBOARD
  wait4start();

  //request takeoff
  takeoff(altitude);
  ros::Time takeoff_time = (ros::Time::now());
  ros::Rate rate(20);
  bool landing = false;

  // create waypoint list
  std::vector<gnc_api_waypoint> waypointList;
  gnc_api_waypoint nextWayPoint;
  nextWayPoint.x = 0;
  nextWayPoint.y = 0;
  nextWayPoint.z = altitude;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = square_size;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.y = square_size;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = 0;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.y = 0;
  waypointList.push_back(nextWayPoint);

  int counter = 0;
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
    if(check_waypoint_reached(.3) == 1) {
      if (counter < waypointList.size())
        {
          set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
          counter++;
        }else{
         //land after all waypoints are reached
          land();
      }
    }
  }
  return 0;
}
