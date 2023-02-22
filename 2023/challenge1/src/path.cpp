#include <gnc_functions.hpp>
//include API 

int main(int argc, char** argv)
{
  //initialize ros 
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node;
  
  //initialize control publisher/subscribers
  init_publisher_subscriber(gnc_node);

  // wait for FCU connection
  wait4connect();

  //wait for used to switch to mode OFFBOARD
  wait4start();

  //create local reference frame 
  // initialize_local_frame();

  //request takeoff
  takeoff(2);

  //specify some waypoints
  //drone looking south, starts at 0,0,0,0
  // double xPos = 0;
  // double yPos = 0;
  // double zPos = 7.32;//8 yards -> meters Search 8*8 area
  // std::vector<gnc_api_waypoint> waypointList;
  // gnc_api_waypoint nextWayPoint;
  // nextWayPoint.x = xPos;
  // nextWayPoint.y = yPos;
  // nextWayPoint.z = zPos;
  // nextWayPoint.psi = 0;
  // waypointList.push_back(nextWayPoint);
  // nextWayPoint.x = -3.66;//align with first ugv row 
  // nextWayPoint.y = 3.66;
  // nextWayPoint.z = zPos;
  // nextWayPoint.psi = 0;
  // waypointList.push_back(nextWayPoint);
  // for(int i = 1; (xPos-3.66) < 45.72; i++){
  //   if (i%2 == 0){
  //     waypointList.push_back(nextWayPoint);
  //     nextWayPoint.x = xPos;
  //     nextWayPoint.y = yPos-38.4;
  //     nextWayPoint.z = zPos;
  //     nextWayPoint.psi = 0;
  //     waypointList.push_back(nextWayPoint);
  //     nextWayPoint.x = xPos-3.66;
  //     nextWayPoint.y = yPos;
  //     nextWayPoint.z = zPos;
  //     nextWayPoint.psi = 0;
  //     waypointList.push_back(nextWayPoint);
  //   }
  //       else 
  //   {
  //     nextWayPoint.x = xPos;
  //     nextWayPoint.y = yPos+38.4;
  //     nextWayPoint.z = zPos;
  //     nextWayPoint.psi = 0;
  //     waypointList.push_back(nextWayPoint);
  //     nextWayPoint.x = xPos-3.66;
  //     nextWayPoint.y = yPos;
  //     nextWayPoint.z = zPos;
  //     nextWayPoint.psi = 0;
  //     waypointList.push_back(nextWayPoint);
  //   }
  // }
  
  // nextWayPoint.x = -46.6;//cross finish line
  // nextWayPoint.y = yPos;
  // nextWayPoint.z = 7.32;
  // nextWayPoint.psi = 0;
  // waypointList.push_back(nextWayPoint);//since drone at 9 feet sees about 9*9 it is better if we move to x= -4.5. Moving drone to 4.5 puts it in the middle of first ugv row 
  
  //specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
  ros::Rate rate(20);
  int counter = 0;
  while(ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
      // if(check_waypoint_reached(.3) == 1) {
      //   if (counter < waypointList.size())
      //     {
      //       set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
      //       counter++;
      //     }else{
      //     //land after all waypoints are reached
      //     land();
      //   }
      // }
      set_destination(0,0,2,0);
      
    }
  return 0;
}
