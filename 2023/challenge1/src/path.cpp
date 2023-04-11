#include <first_flight/gnc_functions.hpp>
//include API 

const double ALTITUDE = 7.32;
const double ROW_DIST = 3.66;
const double COL_DIST = 38.4;
const double MIN_X = -45.72;
const double FINISH_LINE_X = -46.6;

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

/*

<--- +x

    |
    |
    v +y

    +z is out of the page

    --------------------
    |     |     |     |
    |     |    1 0    |
    |     |     |     |
    |     |     |     |
    |     |     |     |
    |     |     |     |
    |     |     |     |
    |     |     |     |
    |     |     |     |
    |     |     |     |
    |     |    1 0    |
    |     |     |     |
    --------------------

*/


  //specify some waypoints
  //drone looking south, starts at 0,0,0,0
  double xPos = 0;
  double yPos = 0;
  double zPos = ALTITUDE;//8 yards -> meters Search 8*8 area
  std::vector<gnc_api_waypoint> waypointList;
  gnc_api_waypoint nextWayPoint;
  nextWayPoint.x = xPos;
  nextWayPoint.y = yPos;
  nextWayPoint.z = zPos;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);
  xPos = -ROW_DIST;//align with first ugv row 
  yPos = ROW_DIST;
  nextWayPoint.x = xPos;
  nextWayPoint.y = yPos;
  waypointList.push_back(nextWayPoint);
  for(int i = 1; (xPos-2*ROW_DIST) > MIN_X; i++){
    if (i%2 == 0){
      waypointList.push_back(nextWayPoint);
      nextWayPoint.x = xPos;
      nextWayPoint.y = (yPos-=COL_DIST);
      waypointList.push_back(nextWayPoint);
      nextWayPoint.x = (xPos-=ROW_DIST);
      nextWayPoint.y = yPos;
      waypointList.push_back(nextWayPoint);
    }
    else {
      nextWayPoint.x = xPos;
      nextWayPoint.y = (yPos+=COL_DIST);
      waypointList.push_back(nextWayPoint);
      nextWayPoint.x = (xPos-=ROW_DIST);
      nextWayPoint.y = yPos;
      waypointList.push_back(nextWayPoint);
    }
  }
  
  nextWayPoint.x = FINISH_LINE_X;//cross finish line
  nextWayPoint.y = yPos;
  waypointList.push_back(nextWayPoint);//since drone at 9 feet sees about 9*9 it is better if we move to x= -4.5. Moving drone to 4.5 puts it in the middle of first ugv row 
  
  //specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
  ros::Rate rate(20);
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
