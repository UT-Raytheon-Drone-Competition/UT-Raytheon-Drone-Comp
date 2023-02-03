#include <gnc_functions.hpp>
//include API 

void nextRowWayPoint (double xPos, double yPos)//make a waypoint to the next row
{

}
void reverseSearch (std::vector<gnc_api_waypoint> wayPointList)
{
  gnc_api_waypoint nextWayPoint;
  double xPos = 45.72 - 3.66;
  dobule yPos = 45.72 - 3.66;
  double zPos = 8;
  nextWayPoint.x = xPos;
  nextWayPoint.y = yPos;
  nextWayPoint.z = zPos;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);

  if (int i = 1; (yPos - 7.32) > 3.66; i++)//|-     
    {
      if (i%2 == 0)
        {
          waypointList.push_back(nextWayPoint);
	  xPos = 45.72-3.66;
          nextWayPoint.x = xPos;
          nextWayPoint.y = yPos;
          nextWayPoint.z = zPos;
          nextWayPoint.psi = 0;
          waypointList.push_back(nextWayPoint);
          nextWayPoint.x = xPos;
          nextWayPoint.y = yPos-7.32;
          nextWayPoint.z = zPos;
          nextWayPoint.psi = 0;
          waypointList.push_back(nextWayPoint);
        }
      else//-|
        {
	  xPos = -3.66;
          nextWayPoint.x = xPos;
          nextWayPoint.y = yPos;
          nextWayPoint.z = zPos;
          nextWayPoint.psi = 0;
          waypointList.push_back(nextWayPoint);
	  yPos = yPos -7.32;
          nextWayPoint.x = xPos;
          nextWayPoint.y = yPos;
          nextWayPoint.z = zPos;
          nextWayPoint.psi = 0;
          waypointList.push_back(nextWayPoint);
        }
    }  
}
int main(int argc, char** argv)
{
  //initialize ros 
  ros::init(argc, argv, "gnc_node");
  ros::NodeHandle gnc_node;
  
  //initialize control publisher/subscribers
  init_publisher_subscriber(gnc_node);

  // wait for FCU connection
  wait4connect();

  //wait for used to switch to mode GUIDED
  wait4start();

  //create local reference frame 
  initialize_local_frame();

  //request takeoff
  takeoff(10);

  //specify some waypoints
  //drone looking south, starts at 0,0,0,0
  double xPos = 0;
  double yPos = 0;
  dobule zPos = 7.32;//8 yards -> meters Search 8*8 area
  std::vector<gnc_api_waypoint> waypointList;
  gnc_api_waypoint nextWayPoint;
  nextWayPoint.x = xPos;
  nextWayPoint.y = yPos;
  nextWayPoint.z = zPos;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = -3.66;//align with first ugv row 
  nextWayPoint.y = 3.66;
  nextWayPoint.z = zPos;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);
  
  nextWayPoint.x = -46.6;//cross finish line
  nextWayPoint.y = yPos;
  nextWayPoint.z = 7.32;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);//since drone at 9 feet sees about 9*9 it is better if we move to x= -4.5. Moving drone to 4.5 puts it in the middle of first ugv row 

  ////////////////////////

  nextWayPoint.x = xPos;
  nextWayPoint.y = yPos;
  nextWayPoint.z = zPos;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);
  nextWayPoint.x = -3.66;//align with first ugv row
  nextWayPoint.y = 3.66;
  nextWayPoint.z = zPos;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);


  waypointList.push_back(nextWayPoint);//go to the end of first row. Aka search first row for UGV
  nextWayPoint.x = xPos;
  nextWayPoint.y = yPos-38.4;
  nextWayPoint.z = zPos;
  nextWayPoint.psi = 0;
  //specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
  ros::Rate rate(2.0);
  int counter = 0;
  double UGVx = 0;
  while(ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
      UGVx = UGVx-0.0381  //UGVs are moving .031 meters per .5 seconds 

	/*if (ugv detected)
	    if (friendly)
	       move to next row
	    else if (enemy)
	       attack()
	       bool target_immobalized
	       if (target_immobalized)
	           move on to next row
               else if (!target_immobalized)
	           attack()
	  else if(reached end of row and did not find ugv)
	      move to next row ((UGVx),y+8yards)

	  
	  //if (last row reached and searched  && all enemy ugvs were not elimiated)
	  //use backwards search algorithm
	  //cross finish line
	       
	           
	*/        
	if(check_waypoint_reached(.3) == 1)//if checkpoint reached or UGV in the row was hit then we go to the waypoint (y-8yards and ugvx)
	{
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
}//Path for second challenge
