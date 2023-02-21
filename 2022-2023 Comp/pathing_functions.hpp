//File for functions relating to the pathing of the drone.
#include <gnc_functions.hpp>
#pragma once

void nextRow (std::vector<gnc_api_waypoint> waypoints,int x0,int y0,int x1){//x1 will be the curent ugv positions



}
void nextRow(std::vector<gnc_api_waypoint>waypoints, int x0, int y0){//search next row
  gnc_api_waypoint nextWayPoint;

  if (y0 + 7.32 > 45.72){
    //either reverse search or 
  }
  nextWayPoint.x = x0;
  nextWayPoint.y = y0 + 7.32;
  nextWayPoint.z = 7.32;
  nextWayPoint.psi = 0;
  waypointList.push_back(nextWayPoint);
  if(x0 > 10){
    nextWayPoint.x = x0 - 38.4; 
    nextWayPoint.y = y0; 
    nextWayPoint.z = 7.32;
    nextWayPoint.psi = 0;
    waypointList.push_back(nextWayPoint); 
  }
  else{
    nextWayPoint.x = x0 + 38.4; 
    nextWayPoint.y = y0; 
    nextWayPoint.z = 7.32;
    nextWayPoint.psi = 0;
    waypointList.push_back(nextWayPoint); 
  }
}
void reverseSearch (std::vector<gnc_api_waypoint> wayPointList)
{
  gnc_api_waypoint nextWayPoint;
  double xPos = 45.72 - 3.66;
  double yPos = 45.72 - 3.66;
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

void ForwardSearch (std::vector<gnc_api_waypoint> wayPointList)//creates list of waypoints to search from 50*50 grid from left to right
{
  //drone looking south, starts at 0,0,0,0
  double xPos = x0;
  double yPos = y0;
  dobule zPos = z0;//8 yards -> meters Search 8*8 area
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
  if (int i = 1; (xPos-3.66) < 45.72; i++)
    {
      if (i%2 == 0)
  {
    waypointList.push_back(nextWayPoint);
    nextWayPoint.x = xPos;
    nextWayPoint.y = yPos-38.4;
    nextWayPoint.z = zPos;
    nextWayPoint.psi = 0;
    waypointList.push_back(nextWayPoint);
    nextWayPoint.x = xPos-3.66;
    nextWayPoint.y = yPos;
    nextWayPoint.z = zPos;
    nextWayPoint.psi = 0;
    waypointList.push_back(nextWayPoint);
  }
      else
  {
    nextWayPoint.x = xPos;
    nextWayPoint.y = yPos+38.4;
    nextWayPoint.z = zPos;
    nextWayPoint.psi = 0;
    waypointList.push_back(nextWayPoint);
    nextWayPoint.x = xPos-3.66;
    nextWayPoint.y = yPos;
    nextWayPoint.z = zPos;
    nextWayPoint.psi = 0;
    waypointList.push_back(nextWayPoint);
   }
    }
}




