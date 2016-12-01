#include <ros/ros.h>
#include "rags_planner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rags_planner") ;

  ros::NodeHandle nHandle ;
  
  RagsRos RAGSPlanner(nHandle) ;
  
  ros::spin();
  return 0;
}
