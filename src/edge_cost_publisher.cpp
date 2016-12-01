#include <ros/ros.h>
#include "edge_cost_publisher.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "edge_cost_publisher") ;

  ros::NodeHandle nHandle ;
  
  TrueCosts true_costs(nHandle) ;
  
  double t ;
  ros::param::get("/edge_cost_rate",t) ;
  ros::Timer edgeCostTimer = nHandle.createTimer(ros::Duration(t), &TrueCosts::edgeCostCallback, &true_costs) ;
  
  ros::spin();
  return 0;
}
