#include <ros/ros.h>
#include "speed_regulator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "speed_regulator") ;

  ros::NodeHandle nHandle ;
  
  SpeedRegulator speed_regulator(nHandle) ;
  
  ros::spin();
  return 0;
}
