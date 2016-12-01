#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include <ros/console.h>
#include <vector>
#include <math.h>
#include "rags_ros/EdgeCosts.h"

using namespace std ;

class SpeedRegulator
{
  public:
    SpeedRegulator(ros::NodeHandle) ;
    ~SpeedRegulator() {}
    
  private:
    ros::Subscriber subSpeeds ;
    ros::Subscriber subCmdVel ;
    ros::Subscriber subEdgeIndex ;
    ros::Publisher pubCmdVel ;
    
    bool fSpeeds ;
    bool fEdge ;
    vector<double> latestSpeeds ;
    int edgeIndex ;
    double speedLimit ;
    
    void speedCallback(const rags_ros::EdgeCosts&) ;
    void cmdVelCallback(const geometry_msgs::Twist&) ;
    void edgeCallback(const std_msgs::Int16&) ;
};

SpeedRegulator::SpeedRegulator(ros::NodeHandle nh){
  subSpeeds = nh.subscribe("/edge_speeds", 10, &SpeedRegulator::speedCallback, this) ;
  subCmdVel = nh.subscribe("recovery_cmd_vel", 10, &SpeedRegulator::cmdVelCallback, this) ;
  subEdgeIndex = nh.subscribe("edge_index", 10, &SpeedRegulator::edgeCallback, this) ;
  pubCmdVel = nh.advertise<geometry_msgs::Twist>("pioneer/cmd_vel", 10) ;
  
  fSpeeds = false ;
  fEdge = false ;
}

void SpeedRegulator::speedCallback(const rags_ros::EdgeCosts& msg){
  if (!fSpeeds)
    fSpeeds = true ;
  latestSpeeds = msg.costs ;
}

void SpeedRegulator::edgeCallback(const std_msgs::Int16& msg){
  if (!fEdge)
    fEdge = true ;
  edgeIndex = msg.data ;
  if (fSpeeds){
    speedLimit = latestSpeeds[edgeIndex] ;
    ROS_INFO_STREAM("Enforced speed limit: " << speedLimit << " m/s") ;
  }
}

void SpeedRegulator::cmdVelCallback(const geometry_msgs::Twist& msg){
  if (fSpeeds && fEdge){
    geometry_msgs::Twist cmd = msg ;
    if (cmd.linear.x > speedLimit)
      cmd.linear.x = speedLimit ;
    
    pubCmdVel.publish(cmd) ;
  }
}
