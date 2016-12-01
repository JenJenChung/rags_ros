#include "ros/ros.h"
#include <fstream> // std::ifstream, std::ofstream
#include <sstream> // std::stringstream
#include <random> // std::default_random_engine, std::normal_distribution
#include <vector> // std::vector
#include <chrono> // std::chrono_system_clock::now
#include "rags_ros/EdgeCosts.h"

using std::ofstream ;
using std::stringstream ;
using std::default_random_engine ;
using std::normal_distribution ;
using std::vector ;

class TrueCosts {
  public:
    TrueCosts(ros::NodeHandle) ;
    ~TrueCosts(){
    	trueCostsFile.close() ;
  	}
    
    void edgeCostCallback(const ros::TimerEvent&) ;
  private:
    ros::Publisher pubTrueCosts ;
    ros::Publisher pubSpeedLimits ;
    
    int numEdges ;
    vector< vector<double> > cost_distributions ;
    
    stringstream tcFileName ;
    ofstream trueCostsFile ;
} ;

TrueCosts::TrueCosts(ros::NodeHandle nh){
  
  pubTrueCosts = nh.advertise<rags_ros::EdgeCosts>("/edge_costs", 10, true) ;
  pubSpeedLimits = nh.advertise<rags_ros::EdgeCosts>("/edge_speeds", 10, true) ;
  
  ros::param::get("/num_edges", numEdges);
  char buffer[50] ;
  // Read in edges
  for (int i = 0; i < numEdges; i++){
    vector<double> e ;
    sprintf(buffer,"/edge%d",i) ;
    ros::param::get(buffer,e) ;
		
		vector<double> cd ;
		cd.push_back(e[2]) ;
		cd.push_back(e[3]) ;
		cd.push_back(e[4]) ;
		cost_distributions.push_back(cd) ;
  }
  
  int trialNum ;
  ros::param::get("/trial_number",trialNum) ;
  tcFileName << "/home/jchu2041/catkin_ws/src/rags_ros/test_config/true_costs_" << trialNum << ".csv" ;
  trueCostsFile.open(tcFileName.str().c_str()) ;
}

void TrueCosts::edgeCostCallback(const ros::TimerEvent&){
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  default_random_engine generator(seed);
  
  // Query for true edge costs and corresponding traversal speeds and write to csv file
  vector<double> true_costs ;
  vector<double> speed_limits ;
  for (int i = 0; i < numEdges; i++){
    normal_distribution<double> distribution(cost_distributions[i][0], cost_distributions[i][1]) ;
    double c ;
    double s ;
    while (true){
      c = distribution(generator) ;
      s = cost_distributions[i][2]/c ;
      if (s > 0.01 && s < 1.0) // do not allow speeds less than 0.01m/s or faster than 1.0m/s
        break ;
    }
    true_costs.push_back(c) ;
    speed_limits.push_back(s) ;
    trueCostsFile << true_costs[i] << "," ;
  }
  trueCostsFile << "\n" ;
  
  rags_ros::EdgeCosts trueEdgeCosts ;
  trueEdgeCosts.costs = true_costs ;
  rags_ros::EdgeCosts speedLimits ;
  speedLimits.costs = speed_limits ;
  
  pubTrueCosts.publish(trueEdgeCosts) ;
  pubSpeedLimits.publish(speedLimits) ;
}
