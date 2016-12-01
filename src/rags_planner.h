#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Time.h"
#include "std_msgs/Int16.h"
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/console.h>
#include <vector>
#include <string>
#include <chrono> // std::chrono_system_clock::now
#include <random> // std::default_random_engine, std::uniform_real_distribution
#include <fstream> // std::ifstream, std::ofstream
#include <sstream> // std::stringstream
#include <math.h>
#include "rags_ros/EdgeCosts.h"
#include "rags_ros/PathCost.h"
#include "rags_ros/RAGS.h"
#include "rags_ros/XY.h"

using namespace std ;
using namespace easymath ;

typedef unsigned int UINT ;

class RagsRos
{
  public:
    typedef pair<int, int> edge ;
    RagsRos(ros::NodeHandle) ;
    ~RagsRos() {
      if (fPlanner)
        delete RAGSPlanner ;
    }
    
  private:
    ros::Subscriber subResult ;
    ros::Subscriber subPose ;
    ros::Subscriber subCost ;
    ros::Subscriber subGoal ;
    ros::Subscriber subGoalIndex ;
    ros::Publisher pubWaypoint ;
    ros::Publisher pubEdgeIndex ;
    ros::Publisher pubPathCost ;
    ros::Publisher pubGoal ;
    bool fPose ;
    bool fCosts ;
    bool fGoal ;
    bool fPlanner ;
    move_base_msgs::MoveBaseActionResult result ;
    geometry_msgs::Pose pose ;
    geometry_msgs::Twist waypoint ;
    
    int numVertices ;
    int numEdges ;
    XY curLoc ;
    XY nextLoc ;
    XY goal ;
    vector<XY> vertices ;
    vector<edge> edges ;
    vector< vector<double> > cost_distributions ;
    vector<double> true_costs ;
    double goalTolerance ;
    
    string robotName ; // logged values
    string planner ; // logged values
    vector<double> Start ; // logged values
    vector<double> Goal ; // logged values
    vector<double> Path ; // logged values
    ros::Time goalReceiveTime ;
    
    RAGS * RAGSPlanner ;
    
    void actionCallback(const move_base_msgs::MoveBaseActionResult&) ;
    void poseCallback(const nav_msgs::Odometry&) ;
    void costCallback(const rags_ros::EdgeCosts&) ;
    void goalCallback(const geometry_msgs::Twist&) ;
    void goalIndexCallback(const std_msgs::Int16&) ;
    void ExecuteTransition() ;
    int GetEdgeIndex(XY, XY) ;
};

RagsRos::RagsRos(ros::NodeHandle nh){
  subResult = nh.subscribe("move_base/result", 10, &RagsRos::actionCallback, this) ;
  subPose = nh.subscribe("odom", 10, &RagsRos::poseCallback, this) ;
  subCost = nh.subscribe("/edge_costs", 10, &RagsRos::costCallback, this) ;
  subGoal = nh.subscribe("global_goal", 10, &RagsRos::goalCallback, this) ;
  subGoalIndex = nh.subscribe("global_goal_index", 10, &RagsRos::goalIndexCallback, this) ;
  pubWaypoint = nh.advertise<geometry_msgs::Twist>("map_goal", 10) ;
  pubEdgeIndex = nh.advertise<std_msgs::Int16>("edge_index", 10, true) ;
  pubPathCost = nh.advertise<rags_ros::PathCost>("path_cost", 10, true) ;
  pubGoal = nh.advertise<geometry_msgs::Twist>("global_goal", 10, true) ;
  
  fPose = false ;
  fCosts = false ;
  fGoal = false ;
  fPlanner = false ;
  
  // Read in parameters
  ros::param::get("/num_vertices", numVertices);
  ros::param::get("/num_edges", numEdges);
  ros::param::get("planner", planner) ;
  ros::param::get("robot_name", robotName) ;
  char buffer[50] ;
  ros::param::get("x", curLoc.x) ;
  ros::param::get("y", curLoc.y) ;
  ROS_INFO_STREAM("Robot current position: (" << curLoc.x << "," << curLoc.y << ")") ;
  ros::param::get("move_base/TrajectoryPlannerROS/xy_goal_tolerance", goalTolerance) ;
  
  // Read in vertices
  for (int i = 0; i < numVertices; i++){
    vector<double> v ;
    sprintf(buffer,"/vertex%d",i) ;
    ros::param::get(buffer,v) ;
		vertices.push_back(XY(v[0],v[1])) ;
  }
  
  // Read in edges
  for (int i = 0; i < numEdges; i++){
    vector<double> e ;
    sprintf(buffer,"/edge%d",i) ;
    ros::param::get(buffer,e) ;
    
    edge ee((int)e[0],(int)e[1]) ;
		edges.push_back(ee) ;
		
		vector<double> cd ;
		cd.push_back(e[2]) ;
		cd.push_back(e[3]) ;
		cost_distributions.push_back(cd) ;
  }
  
//  // Randomly generate edge cost distributions
//	ROS_INFO("Generating edge cost distribution parameter values...") ;
//  // Write to csv file
//  stringstream cdFileName ;
//  cdFileName << "/home/jchu2041/catkin_ws/src/rags_ros/test_config/cost_distributions_" << ros::WallTime::now().toSec() << ".csv" ;
//  ofstream costsFile ;
//  costsFile.open(cdFileName.str().c_str()) ;
//  
//	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
//  default_random_engine generator(seed);
//  uniform_real_distribution<double> mean_distribution(2.0,4.0); // mean traversal speeds at 0.5-0.25 m/s 
//  uniform_real_distribution<double> std_distribution(0.1,0.7);
//  
//  for (unsigned i = 0; i < edges.size(); i++){
//    vector<double> cost ;
//    double diffx = vertices[edges[i].first].x - vertices[edges[i].second].x ;
//    double diffy = vertices[edges[i].first].y - vertices[edges[i].second].y ;
//    double dist = sqrt(pow(diffx,2)+pow(diffy,2)) ;
//    bool repeat = true ;
//    while (repeat){
//      cost.clear() ;
//      cost.push_back(mean_distribution(generator)) ;
//      cost.push_back(std_distribution(generator)) ;
//      if (cost[1] < cost[0]){ // do not allow std > mean
//        repeat = false ;
//        cost[0] *= dist ; // cost_distributions represent time of traversal
//        cost[1] *= dist ; // multiplication to distance is for conversion to units of time
//      }
//    }
//    cost_distributions.push_back(cost) ;
//    costsFile << cost[0] << "," << cost[1] << "\n" ;
//  }
//	costsFile.close() ;
//	ROS_INFO("Edge cost distributions defined.") ;
//  ROS_INFO_STREAM("First edge cost drawn from: N(" << cost_distributions[0][0] << ","
//    << cost_distributions[0][1] << ")") ;
}

void RagsRos::actionCallback(const move_base_msgs::MoveBaseActionResult& msg){
  if (msg.status.status == 3){
    // check if nextLoc is nearest waypoint
    double diffx = pose.position.x - nextLoc.x ;
    double diffy = pose.position.y - nextLoc.y ;
    double dist = sqrt(pow(diffx,2)+pow(diffy,2)) ;
    if (dist < 2.0*goalTolerance){ // successfully reached next location
      curLoc = nextLoc ;
      ROS_INFO_STREAM("Transition complete, robot now at (" << curLoc.x << "," << curLoc.y <<
        "), continuing through to (" << goal.x << "," << goal.y << ")") ;
      Path.push_back(curLoc.x) ;
      Path.push_back(curLoc.y) ;
      double diffx = curLoc.x - goal.x ;
      double diffy = curLoc.y - goal.y ;
      double dist = sqrt(pow(diffx,2)+pow(diffy,2)) ;
      if (dist < 0.01){
        ROS_INFO("Global goal reached!") ;
        ros::Duration plan_time = ros::Time::now() - goalReceiveTime ;
        ROS_INFO_STREAM("Total execution time: " << plan_time.toSec() << "s") ;
        rags_ros::PathCost p ;
        p.robot_name = robotName ;
        p.planner = planner ;
        p.start = Start ;
        p.goal = Goal ;
        p.path = Path ;
        p.execution_time = plan_time ;
        pubPathCost.publish(p) ;
        fGoal = false ;
      }
      else{
        ROS_INFO("Interim goal reached!") ;
        ExecuteTransition() ;
      }
    }
  }
  else if (msg.status.status == 6 || msg.status.status == 2 || msg.status.status == 4 || msg.status.status == 5) {
    ROS_INFO("Plan failed. Resetting RAGS object and waiting for new global goal...") ;
    fGoal = false ;
  }
}

void RagsRos::poseCallback(const nav_msgs::Odometry& msg){
  fPose = true ;
  pose.position.x = msg.pose.pose.position.x ;
  pose.position.y = msg.pose.pose.position.y ;
  pose.position.z = msg.pose.pose.position.z ;
  pose.orientation.x = msg.pose.pose.orientation.x ;
  pose.orientation.y = msg.pose.pose.orientation.y ;
  pose.orientation.z = msg.pose.pose.orientation.z ;
  pose.orientation.w = msg.pose.pose.orientation.w ;
}

void RagsRos::costCallback(const rags_ros::EdgeCosts& msg){
  true_costs = msg.costs ;
  if (!fCosts){
    fCosts = true ;
    ExecuteTransition() ;
  }
}

void RagsRos::goalCallback(const geometry_msgs::Twist& msg){
  if (!fGoal){
    if (fPlanner){
      delete RAGSPlanner ;
      fPlanner = false ;
    }
    ROS_INFO("Global goal received, creating new RAGS object...") ;
    RAGSPlanner = new RAGS(vertices, edges, cost_distributions) ;
    fPlanner = true ;
    ROS_INFO("RAGS object created...") ;
    goal.x = msg.linear.x ;
    goal.y = msg.linear.y ;
    fGoal = true ;
    Start.clear() ;
    Start.push_back(curLoc.x) ;
    Start.push_back(curLoc.y) ;
    Goal.clear() ;
    Goal.push_back(goal.x) ;
    Goal.push_back(goal.y) ;
    Path.clear() ;
    Path.push_back(curLoc.x) ;
    Path.push_back(curLoc.y) ;
    goalReceiveTime = ros::Time::now() ;
    ExecuteTransition() ;
  }
  else
    ROS_INFO("No new global goals accepted until robot has fully executed current plan.") ;
}

void RagsRos::goalIndexCallback(const std_msgs::Int16& msg){
  geometry_msgs::Twist g ;
  g.linear.x = vertices[msg.data].x ;
  g.linear.y = vertices[msg.data].y ;
  g.linear.z = 0.0 ;
  g.angular.x = 0.0 ;
  g.angular.y = 0.0 ;
  g.angular.z = 0.0 ;
  
  pubGoal.publish(g) ;
}

void RagsRos::ExecuteTransition(){
  if (!fCosts)
    ROS_INFO("Waiting to receive next transition costs...") ;
  if (!fPose)
    ROS_INFO("Waiting to receive updated pose information...") ;
  if (!fGoal)
    ROS_INFO("Waiting to receive global goal...") ;
  
  if (fCosts && fPose && fGoal) {
    geometry_msgs::Twist waypoint ;
    std_msgs::Int16 edgeInd ;
    
    nextLoc = RAGSPlanner->SearchGraph(curLoc,goal,true_costs) ;
    int e = GetEdgeIndex(curLoc,nextLoc) ;
    edgeInd.data = e ;
    
    waypoint.linear.x = nextLoc.x ;
    waypoint.linear.y = nextLoc.y ;
    waypoint.linear.z = 0.0 ;
    waypoint.angular.x = 0.0 ;
    waypoint.angular.y = 0.0 ;
    waypoint.angular.z = 0.0 ;
    
    pubWaypoint.publish(waypoint) ;
    pubEdgeIndex.publish(edgeInd) ;
  }
}

int RagsRos::GetEdgeIndex(XY s, XY g){
	int startID ;
	int goalID ;
	bool foundStart = false ;
	bool foundGoal = false ;
	for (UINT i = 0; i < vertices.size(); i++){
		if (fabs(s.x - vertices[i].x) < 0.001 && fabs(s.y - vertices[i].y) < 0.001){
//      ROS_INFO_STREAM("Robot current position at vertex " << i << ": (" << s.x << "," << s.y << ")") ;
			startID = i ;
			foundStart = true ;
		}
		if (fabs(g.x - vertices[i].x) < 0.001 && fabs(g.y - vertices[i].y) < 0.001){
			goalID = i ;
			foundGoal = true ;
		}
		if (foundStart && foundGoal)
			break ;
	}
	if (!foundStart){
		ROS_INFO_STREAM("ERROR: Did not find current vertex index for (" << s.x << "," << s.y << ").") ;
		exit(1) ;
	}
	if (!foundGoal){
		ROS_INFO_STREAM("ERROR: Did not find next vertex index for (" << g.x << "," << g.y << ").") ;
		exit(1) ;
	}
	for (UINT i = 0; i < edges.size(); i++){
		if (edges[i].first == startID && edges[i].second == goalID)
			return i ;
	}
	ROS_INFO_STREAM("Error: Did not find edge index for (" << startID << "," << goalID << "). Exiting.") ;
	exit(1) ;
}
