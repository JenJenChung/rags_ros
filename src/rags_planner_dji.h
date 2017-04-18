#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Time.h"
#include <ros/console.h>
#include <vector>
#include <string>
#include <math.h>
#include <fstream> // std::ifstream, std::ofstream
#include <sstream> // std::stringstream
#include "custom_messages/Edge_Costs.h"
#include "custom_messages/Nav_Goal.h"
#include "custom_messages/Query_RAGS.h"
#include "custom_messages/Scan_Goals.h"
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
    ~RagsRos() {delete RAGSPlanner ;}
    
  private:
    ros::Subscriber subQueries ;
    ros::Subscriber subCost ;
    ros::Publisher pubEdgesToScan ;
    ros::Publisher pubWaypoint ;
    ros::Publisher pubPathCost ;
    ros::Publisher pubGoal ;
    bool fCosts ;
    int query ;
    custom_messages::Nav_Goal waypoint ;
    
    int numVertices ;
    int numEdges ;
    XY curLoc ;
    XY nextLoc ;
    XY goal ;
    XY qLoc, eLoc ; // error checking to confirm requests are made from same vertex
    int qInd, eInd ;
    vector<XY> vertices ;
    vector<edge> edges ;
    vector< vector<double> > cost_distributions ;
    vector<double> true_costs ;
    
    vector<double> Start ; // logged values
    vector<double> Goal ; // logged values
    vector<double> Path ; // logged values
    ros::Time goalReceiveTime ;
    
    RAGS * RAGSPlanner ;
    
    void actionCallback(const custom_messages::Query_RAGS&) ;
    void costCallback(const custom_messages::Edge_Costs&) ;
    void ExecuteTransition() ;
    void ComputeNeighbouringEdges() ;
};

RagsRos::RagsRos(ros::NodeHandle nh){
  subQueries = nh.subscribe("/waiting_on_RAGS", 10, &RagsRos::actionCallback, this) ;
  subCost = nh.subscribe("/edge_costs", 10, &RagsRos::costCallback, this) ;
  pubEdgesToScan = nh.advertise<custom_messages::Scan_Goals>("/edges_to_scan", 10, true) ;
  pubWaypoint = nh.advertise<custom_messages::Nav_Goal>("/waypoint_to_travel", 10) ;
  pubPathCost = nh.advertise<rags_ros::PathCost>("path_cost", 10, true) ;
  pubGoal = nh.advertise<geometry_msgs::Twist>("global_goal", 10, true) ;
  
  query = -1 ; // note: initial query message should always be 0 for edges to scan
  fCosts = false ;
  
  // Read in parameters
  ros::param::get("/num_vertices", numVertices);
  ros::param::get("/num_edges", numEdges);
  char buffer[50] ;
  
  // Read in vertices
  for (int i = 0; i < numVertices; i++){
    vector<double> v ;
    sprintf(buffer,"/vertex%d",i) ;
    ros::param::get(buffer,v) ;
		vertices.push_back(XY(v[0],v[1])) ;
  }
  
  // Set current location as first vertex and goal as final vertex
  curLoc = vertices[0] ;
  goal = vertices[vertices.size()-1] ;
  
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
		
    // Initialise true costs
		true_costs.push_back(0.0) ;
  }
  
  // Log data
  Start.clear() ;
  Start.push_back(curLoc.x) ;
  Start.push_back(curLoc.y) ;
  Goal.clear() ;
  Goal.push_back(goal.x) ;
  Goal.push_back(goal.y) ;
  Path.clear() ;
  Path.push_back(curLoc.x) ;
  Path.push_back(curLoc.y) ;
  
  geometry_msgs::Twist GG ;
  GG.linear.x = goal.x ;
  GG.linear.y = goal.y ;
  GG.linear.z = 0.0 ;
  GG.angular.x = 0.0 ;
  GG.angular.y = 0.0 ;
  GG.angular.z = 0.0 ;
  
  pubGoal.publish(GG) ;
  
  // Initialise RAGS object and non-dominated path set
  RAGSPlanner = new RAGS(vertices, edges, cost_distributions) ;
  XY s ;
  RAGSPlanner->InitialiseNDSet(curLoc,goal,s) ;
}

void RagsRos::actionCallback(const custom_messages::Query_RAGS& msg){
  if (query == -1)
    goalReceiveTime = ros::Time::now() ; // log first instance of path query
  query = msg.request_type ;
  qInd = msg.my_vertex_index ;
  qLoc = vertices[qInd] ;
  if (query == 0){ // query for edges to scan
    ComputeNeighbouringEdges() ;
  }
  else if (query == 1){ // query for next waypoint
    ExecuteTransition() ;
  }
  else{
    ROS_INFO_STREAM("Unknown query ID: " << query << ". Ignoring...") ;
  }
}

void RagsRos::costCallback(const custom_messages::Edge_Costs& msg){
  eInd = msg.my_vertex_index ;
  eLoc = vertices[eInd] ;
  for (size_t i = 0; i < msg.indices.size(); i++){
    true_costs[msg.indices[i]] = msg.costs[i] ;
  }
  fCosts = true ; // indicate that new edge costs have been received
}

void RagsRos::ExecuteTransition(){
  if (!fCosts)
    ROS_INFO("Waiting to receive next transition costs...") ;
  else{
    if (qInd != eInd){
      ROS_INFO_STREAM("Mismatching vertex IDs from /waiting_on_RAGS [" << qInd << "] and /edge_costs [" << eInd << "]! Waiting for consensus to continue...") ;
    }
    else{
      curLoc = qLoc ;
      nextLoc = RAGSPlanner->SearchGraph(curLoc,goal,true_costs) ;
      
      bool vFound = false ;
      for (int i = 0; i < numVertices; i++){
        if (nextLoc == vertices[i]){
          vFound = true ;
          waypoint.goal_index = i ;
          waypoint.goal_lat = vertices[i].x ;
          waypoint.goal_lon = vertices[i].y ;
          break ;
        }
      }
      if (!vFound){
        ROS_INFO_STREAM("ERROR: Next commanded vertex (" << nextLoc.x << "," << nextLoc.y << ") not found! Exiting.") ;
        exit(1) ;
      }
      pubWaypoint.publish(waypoint) ;
      fCosts = false ; // flag current edge costs as out of date
      
      // Log
      Path.push_back(nextLoc.x) ;
      Path.push_back(nextLoc.y) ;
      
      if (nextLoc == goal){
        ros::Duration plan_time = ros::Time::now() - goalReceiveTime ;
        rags_ros::PathCost p ;
        p.robot_name = "DJI Matrice 100" ;
        p.planner = "RAGS" ;
        p.start = Start ;
        p.goal = Goal ;
        p.path = Path ;
        p.execution_time = plan_time ; // note: only logs time between first and last query from platform
        pubPathCost.publish(p) ;
      }
    }
  }
}

void RagsRos::ComputeNeighbouringEdges(){
  vector<Node *> ndSet = RAGSPlanner->GetNDSet() ;
  
  // Error checking
  if (ndSet[0]->GetVertex()->GetX() != qLoc.x || ndSet[0]->GetVertex()->GetY() != qLoc.y){
    ROS_INFO_STREAM("ERROR: Mismatching vertices between stored non-dominated path set (" << ndSet[0]->GetVertex()->GetX() << "," << ndSet[0]->GetVertex()->GetY() << ") and received current vertex (" << qLoc.x << "," << qLoc.y << ")! Exiting.") ;
    exit(1) ;
  }
  custom_messages::Scan_Goals e ;
  
  for (size_t i = 0; i < RAGSPlanner->GetNDSetSize(); i++){
    XY v2 ;
    int v2Ind ;
    v2.x = ndSet[i]->GetParent()->GetVertex()->GetX() ;
    v2.y = ndSet[i]->GetParent()->GetVertex()->GetY() ;
    
    bool vFound = false ;
    for (int j = 0; j < numVertices; j++){
      if (v2 == vertices[j]){
        v2Ind = j ;
        vFound = true ;
        break ;
      }
    }
    if (!vFound)
      ROS_INFO_STREAM("Vertex (" << v2.x << "," << v2.y << ") not found! Skipping...") ;
    else{
      bool eFound = false ;
      for (int j = 0; j < numEdges; j++){
        if (edges[j].first == qInd && edges[j].second == v2Ind){
          e.indices.push_back(j) ;
          e.lats.push_back(v2.x) ;
          e.lons.push_back(v2.y) ;
          eFound = true ;
          break ;
        }
      }
      if (!eFound)
        ROS_INFO_STREAM("Edge connecting vertices (" << qInd << "," << v2Ind << ") not found! Skipping...") ;
    }
  }
  pubEdgesToScan.publish(e) ;
}
