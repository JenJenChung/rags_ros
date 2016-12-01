#ifndef SEARCH_H_
#define SEARCH_H_

#include <vector> // std::vector, std::cout
#include <time.h> // clock_t, clock, CLOCKS_PER_SEC
#include <math.h> // pow, abs, sqrt
#include "Vertex.h"
#include "Edge.h"
#include "Graph.h"
#include "Node.h"
#include "Queue.h"

using std::vector ;
using std::cout ;

typedef unsigned long int ULONG ;

//typedef unsigned long int ULONG ;
enum searchType {ASTAR, DIJKSTRA} ; // BREADTH, DEPTH
enum heuristic {ZERO, MANHATTAN, EUCLIDEAN} ;
enum pathOut {BEST,ALL} ;

// Path search class to store and search a graph
// A* search with zero, Manhattan or Euclidean distance heuristics
class Search
{
	public:
		Search(Graph * graph, Vertex * source, Vertex * goal):itsGraph(graph), itsSource(source), itsGoal(goal){
	  	SEARCH_TYPE = ASTAR ;
	  	HEURISTIC = ZERO ;
  	}

		~Search(){
	    delete itsQueue ;
	    itsQueue = 0 ;
    }
		
		Graph * GetGraph() const {return itsGraph ;}
		Queue * GetQueue() const {return itsQueue ;}
		void SetQueue(Queue * queue) {itsQueue = queue ;}
		Vertex * GetSource() const {return itsSource ;}
		Vertex * GetGoal() const {return itsGoal ;}
		vector<Node *> PathSearch(pathOut pType) ;

	private:
		Graph * itsGraph ;
		Queue * itsQueue ;
		Vertex * itsSource ;
		Vertex * itsGoal ;
		searchType SEARCH_TYPE ;
		heuristic HEURISTIC ;
		
		ULONG FindSourceID() ;
		double ManhattanDistance(Vertex * v1, Vertex * v2) ;
		double EuclideanDistance(Vertex * v1, Vertex *v2) ;
		void UpdateNode(Node * n) ;
} ;

vector<Node *> Search::PathSearch(pathOut pType)
{
  ULONG sourceID = FindSourceID() ;
  itsQueue = new Queue(new Node(itsGraph->GetVertices()[sourceID], SOURCE)) ;

  clock_t t_start = clock() ;
  double t_elapse = 0.0 ;

  while (!itsQueue->EmptyQueue() && t_elapse < 5.0){
    // Pop cheapest node from queue
    Node * currentNode = itsQueue->PopQueue() ;
    if (!currentNode){
    	// Dominated node was popped off queue
    	continue ;
  	}

    // Terminate search once one path is found
    if (pType == BEST)
	    if (currentNode->GetVertex() == itsGoal)
		    break ;

    // Find all neighbours excluding ancestor vertices if any
    vector<Edge *> neighbours = itsGraph->GetNeighbours(currentNode) ;

    // Update neighbours
    for (ULONG i = 0; i < (ULONG)neighbours.size(); i++){
	    // Check if neighbour vertex is already in closed set
	    bool newNeighbour = true ;
	    if (pType == BEST){
		    Vertex * vcheck = neighbours[i]->GetVertex2() ;
		    for (ULONG j = 0; j < itsQueue->GetClosed().size(); j++){
			    if (itsQueue->GetClosed()[j]->GetVertex() == vcheck){
				    newNeighbour = false ;
				    break ;
			    }
		    }
	    }
	
	    if (newNeighbour){
		    // Create neighbour node
		    Node * currentNeighbour = new Node(currentNode, neighbours[i]) ;
		    UpdateNode(currentNeighbour) ;
		    itsQueue->UpdateQueue(currentNeighbour) ;
	    }
    }

    t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;
  }

	if (t_elapse >= 5.0)
		cout << "Search timed out!\n" ;
	
  // Check if a path is found
  bool ClosedAll = false ;
  for (ULONG i = 0; i < itsQueue->GetClosed().size(); i++){
    if (itsQueue->GetClosed()[i]->GetVertex() == itsGoal){
	    ClosedAll = true ;
	    break ;
    }
  }

  if (!ClosedAll){
    cout << "No path found from source to goal.\n" ;
    vector<Node *> bestPath ;
    return bestPath ;
  }
  else{
    ULONG k = 0 ;
    vector<Node *> bestPath((ULONG)itsQueue->GetClosed().size()) ;

    for (ULONG i = 0; i < (ULONG)itsQueue->GetClosed().size(); i++){
	    if (itsGoal == itsQueue->GetClosed()[i]->GetVertex()){
		    bestPath[k] = itsQueue->GetClosed()[i] ;
		    k++ ;
	    }
    }

    bestPath.resize(k) ;
    return bestPath ;
  }
}

ULONG Search::FindSourceID()
{
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
    if (itsSource == itsGraph->GetVertices()[i])
      return i ;
  cout << "Error: souce ID not found. Exiting.\n" ;
  exit(1) ;
}

double Search::ManhattanDistance(Vertex * v1, Vertex * v2)
{
  double diffX = abs(v1->GetX() - v2->GetX()) ;
  double diffY = abs(v1->GetY() - v2->GetY()) ;
  double diff = diffX + diffY ;
  return diff ;
}

double Search::EuclideanDistance(Vertex * v1, Vertex *v2)
{
  double diffX = pow(v1->GetX() - v2->GetX(),2) ;
  double diffY = pow(v1->GetY() - v2->GetY(),2) ;
  double diff = sqrt(diffX+diffY) ;
  return diff ;
}

void Search::UpdateNode(Node * n)
{
  if (SEARCH_TYPE == ASTAR)
  {
    double diff ;
    switch (HEURISTIC)
    {
	    case ZERO:
		    diff = 0.0 ;
		    n->SetHeuristic(diff) ;
		    break ;
	    case MANHATTAN:
		    diff = ManhattanDistance(itsGoal, n->GetVertex()) ;
		    n->SetHeuristic(diff) ;
		    break ;
	    case EUCLIDEAN:
		    diff = EuclideanDistance(itsGoal, n->GetVertex()) ;
		    n->SetHeuristic(diff) ;
		    break ;
    }
  }
  else if (SEARCH_TYPE == DIJKSTRA)
  {
    HEURISTIC = ZERO ;
    SEARCH_TYPE = ASTAR ;
    UpdateNode(n) ;
  }
}

#endif // SEARCH_H_
