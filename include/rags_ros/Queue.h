#ifndef QUEUE_H_
#define QUEUE_H_

#include <vector> // std::vector
#include <queue> // std::priority_queue
#include "Node.h"
#include "CompareNode.h"

using std::vector ;
using std::priority_queue ;

typedef unsigned long int ULONG ;

// Custom queue type to perform priority queue updates
class Queue
{
	public:
  	typedef priority_queue<Node *, vector<Node *>, CompareNode> QUEUE ;
		Queue(Node * source){
	    itsPQ = new QUEUE ;
	    itsPQ->push(source) ;
    }
    
		~Queue(){
			while (!itsPQ->empty()){
				Node * temp = itsPQ->top() ;
				delete temp ;
				temp = 0 ;
				itsPQ->pop() ;
			}
	    delete itsPQ ;
	    itsPQ = 0 ;
	    for (ULONG i = 0; i < closed.size(); i ++){
		    delete closed[i] ;
		    closed[i] = 0 ;
	    }
    }
		
		vector<Node *> GetClosed() const {return closed ;}
		bool EmptyQueue() const {return itsPQ->empty() ;}
		ULONG SizeQueue() const {return (ULONG)itsPQ->size() ;}
		void UpdateQueue(Node * newNode) ;
		Node * PopQueue() ;
    
	private:
		QUEUE * itsPQ ;
		vector<Node *> closed ;
		
		bool CompareNodes(const Node * n1, const Node * n2) const ;
} ;

void Queue::UpdateQueue(Node * newNode)
{
  // Compare newNode to nodes in closed set
  // if closed contains node with same vertex, compare their costs
  // choose whether or not to create a new node
  bool dom = false ;
  for (ULONG i = 0; i < closed.size(); i++){
    if (closed[i]->GetVertex() == newNode->GetVertex()){
	    dom = CompareNodes(newNode, closed[i]) ;
	    if (dom){
	    	delete newNode ;
	    	newNode = 0 ;
		    return ;
	    }
    }
  }
  itsPQ->push(newNode) ;
}

Node * Queue::PopQueue()
{
  // Check if next node is already dominated by existing node in closed set
  Node * newNode = itsPQ->top() ;
  bool dom = false ;
  for (ULONG i = 0; i < closed.size(); i++){
    if (closed[i]->GetVertex() == newNode->GetVertex()){
      dom = CompareNodes(newNode, closed[i]) ;
      if (dom){
      	delete newNode ;
      	newNode = 0 ;
	      itsPQ->pop() ;
	      return 0 ;
      }
    }
  }
  closed.push_back(itsPQ->top()) ;
  itsPQ->pop() ;
  return closed[(ULONG)closed.size()-1] ;
}

bool Queue::CompareNodes(const Node * n1, const Node * n2) const
{
	// out: Is n1 worse than or equal to n2?
  double n1Cost = n1->GetMeanCost() ;
  double n2Cost = n2->GetMeanCost() ;
  return (n1Cost >= n2Cost && n1->GetVarCost() >= n2->GetVarCost()) ;
  /*	if (n1Cost > n2Cost && n1->GetVarCost() > n2->GetVarCost())
    return true ;
  else if (n2Cost > n1Cost && n2->GetVarCost() > n1->GetVarCost())
    return false ;
  else
    return (n1Cost > n2Cost) ;*/
}

#endif //QUEUE_H_
