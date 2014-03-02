#ifndef MAPSEARCHNODE_H
#define MAPSEARCHNODE_H
#include "stlastar.h" // See header for copyright and usage information
#include "FootstepPlanner.h"
#include <eigen3/Eigen/Core>

#include <iostream>
#include <stdio.h>

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

using namespace std;

namespace astar
{

// Definitions
class MapSearchNode
{
public:
	unsigned int x;	 // the (x,y) positions of the node
    unsigned int y;
	
    MapSearchNode();
    MapSearchNode(Eigen::Vector2i p);
    MapSearchNode(unsigned int px, unsigned int py);

    int GetMap( int x, int y );
	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );

    void PrintNodeInfo();
};
}

#endif
