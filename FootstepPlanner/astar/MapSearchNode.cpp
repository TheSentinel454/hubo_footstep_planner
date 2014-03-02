#include "MapSearchNode.h"

using namespace std;
using namespace astar;
using namespace Eigen;
using namespace fsp;

int FootstepPlanner::MAP_WIDTH;
int FootstepPlanner::MAP_HEIGHT;
float FootstepPlanner::DISCRETIZATION_RES;
int* FootstepPlanner::ENVIRONMENT_MAP;
Vector2d FootstepPlanner::MIN_POINT;
Vector2d FootstepPlanner::INV_MIN_POINT;
Vector2d FootstepPlanner::MAX_POINT;

MapSearchNode::MapSearchNode()
{
    x = y = 0;
}

MapSearchNode::MapSearchNode(Vector2i p)
{
    x = p[0];
    y = p[1];
}

MapSearchNode::MapSearchNode(unsigned int px, unsigned int py)
{
    x=px;
    y=py;
}

int MapSearchNode::GetMap( int x, int y )
{
    if( x < 0 || x >= FootstepPlanner::MAP_WIDTH ||
        y < 0 || y >= FootstepPlanner::MAP_HEIGHT)
    {
        return 9;
    }
    return FootstepPlanner::ENVIRONMENT_MAP[(y*FootstepPlanner::MAP_WIDTH)+x];
}

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{
    // Same state in a map search is simply when (x,y) are the same
    return ((x == rhs.x) && (y == rhs.y));
}

void MapSearchNode::PrintNodeInfo()
{
    char str[100];
    sprintf( str, "Node position : (%d,%d)\n", x,y );

    cout << str;
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal.
float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
    float xd = float( ( (float)x - (float)nodeGoal.x ) );
    float yd = float( ( (float)y - (float)nodeGoal.y) );

    return xd + yd;

}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

    if( (x == nodeGoal.x) &&
        (y == nodeGoal.y) )
    {
        return true;
    }

    return false;
}

///
/// \fn MapSearchNode::GetSuccessors
/// \brief This generates the successors to the given Node. It uses a helper function called
///        AddSuccessor to give the successors to the AStar class. The A* specific initialisation
///        is done for each node internally, so here you just set the state information that
///        is specific to the application
/// \param astarsearch
/// \param parent_node
/// \return
///
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

    int parent_x = -1;
    int parent_y = -1;

    if( parent_node )
    {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }


    MapSearchNode NewNode;

    // push each possible move except allowing the search to go backwards

    if( (GetMap( x-1, y ) < 9)
        && !((parent_x == x-1) && (parent_y == y))
      )
    {
        NewNode = MapSearchNode( x-1, y );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x, y-1 ) < 9)
        && !((parent_x == x) && (parent_y == y-1))
      )
    {
        NewNode = MapSearchNode( x, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }

    if( (GetMap( x+1, y ) < 9)
        && !((parent_x == x+1) && (parent_y == y))
      )
    {
        NewNode = MapSearchNode( x+1, y );
        astarsearch->AddSuccessor( NewNode );
    }


    if( (GetMap( x, y+1 ) < 9)
        && !((parent_x == x) && (parent_y == y+1))
        )
    {
        NewNode = MapSearchNode( x, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }

    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is
// conceptually where we're moving
float MapSearchNode::GetCost( MapSearchNode &successor )
{
    return (float) GetMap( x, y );

}
