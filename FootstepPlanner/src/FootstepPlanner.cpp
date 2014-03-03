/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Luke Tornquist <luke.tornquist@gatech.edu>
 * Date: Jan 2014
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the Humanoid Robotics Lab nor the names of
 *     its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "FootstepPlanner.h"
#include <queue>
#define DISPLAY_SOLUTION

using namespace fsp;
using namespace std;
using namespace Eigen;
using namespace flann;
using namespace astar;

///
/// \brief FootstepPlanner::FootstepPlanner
///
FootstepPlanner::FootstepPlanner(vector<Foot> ft)
{
    _Feet = ft;
}

///
/// \brief FootstepPlanner::generatePlan
/// \param plannerType
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \return
///
vector<FootLocation> FootstepPlanner::generatePlan(int plannerType, vector<FootConstraint> constraints, vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles)
{
    // Take a starting time stamp
    clock_t tStart = clock();
    // Initialize plan
    vector<FootLocation> plan;
    vector<Vector2i> mapPlan;
    switch(plannerType)
    {
        case PLANNER_TYPE_R_STAR:
            plan = runRStarPlanner(constraints, currentLocation, goalLocation, obstacles);
            break;
        case PLANNER_TYPE_A_STAR:
            plan = runAStarPlanner(constraints, currentLocation, goalLocation, obstacles, mapPlan);
            break;
        case PLANNER_TYPE_RRT:
        default:
            plan = runRRTPlanner(constraints, currentLocation, goalLocation, obstacles);
            break;
    }
    // Take the ending time stamp
    clock_t tEnd = clock();

    // Calculate the planning time
    double dPlanningTime = (double)((tEnd - tStart)/CLOCKS_PER_SEC);

    // Save the planning data
    _writePlannerOutput(dPlanningTime, plan);

    // Return the plan
    return plan;
}

///
/// \brief FootstepPlanner::runRRTPlanner
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \return
///
vector<FootLocation> FootstepPlanner::runRRTPlanner(vector<FootConstraint> constraints, vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles)
{
    // Initialize the plan
    vector<FootLocation> plan;
    // Initialize Current Position RRT
    flann::Matrix<double> initialState(new double[currentLocation.size() * 2], currentLocation.size(), 2);
    // Add each of the start locations
    for(int i = 0; i < currentLocation.size(); i++)
    {
        initialState[i][0] = currentLocation[i].getLocation()[0]; // X Position
        initialState[i][1] = currentLocation[i].getLocation()[1]; // Y Position
        // Update random min/max values
        _updateRandomMinMaxValues(initialState[i][0], initialState[i][1]);
    }
    // Create the index with only the initial state
    flann::Index< flann::L2<double> > startRRT(initialState, flann::KDTreeIndexParams(4));
    startRRT.buildIndex();

    // Initialize Goal Position RRT
    flann::Matrix<double> goalState(new double[goalLocation.size() * 2], goalLocation.size(), 2);
    // Add each of the goal locations
    for(int i = 0; i < goalLocation.size(); i++)
    {
        goalState[i][0] = goalLocation[i].getLocation()[0]; // X Position
        goalState[i][1] = goalLocation[i].getLocation()[1]; // Y Position
        // Update random min/max values
        _updateRandomMinMaxValues(goalState[i][0], goalState[i][1]);
    }
    // Create the index with only the goal state
    flann::Index< flann::L2<double> > goalRRT(goalState, flann::KDTreeIndexParams(4));
    goalRRT.buildIndex();

    FootLocationNode startFootLocationRoot(currentLocation[0], &_Feet);
    startFootLocationRoot.setParent(NULL);
    for(int i = 1; i < currentLocation.size(); i++)
        startFootLocationRoot.addChild(currentLocation[i], &_Feet);

    FootLocationNode goalFootLocationRoot(goalLocation[0], &_Feet);
    goalFootLocationRoot.setParent(NULL);
    for(int i = 1; i < goalLocation.size(); i++)
        goalFootLocationRoot.addChild(goalLocation[i], &_Feet);

    FootLocation* lastStartNode = &currentLocation[0];
    FootLocation* lastGoalNode = &goalLocation[0];
    int i = 0;
    // Iterate until we find a path
    do
    {
        // First let's grow the start RRT
        // Get the Random Point we want to grow towards
        // We are going to flip a coin to see if we use the goal RRT's
        // latest point, or we use a randomly generated point
        Vector2d vRand = _getNextRandomPoint(lastGoalNode);
        cout << "Random Point(S): " << endl << vRand << endl;

        // Find the nearest neighbor in the start RRT
        Vector2d vNearestNeighbor = _findNearestNeighbor(vRand, startRRT);
        cout << "Nearest Neighbor(S): " << endl << vNearestNeighbor << endl;

        // Find the corresponding Foot Location Node
        FootLocationNode* flnNearestNeighbor = _findFootLocationNode(vNearestNeighbor, &startFootLocationRoot);

        // Randomly generate foot location configuration (Collision detection is done when generating the foot)
        FootLocation* flNewStart = _getRandomFootLocation(constraints, obstacles, flnNearestNeighbor->getFootLocation(), vRand);
        if (flNewStart != NULL)
        {
            cout << "Found valid Foot Location(S): " << endl
                 << flNewStart->getLocation() << endl;
            // Add to the start RRT
            flann::Matrix<double> mNewPoint(new double[1 * 2], 1, 2);
            mNewPoint[0][0] = flNewStart->getLocation()[0];   // X coordinate of new point
            mNewPoint[0][1] = flNewStart->getLocation()[1];   // Y coordinate of new point
            startRRT.addPoints(mNewPoint);

            // Save the last Start Node
            lastStartNode = flNewStart;
            plan.push_back((*flNewStart));
            // Add the new foot location node to the tree
            flnNearestNeighbor->addChild((*flNewStart), &_Feet);
            // Check to see if we are close enough to the goal
            double x2 = (*flNewStart).getLocation()[0];
            double x1 = goalLocation[(*flNewStart).getFootIndex()].getLocation()[0];
            double y2 = (*flNewStart).getLocation()[1];
            double y1 = goalLocation[(*flNewStart).getFootIndex()].getLocation()[1];
            double dstnc = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
            if (dstnc <= 0.5d)
            {
                // Good enough, let's find the path
                cout << "Found path: " << i << endl;
                cout << "Final Node: " << (*flNewStart).getFootIndex() << endl
                     << (*flNewStart).getLocation() << endl;
                cout << "Goal Location: " << endl;
                cout << goalLocation[(*flNewStart).getFootIndex()].getLocation() << endl;
                cout << "Distance: " << dstnc << endl;
                FootLocationNode* flnPath = flnNearestNeighbor;
                vector<FootLocation> locs;
                do
                {
                    if (flnPath->getParent() == NULL)
                        break;
                    else
                    {
                        locs.push_back(flnPath->getFootLocation());
                        flnPath = flnPath->getParent();
                    }
                }
                while(true);
                plan.clear();
                // Reverse the plan
                for(int i = locs.size() - 1; i >= 0; i--)
                {
                    plan.push_back(locs[i]);
                }
                break;
            }
        }
        i++;
        cout << "Iteration: " << i << endl;

        // Now let's grow the goal RRT
        /*
        // Get the Random Point we want to grow towards
        // Same as above, coin flipping time
        Vector2d vRandG = _getNextRandomPoint(lastStartNode);
        cout << "Random Point(G): " << endl << vRandG << endl;

        // Find the nearest neighbor in the start RRT
        Vector2d vNearestNeighborG = _findNearestNeighbor(vRandG, goalRRT);
        cout << "Nearest Neighbor(G): " << endl << vNearestNeighborG << endl;

        // Find the corresponding Foot Location Node
        FootLocationNode* flnNearestNeighborG = _findFootLocationNode(vNearestNeighborG, &goalFootLocationRoot);

        // Randomly generate foot location configuration (Collision detection is done when generating the foot)
        FootLocation* flNewGoal = _getRandomFootLocation(constraints, obstacles, flnNearestNeighborG->getFootLocation(), vRandG);
        if (flNewGoal != NULL)
        {
            cout << "Found valid Foot Location(G): " << flNewGoal->getLocation() << endl;
            // Add to the start RRT
            flann::Matrix<double> mNewPoint(new double[1 * 2], 1, 2);
            mNewPoint[0][0] = flNewGoal->getLocation()[0];   // X coordinate of new point
            mNewPoint[0][1] = flNewGoal->getLocation()[1];   // Y coordinate of new point
            goalRRT.addPoints(mNewPoint);

            // Save the last Goal Node
            lastGoalNode = flNewGoal;
            // Add the new foot location node to the tree
            flnNearestNeighborG->addChild((*flNewGoal), &_Feet);
        }
        */

        // Check for goal/start RRT connectivity

    }
    while(i < 5000);
    cout << "X: " << _minimumRandomX << "-" << _maximumRandomX << endl;
    cout << "Y: " << _minimumRandomY << "-" << _maximumRandomY << endl;
    return plan;
}

vector<FootLocation> FootstepPlanner::runAStarPlanner(vector<FootConstraint> constraints, vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles, vector<Vector2i>& mapPlan)
{
    AStarSearch<MapSearchNode> astarsearch;

    // Get the environment map based on the start/goal location and obstacles
    DISCRETIZATION_RES = _getDiscretizationResolution(constraints);
    ENVIRONMENT_MAP = _getEnvironmentMap(currentLocation, goalLocation, obstacles);

    // Create a start state
    MapSearchNode nodeStart(_getMapCoord(currentLocation[0].getLocation()));

    // Define the goal state
    MapSearchNode nodeEnd(_getMapCoord(goalLocation[0].getLocation()));

    // Set Start and goal states
    astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

    unsigned int SearchState;
    unsigned int SearchSteps = 0;

    do
    {
        SearchState = astarsearch.SearchStep();
        SearchSteps++;

#if DEBUG_LISTS
        cout << "Steps:" << SearchSteps << "\n";
        int len = 0;

        cout << "Open:\n";
        MapSearchNode *p = astarsearch.GetOpenListStart();
        while( p )
        {
            len++;
#if !DEBUG_LIST_LENGTHS_ONLY
            ((MapSearchNode *)p)->PrintNodeInfo();
#endif
            p = astarsearch.GetOpenListNext();
        }
        cout << "Open list has " << len << " nodes\n";
        len = 0;

        cout << "Closed:\n";
        p = astarsearch.GetClosedListStart();
        while( p )
        {
            len++;
#if !DEBUG_LIST_LENGTHS_ONLY
            p->PrintNodeInfo();
#endif
            p = astarsearch.GetClosedListNext();
        }

        cout << "Closed list has " << len << " nodes\n";
#endif
    }
    while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

    vector<FootLocation> plan;
    if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
    {
        cout << "Search found goal state\n";
        MapSearchNode *node = astarsearch.GetSolutionStart();

#ifdef DISPLAY_SOLUTION
        cout << "Displaying solution\n";
        node->PrintNodeInfo();
#endif
        int steps = 0;
        mapPlan.push_back(Vector2i(node->x, node->y));

        for(;;)
        {
            node = astarsearch.GetSolutionNext();
            if(!node)
                break;
#ifdef DISPLAY_SOLUTION
            node->PrintNodeInfo();
#endif
            mapPlan.push_back(Vector2i(node->x, node->y));
            steps++;
        };
        cout << "Solution steps " << steps << endl;
        // Once you're done with the solution you can free the nodes up
        astarsearch.FreeSolutionNodes();
    }
    else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
    {
        cout << "Search terminated. Did not find goal state\n";
    }

    // Display the number of loops the search went through
    cout << "SearchSteps : " << SearchSteps << "\n";
    astarsearch.EnsureMemoryFreed();

    int prevDirection = 0;
    // 0 = Moving Right
    // 1 = Moving Left
    // 2 = Moving Up
    // 3 = Moving Down
    vector<int> directions;
    // 0 = Continue Right
    // 1 = Continue Left
    // 2 = Continue Up
    // 3 = Continue Down
    // 4 = Turn Left
    // 5 = Turn Right
    // Now we need to generate the foot step path
    for(int i = 0; i < (mapPlan.size() - 1); i++)
    {
        // Make sure we have the next location
        if ((i + 1) < mapPlan.size())
        {
            // Check for X increasing (Moving Right)
            if (mapPlan[i][0] < mapPlan[i+1][0])
            {
                switch(prevDirection)
                {
                case 0:
                    directions.push_back(0);
                    cout << "Continue Right" << endl;
                    break;
                case 2:
                    directions.push_back(5);
                    cout << "Turning Right" << endl;
                    break;
                case 3:
                    directions.push_back(4);
                    cout << "Turning Left" << endl;
                    break;
                case 1:
                default:
                    cout << "WTF?!?!" << endl;
                    break;
                }
                prevDirection = 0;
            }
            // Check for X decreasing (Moving Left)
            if (mapPlan[i][0] > mapPlan[i+1][0])
            {
                switch(prevDirection)
                {
                case 1:
                    directions.push_back(1);
                    cout << "Continue Left" << endl;
                    break;
                case 2:
                    directions.push_back(4);
                    cout << "Turning Left" << endl;
                    break;
                case 3:
                    directions.push_back(5);
                    cout << "Turning Right" << endl;
                    break;
                case 0:
                default:
                    cout << "WTF?!?!" << endl;
                    break;
                }
                prevDirection = 1;
            }
            // Check for Y increasing (Moving Up)
            if (mapPlan[i][1] < mapPlan[i+1][1])
            {
                switch(prevDirection)
                {
                case 2:
                    directions.push_back(2);
                    cout << "Continue Up" << endl;
                    break;
                case 0:
                    directions.push_back(4);
                    cout << "Turning Left" << endl;
                    break;
                case 1:
                    directions.push_back(5);
                    cout << "Turning Right" << endl;
                    break;
                case 3:
                default:
                    cout << "WTF?!?!" << endl;
                    break;
                }
                prevDirection = 2;
            }
            // Check for Y decreasing (Moving Down)
            if (mapPlan[i][1] > mapPlan[i+1][1])
            {
                switch(prevDirection)
                {
                case 3:
                    directions.push_back(3);
                    cout << "Continue Down" << endl;
                    break;
                case 1:
                    directions.push_back(4);
                    cout << "Turning Left" << endl;
                    break;
                case 0:
                    directions.push_back(5);
                    cout << "Turning Right" << endl;
                    break;
                case 2:
                default:
                    cout << "WTF?!?!" << endl;
                    break;
                }
                prevDirection = 3;
            }
        }
    }
    cout << "MapPlan Size: " << mapPlan.size() << endl;
    cout << "Directions Size: " << directions.size() << endl;

    int previousFootIndex = currentLocation[0].getFootIndex();
    int nextFootIndex = (previousFootIndex + 1);
    nextFootIndex = nextFootIndex % _Feet.size();
    Vector2d vdLatestFootLocation[_Feet.size()];
    Vector2i viLatestFootLocation[_Feet.size()];
    float theta = 0.0f;
    for(int i = 0; i < mapPlan.size()-1; i++)
    {
        do
        {
            switch(directions[i])
            {
            case 1:
                theta = 180.0f;
                break;
            case 2:
                theta = 90.0f;
                break;
            case 3:
                theta = 270.0f;
                break;
            case 4:
                theta = ((int)theta + 90) % 360;
                break;
            case 5:
                theta = ((int)theta - 90 + 360) % 360;
                break;
            case 0:
            default:
                theta = 0.0f;
                break;
            }

            // Add a location
            plan.push_back(FootLocation(_getWorldCoord(mapPlan[i + 1]), theta, nextFootIndex, &_Feet));

            // Save the latest foot location
            vdLatestFootLocation[nextFootIndex] = plan.back().getLocation();
            viLatestFootLocation[nextFootIndex] = _getMapCoord(vdLatestFootLocation[nextFootIndex]);

            // Move to the next foot
            previousFootIndex = nextFootIndex;
            nextFootIndex = (nextFootIndex + 1) % _Feet.size();
        }
        while(viLatestFootLocation[previousFootIndex][0] == mapPlan[i][0] &&
              viLatestFootLocation[previousFootIndex][1] == mapPlan[i][1]);

    }

    return plan;
}

int* FootstepPlanner::_getEnvironmentMap(vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles)
{
    // Current location
    for(int i = 0; i < currentLocation.size(); i++)
    {
        // Check for the minimum point
        if (currentLocation[i].getLocation()[0] < MIN_POINT[0])
            MIN_POINT[0] = currentLocation[i].getLocation()[0];
        if (currentLocation[i].getLocation()[1] < MIN_POINT[1])
            MIN_POINT[1] = currentLocation[i].getLocation()[1];
        // Check for the maximum point
        if (currentLocation[i].getLocation()[0] > MAX_POINT[0])
            MAX_POINT[0] = currentLocation[i].getLocation()[0];
        if (currentLocation[i].getLocation()[1] > MAX_POINT[1])
            MAX_POINT[1] = currentLocation[i].getLocation()[1];
    }
    // Goal Location
    for(int i = 0; i < goalLocation.size(); i++)
    {
        // Check for the minimum point
        if (goalLocation[i].getLocation()[0] < MIN_POINT[0])
            MIN_POINT[0] = goalLocation[i].getLocation()[0];
        if (goalLocation[i].getLocation()[1] < MIN_POINT[1])
            MIN_POINT[1] = goalLocation[i].getLocation()[1];
        // Check for the maximum point
        if (goalLocation[i].getLocation()[0] > MAX_POINT[0])
            MAX_POINT[0] = goalLocation[i].getLocation()[0];
        if (goalLocation[i].getLocation()[1] > MAX_POINT[1])
            MAX_POINT[1] = goalLocation[i].getLocation()[1];
    }
    // Obstacle locations
    for(int i = 0; i < obstacles.size(); i++)
    {
        // Check for the minimum point (Start)
        if (obstacles[i].getStart()[0] < MIN_POINT[0])
            MIN_POINT[0] = obstacles[i].getStart()[0];
        if (obstacles[i].getStart()[1] < MIN_POINT[1])
            MIN_POINT[1] = obstacles[i].getStart()[1];
        // Check for the minimum point (End)
        if (obstacles[i].getEnd()[0] < MIN_POINT[0])
            MIN_POINT[0] = obstacles[i].getEnd()[0];
        if (obstacles[i].getEnd()[1] < MIN_POINT[1])
            MIN_POINT[1] = obstacles[i].getEnd()[1];

        // Check for the maximum point (Start)
        if (obstacles[i].getStart()[0] > MAX_POINT[0])
            MAX_POINT[0] = obstacles[i].getStart()[0];
        if (obstacles[i].getStart()[1] > MAX_POINT[1])
            MAX_POINT[1] = obstacles[i].getStart()[1];
        // Check for the maximum point (End)
        if (obstacles[i].getEnd()[0] > MAX_POINT[0])
            MAX_POINT[0] = obstacles[i].getEnd()[0];
        if (obstacles[i].getEnd()[1] > MAX_POINT[1])
            MAX_POINT[1] = obstacles[i].getEnd()[1];
    }
    // Add some padding to the Min/Max points
    MIN_POINT[0] -= (2 * DISCRETIZATION_RES);
    MIN_POINT[1] -= (2 * DISCRETIZATION_RES);
    MAX_POINT[0] += (2 * DISCRETIZATION_RES);
    MAX_POINT[1] += (2 * DISCRETIZATION_RES);

    // Set the inverse minimum
    INV_MIN_POINT[0] = -MIN_POINT[0];
    INV_MIN_POINT[1] = -MIN_POINT[1];

    // Find the map width based on the min/max point and discretization resolution
    MAP_WIDTH = ((MAX_POINT[0] - MIN_POINT[0]) / DISCRETIZATION_RES) + 1;

    // Find the map height based on the min/max point and discretization resolution
    MAP_HEIGHT = ((MAX_POINT[1] - MIN_POINT[1]) / DISCRETIZATION_RES) + 1;

    // Initialize map array
    int* mapPtr = new int[MAP_WIDTH * MAP_HEIGHT];
    // Set all the states to open initially
    for(int i = 0; i < (MAP_WIDTH * MAP_HEIGHT); i++)
        mapPtr[i] = 1;

    // Set all the obstacle locations to 9
    // Go through each of the obstacles
    for(int i = 0; i < obstacles.size(); i++)
    {
        // Add the start tile
        Vector2i startCoord = _getMapCoord(obstacles[i].getStart());
        // Check for valid coord
        if (_isValidMapCoord(startCoord))
            mapPtr[(startCoord[1] * MAP_WIDTH) + startCoord[0]] = 9;
        // Get slope
        float slopeRise = (obstacles[i].getEnd()[1] - obstacles[i].getStart()[1]);
        float slopeRun = (obstacles[i].getEnd()[0] - obstacles[i].getStart()[0]);
        // Iterate across the slope and add the obstacle points
        for(float j = 0.00f; j <= 1.00f; j+= 0.01f)
        {
            Vector2i midCoord = _getMapCoord(Vector2d(obstacles[i].getStart()[0] + (slopeRun * j),
                                                      obstacles[i].getStart()[1] + (slopeRise * j)));
            // Check for valid coord
            if (_isValidMapCoord(midCoord))
                mapPtr[(midCoord[1] * MAP_WIDTH) + midCoord[0]] = 9;
        }
        // Add the end tile
        Vector2i endCoord = _getMapCoord(obstacles[i].getEnd());
        if (_isValidMapCoord(endCoord))
            mapPtr[(endCoord[1] * MAP_WIDTH) + endCoord[0]] = 9;
    }

    // Return the pointer to the map
    return mapPtr;
}

bool FootstepPlanner::_isValidMapCoord(Vector2i coord)
{
    return (coord[0] >= 0 && coord[0] < MAP_WIDTH &&
            coord[1] >= 0 && coord[1] < MAP_HEIGHT);
}

float FootstepPlanner::_getDiscretizationResolution(vector<FootConstraint> constraints)
{
    float dr = 0;
    // Go through each of the feet and make it the max width/length of either foot
    for(int i = 0; i < _Feet.size(); i++)
    {
        // Check the width for a greater value
        dr = max(_Feet[i].getWidth(), dr);
        // Check the length for a greater value
        dr = max(_Feet[i].getLength(), dr);
    }

    // Now let's check the constraints
    for(int i = 0; i < constraints.size(); i++)
    {
        FootConstraint fc = constraints[i];
        float widthRange = _Feet[fc.getRefFootIndex()].getWidth() + _Feet[fc.getFootIndex()].getWidth() + fc.getMaximumDeltaY();
        float lengthRange = max(fc.getMaximumDeltaX() + 0.5d * _Feet[fc.getFootIndex()].getLength(), 0.5d * _Feet[fc.getRefFootIndex()].getLength()) +
                            max(abs(fc.getMinimumDeltaX()) + 0.5d * _Feet[fc.getFootIndex()].getLength(), 0.5d * _Feet[fc.getRefFootIndex()].getLength());
        dr = max(widthRange, dr);
        dr = max(lengthRange, dr);

    }
    return dr;
}

Vector2i FootstepPlanner::_getMapCoord(Vector2d worldCoord)
{
    return Vector2i(floor((worldCoord[0] + INV_MIN_POINT[0]) / DISCRETIZATION_RES), floor((worldCoord[1] + INV_MIN_POINT[1]) / DISCRETIZATION_RES));
}

Vector2d FootstepPlanner::_getWorldCoord(Vector2i mapCoord)
{
    return Vector2d((mapCoord[0] * DISCRETIZATION_RES) - INV_MIN_POINT[0],
                    (mapCoord[1] * DISCRETIZATION_RES) - INV_MIN_POINT[1]);
}

///
/// \brief FootstepPlanner::runRStarPlanner
/// \param constraints
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \return
///
vector<FootLocation> FootstepPlanner::runRStarPlanner(vector<FootConstraint> constraints, vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles)
{
    // TODO: Mohit, add the R Star Planner here
		//Start out with the initial state in the queue. 
		std::priority_queue<FootLocationNode> path_queue;
		//Tree which will contain all the desired nodes 
		FootLocationnode r_star_tree;

		//Boolean value to check for the goal value. 
		bool is_goal_reached = false;
		path_queue.push(new FootLocationNode(currentLocation, &_Feet));

		while(!is_goal_reached) {
				//Start by expanding the first node from the priority queue. 
				FootLocationNode curent_goal = path_queue.top();
				path_queue.pop();
				//If there is no path to this node, and its not avoid, then find the path to the node. 
				if (current_goal.path == false) {
						//Send the parent of the current goal along with the current start to get the path using weighted a_star.
						//This function would mark the path as avoid in the following two situations...
						//1) If the number of states expanded goes beyond some number, 2) If the cost of the path becomes ridiculous.    
						find_path_using_weighted_a_star(&(current_goal.parent), current_goal);
				}
				//If a path has been found, check for the final goal. Also, add k more states...  
				if (current_goal.should_avoid == false) {
						if (current_goal.is_end()) {
								is_goal_reached = true;
						} else {
								// if the goal has not been reached, do attach k more states to the queue,
								for (int i=0;i< 10;i++) {
										path_queue.push(get_random_goal(current_goal), 10);
								}
								//Add the goal in case its within the range..  
								if (get_euclid_dist(current_goal, goalLocation) < 10) {
										path_queue.push(get_random_goal(current_goal), 10);
								}
						}
				}
		}

		//Creating the final path,and putting it in a vector.
		Vector<FootLocation> finalPath;
		while (current_goal.parent!=NULL) {
			finalPath.push(current_goal.getFootLocation());	
		}

		return finalPath; 
}

///
/// \brief FootstepPlanner::weighted_astar_search()
/// \param constraints
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \return
///
void find_path_using_weighted_a_star(FootLocationNode* parent, FootLocationNode goal, int desired_weight) {
		//Start out with the initial state in the queue. 
		std::priority_queue<FootLocationNode> path_queue;
		//Boolean value to check for the goal value. 
		bool is_goal_reached = false;
		path_queue.push(new FootLocationNode(currentLocation, &_Feet));
		
		while (true) {
				//Start by expanding the first node from the priority queue. 
				FootLocationNode curent_node = path_queue.top();
				path_queue.pop();

				if ((current_node.getLocation()[0]==goal.getLocation()[0]) &&(current_node.getLocation()[1]==goal.getLocation[1])) {
						//Goal has been reached.
						return; 
				}
				//Explore from the current node.We add the heuristic towards the goal while adding steps and then push accordingly. 
				add_possible_steps(&current_node);
				//Add the goal in case its within the range..  
				if (get_euclid_dist(current_goal, goalLocation) < 10) {
						path_queue.push(get_random_goal(current_goal), 10);
				}
		}
}

public void add_possible_steps(FootLocationNode* parent) {
		//For now, add some specific values, and also be vary of the obstacles...
}

///
/// \brief FootstepPlanner::getStaticPlan
/// \return
///
/// \brief FootstepPlanner::getStaticPlan
/// \return
///
vector<FootLocation> FootstepPlanner::getStaticPlan()
{
    vector<FootLocation> plan;
    plan.push_back(FootLocation(Vector2d(5.0d, 3.0d), 0.0f, 0, &_Feet));
    plan.push_back(FootLocation(Vector2d(10.0d, 0.0d), 0.0f, 1, &_Feet));
    plan.push_back(FootLocation(Vector2d(16.0d, 3.0d), 0.0f, 0, &_Feet));
    plan.push_back(FootLocation(Vector2d(22.0d, 0.0d), 0.0f, 1, &_Feet));
    plan.push_back(FootLocation(Vector2d(29.0d, 3.0d), 0.0f, 0, &_Feet));
    plan.push_back(FootLocation(Vector2d(36.0d, 0.0d), 0.0f, 1, &_Feet));
    plan.push_back(FootLocation(Vector2d(43.0d, 3.0d), 0.0f, 0, &_Feet));
    plan.push_back(FootLocation(Vector2d(50.0d, 0.0d), 0.0f, 1, &_Feet));
    plan.push_back(FootLocation(Vector2d(57.0d, 3.0d), 0.0f, 0, &_Feet));
    plan.push_back(FootLocation(Vector2d(62.0d, 0.0d), 0.0f, 1, &_Feet));
    plan.push_back(FootLocation(Vector2d(67.0d, 3.0d), 0.0f, 0, &_Feet));
    plan.push_back(FootLocation(Vector2d(67.0d, 0.0d), 0.0f, 1, &_Feet));

    return plan;
}

///
/// \brief FootstepPlanner::writePlannerOutput
/// \param time
/// \param plan
///
void FootstepPlanner::_writePlannerOutput(double time, vector<FootLocation> plan)
{
    printf("Time taken: %.2fs\n", time);
    // Write the edges out to a file
    ofstream myfile ("PlannerOutput.txt");
    if (myfile.is_open())
    {
        myfile << "Planning Time: " << time << endl;
        for(int i = 0; i < plan.size(); i++)
        {
            myfile << plan[i].getLocation()[0] << ",";
            myfile << plan[i].getLocation()[1] << ",";
            myfile << plan[i].getTheta() << ",";
            myfile << _Feet[plan[i].getFootIndex()].getName() << endl;
        }
        //myfile << plan << endl;
        myfile.close();
    }
    else cout << "Unable to open file";
}

///
/// \brief FootstepPlanner::_getNextRandomPoint
/// \param lastPoint
/// \return
///
Vector2d FootstepPlanner::_getNextRandomPoint(FootLocation* lastFootNode)
{
    // Flip coin
    // If true we use the last foot node location
    if ((rand() % 100) >= 50)
    {
        cout << "Selecting last foot node as Random Point" << endl;
        return (*lastFootNode).getLocation();
    }
    // If false, we use a randomly selected location
    else
    {
        cout << "Selecting random location as Random Point" << endl;
        return _getRandomLocation();
    }
}

///
/// \brief FootstepPlanner::_getRandomFootLocation
/// \param constraints
/// \param flStanceFoot
/// \param flNewStart
/// \return
///
FootLocation* FootstepPlanner::_getRandomFootLocation(vector<FootConstraint> constraints, vector<Line> obstacles, FootLocation flStanceFoot, Vector2d randomPoint)
{
    // Determine the next foot in our step sequence (Left -> Right or Right -> Left)
    int previousFootIndex = flStanceFoot.getFootIndex();
    int nextFootIndex = (previousFootIndex + 1);
    nextFootIndex = nextFootIndex % _Feet.size();
    cout << "Previous Foot Index: " << previousFootIndex << " Next Foot Index: " << nextFootIndex << endl;

    int iteration = 0;
    // Randomly generate valid foot configuration
    while(iteration < 50)
    {
        FootLocation flFootConfig = _generateRandomFootConfig(previousFootIndex, nextFootIndex, constraints, flStanceFoot, randomPoint);
        cout << "Random Foot Config Success: " << iteration << endl;
        // Check for collision
        if (_isCollision(flFootConfig, flStanceFoot, obstacles))
        {
            cout << "Collision!" << endl;
            iteration++;
        }
        // No collision
        else
        {
            cout << "No collision!" << endl;
            // Return the foot location
            return new FootLocation(flFootConfig.getLocation(), flFootConfig.getTheta(), flFootConfig.getFootIndex(), &_Feet);
        }
    }
    // Return failure
    return NULL;
}

///
/// \brief FootstepPlanner::_updateRandomMinMaxValues
/// \param xValue
/// \param yValue
///
void FootstepPlanner::_updateRandomMinMaxValues(double xValue, double yValue)
{
    // Check for X Maximum
    if (xValue > _maximumRandomX)
        _maximumRandomX = xValue;
    // Check for X Minimum
    if (xValue < _minimumRandomX)
        _minimumRandomX = xValue;
    // Check for Y Maximum
    if (yValue > _maximumRandomY)
        _maximumRandomY = yValue;
    // Check for Y Minimum
    if (yValue < _minimumRandomY)
        _minimumRandomY = yValue;
}

///
/// \brief _generateRandomFootConfig
/// \param previousFootIndex
/// \param nextFootIndex
/// \param flFootConfig
/// \param constraints
/// \param flStanceFoot
/// \return
///
FootLocation FootstepPlanner::_generateRandomFootConfig(int previousFootIndex, int nextFootIndex, const vector<FootConstraint>& constraints, FootLocation flStanceFoot, Vector2d randomPoint)
{
    // Find the corresponding constraint
    const FootConstraint* fc;
    for(int i = 0; i < constraints.size(); i++)
    {
        // Look for the constraint that corresponds to the next foot
        // and that references the previous foot
        if (constraints[i].getFootIndex() == nextFootIndex &&
            constraints[i].getRefFootIndex() == previousFootIndex)
            // Save the pointer to that constraint
            fc = &constraints[i];
    }
    // Get the previous foot location so we know where to start from
    Vector2d minPoint(flStanceFoot.getLocation()[0] + fc->getMinimumDeltaX(),
                      flStanceFoot.getLocation()[1] + fc->getMinimumDeltaY());
    Vector2d maxPoint(flStanceFoot.getLocation()[0] + fc->getMaximumDeltaX(),
                      flStanceFoot.getLocation()[1] + fc->getMaximumDeltaY());

    // Check to see if the random point is within the threshold
    if (randomPoint[0] >= minPoint[0] && randomPoint[0] <= maxPoint[0] &&
        randomPoint[1] >= minPoint[1] && randomPoint[1] <= maxPoint[1])
    {
        // It seems we've found the foot location already
        return FootLocation(randomPoint, 0.0f, nextFootIndex, &_Feet);
    }
    else
    {
        // Generate a random X in the valid range (bias towards randomPoint)
        double dRandX = frand(minPoint[0], maxPoint[0]);
        // Generate a random Y in the valid range (bias towards randomPoint)
        double dRandY = frand(minPoint[1], maxPoint[1]);
        // Generate a random Theta in the valid range
        double dRandTheta = fc->getMinimumDeltaTheta() + frand(0, fc->getMaximumDeltaTheta() - fc->getMaximumDeltaTheta());
        // Initialize the foot location
        return FootLocation(Vector2d(dRandX, dRandY), dRandTheta, nextFootIndex, &_Feet);
    }
}

double FootstepPlanner::frand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

///
/// \brief FootstepPlanner::_isCollision
/// \param flFootConfig
/// \param flStanceFoot
/// \param obstacles
/// \return
///
bool FootstepPlanner::_isCollision(const FootLocation& flFootConfig, const FootLocation& flStanceFoot, vector<Line> obstacles)
{
    // Check for collision with nearest neighbor
    if (flFootConfig.isCollision(flStanceFoot))
        // Collision
        return true;
    // No collision with nearest neighbor
    else
    {
        // Iterate through each of the foot bounds
        for(int i = 0; i < flFootConfig.getBounds().size(); i++)
        {
            // Iterate through each of the obstacle lines
            for(int j = 0; j < obstacles.size(); j++)
            {
                // Check for collision with the obstacle
                if (flFootConfig.getBounds()[i].isCollision(obstacles[j]))
                    // Collision
                    return true;
            }
        }
    }
    // No collision
    return false;
}

///
/// \brief FootstepPlanner::_getRandomLocation
/// \return
///
Vector2d FootstepPlanner::_getRandomLocation()
{
    // Randomize a value from min to max
    return Vector2d(_minimumRandomX + (((double)rand() / RAND_MAX) * (_maximumRandomX - _minimumRandomX)),
                    _minimumRandomY + (((double)rand() / RAND_MAX) * (_maximumRandomY - _minimumRandomY)));
}

///
/// \brief FootstepPlanner::_findNearestNeighbor
/// \param location
/// \param points
/// \return
///
Vector2d FootstepPlanner::_findNearestNeighbor(Vector2d location, flann::Index< flann::L2<double> > points)
{
    // Find the nearest neighbor
    flann::Matrix<double> query(new double[1 * 2], 1, 2);
    flann::Matrix<int> indices(new int[query.rows * query.cols], query.rows, query.cols);
    flann::Matrix<double> dists(new double[query.rows * query.cols], query.rows, query.cols);
    // Set the query point
    query[0][0] = location[0];
    query[0][1] = location[1];
    // Search for the nearest neighbor
    points.knnSearch(query, indices, dists, 1, flann::SearchParams(128));
    // Get the nearest neighbor
    double* neighbor = points.getPoint(indices[0][0]);
    // Initialize the nearest neighbor vector
    return Vector2d(neighbor[0], neighbor[1]);
}

///
/// \brief FootstepPlanner::_findFootLocation
/// \param location
/// \param root
/// \return
///
FootLocationNode* FootstepPlanner::_findFootLocationNode(Vector2d location, FootLocationNode* root)
{
    FootLocationNode* node;
    // Check for a valid value
    if (root != NULL)
    {
        //cout << "Checking Location: " << endl << location << endl
        //     << "Against: " << endl << root->getLocation() << endl;
        // Check for a match on the root
        if (location[0] == root->getLocation()[0] &&
            location[1] == root->getLocation()[1])
            // Return the root node
            return root;
        // No match
        else
        {
            // Let's search the children
            for(int i = 0; i < root->getChildren().size(); i++)
            {
                // Recursively call to find the foot location
                node = _findFootLocationNode(location, root->getChild(i));
                // See if we found a match
                if (node != NULL)
                    // Return the match
                    return node;
            }
            // No match across all children
            return NULL;
        }
    }
    // No valid root
    else
        // No valid match
        return NULL;
}
