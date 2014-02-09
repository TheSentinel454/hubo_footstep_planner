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

using namespace fsp;
using namespace std;
using namespace Eigen;
using namespace flann;

///
/// \brief FootstepPlanner::FootstepPlanner
///
FootstepPlanner::FootstepPlanner(){}

///
/// \brief FootstepPlanner::generatePlan
/// \param plannerType
/// \param feet
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \return
///
vector<FootLocation> FootstepPlanner::generatePlan(int plannerType, const vector<Foot> feet, vector<FootConstraint> constraints, vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles)
{
    // Take a starting time stamp
    clock_t tStart = clock();
    // Initialize plan
    vector<FootLocation> plan;
    switch(plannerType)
    {
        case PLANNER_TYPE_RRT:
        default:
            plan = runRRTPlanner(feet, constraints, currentLocation, goalLocation, obstacles);
            break;
    }
    // Take the ending time stamp
    clock_t tEnd = clock();

    // Calculate the planning time
    double dPlanningTime = (double)((tEnd - tStart)/CLOCKS_PER_SEC);

    // Save the planning data
    _writePlannerOutput(dPlanningTime, feet, plan);

    // Return the plan
    return plan;
}

///
/// \brief FootstepPlanner::runRRTPlanner
/// \param feet
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \return
///
vector<FootLocation> FootstepPlanner::runRRTPlanner(const vector<Foot> feet, vector<FootConstraint> constraints, vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles)
{
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

    FootLocationNode footLocationRoot;

    FootLocation* lastStartNode = &currentLocation[0];
    FootLocation* lastGoalNode = &goalLocation[0];
    // Iterate until we find a path
    do
    {
        // First let's grow the start RRT
        // Get the Random Point we want to grow towards
        // We are going to flip a coin to see if we use the goal RRT's
        // latest point, or we use a randomly generated point
        Vector2d vRand = _getNextRandomPoint(lastGoalNode);

        // Find the nearest neighbor in the start RRT
        Vector2d vNearestNeighbor = _findNearestNeighbor(vRand, startRRT);

        // Find the corresponding Foot Location object
        FootLocation flNearestNeighbor = _findFootLocation(vNearestNeighbor, footLocationRoot);

        // Randomly generate foot location configuration (Collision detection is done when generating the foot)
        FootLocation* flNewStart;
        if (_getRandomFootLocation(feet, constraints, obstacles, flNearestNeighbor, flNewStart))
        {
            // Add to the start RRT
            flann::Matrix<double> mNewPoint(new double[1 * 2], 1, 2);
            mNewPoint[0][0] = (*flNewStart).getLocation()[0];   // x coordinate of new point
            mNewPoint[0][1] = (*flNewStart).getLocation()[1];   // y coordinate of new point
            startRRT.addPoints(mNewPoint);

            // Save the last Start Node
            lastStartNode = flNewStart;
        }

        // Now let's grow the goal RRT

        // Get the Random Point we want to grow towards
        // Same as above, coin flipping time

        // Find the nearest neighbor in the goal RRT

        // Determine the next foot in our step sequence (Left -> Right or Right -> Left)

        // Randomly generate foot location configuration

        // Check for collision (possibly regenerate)

        // Add to the goal RRT

        // Check for goal/start RRT connectivity

    }
    while(true);
}

///
/// \brief FootstepPlanner::getStaticPlan
/// \param feet
/// \return
///
vector<FootLocation> FootstepPlanner::getStaticPlan()
{
    vector<FootLocation> plan;
    plan.push_back(FootLocation(Vector2d(5.0d, 3.0d), 0.0f, 0));
    plan.push_back(FootLocation(Vector2d(10.0d, 0.0d), 0.0f, 1));
    plan.push_back(FootLocation(Vector2d(16.0d, 3.0d), 0.0f, 0));
    plan.push_back(FootLocation(Vector2d(22.0d, 0.0d), 0.0f, 1));
    plan.push_back(FootLocation(Vector2d(29.0d, 3.0d), 0.0f, 0));
    plan.push_back(FootLocation(Vector2d(36.0d, 0.0d), 0.0f, 1));
    plan.push_back(FootLocation(Vector2d(43.0d, 3.0d), 0.0f, 0));
    plan.push_back(FootLocation(Vector2d(50.0d, 0.0d), 0.0f, 1));
    plan.push_back(FootLocation(Vector2d(57.0d, 3.0d), 0.0f, 0));
    plan.push_back(FootLocation(Vector2d(62.0d, 0.0d), 0.0f, 1));
    plan.push_back(FootLocation(Vector2d(67.0d, 3.0d), 0.0f, 0));
    plan.push_back(FootLocation(Vector2d(67.0d, 0.0d), 0.0f, 1));

    return plan;
}

///
/// \brief FootstepPlanner::writePlannerOutput
/// \param time
/// \param plan
///
void FootstepPlanner::_writePlannerOutput(double time, const vector<Foot> feet, vector<FootLocation> plan)
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
            myfile << feet[plan[i].getFootIndex()].getName() << endl;
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
        return (*lastFootNode).getLocation();
    // If false, we use a randomly selected location
    else
        return _getRandomLocation();
}

///
/// \brief FootstepPlanner::_getRandomFootLocation
/// \param constraints
/// \param flNearestNeighbor
/// \param flNewStart
/// \return
///
bool FootstepPlanner::_getRandomFootLocation(const vector<Foot> feet, vector<FootConstraint> constraints, vector<Line> obstacles, FootLocation flNearestNeighbor, FootLocation* flNewStart)
{
    // Determine the next foot in our step sequence (Left -> Right or Right -> Left)
    int previousFootIndex = flNearestNeighbor.getFootIndex();
    int nextFootIndex = (previousFootIndex + 1);
    nextFootIndex = nextFootIndex % feet.size();

    FootLocation* flFootConfig;
    int iteration = 0;
    // Randomly generate valid foot configuration
    while(_generateRandomFootConfig(previousFootIndex, nextFootIndex, flFootConfig, constraints, flNearestNeighbor) &&
          iteration < 50)
    {
        // Check for collision
        if (_isCollision(*flFootConfig, flNearestNeighbor, obstacles))
            iteration++;
        // No collision
        else
        {
            // Initialize the new footlocation
            flNewStart = flFootConfig;
            // Good to go
            return true;
        }
    }
    // Return failure
    return false;
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
/// \param flNearestNeighbor
/// \return
///
bool FootstepPlanner::_generateRandomFootConfig(int previousFootIndex, int nextFootIndex, FootLocation* flFootConfig, vector<FootConstraint> constraints, FootLocation flNearestNeighbor)
{

    return true;
}

///
/// \brief _isCollision
/// \param flFootConfig
/// \param flNearestNeighbor
/// \param obstacles
/// \return
///
bool FootstepPlanner::_isCollision(FootLocation flFootConfig, FootLocation flNearestNeighbor, vector<Line> obstacles)
{
    return false;
}

///
/// \brief FootstepPlanner::_getRandomLocation
/// \return
///
Vector2d FootstepPlanner::_getRandomLocation()
{
    //double f = (double)rand() / RAND_MAX;
    //return fMin + f * (fMax - fMin);
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
FootLocation FootstepPlanner::_findFootLocation(Vector2d location, FootLocationNode root)
{

}

