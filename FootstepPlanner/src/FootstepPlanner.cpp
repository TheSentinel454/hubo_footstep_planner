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
vector<FootLocation> FootstepPlanner::generatePlan(int plannerType, vector<Foot> feet, vector<FootConstraint> constraints, vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles)
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
    _writePlannerOutput(dPlanningTime, plan);

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
vector<FootLocation> FootstepPlanner::runRRTPlanner(vector<Foot> feet, vector<FootConstraint> constraints, vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles)
{
    // Initialize Current Position RRT
    flann::Matrix<double> initialState(new double[currentLocation.size() * 2], currentLocation.size(), 2);
    // Add each of the start locations
    for(int i = 0; i < currentLocation.size(); i++)
    {
        initialState[i][0] = currentLocation[i].getLocation()[0]; // X Position
        initialState[i][1] = currentLocation[i].getLocation()[1]; // Y Position
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
    }
    // Create the index with only the goal state
    flann::Index< flann::L2<double> > goalRRT(goalState, flann::KDTreeIndexParams(4));
    goalRRT.buildIndex();

    // Iterate until we find a path
    /*
    do
    {

    }
    while(true);
    */
}

///
/// \brief FootstepPlanner::getStaticPlan
/// \param feet
/// \return
///
vector<FootLocation> FootstepPlanner::getStaticPlan(vector<Foot> feet)
{
    vector<FootLocation> plan;
    plan.push_back(FootLocation(Vector2d(5.0d, 3.0d), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2d(10.0d, 0.0d), 0.0f, feet[1]));
    plan.push_back(FootLocation(Vector2d(16.0d, 3.0d), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2d(22.0d, 0.0d), 0.0f, feet[1]));
    plan.push_back(FootLocation(Vector2d(29.0d, 3.0d), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2d(36.0d, 0.0d), 0.0f, feet[1]));
    plan.push_back(FootLocation(Vector2d(43.0d, 3.0d), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2d(50.0d, 0.0d), 0.0f, feet[1]));
    plan.push_back(FootLocation(Vector2d(57.0d, 3.0d), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2d(62.0d, 0.0d), 0.0f, feet[1]));
    plan.push_back(FootLocation(Vector2d(67.0d, 3.0d), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2d(67.0d, 0.0d), 0.0f, feet[1]));

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
            myfile << plan[i].getFoot().getName() << endl;
        }
        //myfile << plan << endl;
        myfile.close();
    }
    else cout << "Unable to open file";
}
