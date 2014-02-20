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

#include "main.h"

using namespace std;
using namespace fsp;
using namespace Eigen;

///
/// \brief main
/// \return
///
int main()
{
    // Initialize random seed
    srand(time(NULL));

    // Initialize the feet
    //vector<Foot> feet;
    FEET.push_back(Foot(2.0f, 4.0f, "Left"));
    FEET.push_back(Foot(2.0f, 4.0f, "Right"));

    // Initialize the foot constraints
    vector<FootConstraint> constraints;
    constraints.push_back(FootConstraint(0, 1, 2.0d, 4.0d, -4.0d, 4.0d, -25.0d, 30.0d));
    constraints.push_back(FootConstraint(1, 0, 2.0d, 4.0d, -4.0d, 4.0d, -25.0d, 30.0d));

    // Initialize the current location
    vector<FootLocation> currentLoc;
    currentLoc.push_back(FootLocation(Vector2d(0.0d, 3.0d), 0.0f, 0, &FEET));
    currentLoc.push_back(FootLocation(Vector2d(0.0d, 0.0d), 0.0f, 1, &FEET));

    // Initialize the goal location
    vector<FootLocation> goalLoc;
    goalLoc.push_back(FootLocation(Vector2d(67.0d, 3.0d), 0.0f, 0, &FEET));
    goalLoc.push_back(FootLocation(Vector2d(67.0d, 0.0d), 0.0f, 1, &FEET));

    // Initialize the obstacles
    vector<Line> obs;
    // First obstacle
    obs.push_back(Line(Vector2d(-10.0d, 7.0d), Vector2d(55.0d, 18.0d)));
    obs.push_back(Line(Vector2d(55.0d, 18.0d), Vector2d(0.0d, 23.0d)));
    obs.push_back(Line(Vector2d(0.0d, 23.0d), Vector2d(-10.0d, 7.0d)));
    // Second obstacle
    obs.push_back(Line(Vector2d(20.0d, -7.0d), Vector2d(25.0d, -38.0d)));
    obs.push_back(Line(Vector2d(25.0d, -38.0d), Vector2d(25.0d, -50.0d)));
    obs.push_back(Line(Vector2d(25.0d, -50.0d), Vector2d(10.0d, -17.0d)));
    obs.push_back(Line(Vector2d(10.0d, -17.0d), Vector2d(20.0d, -7.0d)));

    // Initialize the planner
    FootstepPlanner planner(FEET);
    //vector<FootLocation> plan = planner.getStaticPlan();
    vector<FootLocation> plan2 = planner.generatePlan(PLANNER_TYPE_RRT, constraints, currentLoc, goalLoc, obs);
    //vector<FootLocation> plan3 = planner.generatePlan(PLANNER_TYPE_R_STAR, constraints, currentLoc, goalLoc, obs);

    // Initialize the visualizer
    FootstepPlanVisualizer visualizer(FEET);
    visualizer.visualizePlanUsingTransform(currentLoc, goalLoc, obs, plan2);
    //visualizePlan(currentLoc, goalLoc, obs, plan);
    return 0;
}
