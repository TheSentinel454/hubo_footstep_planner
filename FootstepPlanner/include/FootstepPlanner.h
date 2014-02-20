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

#ifndef FOOTSTEPPLANNER_H
#define FOOTSTEPPLANNER_H

#include <string>
#include <vector>
#include "FootLocation.h"
#include "FootLocationNode.h"
#include "Line.h"
#include "FootConstraint.h"
#include <eigen3/Eigen/Core>
#include <stdlib.h>
#include <time.h>
#include <flann/flann.hpp>
#include <iostream>
#include <fstream>
#include <boost/graph/adjacency_list.hpp>

namespace fsp {

///
/// \brief PLANNER_TYPE_RRT
///
const int PLANNER_TYPE_RRT = 0;
///
/// \brief PLANNER_TYPE_R_STAR
///
const int PLANNER_TYPE_R_STAR = 1;

/*!
  * \class Footstep Planner
  * \brief Footstep planner is in charge of taking in a set of parameters, and calculating a plan based on those parameters.
  *
  *
  */
class FootstepPlanner
{
    public:
        FootstepPlanner(std::vector<Foot> ft);

        std::vector<FootLocation> generatePlan(int plannerType, std::vector<FootConstraint> constraints, std::vector<FootLocation> currentLocation, std::vector<FootLocation> goalLocation, std::vector<Line> obstacles);
        std::vector<FootLocation> getStaticPlan();

        std::vector<FootLocation> runRRTPlanner(std::vector<FootConstraint> constraints, std::vector<FootLocation> currentLocation, std::vector<FootLocation> goalLocation, std::vector<Line> obstacles);
        std::vector<FootLocation> runRStarPlanner(std::vector<FootConstraint> constraints, std::vector<FootLocation> currentLocation, std::vector<FootLocation> goalLocation, std::vector<Line> obstacles);
    protected:

    private:
        ///
        /// \brief _writePlannerOutput
        /// \param time
        /// \param plan
        ///
        void _writePlannerOutput(double time, std::vector<FootLocation> plan);
        Eigen::Vector2d _getNextRandomPoint(FootLocation* lastFootNode);
        Eigen::Vector2d _getRandomLocation();
        bool _getRandomFootLocation(std::vector<FootConstraint> constraints, std::vector<Line> obstacles, FootLocation flNearestNeighbor, Eigen::Vector2d randomPoint, FootLocation* flNewStart);
        Eigen::Vector2d _findNearestNeighbor(Eigen::Vector2d location, flann::Index< flann::L2<double> > points);
        FootLocationNode* _findFootLocationNode(Eigen::Vector2d location, fsp::FootLocationNode* root);
        bool _isCollision(fsp::FootLocation flFootConfig, fsp::FootLocation flNearestNeighbor, std::vector<Line> obstacles);
        void _updateRandomMinMaxValues(double xValue, double yValue);
        bool _generateRandomFootConfig(int previousFootIndex, int nextFootIndex, fsp::FootLocation* flFootConfig, std::vector<FootConstraint> constraints, fsp::FootLocation flNearestNeighbor, Eigen::Vector2d randomPoint);
        double _minimumRandomX;
        double _maximumRandomX;
        double _minimumRandomY;
        double _maximumRandomY;
        std::vector<Foot> _Feet;
};

} // namespace fsp

#endif
