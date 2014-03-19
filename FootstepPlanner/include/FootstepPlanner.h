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
#include "MapSearchNode.h"
#include <eigen3/Eigen/Core>
#include <stdlib.h>
#include <time.h>
#include <flann/flann.hpp>
#include <iostream>
#include <fstream>

namespace fsp {

///
/// \brief PLANNER_TYPE_RRT
///
const int PLANNER_TYPE_RRT = 0;
///
/// \brief PLANNER_TYPE_R_STAR
///
const int PLANNER_TYPE_R_STAR = 1;
///
/// \brief PLANNER_TYPE_A_STAR
///
const int PLANNER_TYPE_A_STAR = 2;

/*!
  * \class Footstep Planner
  * \brief Footstep planner is in charge of taking in a set of parameters, and calculating a plan based on those parameters.
  *
  *
  */
class FootstepPlanner
{
    public:
        static int* ENVIRONMENT_MAP;
        static float DISCRETIZATION_RES;
        static Eigen::Vector2d MIN_POINT;
        static Eigen::Vector2d INV_MIN_POINT;
        static Eigen::Vector2d MAX_POINT;
        static int MAP_WIDTH;
        static int MAP_HEIGHT;

        FootstepPlanner(std::vector<Foot> ft);

        std::vector<FootLocation> generatePlan(int plannerType, std::vector<FootConstraint> constraints, std::vector<FootLocation> currentLocation, std::vector<FootLocation> goalLocation, std::vector<Line> obstacles);
        std::vector<FootLocation> getStaticPlan();

        std::vector<FootLocation> runRRTPlanner(std::vector<FootConstraint> constraints, std::vector<FootLocation> currentLocation, std::vector<FootLocation> goalLocation, std::vector<Line> obstacles);
        std::vector<FootLocation> runAStarPlanner(std::vector<FootConstraint> constraints, std::vector<FootLocation> currentLocation, std::vector<FootLocation> goalLocation, std::vector<Line> obstacles, std::vector<Eigen::Vector2i>& mapPlan);
        std::vector<FootLocation> runRStarPlanner(std::vector<FootConstraint> constraints, std::vector<FootLocation> currentLocation, std::vector<FootLocation> goalLocation, std::vector<Line> obstacles);
    protected:

    private:
        ///
        /// \brief _writePlannerOutput
        /// \param time
        /// \param plan
        ///
        void _writePlannerOutput(double time, std::vector<FootLocation> plan);
        double frand(double fMin, double fMax);

        bool _isValidMapCoord(Eigen::Vector2i coord);
        int* _getEnvironmentMap(vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles);
        float _getDiscretizationResolution(vector<FootConstraint> constraints);
        Eigen::Vector2i _getMapCoord(Eigen::Vector2d worldCoord);
        Eigen::Vector2d _getWorldCoord(Eigen::Vector2i mapCoord);

        Eigen::Vector2d _getNextRandomPoint(FootLocation* lastFootNode);
        Eigen::Vector2d _getRandomLocation();
        FootLocation* _getRandomFootLocation(std::vector<FootConstraint> constraints, std::vector<Line> obstacles, FootLocation flStanceFoot, Eigen::Vector2d randomPoint);
        Eigen::Vector2d _findNearestNeighbor(Eigen::Vector2d location, flann::Index< flann::L2<double> > points);
        FootLocationNode* _findFootLocationNode(Eigen::Vector2d location, fsp::FootLocationNode* root);
        bool _isCollision(const fsp::FootLocation& flFootConfig, const fsp::FootLocation* flStanceFoot, std::vector<Line> obstacles);
        void _updateRandomMinMaxValues(double xValue, double yValue);
        FootLocation _generateRandomFootConfig(int previousFootIndex, int nextFootIndex, const std::vector<FootConstraint>& constraints, fsp::FootLocation flStanceFoot, Eigen::Vector2d randomPoint);
				double get_euclid_distance(FootLocationNode* node, FootLocation footLocation);
				void find_path_using_weighted_a_star(FootLocationNode* parent, FootLocationNode* goal, int desired_weight, vector<Line> obstacles);
				FootLocationNode* get_random_goal(FootLocationNode* currentLocation, int radius, vector<Line> obstacles); 
				vector<FootLocationNode*> get_possible_configurations(FootLocationNode* currentLocation, int radius, vector<Line> obstacles, int num_config);
        double _minimumRandomX;
        double _maximumRandomX;
        double _minimumRandomY;
        double _maximumRandomY;
        std::vector<Foot> _Feet;
};

} // namespace fsp

#endif
