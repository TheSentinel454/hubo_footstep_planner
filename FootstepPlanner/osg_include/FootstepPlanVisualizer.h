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

#ifndef FOOTSTEPPLANVISUALIZER_H
#define FOOTSTEPPLANVISUALIZER_H

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/LineSegment>
#include <osgViewer/Viewer>
#include <osg/LineWidth>
#include <osg/MatrixTransform>

#include <eigen3/Eigen/Core>

#include "FootLocation.h"
#include "Foot.h"
#include "Line.h"

namespace fsp {

/*!
  * \class FootstepPlanVisualizer
  * \brief A Footstep Plan Visualizer helps in visualizing a footstep plan
  *
  *
  */
class FootstepPlanVisualizer
{
    public:
        FootstepPlanVisualizer(std::vector<fsp::Foot> ft);

        void visualizePlan(std::vector<fsp::FootLocation>, std::vector<fsp::FootLocation>, std::vector<fsp::Line>, std::vector<fsp::FootLocation>);
        void visualizePlanUsingTransform(std::vector<fsp::FootLocation>, std::vector<fsp::FootLocation>, std::vector<fsp::Line>, std::vector<fsp::FootLocation>);
        void visualizePlan2(Eigen::Vector2d minPoint, Eigen::Vector2d maxPoint, float discretizationResolution, std::vector<fsp::FootLocation>, std::vector<fsp::FootLocation>, std::vector<fsp::Line>, std::vector<fsp::FootLocation>, std::vector<Eigen::Vector2i>);

    protected:

    private:
        osg::PositionAttitudeTransform* _getBoxObstacle(osg::Vec3 center, float lengthX, float lengthY, float lengthZ, float theta);
        osg::Geode* _getObstacle(std::vector<fsp::Line> obstacles);
        osg::PositionAttitudeTransform* _getFootTransform(fsp::FootLocation location, osg::Vec4 color);
        osg::Geode* _getTileFromMap(float mapX, float mapY, float discretizationResolution, Eigen::Vector2d invMinPoint, osg::Vec4 color);
        osg::Geode* _getTileFromWorld(float worldX, float worldY, float discretizationResolution, Eigen::Vector2d invMinPoint, osg::Vec4 color);
        Eigen::Vector2i _getMapCoord(Eigen::Vector2d worldCoord, Eigen::Vector2d invMinPoint, float discretizationResolution);
        Eigen::Vector2d _getWorldCoord(Eigen::Vector2i mapCoord, Eigen::Vector2d invMinPoint, float discretizationResolution);

        std::vector<Foot> _Feet;
    };
} // namespace fsp

#endif
