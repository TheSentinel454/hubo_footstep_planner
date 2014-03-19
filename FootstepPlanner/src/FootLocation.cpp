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

#include "FootLocation.h"

using namespace fsp;
using namespace Eigen;
using namespace std;

///
/// \brief FootLocation::FootLocation
///
FootLocation::FootLocation()
{
    _Location = Vector2d(0.0d, 0.0d);
    _WorldTheta = 0.0f;
    _FootIndex = -1;
}

///
/// \brief FootLocation::FootLocation
/// \param location
/// \param theta
/// \param footIndex
/// \param feet
///
FootLocation::FootLocation(Vector2d location, float worldTheta, float theta, int footIndex, vector<Foot>* feet)
{
    _Location = location;
    _Theta = theta;
    _WorldTheta = worldTheta;
    _FootIndex = footIndex;
    float yOffset = (*feet)[footIndex].getWidth() / 2;
    float xOffset = (*feet)[footIndex].getLength() / 2;

    // Set a point of 2,1
    Vector2d p1(xOffset, yOffset);
    Vector2d p2(-xOffset, yOffset);
    Vector2d p3(-xOffset, -yOffset);
    Vector2d p4(xOffset, -yOffset);
    // Calculate the Transformation
    Transform<double,2,Affine> t0 = Translation<double,2>(location) *
                                    Rotation2Dd(worldTheta * (M_PI / 180.0d));
    // Calculate the bound points
    p1 = (t0.linear() * p1) + t0.translation();
    p2 = (t0.linear() * p2) + t0.translation();
    p3 = (t0.linear() * p3) + t0.translation();
    p4 = (t0.linear() * p4) + t0.translation();
    // Add the bounds for the foot
    _Bounds.push_back(Line(p1, p2));
    _Bounds.push_back(Line(p2, p3));
    _Bounds.push_back(Line(p3, p4));
    _Bounds.push_back(Line(p4, p1));
}

///
/// \brief FootLocation::isCollision
/// \param location
/// \return
///
bool FootLocation::isCollision(const fsp::FootLocation& location) const
{
    // Iterate across this object's bounds
    for(int i = 0; i < _Bounds.size(); i++)
    {
        // Iterate across the passed in object's bounds
        for(int j = 0; j < location.getBounds().size(); j++)
        {
            // Check for collision
            if (_Bounds[i].isCollision(location.getBounds()[j]))
                // Collision
                return true;
        }
    }
    // No collision
    return false;
}

///
/// \brief FootLocation::getLocation
/// \return
///
Vector2d FootLocation::getLocation() const { return _Location; }
///
/// \brief FootLocation::getWorldTheta
/// \return
///
float FootLocation::getWorldTheta() const { return _WorldTheta; }
///
/// \brief FootLocation::getTheta
/// \return
///
float FootLocation::getTheta() const { return _Theta; }
///
/// \brief FootLocation::getFootIndex
/// \return
///
int FootLocation::getFootIndex() const { return _FootIndex; }
///
/// \brief FootLocation::getBounds
/// \return
///
vector<Line> FootLocation::getBounds() const { return _Bounds; }
