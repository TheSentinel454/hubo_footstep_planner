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

#include "Line.h"

using namespace fsp;

///
/// \brief Line::Line
/// \param start
/// \param end
///
Line::Line(Eigen::Vector2d start, Eigen::Vector2d end)
{
    _Start = start;
    _End = end;
}

///
/// \brief Line::getStart
/// \return
///
Eigen::Vector2d Line::getStart() const { return _Start; }

///
/// \brief Line::getEnd
/// \return
///
Eigen::Vector2d Line::getEnd() const { return _End; }

///
/// \brief Line::isCollision
/// \param line
/// \return
///
bool Line::isCollision(Line line) const
{
    // Calculate values for collision detection
    float denominator = ((_End[0] - _Start[0]) * (line.getEnd()[1] - line.getStart()[1])) - ((_End[1] - _Start[1]) * (line.getEnd()[0] - line.getStart()[0]));
    float numerator1 = ((_Start[1] - line.getStart()[1]) * (line.getEnd()[0] - line.getStart()[0])) - ((_Start[0] - line.getStart()[0]) * (line.getEnd()[1] - line.getStart()[1]));
    float numerator2 = ((_Start[1] - line.getStart()[1]) * (_End[0] - _Start[0])) - ((_Start[0] - line.getStart()[0]) * (_End[1] - _Start[1]));

    // Detect coincident lines
    // TODO: I'm getting false positives here when the lines don't overlap...
    //if (denominator == 0) return numerator1 == 0 && numerator2 == 0;

    float r = numerator1 / denominator;
    float s = numerator2 / denominator;

    return (r >= 0 && r <= 1) && (s >= 0 && s <= 1);
}
