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

#ifndef FOOTCONSTRAINT_H
#define FOOTCONSTRAINT_H

#include "Foot.h"
#include <eigen3/Eigen/Core>
#include "fcl/collision.h"

namespace fsp {

/*!
  * \class FootConstraint
  * \brief A FootConstraint keeps track of a constraint for a particular foot in reference to another foot.
  *
  *
  */
class FootConstraint
{
    public:
        FootConstraint( int footIndex, int refFootIndex,
                        double minDeltaX, double maxDeltaX,
                        double minDeltaY, double maxDeltaY,
                        double minDeltaTheta, double maxDeltaTheta);

        int getFootIndex() const;
        int getRefFootIndex() const;
        double getMinimumDeltaLength() const;
        double getMaximumDeltaLength() const;
        double getMinimumDeltaWidth() const;
        double getMaximumDeltaWidth() const;
        double getMinimumDeltaTheta() const;
        double getMaximumDeltaTheta() const;
    protected:

    private:
        double _MinDeltaLength;
        double _MaxDeltaLength;
        double _MinDeltaWidth;
        double _MaxDeltaWidth;
        double _MinDeltaTheta;
        double _MaxDeltaTheta;
        int _FootIndex;
        int _ReferenceFootIndex;
    };
} // namespace fsp

#endif
