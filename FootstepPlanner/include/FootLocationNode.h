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

#ifndef FOOTLOCATIONNODE_H
#define FOOTLOCATIONNODE_H

#include <eigen3/Eigen/Core>
#include <vector>

#include "FootLocation.h"

namespace fsp {

/*!
  * \class FootLocationNode
  * \brief A FootLocationNode keeps track of the foot location, angle, and a reference to a foot.
  * \brief shouldAvoid parameter tells the planner if it should avoid this node or not.  
  *
  */
class FootLocationNode
{
    public:
        FootLocationNode();
        FootLocationNode(fsp::FootLocation& location, std::vector<Foot>* feet);
        FootLocationNode(Eigen::Vector2d location, float worldTheta, float theta, int footIndex, std::vector<Foot>* feet);
        ~FootLocationNode();

        FootLocation getFootLocation() const;
        Eigen::Vector2d getLocation() const;
        float getTheta() const;
        int getFootIndex() const;
        std::vector<Line> getBounds() const;
        std::vector<FootLocationNode*> getChildren() const;
        FootLocationNode* getChild(int index) const;
        FootLocationNode* getParent() const;
        void setParent(FootLocationNode* parent);
        void addChild(FootLocationNode* child);
        void addChild(FootLocation& child, std::vector<Foot>* feet);
				bool shouldAvoid() const;
				void setShouldAvoid(bool shouldAvoid);
				bool doesPathExist() const;
				void setDoesPathExist(bool doesPathExist); 
    protected:

    private:
        fsp::FootLocation _Location;
        FootLocationNode* _Parent;
        std::vector<FootLocationNode*> _Children;
				bool _shouldAvoid;
				bool _doesPathExist;
    };
} // namespace fsp


#endif
