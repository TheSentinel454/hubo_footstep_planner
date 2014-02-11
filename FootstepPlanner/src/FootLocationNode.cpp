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

#include "FootLocationNode.h"

using namespace fsp;
using namespace std;
using namespace Eigen;

FootLocationNode::FootLocationNode(){}

FootLocationNode::FootLocationNode(FootLocation location)
{
    _Location = location.getLocation();
    _Theta = location.getTheta();
    _FootIndex = location.getFootIndex();
}

FootLocationNode::FootLocationNode(Vector2d location, float theta, int footIndex)
{
    _Location = location;
    _Theta = theta;
    _FootIndex = footIndex;
}

Vector2d FootLocationNode::getLocation() const { return _Location; }
float FootLocationNode::getTheta() const { return _Theta; }
int FootLocationNode::getFootIndex() const { return _FootIndex; }
vector<FootLocationNode*> FootLocationNode::getChildren() const { return _Children; }

FootLocationNode* FootLocationNode::getChild(int index) const
{
    if (_Children.size() > index)
        return _Children[index];
    else
        return NULL;
}

void FootLocationNode::setParent(FootLocationNode* parent)
{
    _Parent = parent;
}

void FootLocationNode::addChild(FootLocationNode* child)
{
    _Children.push_back(child);
}

void FootLocationNode::addChild(FootLocation child)
{
    FootLocationNode node(child);
    _Children.push_back(&node);
}
