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

FootLocationNode::FootLocationNode()
{
    _shouldAvoid = false;
}

FootLocationNode::FootLocationNode(FootLocation& location, vector<Foot>* feet)
{
    _Location = FootLocation(location.getLocation(), location.getWorldTheta(), location.getTheta(), location.getFootIndex(), feet);
    _shouldAvoid = false;
}

FootLocationNode::FootLocationNode(Vector2d location, float worldTheta, float theta, int footIndex, vector<Foot>* feet)
{
    _Location = FootLocation(location, worldTheta, theta, footIndex, feet);
    _shouldAvoid = false;
}

FootLocationNode::~FootLocationNode()
{
    for(int i = 0; i < _Children.size(); i++)
        delete _Children[i];
}

FootLocation FootLocationNode::getFootLocation() const { return _Location; }
Vector2d FootLocationNode::getLocation() const { return _Location.getLocation(); }
float FootLocationNode::getTheta() const { return _Location.getWorldTheta(); }
int FootLocationNode::getFootIndex() const { return _Location.getFootIndex(); }
vector<Line> FootLocationNode::getBounds() const { return _Location.getBounds(); }
vector<FootLocationNode*> FootLocationNode::getChildren() const { return _Children; }
bool FootLocationNode::shouldAvoid() const {return _shouldAvoid;}
bool FootLocationNode::doesPathExist() const {return _doesPathExist;}

FootLocationNode* FootLocationNode::getChild(int index) const
{
    if (_Children.size() > index)
        return _Children[index];
    else
        return NULL;
}

FootLocationNode* FootLocationNode::getParent() const
{
    return _Parent;
}

void FootLocationNode::setParent(FootLocationNode* parent)
{
    _Parent = parent;
}

void FootLocationNode::addChild(FootLocationNode* child)
{
    _Children.push_back(child);
}

void FootLocationNode::addChild(FootLocation& child, vector<Foot>* feet)
{
    // Initialize the node
    FootLocationNode* newNode = new FootLocationNode(child, feet);
    // Set the parent
    newNode->setParent(this);
    // Add the child
    _Children.push_back(newNode);
}

void FootLocationNode::setShouldAvoid(bool shouldAvoid) {
		_shouldAvoid = shouldAvoid;
}

void FootLocationNode::setDoesPathExist(bool doesPathExist) {
		_doesPathExist = doesPathExist;
}
