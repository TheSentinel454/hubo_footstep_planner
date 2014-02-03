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

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/PositionAttitudeTransform>
#include <osg/LineSegment>
#include <osgViewer/Viewer>
#include <osg/LineWidth>
#include <osg/MatrixTransform>

#include <iostream>

#include "../include/main.h"
#include "../include/Foot.h"
#include "../include/FootLocation.h"

using namespace std;
using namespace osg;
using namespace fsp;
using namespace Eigen;

int main()
{
    //pyramidTest();x
    Foot leftFoot(2.0f, 4.0f, "Left");
    Foot rightFoot(2.0f, 4.0f, "Right");
    Foot feet[2] = {leftFoot, rightFoot};

    cout << leftFoot.getName() << ": " << leftFoot.getWidth() << "x" << leftFoot.getLength() << endl;
    cout << rightFoot.getName() << ": " << rightFoot.getWidth() << "x" << rightFoot.getLength() << endl;

    Vector2f currentLeftPos(0.0f, 2.0f);
    Vector2f currentRightPos(0.0f, 0.0f);
    FootLocation currentLeftFoot(currentLeftPos, 0.0f, leftFoot);
    FootLocation currentRightFoot(currentRightPos, 0.0f, rightFoot);
    FootLocation currentLoc[2] = {currentLeftFoot, currentRightFoot};

    Vector2f goalLeftPos(67.0f, 85.0f);
    Vector2f goalRightPos(67.0f, 83.0f);
    FootLocation goalLeftFoot(goalLeftPos, 0.0f, leftFoot);
    FootLocation goalRightFoot(goalRightPos, 0.0f, rightFoot);
    FootLocation goalLoc[2] = {goalLeftFoot, goalRightFoot};


    visualizePlan(currentLoc, goalLoc, NULL, NULL);//obs, plan);
    return 0;
}

///
/// \fn visualizePla
/// \brief visualizePlan
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \param plan
///
void visualizePlan(FootLocation currentLocation[], FootLocation goalLocation[], Line obstacles[], FootLocation plan[])
{

}

void pyramidTest()
{
    Group* root = new Group();
    Geode* pyramidGeode = new Geode();
    Geometry* pyramidGeometry = new Geometry();

    pyramidGeode->addDrawable(pyramidGeometry);
    root->addChild(pyramidGeode);

    Vec3Array* pyramidVertices = new Vec3Array;
    pyramidVertices->push_back(Vec3(0,0,0));
    pyramidVertices->push_back(Vec3(10,0,0));
    pyramidVertices->push_back(Vec3(10,10,0));
    pyramidVertices->push_back(Vec3(0,10,0));
    pyramidVertices->push_back(Vec3(5,5,10));

    pyramidGeometry->setVertexArray(pyramidVertices);
    DrawElementsUInt* pyramidBase = new DrawElementsUInt(PrimitiveSet::QUADS, 0);
    pyramidBase->push_back(3);
    pyramidBase->push_back(2);
    pyramidBase->push_back(1);
    pyramidBase->push_back(0);
    pyramidGeometry->addPrimitiveSet(pyramidBase);

    DrawElementsUInt* pyramidFaceOne = new DrawElementsUInt(PrimitiveSet::TRIANGLES, 0);
    pyramidFaceOne->push_back(0);
    pyramidFaceOne->push_back(1);
    pyramidFaceOne->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceOne);

    DrawElementsUInt* pyramidFaceTwo = new DrawElementsUInt(PrimitiveSet::TRIANGLES, 0);
    pyramidFaceTwo->push_back(1);
    pyramidFaceTwo->push_back(2);
    pyramidFaceTwo->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceTwo);

    DrawElementsUInt* pyramidFaceThree = new DrawElementsUInt(PrimitiveSet::TRIANGLES, 0);
    pyramidFaceThree->push_back(2);
    pyramidFaceThree->push_back(3);
    pyramidFaceThree->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceThree);

    DrawElementsUInt* pyramidFaceFour = new DrawElementsUInt(PrimitiveSet::TRIANGLES, 0);
    pyramidFaceFour->push_back(3);
    pyramidFaceFour->push_back(0);
    pyramidFaceFour->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceFour);

    Vec4Array* colors = new Vec4Array;
    colors->push_back(Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back(Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back(Vec4(0.0f, 0.0f, 1.0f, 1.0f));
    colors->push_back(Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    colors->push_back(Vec4(1.0f, 0.0f, 0.0f, 1.0f));

    pyramidGeometry->setColorArray(colors);
    pyramidGeometry->setColorBinding(Geometry::BIND_PER_VERTEX);

//    osg::PositionAttitudeTransform* pyramidTwoXForm =
//            new osg::PositionAttitudeTransform();

//    root->addChild(pyramidTwoXForm);
//    pyramidTwoXForm->addChild(pyramidGeode);

//    osg::Vec3 pyramidTwoPosition(15, 0, 0);
//    pyramidTwoXForm->setPosition(pyramidTwoPosition);

    osgViewer::Viewer viewer;
    viewer.setSceneData(root);
    viewer.run();
}

