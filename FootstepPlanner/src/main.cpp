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

#include "main.h"
#include "FootstepPlanner.h"

using namespace std;
using namespace osg;
using namespace fsp;
using namespace Eigen;

int main()
{
    //pyramidTest();
    vector<Foot> feet;
    feet.push_back(Foot(2.0f, 4.0f, "Left"));
    feet.push_back(Foot(2.0f, 4.0f, "Right"));

    Vector2f currentLeftPos(0.0f, 3.0f);
    Vector2f currentRightPos(0.0f, 0.0f);
    FootLocation currentLeftFoot(currentLeftPos, 0.0f, feet[0]);
    FootLocation currentRightFoot(currentRightPos, 0.0f, feet[1]);
    vector<FootLocation> currentLoc;
    currentLoc.push_back(currentLeftFoot);
    currentLoc.push_back(currentRightFoot);

    Vector2f goalLeftPos(67.0f, 3.0f);
    Vector2f goalRightPos(67.0f, 0.0f);
    FootLocation goalLeftFoot(goalLeftPos, 0.0f, feet[0]);
    FootLocation goalRightFoot(goalRightPos, 0.0f, feet[1]);
    vector<FootLocation> goalLoc;
    goalLoc.push_back(goalLeftFoot);
    goalLoc.push_back(goalRightFoot);

    vector<FootLocation> plan;
    plan.push_back(FootLocation(Vector2f(5.0f, 3.0f), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2f(10.0f, 0.0f), 0.0f, feet[1]));
    plan.push_back(FootLocation(Vector2f(16.0f, 3.0f), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2f(22.0f, 0.0f), 0.0f, feet[1]));
    plan.push_back(FootLocation(Vector2f(29.0f, 3.0f), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2f(36.0f, 0.0f), 0.0f, feet[1]));
    plan.push_back(FootLocation(Vector2f(43.0f, 3.0f), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2f(50.0f, 0.0f), 0.0f, feet[1]));
    plan.push_back(FootLocation(Vector2f(57.0f, 3.0f), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2f(62.0f, 0.0f), 0.0f, feet[1]));
    plan.push_back(FootLocation(Vector2f(67.0f, 3.0f), 0.0f, feet[0]));
    plan.push_back(FootLocation(Vector2f(67.0f, 0.0f), 0.0f, feet[1]));

    vector<Line> obs;
    // First obstacle
    obs.push_back(Line(Vector2f(-10.0f, 7.0f), Vector2f(55.0f, 18.0f)));
    obs.push_back(Line(Vector2f(55.0f, 18.0f), Vector2f(0.0f, 23.0f)));
    obs.push_back(Line(Vector2f(0.0f, 23.0f), Vector2f(-10.0f, 7.0f)));
    // Second obstacle
    obs.push_back(Line(Vector2f(20.0f, -7.0f), Vector2f(25.0f, -38.0f)));
    obs.push_back(Line(Vector2f(25.0f, -38.0f), Vector2f(25.0f, -50.0f)));
    obs.push_back(Line(Vector2f(25.0f, -50.0f), Vector2f(10.0f, -17.0f)));
    obs.push_back(Line(Vector2f(10.0f, -17.0f), Vector2f(20.0f, -7.0f)));

    FootstepPlanner planner;
    vector<FootLocation> plan2 = planner.generatePlan(PLANNER_TYPE_RRT, feet, currentLoc, goalLoc, obs);
    //visualizePlanUsingTransform(currentLoc, goalLoc, obs, plan);
    visualizePlan(currentLoc, goalLoc, obs, plan);
    return 0;
}

/*
 * Function to get the transform for each of the foot. 
 * Input Params : 
 * Vector<FootLocation> :  To get the location/orientation of the transform to be generated. 
 * Vec4 Color : To signify the color of the transform to be generated. 
 * Output Values :
 * To throw the output transform which would be added to the root. 
 */
osg::PositionAttitudeTransform* getFootTransform(FootLocation location, Vec4 color) 
{
    //Create the geode, and set the location values according to the given location. 
    Geode* currentPositionGeode = new Geode();
    Geometry* currentPositionGeometry = new Geometry();
    Vec4Array* currentPositionColors = new Vec4Array;
    Vec3Array* currentPositionVertices = new Vec3Array;
    
    Vector2f loc = location.getLocation();
    float xOffset = location.getFoot().getLength() / 2;
    float yOffset = location.getFoot().getWidth() / 2;
		float theta = location.getTheta();
    currentPositionVertices->push_back(Vec3(loc[0] - xOffset, loc[1] - yOffset, 0));
    currentPositionVertices->push_back(Vec3(loc[0] + xOffset, loc[1] - yOffset, 0));
    currentPositionVertices->push_back(Vec3(loc[0] + xOffset, loc[1] + yOffset, 0));
    currentPositionVertices->push_back(Vec3(loc[0] - xOffset, loc[1] + yOffset, 0));
    
		// Set the Vertex array
    currentPositionGeometry->setVertexArray(currentPositionVertices);
		// Add foot
		DrawElementsUInt* currentFoot = new DrawElementsUInt(PrimitiveSet::QUADS, 0);
		currentFoot->push_back(0);
		currentFoot->push_back(1);
		currentFoot->push_back(2);
		currentFoot->push_back(3);
		currentPositionGeometry->addPrimitiveSet(currentFoot);
		currentPositionColors->push_back(color);

    currentPositionGeometry->setColorArray(currentPositionColors);
    currentPositionGeometry->setColorBinding(Geometry::BIND_PER_PRIMITIVE_SET);
    currentPositionGeode->addDrawable(currentPositionGeometry);

		//Create the transform, and set the rotation for the given foot location.
    osg::PositionAttitudeTransform* footTransform = new osg::PositionAttitudeTransform();

		footTransform->addChild(currentPositionGeode);
 
    osg::Vec3 footPosition(loc[0], loc[1], 0);
    footTransform->setPosition(footPosition);
    footTransform->setAttitude(osg::Quat(osg::DegreesToRadians(theta), osg::Vec3(0, 0, 1)));

		return footTransform;
}
 
///
/// \fn visualizePla
/// \brief visualizePlan
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \param plan
///
void visualizePlanUsingTransform(vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles, vector<FootLocation> plan)
{
    // Initialize the root
    Group* root = new Group();

    // Initialize colors
    Vec4 blue = Vec4(0.0f, 0.0f, 1.0f, 1.0f);
    Vec4 green = Vec4(0.0f, 1.0f, 0.0f, 1.0f);
    Vec4 red = Vec4(1.0f, 0.0f, 0.0f, 1.0f);
    Vec4 yellow = Vec4(1.0f, 1.0f, 0.0f, 1.0f);
    Vec4 black = Vec4(0.0f, 0.0f, 0.0f, 1.0f);
    Vec4 white = Vec4(1.0f, 1.0f, 1.0f, 1.0f);

		// Add the current position(blue). 
		for(int i = 0; i < currentLocation.size();i++) 
		{
				root->addChild(getFootTransform(currentLocation[i], blue));
		}

    // Add the goal position (green)
		for(int i = 0; i < goalLocation.size();i++) 
		{
				root->addChild(getFootTransform(goalLocation[i], green));
		}

    // Add the obstacles (red)
    Geode* obstacleGeode = new Geode();
    Geometry* obstacleGeometry = new Geometry();
    Vec4Array* obstacleColors = new Vec4Array;
    obstacleColors->push_back(red);

    Vec3Array* obstacleVertices = new Vec3Array;
    for(int i = 0; i < obstacles.size(); i++)
    {
        Line line = obstacles[i];
        Vector2f start = line.getStart();
        Vector2f end = line.getEnd();
        obstacleVertices->push_back(Vec3(start[0], start[1], 0));
        obstacleVertices->push_back(Vec3(end[0], end[1], 0));
    }
    // Set the Vertex array
    obstacleGeometry->setVertexArray(obstacleVertices);
    obstacleGeometry->setColorArray(obstacleColors);
    obstacleGeometry->setColorBinding(Geometry::BIND_OVERALL);

    obstacleGeometry->addPrimitiveSet(new DrawArrays(GL_LINES,0,obstacleVertices->size()));
    obstacleGeode->addDrawable(obstacleGeometry);
    root->addChild(obstacleGeode);

    // Add the steps (yellow)
    for(int i = 0; i < plan.size(); i++)
    {
				root->addChild(getFootTransform(plan[i], yellow));
    }

    // Add the plan outline (black)
    Geode* planOutlineGeode = new Geode();
    Geometry* planOutlineGeometry = new Geometry();
    Vec4Array* planOutlineColors = new Vec4Array;
    planOutlineColors->push_back(black);

    // Add the points for the lines
    Vec3Array* planOutlineVertices = new Vec3Array;
    planOutlineVertices->push_back(Vec3(currentLocation[0].getLocation()[0], currentLocation[0].getLocation()[1], 0));
    for(int i = 0; i < plan.size(); i++)
    {
        FootLocation fl = plan[i];
        Vector2f loc = fl.getLocation();
        planOutlineVertices->push_back(Vec3(loc[0], loc[1], 0));
        planOutlineVertices->push_back(Vec3(loc[0], loc[1], 0));
    }
    // Set the Vertex array
    planOutlineGeometry->setVertexArray(planOutlineVertices);

    // Set the color
    planOutlineGeometry->setColorArray(planOutlineColors);
    planOutlineGeometry->setColorBinding(Geometry::BIND_OVERALL);
    // Add the primitive set
    planOutlineGeometry->addPrimitiveSet(new DrawArrays(GL_LINES,0,planOutlineVertices->size()));
    planOutlineGeode->addDrawable(planOutlineGeometry);
    root->addChild(planOutlineGeode);

    osgViewer::Viewer viewer;
    viewer.setSceneData(root);
    viewer.run();
}
///
/// \fn visualizePlan
/// \brief visualizePlan
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \param plan
///
void visualizePlan(vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles, vector<FootLocation> plan)
{
    // Initialize the root
    Group* root = new Group();

    // Initialize colors
    Vec4 blue = Vec4(0.0f, 0.0f, 1.0f, 1.0f);
    Vec4 green = Vec4(0.0f, 1.0f, 0.0f, 1.0f);
    Vec4 red = Vec4(1.0f, 0.0f, 0.0f, 1.0f);
    Vec4 yellow = Vec4(1.0f, 1.0f, 0.0f, 1.0f);
    Vec4 black = Vec4(0.0f, 0.0f, 0.0f, 1.0f);
    Vec4 white = Vec4(1.0f, 1.0f, 1.0f, 1.0f);

    // Add the current position (blue)
    Geode* currentPositionGeode = new Geode();
    Geometry* currentPositionGeometry = new Geometry();
    Vec4Array* currentPositionColors = new Vec4Array;

    Vec3Array* currentPositionVertices = new Vec3Array;
    for(int i = 0; i < currentLocation.size(); i++)
    {
        FootLocation fl = currentLocation[i];
        Vector2f loc = fl.getLocation();
        float xOffset = fl.getFoot().getLength() / 2;
        float yOffset = fl.getFoot().getWidth() / 2;
        currentPositionVertices->push_back(Vec3(loc[0] - xOffset, loc[1] - yOffset, 0));
        currentPositionVertices->push_back(Vec3(loc[0] + xOffset, loc[1] - yOffset, 0));
        currentPositionVertices->push_back(Vec3(loc[0] + xOffset, loc[1] + yOffset, 0));
        currentPositionVertices->push_back(Vec3(loc[0] - xOffset, loc[1] + yOffset, 0));
    }
    // Set the Vertex array
    currentPositionGeometry->setVertexArray(currentPositionVertices);
    // Add each of the feet
    for(int i = 0, j = 0; i < currentLocation.size(); i++, j+=4)
    {
        // Add foot
        DrawElementsUInt* currentFoot = new DrawElementsUInt(PrimitiveSet::QUADS, 0);
        currentFoot->push_back(j + 0);
        currentFoot->push_back(j + 1);
        currentFoot->push_back(j + 2);
        currentFoot->push_back(j + 3);
        currentPositionGeometry->addPrimitiveSet(currentFoot);
        currentPositionColors->push_back(blue);
    }
    currentPositionGeometry->setColorArray(currentPositionColors);
    currentPositionGeometry->setColorBinding(Geometry::BIND_PER_PRIMITIVE_SET);

    currentPositionGeode->addDrawable(currentPositionGeometry);
    root->addChild(currentPositionGeode);

    // Add the goal position (green)
    Geode* goalPositionGeode = new Geode();
    Geometry* goalPositionGeometry = new Geometry();
    Vec4Array* goalPositionColors = new Vec4Array;

    Vec3Array* goalPositionVertices = new Vec3Array;
    for(int i = 0; i < goalLocation.size(); i++)
    {
        FootLocation fl = goalLocation[i];
        Vector2f loc = fl.getLocation();
        float xOffset = fl.getFoot().getLength() / 2;
        float yOffset = fl.getFoot().getWidth() / 2;
        goalPositionVertices->push_back(Vec3(loc[0] - xOffset, loc[1] - yOffset, 0));
        goalPositionVertices->push_back(Vec3(loc[0] + xOffset, loc[1] - yOffset, 0));
        goalPositionVertices->push_back(Vec3(loc[0] + xOffset, loc[1] + yOffset, 0));
        goalPositionVertices->push_back(Vec3(loc[0] - xOffset, loc[1] + yOffset, 0));
    }
    // Set the Vertex array
    goalPositionGeometry->setVertexArray(goalPositionVertices);
    // Add each of the feet
    for(int i = 0, j = 0; i < goalLocation.size(); i++, j+=4)
    {
        // Add foot
        DrawElementsUInt* goalFoot = new DrawElementsUInt(PrimitiveSet::QUADS, 0);
        goalFoot->push_back(j + 0);
        goalFoot->push_back(j + 1);
        goalFoot->push_back(j + 2);
        goalFoot->push_back(j + 3);
        goalPositionGeometry->addPrimitiveSet(goalFoot);
        goalPositionColors->push_back(green);
    }
    goalPositionGeometry->setColorArray(goalPositionColors);
    goalPositionGeometry->setColorBinding(Geometry::BIND_PER_PRIMITIVE_SET);

    goalPositionGeode->addDrawable(goalPositionGeometry);
    root->addChild(goalPositionGeode);

    // Add the obstacles (red)
    Geode* obstacleGeode = new Geode();
    Geometry* obstacleGeometry = new Geometry();
    Vec4Array* obstacleColors = new Vec4Array;
    obstacleColors->push_back(red);

    Vec3Array* obstacleVertices = new Vec3Array;
    for(int i = 0; i < obstacles.size(); i++)
    {
        Line line = obstacles[i];
        Vector2f start = line.getStart();
        Vector2f end = line.getEnd();
        obstacleVertices->push_back(Vec3(start[0], start[1], 0));
        obstacleVertices->push_back(Vec3(end[0], end[1], 0));
    }
    /*
    for(int i = 0; i < obstacles.size(); i++)
    {
        FootLocation fl = plan[i];
        Vector2f loc = fl.getLocation();
        float xOffset = fl.getFoot().getLength() / 2;
        float yOffset = fl.getFoot().getWidth() / 2;
        planPositionVertices->push_back(Vec3(loc[0] - xOffset, loc[1] - yOffset, 0));
        planPositionVertices->push_back(Vec3(loc[0] + xOffset, loc[1] - yOffset, 0));
        planPositionVertices->push_back(Vec3(loc[0] + xOffset, loc[1] + yOffset, 0));
        planPositionVertices->push_back(Vec3(loc[0] - xOffset, loc[1] + yOffset, 0));
    }
    // Set the Vertex array
    obstacleGeometry->setVertexArray(planPositionVertices);
    // Add each of the lines
    DrawElementsUInt* obstacle = new DrawElementsUInt(PrimitiveSet::LINE_LOOP, 0);
    for(int i = 0; i < obstacles.size(); i++)
    {
        // Add line
        obstacle->push_back(i);
    }
    obstacleGeometry->addPrimitiveSet(obstacle);
    */
    // Set the Vertex array
    obstacleGeometry->setVertexArray(obstacleVertices);
    obstacleGeometry->setColorArray(obstacleColors);
    obstacleGeometry->setColorBinding(Geometry::BIND_OVERALL);

    obstacleGeometry->addPrimitiveSet(new DrawArrays(GL_LINES,0,obstacleVertices->size()));
    obstacleGeode->addDrawable(obstacleGeometry);
    root->addChild(obstacleGeode);


    // Add the steps (yellow)
    Geode* planPositionGeode = new Geode();
    Geometry* planPositionGeometry = new Geometry();
    Vec4Array* planPositionColors = new Vec4Array;

    Vec3Array* planPositionVertices = new Vec3Array;
    for(int i = 0; i < plan.size(); i++)
    {
        FootLocation fl = plan[i];
        Vector2f loc = fl.getLocation();
        float xOffset = fl.getFoot().getLength() / 2;
        float yOffset = fl.getFoot().getWidth() / 2;
        planPositionVertices->push_back(Vec3(loc[0] - xOffset, loc[1] - yOffset, 0));
        planPositionVertices->push_back(Vec3(loc[0] + xOffset, loc[1] - yOffset, 0));
        planPositionVertices->push_back(Vec3(loc[0] + xOffset, loc[1] + yOffset, 0));
        planPositionVertices->push_back(Vec3(loc[0] - xOffset, loc[1] + yOffset, 0));
    }
    // Set the Vertex array
    planPositionGeometry->setVertexArray(planPositionVertices);
    // Add each of the feet
    for(int i = 0, j = 0; i < plan.size(); i++, j+=4)
    {
        // Add foot
        DrawElementsUInt* currentFoot = new DrawElementsUInt(PrimitiveSet::QUADS, 0);
        currentFoot->push_back(j + 0);
        currentFoot->push_back(j + 1);
        currentFoot->push_back(j + 2);
        currentFoot->push_back(j + 3);
        planPositionGeometry->addPrimitiveSet(currentFoot);
        planPositionColors->push_back(yellow);
    }
    planPositionGeometry->setColorArray(planPositionColors);
    planPositionGeometry->setColorBinding(Geometry::BIND_PER_PRIMITIVE_SET);

    planPositionGeode->addDrawable(planPositionGeometry);
    root->addChild(planPositionGeode);

    // Add the plan outline (black)
    Geode* planOutlineGeode = new Geode();
    Geometry* planOutlineGeometry = new Geometry();
    Vec4Array* planOutlineColors = new Vec4Array;
    planOutlineColors->push_back(black);

    // Add the points for the lines
    Vec3Array* planOutlineVertices = new Vec3Array;
    planOutlineVertices->push_back(Vec3(currentLocation[0].getLocation()[0], currentLocation[0].getLocation()[1], 0));
    for(int i = 0; i < plan.size(); i++)
    {
        FootLocation fl = plan[i];
        Vector2f loc = fl.getLocation();
        planOutlineVertices->push_back(Vec3(loc[0], loc[1], 0));
        planOutlineVertices->push_back(Vec3(loc[0], loc[1], 0));
    }
    // Set the Vertex array
    planOutlineGeometry->setVertexArray(planOutlineVertices);

    // Set the color
    planOutlineGeometry->setColorArray(planOutlineColors);
    planOutlineGeometry->setColorBinding(Geometry::BIND_OVERALL);
    // Add the primitive set
    planOutlineGeometry->addPrimitiveSet(new DrawArrays(GL_LINES,0,planOutlineVertices->size()));
    planOutlineGeode->addDrawable(planOutlineGeometry);
    root->addChild(planOutlineGeode);

    osgViewer::Viewer viewer;
    viewer.setSceneData(root);
    viewer.run();
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

