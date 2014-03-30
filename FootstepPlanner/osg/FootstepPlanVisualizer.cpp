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

#include "FootstepPlanVisualizer.h"

//#define START_TILE
//#define GOAL_TILE
#define MAP_TILES
#define OBSTACLE_TILES
#define MAP_GRID
#define MAP_LINES
#define OBSTACLES
#define FOOTSTEPS
#define GOAL_LOCATION
#define START_LOCATION


using namespace osg;
using namespace fsp;
using namespace std;
using namespace Eigen;

///
/// \brief FootstepPlanVisualizer::FootstepPlanVisualizer
/// \param ft
///
FootstepPlanVisualizer::FootstepPlanVisualizer(vector<Foot> ft)
{
    _Feet = ft;
}

///
/// \brief getBoxObstacle
/// \param center
/// \param lengthX
/// \param lengthY
/// \param lengthZ
/// \param theta
/// \return
///
osg::PositionAttitudeTransform* FootstepPlanVisualizer::_getBoxObstacle(Vec3 center, float lengthX, float lengthY, float lengthZ, float theta)
{
    Geode* boxGeode = new Geode();

    osg::Box* boxPointer = new osg::Box(center, lengthX, lengthY, lengthZ);
    osg::ShapeDrawable* boxShape = new osg::ShapeDrawable(boxPointer);
    boxGeode->addDrawable(boxShape);

    osg::PositionAttitudeTransform* boxTransform = new osg::PositionAttitudeTransform();
    boxTransform->addChild(boxGeode);

    osg::Vec3 boxPosition(0, 0, 0);
    boxTransform->setPosition(boxPosition);
    boxTransform->setAttitude(osg::Quat(osg::DegreesToRadians(theta), osg::Vec3(0, 0, 1)));

    return boxTransform;
}

///
/// \fn getObstacle
/// \brief Get the obstacle geode..
/// \param obstacles - vector containing all the lines as we got them.
/// \return
///
Geode* FootstepPlanVisualizer::_getObstacle(vector<Line> obstacles)
{
    // Add the obstacles (red)
    Vec4 red = Vec4(1.0f, 0.0f, 0.0f, 1.0f);
    Geode* obstacleGeode = new Geode();
    Geometry* obstacleGeometry = new Geometry();
    Vec4Array* obstacleColors = new Vec4Array;
    obstacleColors->push_back(red);

    Vec3Array* obstacleVertices = new Vec3Array;
    for(int i = 0; i < obstacles.size(); i++)
    {
        Line line = obstacles[i];
        Vector2d start = line.getStart();
        Vector2d end = line.getEnd();
        obstacleVertices->push_back(Vec3(start[0], start[1], 0));
        obstacleVertices->push_back(Vec3(end[0], end[1], 0));
        obstacleVertices->push_back(Vec3(end[0], end[1], 5));
        obstacleVertices->push_back(Vec3(start[0], start[1], 5));
    }
    // Set the Vertex array
    obstacleGeometry->setVertexArray(obstacleVertices);
    for(int i = 0, j = 0; i < obstacles.size(); i++, j+=4)
    {
        // Add foot
        DrawElementsUInt* obstacle = new DrawElementsUInt(PrimitiveSet::QUADS, 0);
        obstacle->push_back(j + 0);
        obstacle->push_back(j + 1);
        obstacle->push_back(j + 2);
        obstacle->push_back(j + 3);
        obstacleGeometry->addPrimitiveSet(obstacle);
        obstacleColors->push_back(red);
    }
    obstacleGeometry->setColorArray(obstacleColors);
    obstacleGeometry->setColorBinding(Geometry::BIND_PER_PRIMITIVE_SET);
    obstacleGeode->addDrawable(obstacleGeometry);
        return obstacleGeode;
}

///
/// \brief getFootTransform - Function to get the transform for each of the foot.
/// \param location - To get the location/orientation of the transform to be generated.
/// \param color - To signify the color of the transform to be generated.
/// \return - To throw the output transform which would be added to the root.
///
PositionAttitudeTransform* FootstepPlanVisualizer::_getFootTransform(FootLocation location, Vec4 color)
{
    //Create the geode, and set the location values according to the given location.
    Geode* currentPositionGeode = new Geode();
    Geometry* currentPositionGeometry = new Geometry();
    Vec4Array* currentPositionColors = new Vec4Array;
    Vec3Array* currentPositionVertices = new Vec3Array;

    Vector2d loc = location.getLocation();
    double xOffset = _Feet[location.getFootIndex()].getLength() / 2;
    double yOffset = _Feet[location.getFootIndex()].getWidth() / 2;
    float theta = location.getWorldTheta();
    currentPositionVertices->push_back(Vec3(-xOffset, -yOffset, 0));
    currentPositionVertices->push_back(Vec3(xOffset, -yOffset, 0));
    currentPositionVertices->push_back(Vec3(xOffset, yOffset, 0));
    currentPositionVertices->push_back(Vec3(-xOffset, yOffset, 0));

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

    Vec3 footPosition(loc[0], loc[1], 0);
    footTransform->setPosition(footPosition);
    footTransform->setAttitude(osg::Quat(osg::DegreesToRadians(theta), osg::Vec3(0, 0, 1)));

    return footTransform;
}

///
/// \fn visualizePlan
/// \brief visualizePlan
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \param plan
///
void FootstepPlanVisualizer::visualizePlanUsingTransform(vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles, vector<FootLocation> plan)
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
        root->addChild(_getFootTransform(currentLocation[i], blue));
    }

    // Add the goal position (green)
    for(int i = 0; i < goalLocation.size();i++)
    {
        root->addChild(_getFootTransform(goalLocation[i], green));
    }

    // Add the obstacles (red)
    root->addChild(_getObstacle(obstacles));

    // Add the steps (yellow)
    for(int i = 0; i < plan.size(); i++)
    {
        root->addChild(_getFootTransform(plan[i], yellow));
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
        Vector2d loc = plan[i].getLocation();
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


void FootstepPlanVisualizer::visualizePlan2(Vector2d minPoint, Vector2d maxPoint, float discretizationResolution, vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles, vector<FootLocation> plan, vector<Vector2i> mapPlan)
{
    // Initialize the root
    Group* root = new Group();

    // Initialize colors
    Vec4 blue = Vec4(0.0f, 0.0f, 1.0f, 1.0f);
    Vec4 opaque_blue = Vec4(0.0f, 0.0f, 1.0f, 0.3f);
    Vec4 green = Vec4(0.0f, 1.0f, 0.0f, 1.0f);
    Vec4 opaque_green = Vec4(0.0f, 1.0f, 0.0f, 0.3f);
    Vec4 red = Vec4(1.0f, 0.0f, 0.0f, 1.0f);
    Vec4 opaque_red = Vec4(1.0f, 0.0f, 0.0f, 0.3f);
    Vec4 yellow = Vec4(1.0f, 1.0f, 0.0f, 1.0f);
    Vec4 opaque_yellow = Vec4(1.0f, 1.0f, 0.0f, 0.3f);
    Vec4 black = Vec4(0.0f, 0.0f, 0.0f, 1.0f);
    Vec4 gray = Vec4(0.5f, 0.5f, 0.5f, 1.0f);
    Vec4 white = Vec4(1.0f, 1.0f, 1.0f, 1.0f);

#ifdef START_LOCATION
    // Add the current position(blue).
    for(int i = 0; i < currentLocation.size();i++)
    {
        root->addChild(_getFootTransform(currentLocation[i], blue));
    }
#endif

#ifdef GOAL_LOCATION
    // Add the goal position (green)
    for(int i = 0; i < goalLocation.size();i++)
    {
        root->addChild(_getFootTransform(goalLocation[i], green));
    }
#endif

#ifdef OBSTACLES
    // Add the obstacles (red)
    root->addChild(_getObstacle(obstacles));
#endif

#ifdef FOOTSTEPS
    // Add the steps (yellow)
    for(int i = 0; i < plan.size(); i++)
    {
        root->addChild(_getFootTransform(plan[i], (i % 2 ? white : gray)));
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
        Vector2d loc = plan[i].getLocation();
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
#endif

#ifdef MAP_GRID
    // Add the grid (black)
    Geode* gridGeode = new Geode();
    Geometry* gridGeometry = new Geometry();
    Vec4Array* gridColors = new Vec4Array;
    gridColors->push_back(white);

    // Add the points for the lines
    Vec3Array* gridVertices = new Vec3Array;
    for(float x = minPoint[0]; x <= maxPoint[0]; x += discretizationResolution)
    {
        gridVertices->push_back(Vec3(x, minPoint[1], 0));
        gridVertices->push_back(Vec3(x, maxPoint[1], 0));
    }
    for(float y = minPoint[1]; y <= maxPoint[1]; y += discretizationResolution)
    {
        gridVertices->push_back(Vec3(minPoint[0], y, 0));
        gridVertices->push_back(Vec3(maxPoint[0], y, 0));
    }
    // Set the Vertex array
    gridGeometry->setVertexArray(gridVertices);

    // Set the color
    gridGeometry->setColorArray(gridColors);
    gridGeometry->setColorBinding(Geometry::BIND_OVERALL);
    // Add the primitive set
    gridGeometry->addPrimitiveSet(new DrawArrays(GL_LINES,0,gridVertices->size()));
    gridGeode->addDrawable(gridGeometry);
    root->addChild(gridGeode);
#endif

    Vector2d invMinPoint(-minPoint[0], -minPoint[1]);

#ifdef START_TILE
    // Add the Start tile
    root->addChild(_getTileFromWorld(currentLocation[0].getLocation()[0],
                                     currentLocation[0].getLocation()[1],
                                     discretizationResolution,
                                     invMinPoint,
                                     opaque_blue));
#endif
#ifdef GOAL_TILE
    // Add the Goal tile
    root->addChild(_getTileFromWorld(goalLocation[0].getLocation()[0],
                                     goalLocation[0].getLocation()[1],
                                     discretizationResolution,
                                     invMinPoint,
                                     opaque_green));
#endif

#ifdef OBSTACLE_TILES
    // Go through each of the obstacles
    for(int i = 0; i < obstacles.size(); i++)
    {
        // Add the start tile
        root->addChild(_getTileFromWorld(obstacles[i].getStart()[0],
                                         obstacles[i].getStart()[1],
                                         discretizationResolution, invMinPoint, opaque_red));
        // Get slope
        float slopeRise = (obstacles[i].getEnd()[1] - obstacles[i].getStart()[1]);
        float slopeRun = (obstacles[i].getEnd()[0] - obstacles[i].getStart()[0]);
        // Iterate across the slope and add the obstacle points
        for(float j = 0.00f; j <= 1.00f; j+= 0.01f)
        {
            root->addChild(_getTileFromWorld(obstacles[i].getStart()[0] + (slopeRun * j),
                                             obstacles[i].getStart()[1] + (slopeRise * j),
                                             discretizationResolution, invMinPoint, opaque_red));
        }
        // Add the end tile
        root->addChild(_getTileFromWorld(obstacles[i].getEnd()[0],
                                         obstacles[i].getEnd()[1],
                                         discretizationResolution, invMinPoint, opaque_red));
    }
#endif

#ifdef MAP_TILES
    // Go through the map Plan
    for(int i = 0; i < mapPlan.size(); i++)
    {
        // Add the plan tile
        root->addChild(_getTileFromMap(mapPlan[i][0],
                                       mapPlan[i][1],
                                       discretizationResolution, invMinPoint, opaque_yellow));
    }
#endif

#ifdef MAP_LINES
    int prevDirection = 0;
    // 0 = Moving Right
    // 1 = Moving Left
    // 2 = Moving Up
    // 3 = Moving Down
    vector<int> directions;
    // 0 = Continue Right
    // 1 = Continue Left
    // 2 = Continue Up
    // 3 = Continue Down
    // 4 = Turn Left
    // 5 = Turn Right
    // Now we need to generate the foot step path
    for(int i = 0; i < (mapPlan.size() - 1); i++)
    {
        // Make sure we have the next location
        if ((i + 1) < mapPlan.size())
        {
            // Check for X increasing (Moving Right)
            if (mapPlan[i][0] < mapPlan[i+1][0])
            {
                switch(prevDirection)
                {
                case 0:
                    directions.push_back(0);
                    cout << "Continue Right" << endl;
                    break;
                case 2:
                    directions.push_back(5);
                    cout << "Turning Right" << endl;
                    break;
                case 3:
                    directions.push_back(4);
                    cout << "Turning Left" << endl;
                    break;
                case 1:
                default:
                    cout << "WTF?!?!" << endl;
                    break;
                }
                prevDirection = 0;
            }
            // Check for X decreasing (Moving Left)
            if (mapPlan[i][0] > mapPlan[i+1][0])
            {
                switch(prevDirection)
                {
                case 1:
                    directions.push_back(1);
                    cout << "Continue Left" << endl;
                    break;
                case 2:
                    directions.push_back(4);
                    cout << "Turning Left" << endl;
                    break;
                case 3:
                    directions.push_back(5);
                    cout << "Turning Right" << endl;
                    break;
                case 0:
                default:
                    cout << "WTF?!?!" << endl;
                    break;
                }
                prevDirection = 1;
            }
            // Check for Y increasing (Moving Up)
            if (mapPlan[i][1] < mapPlan[i+1][1])
            {
                switch(prevDirection)
                {
                case 2:
                    directions.push_back(2);
                    cout << "Continue Up" << endl;
                    break;
                case 0:
                    directions.push_back(4);
                    cout << "Turning Left" << endl;
                    break;
                case 1:
                    directions.push_back(5);
                    cout << "Turning Right" << endl;
                    break;
                case 3:
                default:
                    cout << "WTF?!?!" << endl;
                    break;
                }
                prevDirection = 2;
            }
            // Check for Y decreasing (Moving Down)
            if (mapPlan[i][1] > mapPlan[i+1][1])
            {
                switch(prevDirection)
                {
                case 3:
                    directions.push_back(3);
                    cout << "Continue Down" << endl;
                    break;
                case 1:
                    directions.push_back(4);
                    cout << "Turning Left" << endl;
                    break;
                case 0:
                    directions.push_back(5);
                    cout << "Turning Right" << endl;
                    break;
                case 2:
                default:
                    cout << "WTF?!?!" << endl;
                    break;
                }
                prevDirection = 3;
            }
        }
    }
    cout << "MapPlan Size: " << mapPlan.size() << endl;
    cout << "Directions Size: " << directions.size() << endl;

    // Add the map lines (black)
    Geode* mapGeode = new Geode();
    Geometry* mapGeometry = new Geometry();
    Vec4Array* mapColors = new Vec4Array;
    mapColors->push_back(black);
    Vec3Array* mapVertices = new Vec3Array;
    float previousDesiredTheta = 0.0f;
    float desiredTheta;
    int x, y;
    for(int a = 0; a < mapPlan.size(); a++)
    {
        Vector2d worldCoord = _getWorldCoord(mapPlan[a], invMinPoint, discretizationResolution);
        if (a < directions.size())
        {
            switch(directions[a])
            {
            case 1: // Continue Left
                cout << "Continue Left" << endl;
                desiredTheta = 180.0f;
                mapVertices->push_back(Vec3(worldCoord[0], worldCoord[1] + (discretizationResolution/2.0d), 0));
                mapVertices->push_back(Vec3(worldCoord[0] + discretizationResolution, worldCoord[1] + (discretizationResolution/2.0d), 0));
                break;
            case 2: // Continue Up
                cout << "Continue Up" << endl;
                desiredTheta = 90.0f;
                mapVertices->push_back(Vec3(worldCoord[0] + (discretizationResolution/2.0d), worldCoord[1], 0));
                mapVertices->push_back(Vec3(worldCoord[0] + (discretizationResolution/2.0d), worldCoord[1] + discretizationResolution, 0));
                break;
            case 3: // Continue Down
                cout << "Continue Down" << endl;
                desiredTheta = 270.0f;
                mapVertices->push_back(Vec3(worldCoord[0] + (discretizationResolution/2.0d), worldCoord[1], 0));
                mapVertices->push_back(Vec3(worldCoord[0] + (discretizationResolution/2.0d), worldCoord[1] + discretizationResolution, 0));
                break;
            case 4: // Turn Left
                cout << "Turn Left" << endl;
                desiredTheta = ((int)previousDesiredTheta + 90) % 360;
                // First step
                if (mapPlan[a-1][0] > mapPlan[a][0])
                    // Decrease X
                    x = -1;
                else if (mapPlan[a-1][0] < mapPlan[a][0])
                    // Increase X
                    x = 1;
                else if (mapPlan[a-1][1] < mapPlan[a][1])
                    // Increase Y
                    y = 1;
                else if (mapPlan[a-1][1] > mapPlan[a][1])
                    // Decrease Y
                    y = -1;
                // Second step
                if (mapPlan[a][0] > mapPlan[a+1][0])
                    // Decrease X
                    x = -1;
                else if (mapPlan[a][0] < mapPlan[a+1][0])
                    // Increase X
                    x = 1;
                else if (mapPlan[a][1] < mapPlan[a+1][1])
                    // Increase Y
                    y = 1;
                else if (mapPlan[a][1] > mapPlan[a+1][1])
                    // Decrease Y
                    y = -1;
                if (x < 0)
                {
                    // Decrease X
                    if (y < 0)
                    {
                        // Decrease Y
                        mapVertices->push_back(Vec3(worldCoord[0] + discretizationResolution, worldCoord[1] + (discretizationResolution/2.0d), 0));
                        mapVertices->push_back(Vec3(worldCoord[0] + (discretizationResolution/2.0d), worldCoord[1], 0));
                    }
                    else if (y > 0)
                    {
                        // Increase Y
                        mapVertices->push_back(Vec3(worldCoord[0] + (discretizationResolution/2.0d), worldCoord[1], 0));
                        mapVertices->push_back(Vec3(worldCoord[0], worldCoord[1] + (discretizationResolution/2.0d), 0));
                    }
                }
                else if (x > 0)
                {
                    // Increase X
                    if (y < 0)
                    {
                        // Decrease Y
                        mapVertices->push_back(Vec3(worldCoord[0] + discretizationResolution/2.0d, worldCoord[1] + discretizationResolution, 0));
                        mapVertices->push_back(Vec3(worldCoord[0] + discretizationResolution, worldCoord[1] + discretizationResolution/2.0d, 0));
                    }
                    else if (y > 0)
                    {
                        // Increase Y
                        mapVertices->push_back(Vec3(worldCoord[0], worldCoord[1] + (discretizationResolution/2.0d), 0));
                        mapVertices->push_back(Vec3(worldCoord[0] + (discretizationResolution/2.0d), worldCoord[1] + discretizationResolution, 0));
                    }
                }
                break;
            case 5: // Turn Right
                cout << "Turn Right" << endl;
                desiredTheta = ((int)previousDesiredTheta - 90 + 360) % 360;
                // First step
                if (mapPlan[a-1][0] > mapPlan[a][0])
                    // Decrease X
                    x = -1;
                else if (mapPlan[a-1][0] < mapPlan[a][0])
                    // Increase X
                    x = 1;
                else if (mapPlan[a-1][1] < mapPlan[a][1])
                    // Increase Y
                    y = 1;
                else if (mapPlan[a-1][1] > mapPlan[a][1])
                    // Decrease Y
                    y = -1;
                // Second step
                if (mapPlan[a][0] > mapPlan[a+1][0])
                    // Decrease X
                    x = -1;
                else if (mapPlan[a][0] < mapPlan[a+1][0])
                    // Increase X
                    x = 1;
                else if (mapPlan[a][1] < mapPlan[a+1][1])
                    // Increase Y
                    y = 1;
                else if (mapPlan[a][1] > mapPlan[a+1][1])
                    // Decrease Y
                    y = -1;
                if (x < 0)
                {
                    // Decrease X
                    if (y < 0)
                    {
                        // Decrease Y
                        mapVertices->push_back(Vec3(worldCoord[0] + (discretizationResolution/2.0d), worldCoord[1] + discretizationResolution, 0));
                        mapVertices->push_back(Vec3(worldCoord[0], worldCoord[1] + (discretizationResolution/2.0d), 0));
                    }
                    else if (y > 0)
                    {
                        // Increase Y
                        mapVertices->push_back(Vec3(worldCoord[0] + discretizationResolution, worldCoord[1] + discretizationResolution/2.0d, 0));
                        mapVertices->push_back(Vec3(worldCoord[0] + discretizationResolution/2.0d, worldCoord[1] + discretizationResolution, 0));
                    }
                }
                else if (x > 0)
                {
                    // Increase X
                    if (y < 0)
                    {
                        // Decrease Y
                        mapVertices->push_back(Vec3(worldCoord[0], worldCoord[1] + (discretizationResolution/2.0d), 0));
                        mapVertices->push_back(Vec3(worldCoord[0] + (discretizationResolution/2.0d), worldCoord[1], 0));
                    }
                    else if (y > 0)
                    {
                        // Increase Y
                        mapVertices->push_back(Vec3(worldCoord[0] + (discretizationResolution/2.0d), worldCoord[1], 0));
                        mapVertices->push_back(Vec3(worldCoord[0] + discretizationResolution, worldCoord[1] + (discretizationResolution/2.0d), 0));
                    }
                }
                break;
            case 0: // Continue Right
            default:
                cout << "Continue Right" << endl;
                desiredTheta = 0.0f;
                mapVertices->push_back(Vec3(worldCoord[0], worldCoord[1] + (discretizationResolution/2.0d), 0));
                mapVertices->push_back(Vec3(worldCoord[0] + discretizationResolution, worldCoord[1] + (discretizationResolution/2.0d), 0));
                break;
            }
        }
    }
    // Set the Vertex array
    mapGeometry->setVertexArray(mapVertices);

    // Set the color
    mapGeometry->setColorArray(mapColors);
    mapGeometry->setColorBinding(Geometry::BIND_OVERALL);
    // Add the primitive set
    mapGeometry->addPrimitiveSet(new DrawArrays(GL_LINES,0,mapVertices->size()));
    mapGeode->addDrawable(mapGeometry);
    root->addChild(mapGeode);
#endif

    osgViewer::Viewer viewer;
    viewer.setSceneData(root);
    viewer.run();
}


Vector2i FootstepPlanVisualizer::_getMapCoord(Vector2d worldCoord, Vector2d invMinPoint, float discretizationResolution)
{
    return Vector2i(floor((worldCoord[0] + invMinPoint[0]) / discretizationResolution), floor((worldCoord[1] + invMinPoint[1]) / discretizationResolution));
}

Vector2d FootstepPlanVisualizer::_getWorldCoord(Vector2i mapCoord, Vector2d invMinPoint, float discretizationResolution)
{
    return Vector2d((mapCoord[0] * discretizationResolution) - invMinPoint[0],
                    (mapCoord[1] * discretizationResolution) - invMinPoint[1]);
}

///
/// \brief FootstepPlanVisualizer::_getTileFromWorld
/// \param worldX
/// \param worldY
/// \param discretizationResolution
/// \param color
/// \return
///
Geode* FootstepPlanVisualizer::_getTileFromWorld(float worldX, float worldY, float discretizationResolution, Vector2d invMinPoint, Vec4 color)
{
    return _getTileFromMap(floor((worldX + invMinPoint[0]) / discretizationResolution),
                           floor((worldY + invMinPoint[1]) / discretizationResolution),
                           discretizationResolution, invMinPoint, color);
}

///
/// \brief FootstepPlanVisualizer::_getTileFromMap
/// \param mapX
/// \param mapY
/// \param discretizationResolution
/// \param color
/// \return
///
Geode* FootstepPlanVisualizer::_getTileFromMap(float mapX, float mapY, float discretizationResolution, Vector2d invMinPoint, Vec4 color)
{
    Vec3 drawPos((mapX * discretizationResolution - invMinPoint[0]) + discretizationResolution/2.0f,
                 (mapY * discretizationResolution - invMinPoint[1]) + discretizationResolution/2.0f, -0.25f);

    Geode* boxGeode = new Geode();
    osg::Box* boxPointer = new osg::Box(drawPos, discretizationResolution, discretizationResolution, 0);
    osg::ShapeDrawable* boxShape = new osg::ShapeDrawable(boxPointer);
    boxShape->setColor(color);
    boxGeode->addDrawable(boxShape);
    return boxGeode;
}

///
/// \fn visualizePlan
/// \brief visualizePlan
/// \param currentLocation
/// \param goalLocation
/// \param obstacles
/// \param plan
///
void FootstepPlanVisualizer::visualizePlan(vector<FootLocation> currentLocation, vector<FootLocation> goalLocation, vector<Line> obstacles, vector<FootLocation> plan)
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
        Vector2d loc = fl.getLocation();
        double xOffset = _Feet[fl.getFootIndex()].getLength() / 2;
        double yOffset = _Feet[fl.getFootIndex()].getWidth() / 2;
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
        Vector2d loc = fl.getLocation();
        double xOffset = _Feet[fl.getFootIndex()].getLength() / 2;
        double yOffset = _Feet[fl.getFootIndex()].getWidth() / 2;
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
        Vector2d start = line.getStart();
        Vector2d end = line.getEnd();
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
    Geode* planPositionGeode = new Geode();
    Geometry* planPositionGeometry = new Geometry();
    Vec4Array* planPositionColors = new Vec4Array;

    Vec3Array* planPositionVertices = new Vec3Array;
    for(int i = 0; i < plan.size(); i++)
    {
        FootLocation fl = plan[i];
        Vector2d loc = fl.getLocation();
        double xOffset = _Feet[fl.getFootIndex()].getLength() / 2;
        double yOffset = _Feet[fl.getFootIndex()].getWidth() / 2;
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
        planPositionColors->push_back((i % 2 ? white : black));
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
        Vector2d loc = fl.getLocation();
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
