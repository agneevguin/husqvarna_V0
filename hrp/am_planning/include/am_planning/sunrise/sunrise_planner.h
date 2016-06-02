/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#ifndef BASIC_COVERAGE_H
#define BASIC_COVERAGE_H

#include <ros/ros.h>
#include <am_planning/PathFromPolygon.h>

#include <am_planning/sunrise/cell.h>
#include <am_planning/sunrise/cell_comparators.h>

#include <set>
#include <deque>

namespace Husqvarna
{

class SunrisePlanner
{
public:
    SunrisePlanner(const ros::NodeHandle& nodeh);
    ~SunrisePlanner();

    bool setup();
    bool update(ros::Duration dt);

private:
    bool getPathFromPoly(am_planning::PathFromPolygon::Request& req, am_planning::PathFromPolygon::Response& res);

    // ROS data
    ros::NodeHandle nh;
    ros::ServiceServer covService;
    
    // Algorithm containers
    std::deque<std::set<Cell, RowCellComparator> > rows;
    std::set<Cell, StaticCellComparator> visitedCells;
    std::set<Cell, StaticCellComparator> specialCells;
    
};

typedef boost::shared_ptr<SunrisePlanner> SunrisePlannerPtr;
}

#endif
