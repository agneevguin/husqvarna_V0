/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_planning/sunrise/sunrise_planner.h"

namespace Husqvarna
{

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// SunrisePlanner class
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
bool SunrisePlanner::getPathFromPoly(am_planning::PathFromPolygon::Request& req,
                                     am_planning::PathFromPolygon::Response& res)
{
    ROS_INFO("Sending Path back...");
    return true;
}

SunrisePlanner::SunrisePlanner(const ros::NodeHandle& nodeh)
{
    // Init attributes
    this->nh = nodeh;

    // Setup some ROS stuff
    covService = nh.advertiseService("get_basic_coverage", &SunrisePlanner::getPathFromPoly, this);
    ROS_INFO("Service (Sunrise version) /get_basic_coverage registered.");

    // Parameters
    ros::NodeHandle n_private("~");
}

SunrisePlanner::~SunrisePlanner()
{
}

bool SunrisePlanner::setup()
{
    ROS_INFO("SunrisePlanner::setup()");

    // Add initial rows
    std::set<Cell, RowCellComparator> bottomRow;
    std::set<Cell, RowCellComparator> middleRow;
    std::set<Cell, RowCellComparator> upperRow;
    rows.push_back(bottomRow);
    rows.push_back(middleRow);
    rows.push_back(upperRow);

    return true;
}

bool SunrisePlanner::update(ros::Duration dt)
{
    return true;
}
}
