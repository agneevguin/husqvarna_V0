/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_covmap/covmap.h"

#include <tf/transform_datatypes.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

namespace Husqvarna
{

#define VERY_LARGE_MAP
//#define LARGE_MAP

#define ROUND(num) ((int)(num < 0 ? (num - 0.5) : (num + 0.5)))

#define AM_MAP_WIDTH (2000)
#define AM_MAP_HEIGHT (2000)

#ifdef VERY_LARGE_MAP
#define AM_MAP_SCALE_X (25.0)
#define AM_MAP_SCALE_Y (25.0)
#elif LARGE_MAP
#define AM_MAP_SCALE_X (50.0)
#define AM_MAP_SCALE_Y (50.0)
#else
#define AM_MAP_SCALE_X (100.0)
#define AM_MAP_SCALE_Y (100.0)
#endif

static const unsigned char AM_LM_NO_INFORMATION = 255;
static const unsigned char AM_LM_LETHAL_OBSTACLE = 254;
static const unsigned char AM_LM_INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char AM_LM_FREE_SPACE = 0;

CovMap::CovMap(const ros::NodeHandle& nodeh)
{
    // Init attributes
    this->nh = nodeh;

    // Setup some ROS stuff
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

    // Allocate the map
    mapData = new int8_t[AM_MAP_WIDTH * AM_MAP_HEIGHT];
    memset(mapData, AM_LM_FREE_SPACE, sizeof(mapData));

    // Setup the header & info statically...
    theHeader.frame_id = "map";
    theInfo.width = AM_MAP_WIDTH;
    theInfo.height = AM_MAP_HEIGHT;
#ifdef VERY_LARGE_MAP
    theInfo.resolution = 0.04;
    theInfo.origin.position.x = -40.0;
    theInfo.origin.position.y = -40.0;
#elif LARGE_MAP
    theInfo.resolution = 0.02;
    theInfo.origin.position.x = -20.0;
    theInfo.origin.position.y = -20.0;
#else
    theInfo.resolution = 0.01;
    theInfo.origin.position.x = -10.0;
    theInfo.origin.position.y = -10.0;
#endif
    theInfo.origin.position.z = 0.0;

    // Parameters
    ros::NodeHandle n_private("~");

#ifdef VERY_LARGE_MAP
    double def_stdDev = 1.0;
    double def_offset = 0.0;
    double def_scaleFactor = 1.0;
#elif LARGE_MAP
    double def_stdDev = 2.5;
    double def_offset = 0.0;
    double def_scaleFactor = 4.0;
#else
    double def_stdDev = 9.0;
    double def_offset = 0.0;
    double def_scaleFactor = 15.0;
#endif
    n_private.param("standard_deviation", stdDev, def_stdDev);
    ROS_INFO("Standard deviation: %f", stdDev);

    n_private.param("offset", offset, def_offset);
    ROS_INFO("Offset: %f", offset);

    n_private.param("scale_factor", scaleFactor, def_scaleFactor);
    ROS_INFO("Scale factor: %f", scaleFactor);

    // Create kernels...
    createGaussianFilterVS(gKernelVS, stdDev);
    createGaussianFilterL(gKernelL, stdDev);
    createGaussianFilterS(gKernelS, stdDev);
}

CovMap::~CovMap()
{
    delete[] mapData;
}

bool CovMap::setup()
{
    ROS_INFO("CovMap::setup()");

    return true;
}

void CovMap::updateGrassCut()
{
    tf::StampedTransform transform;
    try
    {
        // CUTTING DISC
        listener.lookupTransform("/map", "/cutting_disc_center", ros::Time(0), transform);

        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();

        updateMap(x, y, 1.0);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

double CovMap::mapFilter(double newValue, double oldValue)
{
    double res = newValue + oldValue;

    // Limit to what is nice for ROS map data [0...100]
    if (res < 0.0)
    {
        res = 0;
    }
    if (res > 100)
    {
        res = 100;
    }
    return res;
}

void CovMap::updateMap(double x, double y, double probValue)
{

    // From costmap_2d
    // 0		Free space (not used/seen yet)
    // 1...127	Non-Free (but no collision)
    // 128..252 Possibly collision
    // 253..255 Definitely collision

    // Calculate map coords
    int mx = ROUND(x * AM_MAP_SCALE_X) + AM_MAP_WIDTH / 2;
    int my = ROUND(y * AM_MAP_SCALE_Y) + AM_MAP_HEIGHT / 2;

    updateProbaInMap(mx, my, probValue * 100.0);
}

void CovMap::updateProbaInMap(int mx, int my, double p)
{
#ifdef VERY_LARGE_MAP
    // Apply a gaussian to this
    double pMap[11][11];

    for (int i = 0; i < 11; ++i)
    {
        for (int j = 0; j < 11; ++j)
        {
            pMap[i][j] = p * scaleFactor * gKernelVS[i][j] - offset;
        }
    }

    // Update in map...
    for (int x = -5; x <= 5; x++)
    {
        for (int y = -5; y <= 5; y++)
        {
            int xx = mx + x;
            int yy = my + y;

            // Check bounds...
            if ((xx >= 0) && (xx < AM_MAP_WIDTH))
            {
                if ((yy >= 0) && (yy < AM_MAP_HEIGHT))
                {
                    double pv = pMap[x + 5][y + 5];
                    mapData[xx + yy * AM_MAP_WIDTH] = (int)mapFilter(pv, (double)mapData[xx + yy * AM_MAP_WIDTH]);
                }
            }
        }
    }

#elif LARGE_MAP
    // Apply a gaussian to this
    double pMap[21][21];

    for (int i = 0; i < 21; ++i)
    {
        for (int j = 0; j < 21; ++j)
        {
            pMap[i][j] = p * scaleFactor * gKernelS[i][j] - offset;
        }
    }

    // Update in map...
    for (int x = -10; x <= 10; x++)
    {
        for (int y = -10; y <= 10; y++)
        {
            int xx = mx + x;
            int yy = my + y;

            // Check bounds...
            if ((xx >= 0) && (xx < AM_MAP_WIDTH))
            {
                if ((yy >= 0) && (yy < AM_MAP_HEIGHT))
                {
                    double pv = pMap[x + 10][y + 10];
                    mapData[xx + yy * AM_MAP_WIDTH] = (int)mapFilter(pv, (double)mapData[xx + yy * AM_MAP_WIDTH]);
                }
            }
        }
    }

#else

    // Apply a gaussian to this
    double pMap[41][41];

    for (int i = 0; i < 41; ++i)
    {
        for (int j = 0; j < 41; ++j)
        {
            pMap[i][j] = p * scaleFactor * gKernelL[i][j] - offset;
        }
    }

    // Update in map...
    for (int x = -20; x <= 20; x++)
    {
        for (int y = -20; y <= 20; y++)
        {
            int xx = mx + x;
            int yy = my + y;

            // Check bounds...
            if ((xx >= 0) && (xx < AM_MAP_WIDTH))
            {
                if ((yy >= 0) && (yy < AM_MAP_HEIGHT))
                {
                    double pv = pMap[x + 20][y + 20];
                    mapData[xx + yy * AM_MAP_WIDTH] = (int)mapFilter(pv, (double)mapData[xx + yy * AM_MAP_WIDTH]);
                }
            }
        }
    }
#endif
}

bool CovMap::update(ros::Duration dt)
{
    ros::Time current_time = ros::Time::now();
    theHeader.stamp = current_time;

    // Calculate the TF from the pose...
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.125));
    tf::Quaternion qyaw = tf::createQuaternionFromYaw(0.0);
    transform.setRotation(qyaw);

    // Send the TF
    br.sendTransform(tf::StampedTransform(transform, current_time, "map", "odom_combined"));

    // Do the mapping
    updateGrassCut();

    // Publish the map
    // nav_msgs::OccupancyGrid *og = theMap->NewOccupancyGrid();
    // og->header = theHeader;
    // og->info = theInfo;

    og.header = theHeader;
    og.info = theInfo;
    og.data = std::vector<int8_t>(mapData, mapData + AM_MAP_WIDTH * AM_MAP_HEIGHT);

    map_pub.publish(og);

    return true;
}

void CovMap::createGaussianFilterL(double gKernel[][41], double std_deviation)
{
    double sigma = std_deviation;
    double r, s = 2.0 * sigma * sigma;

    int w = 41;
    int h = 41;

    int hw = ((w - 1) / 2);
    int hh = ((h - 1) / 2);

    // sum is for normalization
    double sum = 0.0;

    // generate wxh kernel
    for (int x = -hw; x <= hw; x++)
    {
        for (int y = -hh; y <= hh; y++)
        {
            r = sqrt(x * x + y * y);
            gKernel[x + hw][y + hh] = (exp(-(r * r) / s)) / (M_PI * s);
            sum += gKernel[x + hw][y + hh];
        }
    }

    // normalize the Kernel
    for (int i = 0; i < w; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            gKernel[i][j] /= sum;
        }
    }
}

void CovMap::createGaussianFilterS(double gKernel[][21], double std_deviation)
{
    double sigma = std_deviation;
    double r, s = 2.0 * sigma * sigma;

    int w = 21;
    int h = 21;

    int hw = ((w - 1) / 2);
    int hh = ((h - 1) / 2);

    // sum is for normalization
    double sum = 0.0;

    // generate wxh kernel
    for (int x = -hw; x <= hw; x++)
    {
        for (int y = -hh; y <= hh; y++)
        {
            r = sqrt(x * x + y * y);
            gKernel[x + hw][y + hh] = (exp(-(r * r) / s)) / (M_PI * s);
            sum += gKernel[x + hw][y + hh];
        }
    }

    // normalize the Kernel
    for (int i = 0; i < w; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            gKernel[i][j] /= sum;
        }
    }
}

void CovMap::createGaussianFilterVS(double gKernel[][11], double std_deviation)
{
    double sigma = std_deviation;
    double r, s = 2.0 * sigma * sigma;

    int w = 11;
    int h = 11;

    int hw = ((w - 1) / 2);
    int hh = ((h - 1) / 2);

    // sum is for normalization
    double sum = 0.0;
    /*
            // generate wxh kernel
            for (int x = -hw; x <= hw; x++)
            {
                    for(int y = -hh; y <= hh; y++)
                    {
                            r = sqrt(x*x + y*y);
                            gKernel[x + hw][y + hh] = (exp(-(r*r)/s))/(M_PI * s);
                            sum += gKernel[x + hw][y + hh];
                    }
            }

        // normalize the Kernel
            for(int i = 0; i < w; ++i)
            {
                    for(int j = 0; j < w; ++j)
                    {
                            gKernel[i][j] /= sum;
                    }
            }
    */

    // Disabled kernel...
    for (int x = -hw; x <= hw; x++)
    {
        for (int y = -hh; y <= hh; y++)
        {
            r = sqrt(x * x + y * y);
            if (r <= 3)
            {
                gKernel[x + hw][y + hh] = 0.02;
            }
            else
            {
                gKernel[x + hw][y + hh] = 0.00;
            }
        }
    }
}
}
