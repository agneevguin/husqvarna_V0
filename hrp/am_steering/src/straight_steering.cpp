/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_steering/straight_steering.h"
#include <tf/transform_datatypes.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

#define STRAIGHT_INIT 0
#define STRAIGHT_CONTROL 1
#define STRAIGHT_IDLE 2
#define STRAIGHT_AVERAGE 3

#define DEG2RAD(DEG) ((DEG) * ((M_PI) / (180.0)))

#define FIX_ANGLES_RAD(a)       \
    if (a > M_PI * 2.0)     \
    {                       \
        a = a - M_PI * 2.0; \
    }                       \
    else if (a < 0.0)       \
    {                       \
        a = a + M_PI * 2.0; \
    }

#define FIX_ANGLES_DEG(a)       \
    if (a > 360)     \
    {                       \
        a = a - 360; \
    }                       \
    else if (a < 0.0)       \
    {                       \
        a = a + 360; \
    }
namespace Husqvarna
{

StraightSteering::StraightSteering(const ros::NodeHandle& nodeh)
{
    // Init attributes
    nh = nodeh;

    // Parameters
    ros::NodeHandle n_private("~");


    // Setup some ROS stuff
    cmdPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
    joySub = nh.subscribe("nano2", 5, &StraightSteering::joyCallback, this);
    statusSub = nh.subscribe("sensor_status", 1, &StraightSteering::statusCallback, this);
    gpsVelSub = nh.subscribe("vel", 10, &StraightSteering::gpsVelCallback, this);
    gpsFixSub = nh.subscribe("fix", 10, &StraightSteering::gpsFixCallback, this);
    nmeaSub = nh.subscribe("nmea_sentence", 10, &StraightSteering::nmeaSentenceCallback, this);

    state = STRAIGHT_IDLE;
    wMax = 1.0;
    wMin = -1.0;
	statusOutside = false;
	statusCollision = false;
    
    fixStatus = 0;
    referenceHeading = 0.0;
    latestHeading = 0.0;
    
    doAverageHeading = false;
    averageStart = 4.0;
    controlStart = 4.0;
    
    joyDisabled = true;
    startControl = false;
    startInit = false;
    err = 0;
    kp = 0.004;
    speed = 0.3;
}

StraightSteering::~StraightSteering()
{
}

void StraightSteering::printConfig(void)
{
}

bool StraightSteering::setup()
{
    return true;
}

void StraightSteering::joyCallback(const sensor_msgs::Joy::ConstPtr& j)
{
    // Special case to handle key release event...
    if (waitingForRelease)
    {
        waitingForRelease = false;
        return;
    }

    // Need to "enable" this before use of params...
    if (joyDisabled)
    {
        // Pressing R[2]
        if (j->buttons[25 + 16 + 2] == 1)
        {
            joyDisabled = false;
            waitingForRelease = true;
            ROS_INFO("SteerStraight::NANO KONTROL enabled!");
            printConfig();
        }
    }
	else
	{
		// Pressing R[2]
		if (j->buttons[25+16+2] == 1)
		{
			joyDisabled = true;
			waitingForRelease = true;
			ROS_INFO("SteerStraight::NANO KONTROL disabled!");
			return;
		}
	}
    // STOP
    if (j->buttons[6] == 1)
    {
        ROS_INFO("SteerStraight::STOP");
        stop();
        state = STRAIGHT_IDLE;
        doAverageHeading = false;
        return;
    }
    // Play
    else if (j->buttons[7] == 1)
    {
        initStartTime = ros::Time::now();
        std::cout << "start driving" << endl;
        startInit = true;
    }
    // Rec
    /*else if (j->buttons[8] == 1)
    {
        std::cout << "start control" << endl;
        startControl = true;
    }*/
    else
    {
        double p = j->axes[0] / 100.0;
        if (p != 0)
        {
            kp = p;
            ROS_INFO("Steering::kp = %f", kp);
        }
        
        p = j->axes[1] * 10.0;
        if (p != 0)
        {
            averageStart = p;
            ROS_INFO("StraightSteering::averageStart = %f", averageStart);
        }
        
        p = j->axes[2] * 10.0;
        if (p != 0)
        {
            controlStart = p;
            ROS_INFO("StraightSteering::controlStart = %f", controlStart);
        }
        
        double s = j->axes[1+8];
        if (s != 0)
        {
            speed = s;
            ROS_INFO("StraightSteering::speed = %f", speed);
        }
        
        double wAbsMax = j->axes[0+8] * 1.0;
        if (wAbsMax != 0)
        {
            wMax = wAbsMax;
            wMin = -wAbsMax;
            ROS_INFO("Steering::wAbsMax = %f", wAbsMax);
        }
        
    }
    return;
}


void StraightSteering::nmeaSentenceCallback(const nmea_msgs::Sentence::ConstPtr& msg)
{
    nmea_msgs::Sentence sentence;
    sentence = *msg;

    vector<string> strs;
    boost::split(strs, sentence.sentence, boost::is_any_of(","));
    
    if (strs[0] == "$GPGGA")
    {	
		fixStatus = boost::lexical_cast<int>(strs[6]);
	} 
    else if (strs[0] == "$GPRMC")
    {
        latestHeading = boost::lexical_cast<double>(strs[8]);
        FIX_ANGLES_DEG(latestHeading)
        if (doAverageHeading == true)
        {
            if (headingCount < 0.1)
            {
				headingAverageStart= latestHeading;
			}
			
            double diff = headingAverageStart - latestHeading;
			if(diff > 180)
			{
				diff -= 360;
			}
			else if (diff <= -180)
			{
				diff += 360;
			}
            
            headingDiffSum += diff;
            headingCount++;
            headingAverage = headingAverageStart - (headingDiffSum / headingCount);
            FIX_ANGLES_DEG(headingAverage)

            std::cout << "latest: " << latestHeading << ", headingAverage:" << headingAverage << ", headingDiffSum: " << headingDiffSum <<  ", headingAverageStart:" << headingAverageStart << ", cnt:" << headingCount << std::endl;
         
            
        }
        else
        {
            headingCount = 0.0;
            headingAverage = 0.0;
            headingDiffSum = 0.0;
        }
    }
    
}

void StraightSteering::gpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    geometry_msgs::TwistStamped vel;
    vel = *msg;
    std::cout << "heading:" << vel.twist.angular.z << std::endl;
}

void StraightSteering::gpsFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    sensor_msgs::NavSatFix fix;
    fix = *msg;
    std::cout << "lat: " << fix.latitude << ", long: " << fix.longitude << std::endl;
}

void StraightSteering::statusCallback(const am_driver::SensorStatus::ConstPtr& msg)
{
    // Sensor status
    // std::cout << "Sensor Status:" << msg->sensorStatus << std::endl;

    // 0x04 - Collision
    // 0x02 - Out of area

    statusCollision = (msg->sensorStatus & 0x04);
    statusOutside = (msg->sensorStatus & 0x02);

    if (statusCollision || statusOutside)
    {
        stop();
        state = STRAIGHT_IDLE;
    }
}

void StraightSteering::stop()
{
    geometry_msgs::Twist vel;

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = 0.0;

    cmdPub.publish(vel);
}

bool StraightSteering::update(ros::Duration dt)
{

    ros::Time currentTime = ros::Time::now();

    double w;
    geometry_msgs::Twist vel;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;

    switch (state)
    {
        case STRAIGHT_INIT:
            vel.linear.x = speed;
            vel.angular.z = 0.0;

            if (currentTime - initStartTime > ros::Duration(averageStart))
            {
                if (fixStatus == 1)
                {
                    state = STRAIGHT_AVERAGE;
                    averageStartTime = currentTime;
                    doAverageHeading = true;
                    ROS_INFO("Start Averaging GPS heading!");
                    
                }
                else
                {
                    ROS_ERROR("No Fix, will not use GPS, back to IDLE!");
                    stop();
                    state = STRAIGHT_IDLE;
                    
                }
            }
            cmdPub.publish(vel);
            break;
        
        case STRAIGHT_AVERAGE:
            if (currentTime - averageStartTime > ros::Duration(controlStart))
            {
                state = STRAIGHT_CONTROL;  
                referenceHeading = headingAverage;
                ROS_INFO("StraightSteering::STRAIGHT_CONTROL. referenceHeading = %f", referenceHeading);
                
            }
            break;  
            
        case STRAIGHT_CONTROL:
            vel.linear.x = speed;

            /* Angles are in 0-360 degrees */
            if (latestHeading < 0)
            {
                latestHeading += 360;
            }
            if (referenceHeading < 0)
            {
                referenceHeading += 360;
            }
            
            err = referenceHeading - latestHeading;
            
            /* Always choose the error closest to zero degrees */
            if (err < -180)
            {
                err = err + 360;
            }
            else if (err > 180)
            {
                err = err - 360;
            }
                
            w = kp * err;

            /* Threshold to avoid too large turns */
            if (w > wMax)
            {
                w = wMax;
            }
            else if (w < wMin)
            {
                w = wMin;
            }
            
            w = -w;
            std::cout << "ref:" << referenceHeading << ", curr:" << latestHeading << ", avg:" << headingAverage << ", kp:"<< kp << ", wMax:" << wMax << ", w:" << w << ", fix:" << fixStatus << ", spd:" << speed << std::endl;

            vel.angular.z = w;
            cmdPub.publish(vel);
            break;
        

        case STRAIGHT_IDLE:
            vel.linear.x = 0.0;
            vel.angular.z = 0.0;
            
            if (startInit)
            {
                startInit = false;
                state = STRAIGHT_INIT;
            }
        
            break;
    }
    return true;
}
}
