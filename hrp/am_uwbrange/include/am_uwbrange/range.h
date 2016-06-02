/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 * 
 */

#ifndef RANGE_H
#define RANGE_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>
#include <am_driver/AntennaDelay.h>
#include <am_driver/BeaconSleepConfig.h>

namespace Husqvarna
{

typedef std::map<std::string, double> FactorMap;
typedef std::map<std::string, double>::iterator FactorMap_iterator;


class TestRange 
{
public:
	double sum;
	double min;
	double max;
	double num;
	std::vector<double> values;
};

typedef std::map<std::string, TestRange*> RangeValueMap;
typedef std::map<std::string, TestRange*>::iterator RangeValueMap_iterator;
typedef std::map<std::string, int> DurationMap;
typedef std::map<std::string, int>::iterator DurationMap_iterator;
typedef std::map<std::string, bool> BoolMap;
typedef std::map<std::string, int> IndexMap;
class Range
{
public:
	Range(const ros::NodeHandle& nodeh);
	~Range();
	
	bool setup();
	bool run();
	
private:
	// ROS data
	ros::NodeHandle nh;
	ros::Publisher rangePub;
	ros::Publisher multiRangePub;
	ros::Subscriber modeSub;
	ros::Subscriber joySub;
	ros::Subscriber aDelaySub;
	ros::Subscriber sleepSub;

	// Parameters
	std::string pSerialPort;
	
	int debugLog;
	int mode;
	int newMode;
    
    IndexMap indexMap;
    DurationMap sleepTimeMs;
    BoolMap newSleepTime;

    std::string idList[15];
    
	void modeCallback(const std_msgs::UInt16::ConstPtr& msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& j);

    void sleepCallback(const am_driver::BeaconSleepConfig::ConstPtr& msg);
	void antennaDelayCallback(const am_driver::AntennaDelay::ConstPtr& msg);

	// Calibration curve
	FactorMap beaconM;
	FactorMap beaconK;
	
	// Testing
	int startTest;
	double testTime;
	RangeValueMap testData;
	
	// Joy 
	bool waitingForRelease;
	bool joyDisabled;

	// 
	double antennaDelay;
	bool setAntennaDelay;
	
	am_driver::AntennaDelay aDelay;
	
	std::string antennaDelayStr;
	bool antennaDelayStringReady;
};

typedef boost::shared_ptr<Range> RangePtr;

}

#endif
