/*
 * Copyright (c) 2014 - Husqvarna AB, part of HusqvarnaGroup
 * Author: Stefan Grufman
 *
 */

#include "am_uwbrange/range.h"
#include "am_uwbrange/timeoutserial.h"

#include <ros/ros.h>

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#include <am_driver/Range.h>
#include <am_driver/MultiRange.h>

using namespace std;

#define HVA_RANGE_SINGLEMODE (0)
#define HVA_RANGE_MULTIMODE (1)
#define HVA_RANGE_POWERSAVE_MODE (2)
#define HVA_RANGE_BURST_MODE (3)
namespace Husqvarna
{

Range::Range(const ros::NodeHandle& nodeh)
{
    // Init attributes
    nh = nodeh;

    // Setup some ROS stuff
    modeSub = nh.subscribe("uwb_mode", 1, &Range::modeCallback, this);
    rangePub = nh.advertise<am_driver::Range>("uwb", 16);
    sleepSub = nh.subscribe("beacon_sleep", 16, &Range::sleepCallback, this);

    joySub = nh.subscribe("nano2", 1, &Range::joyCallback, this);

    aDelaySub = nh.subscribe("uwb_antenna_delay", 1, &Range::antennaDelayCallback, this);

    debugLog = false;
    mode = HVA_RANGE_SINGLEMODE;

    // Parameters
    ros::NodeHandle n_private("~");

    std::string defPort = "/dev/ttyUSB1";
    n_private.param("serialPort", pSerialPort, defPort);
    ROS_INFO("Param: serialPort: [%s]", pSerialPort.c_str());

    n_private.param("debugLog", debugLog, 0);
    ROS_INFO("Param: debugLog: [%d]", debugLog);

    n_private.param("mode", mode, HVA_RANGE_SINGLEMODE);
    ROS_INFO("Param: mode: [%d]", mode);

    // start with new request so that it is rebooted into correct mode
    newMode = 0xFF;

    // TEMP: Before we move this into calibration of RADIOS!

    std::string idListInit[15] =  {"DECA0100-101","DECA0100-102","DECA0100-103","DECA0100-104",
               "DECA0100-105","DECA0100-106","DECA0100-107","DECA0100-108",
               "DECA0100-109","DECA0100-10A","DECA0100-10B","DECA0100-10C",
               "DECA0100-10D","DECA0100-10E","DECA0100-10F"};
    // For testing
    testData.clear();
    for (int i = 0; i < 8; i++)
    {
		// f(x) = beaconK*x + beaconM
        idList[i] = idListInit[i];        
        indexMap[idList[i]] = i + 1;
        newSleepTime[idList[i]] = true;			//Initially write down the sleep times				
        sleepTimeMs[idList[i]] = (1000 + i * 20) / 3.0;        		        
		beaconK[idList[i]] = 0.0;
        beaconM[idList[i]] = 0.0;
        testData[idList[i]] = new TestRange();
	}
        
    startTest = 0;
    waitingForRelease = false;
    joyDisabled = true;

    setAntennaDelay = false;

    antennaDelay = 0;

    antennaDelayStr = "";
    antennaDelayStringReady = false;
}

Range::~Range()
{
    // TODO: Delete the testData...
}

bool Range::setup()
{
    return true;
}

void Range::antennaDelayCallback(const am_driver::AntennaDelay::ConstPtr& msg)
{
    // Bulid the "rad-string" to set a new antenna delay.

    std::string cmd = "rad ";

    std::cout << "index: " << msg->index << std::endl;
    std::cout << "antennaDelay: " << msg->antennaDelay << std::endl;

    cmd += boost::lexical_cast<std::string>(msg->index);
    cmd += "=";
    cmd += boost::lexical_cast<std::string>(msg->antennaDelay);
    cmd += "\r\n";

    std::cout << "SENDing: " << cmd;

    antennaDelayStr = cmd;
    antennaDelayStringReady = true;
}

void Range::modeCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    // new mode?
    switch (msg->data)
    {
    case 0:
        newMode = HVA_RANGE_SINGLEMODE;
        ROS_INFO("[UWBRANGE] - Requested SINGLE MODE.");
        break;
    case 1:
        newMode = HVA_RANGE_MULTIMODE;
        ROS_INFO("[UWBRANGE] - Requested MULTI MODE.");
        break;
    case 2:
        newMode = HVA_RANGE_BURST_MODE;
        ROS_INFO("[UWBRANGE] - Requested HVA_RANGE_BURST_MODE.");
        break;
    case 3:
        newMode = HVA_RANGE_POWERSAVE_MODE;
        ROS_INFO("[UWBRANGE] - Requested HVA_RANGE_POWERSAVE_MODE.");
        break;        
    case 10:
        startTest = 1;
        ROS_INFO("[UWBRANGE] - Requested Start Test.");
        break;
    default:
        ROS_INFO("[UWBRANGE] - Requested strange mode...nothing done!");
        break;
    }
}

void Range::sleepCallback(const am_driver::BeaconSleepConfig::ConstPtr& msg)
{
    // Scaling due to iplementation in FW...
    sleepTimeMs[msg->beaconId] = (int)(msg->sleepTime * 1000.0 / 3.0);
    ROS_INFO("Range::sleepCallback: Assign %dms to %s",sleepTimeMs[msg->beaconId] * 3, msg->beaconId.c_str());
    newSleepTime[msg->beaconId] = true;
}

void Range::joyCallback(const sensor_msgs::Joy::ConstPtr& j)
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
            ROS_INFO("UWBRANGE::NANO KONTROL enabled!");
            debugLog = true;
        }

        // Always return...
        return;
    }
    else
    {
        // Pressing R[2]
        if (j->buttons[25 + 16 + 2] == 1)
        {
            joyDisabled = true;
            waitingForRelease = true;
            ROS_INFO("UWBRANGE::NANO KONTROL disabled!");
            debugLog = false;
            return;
        }
    }

    // SET
    if (j->buttons[1] == 1)
    {
        waitingForRelease = true;
        setAntennaDelay = true;
    }
    else if (j->buttons[25 + 8 + 2] == 1)
    {
        waitingForRelease = true;
        if (newMode == HVA_RANGE_SINGLEMODE)
        {
            newMode = HVA_RANGE_MULTIMODE;
        }
        else
        {
            newMode = HVA_RANGE_SINGLEMODE;
        }
    }
    else
    {
        //
        // SLIDERS
        //
        float delay = j->axes[2] * 10.0;
        if (delay != 0)
        {
            antennaDelay = 510.000f + delay;
            ROS_INFO("Antenna delay = %f", antennaDelay);
        }
    }
}

bool Range::run()
{
    bool res = true;

    ros::Time last_time = ros::Time::now();

    am_driver::Range rangeMsg;

    std::string reset = "exit\r\n";
    std::string setMultiMode = "set scheme=2\r\nsave\r\n";
    std::string setSingleMode = "set scheme=0\r\nsave\r\n";
    std::string setBurstMode = "set mode=1\r\nsave\r\n";
    std::string setPowerSaveMode = "set mode=2\r\nsave\r\n";
    
    try
    {
        TimeoutSerial serial(pSerialPort, 115200);
        serial.setTimeout(boost::posix_time::seconds(0));

        serial.writeString(reset);

        while (ros::ok())
        {
            ros::Time current_time = ros::Time::now();
            ros::Duration dt = current_time - last_time;
            last_time = current_time;

            // Read from serial port
            string line = serial.readStringUntil("\r\n");

            // Parse data
            if (boost::starts_with(line, "R:"))
            {
                // Format: R:2880:DECA0100-101:DECA0100-100
                vector<string> strs;
                boost::split(strs, line, boost::is_any_of(":"));

                if (strs.size() == 4)
                {
                    // Size=4 means valid line...get data out
                    double range = boost::lexical_cast<double>(strs[1]) / 1000.0;
                    string fromId = strs[2];
                    string toId = strs[3];

                    // Lookup the "offset"
                    double fx_to = beaconK[toId] * range + beaconM[toId];
                    double fx_from = beaconK[fromId] * range + beaconM[fromId];

                    double fx = (fx_to + fx_from) / 2.0;

                    double calc_range = range + fx;

                    if (debugLog)
                    {
                        std::cout << fromId.c_str() << " -> " << toId.c_str() << " = " << range
                                  << " adelay = " << antennaDelay << std::endl;
                    }

                    if (calc_range > 0.8)
                    {
                        // Use value over 0.8 range
                        rangeMsg.header.frame_id = "uwb_base";
                        rangeMsg.header.stamp = ros::Time::now();
                        rangeMsg.fromId = fromId;
                        rangeMsg.toId = toId;
                        rangeMsg.range = calc_range;
                        rangePub.publish(rangeMsg);

                        // For testing
                        if (fromId == "DECA0100-100")
                        {
                            // std::cout << "ToId: " << toId << std::endl;
                            if (testData.find(toId) != testData.end())
                            {
                                TestRange* tst = testData[toId];
                                if (tst != 0)
                                {
                                    tst->sum += range;
                                    tst->num += 1;

                                    tst->values.push_back(range);

                                    if (range > tst->max)
                                    {
                                        tst->max = range;
                                    }
                                    if (range < tst->min)
                                    {
                                        tst->min = range;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else if (debugLog)
            {
                ROS_INFO("[UWBRANGE]-[%s]", line.c_str());
            }

            // Mode change?
            if (newMode != mode)
            {
                mode = newMode;
                switch (mode)
                {
                case HVA_RANGE_SINGLEMODE:
                    serial.writeString(setSingleMode);
                    break;
                case HVA_RANGE_MULTIMODE:
                    serial.writeString(setMultiMode);
                    break;
                case HVA_RANGE_POWERSAVE_MODE:
                    serial.writeString(setPowerSaveMode);
                    break;    
               case HVA_RANGE_BURST_MODE:
                    serial.writeString(setBurstMode);
                    break;    
                }
            }

            // Antenna delay
            if (setAntennaDelay)
            {
                setAntennaDelay = false;

                std::ostringstream valueStream;
                valueStream << (int)(antennaDelay * 1000.0f);

                std::string setAntennaDelayStr = "set adelay=" + valueStream.str() + "\r\nsave\r\n";
                std::cout << setAntennaDelayStr << std::endl;

                serial.writeString(setAntennaDelayStr);
            }

            // Set remote antenna delay from topic
            if (antennaDelayStringReady)
            {
                antennaDelayStringReady = false;
                serial.writeString(antennaDelayStr);
            }

			// New sleep time?
			DurationMap_iterator iter = sleepTimeMs.begin();
			while (iter != sleepTimeMs.end())
			{
				// Check if we have an updated sleep time
				if (newSleepTime[iter->first])
				{
					newSleepTime[iter->first] = false;

					std::ostringstream indexStream;
					indexStream << indexMap[iter->first];
					
					std::ostringstream timeStream;
					timeStream << sleepTimeMs[iter->first];

					std::string setSleepStr = "sleep " + indexStream.str() + "=" + timeStream.str() + "\r\n";
					std::cout << setSleepStr;

					serial.writeString(setSleepStr);
                    break; //break so we only write one per iteration
				}
				iter++;
			}
			
			
            if (startTest == 1)
            {
                startTest = 2;

                testTime = 20.0;

                // Clear values
                RangeValueMap_iterator iter = testData.begin();
                while (iter != testData.end())
                {
                    TestRange* tst = iter->second;

                    if (tst != 0)
                    {

                        tst->sum = 0.0;
                        tst->num = 0.0;
                        tst->max = -1000;
                        tst->min = 1000;

                        tst->values.clear();
                    }

                    iter++;
                }
            }

            if (startTest == 2)
            {
                // Executing test...

                testTime = testTime - dt.toSec();

                if (testTime < 0.0)
                {
                    // Test done
                    std::cout << "Results from robot to beacons:" << std::endl;
                    std::cout << "ID\t\tAVG\tMED\tMIN\tMAX\tNUM\tVAR\t\tSTD\t\tLEN\tERR" << std::endl;
                    startTest = 0;

                    RangeValueMap_iterator iter = testData.begin();
                    while (iter != testData.end())
                    {
                        TestRange* tst = iter->second;
                        string id = iter->first;

                        if (tst != 0)
                        {
                            double mean = tst->sum / tst->num;

                            // Calculate variance
                            double var = 0.0;

                            for (int i = 0; i < tst->values.size(); i++)
                            {
                                double t = (tst->values[i] - mean);
                                var += t * t;
                            }

                            var = var / (tst->values.size());
                            double std_dev = sqrt(var);

                            // Calculate median
                            double median = 0;
                            if (tst->values.size() > 0)
                            {
                                sort(tst->values.begin(), tst->values.end());
                                median = tst->values[tst->values.size() / 2];
                            }

                            // Calculate mean error
                            int assumedLength = round(mean);
                            double mean_err = mean - (double)assumedLength;

                            std::cout << id << "\t" << mean << "\t" << median << "\t" << tst->min << "\t" << tst->max
                                      << "\t" << tst->num << "\t" << var << "\t" << std_dev << "\t"
                                      << (double)assumedLength << "\t" << mean_err << std::endl;
                        }

                        iter++;
                    }
                }
            }

            ros::spinOnce();
        }
    }
    catch (boost::system::system_error& e)
    {
        ROS_ERROR("Range-serial: %s", e.what());
    }

    return res;
}
}
