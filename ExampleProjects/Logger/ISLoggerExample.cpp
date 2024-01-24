/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <string.h>

#define CONNECTION_ATTEMPT_LIMIT 50

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "../../src/InertialSense.h"

bool connecting_;

std::string protocol_; 
std::string ip_;
int port_; 
std::string mount_point_;
std::string username_;
std::string password_;

static void msgHandlerIsb(InertialSense* i, p_data_t* data, int pHandle)
{
	static uint64_t dataCount;
	printf("Data count: %" PRIu64 "          \r", ++dataCount);
}

std::string get_connection_string(std::string protocol_, std::string ip_, int port_, std::string mount_point_, std::string username_, std::string password_) {
    std::string RTK_connection = "TCP:" + protocol_ + ":" + ip_ + ":" + std::to_string(port_);
    if (!mount_point_.empty() || !username_.empty())
        RTK_connection.append(":" + mount_point_);
    if (!username_.empty()) {
        RTK_connection.append(":" + username_);
        if (!password_.empty())
            RTK_connection.append(":" + password_);
    }

    return RTK_connection;
}

//Connect to NTRIP
void connect_rtk_client(InertialSense* is_)
{
    // if (is_ == nullptr) {
    //     // ROS_FATAL("RTK Client connection requested, but configureIS() hasn't been called in the provider.");
    //     ros::shutdown();
    //     connecting_ = false;
    //     return;
    // }
    
    connecting_ = true;

    // [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
    std::string RTK_connection = get_connection_string();

    int RTK_connection_attempt_count = 0;
    while (RTK_connection_attempt_count < CONNECTION_ATTEMPT_LIMIT)
    {
        ++RTK_connection_attempt_count;

        bool connected = is_->OpenConnectionToServer(RTK_connection);

        if (connected)
        {
            // ROS_INFO_STREAM("Successfully connected to " << RTK_connection << " RTK server");
            break;
        }
        else
        {
            std::cout << "\nFailed to connect to base server at " << RTK_connection << "\n";

            if (RTK_connection_attempt_count >= CONNECTION_ATTEMPT_LIMIT)
            {
                std::cout << "\nGiving up after " << RTK_connection_attempt_count << " failed attempts.\n";
            }
            else
            {
                // int sleep_duration = RTK_connection_attempt_count * CONNECTION_ATTEMPT_LIMIT;
                std::cout << "\nRetrying connection in " << "5" << " seconds.\n";
                sleep(5);
            }
        }
    }

    connecting_ = false;
}



int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("Please pass the com port and then optionally the log type as the first two arguments (dat,sdat,csv,kml).\r\n");
		// In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3 kml" 
		return -1;
	}


	// STEP 2: Instantiate InertialSense class
	// InertialSense class wraps communications and logging in a convenient, easy to use class
	InertialSense inertialSense(msgHandlerIsb);
	if (!inertialSense.Open(argv[1]))
	{
		std::cout << "Failed to open com port at " << argv[1] << std::endl;
	}


    //COnfigure as Rover with compassing

    //Connect to RTK Client





	// STEP 3: Enable data logger
	// get log type from command line
	cISLogger::eLogType logType = (argc < 3 ? cISLogger::eLogType::LOGTYPE_DAT : cISLogger::ParseLogType(argv[2]));
	inertialSense.SetLoggerEnabled(true, "", logType);


	// STEP 4: Enable data broadcasting
	// stop current data streaming
	inertialSense.StopBroadcasts();

	// broadcast the standard set of post processing messages (ins, imu, etc.)
	inertialSense.BroadcastBinaryDataRmcPreset(RMC_PRESET_INS_BITS);

	// instead of the rmc preset (real-time message controller) you can request individual messages...
	// Ask for INS message w/ update 40ms period (4ms source period x 10).  Set data rate to zero to disable broadcast and pull a single packet.
// 	inertialSense.BroadcastBinaryData(DID_IMU, 10);


	// utility class for display and ctrl-c handling
	cInertialSenseDisplay display;

	std::cout << "Started logger..." << std::endl;

	while (!display.ExitProgram())
	{
		inertialSense.Update();
	}

	inertialSense.Close();
}

