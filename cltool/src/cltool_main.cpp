/*
MIT LICENSE

Copyright (c) 2014-2024 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
*  main.cpp
*
*  (c) 2014 Inertial Sense, LLC
*
*  The Inertial Sense Command Line Tool (cltool) shows how easy it is to communicate with the uINS, log data, update firmware and more.
*
*  The following keywords are found within this file to identify instructions
*  for SDK implementation.  
*
*    KEYWORD:                  SDK Implementation
*    [C++ COMM INSTRUCTION]    - C++ binding API - InertialSense class with binary communication 
*                                protocol and serial port support for Linux and Windows.
*    [C COMM INSTRUCTION]      - C binding API - Com Manager with binary communication protocol.
*    [LOGGER INSTRUCTION]      - Data logger.
*    [BOOTLOADER INSTRUCTION]  - Firmware update feature.
*
*  This app is designed to be compiled in Linux and Windows.  When using MS
*  Visual Studio IDE, command line arguments can be supplied by right clicking 
*  the project in solution explorer and then selecting properties -> debugging -> command line arguments
*/

#include <signal.h>

// Contains command line parsing and utility functions.  Include this in your project to use these utility functions.
#include "cltool.h"
#include "protocol_nmea.h"
#include "util/natsort.h"

using namespace std;

#define XMIT_CLOSE_DELAY_MS    1000     // (ms) delay prior to cltool close to ensure data transmission

static bool g_killThreadsNow = false;
static bool g_enableDataCallback = false;
int g_devicesUpdating = 0;


static serial_port_t *s_serialPort;


/**
 * requests any data which is not being actively received
 * @param inertialSenseInterface
 * @param datasets a vector of stream_did_t indicating the set of DIDs which should be requested and the rate to be requested at.
 * @return the number of did which were actually requested. This maybe less than the number of items in the specified dataset
 *  if those DIDs have already been recently received. If 0 is returned, it indicates that all requested DIDs are already
 *  streaming.
 */
void cltool_requestDataSets(InertialSense& inertialSenseInterface, std::vector<stream_did_t>& datasets) {
    unsigned int currentTime = current_timeMs();

    for (stream_did_t& dataItem : datasets)
    {   // Datasets to stream
        inertialSenseInterface.BroadcastBinaryData(dataItem.did, dataItem.periodMultiple);
        switch (dataItem.did)
        {
            case DID_RTOS_INFO:
                system_command_t cfg;
                cfg.command = SYS_CMD_ENABLE_RTOS_STATS;
                cfg.invCommand = ~cfg.command;
                inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
                break;
        }
    }
}

// Where we tell the IMX what data to send and at what rate.  
// "cltool_dataCallback()" is registered as the callback functions for all received data.
// All DID messages are found in data_sets.h
static bool cltool_setupCommunications(InertialSense& inertialSenseInterface)
{
    // Stop streaming any messages, wait for buffer to clear, and enable Rx callback
    // inertialSenseInterface.StopBroadcasts();
    SLEEP_MS(100);
    g_enableDataCallback = true;

    {
        // if (g_commandLineOptions.datasets.size() > 0)
        // {   // Select DID for generic display, which support viewing only one DID.
        //     g_inertialSenseDisplay.SelectEditDataset(g_commandLineOptions.datasets.front().did, true);  
        // }
        cltool_requestDataSets(inertialSenseInterface, g_commandLineOptions.datasets);
    }

    if (g_commandLineOptions.sysCommand != 0)
    {   // Send system command to IMX
        cout << "Sending system command: " << g_commandLineOptions.sysCommand;
        bool manfUnlock = false;
        switch(g_commandLineOptions.sysCommand)
        {
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GPS1:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_GPS2:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER0:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER1:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_TO_SER2:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_TO_GPS1:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_GPS1:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_GPS2:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_USB:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER0:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER1:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_TO_SER2:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_USB_LOOPBACK:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER0_LOOPBACK:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER1_LOOPBACK:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_SER2_LOOPBACK:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_LOOPBACK:
            case SYS_CMD_ENABLE_SERIAL_PORT_BRIDGE_CUR_PORT_LOOPBACK_TESTMODE:
                cout << " Enable serial bridge"; break;
            case SYS_CMD_DISABLE_SERIAL_PORT_BRIDGE:
                cout << " Disable serial bridge"; g_commandLineOptions.disableDeviceValidation = true;          break;
            case SYS_CMD_MANF_FACTORY_RESET:            manfUnlock = true;  cout << " Factory Reset";           break;
            case SYS_CMD_MANF_CHIP_ERASE:               manfUnlock = true;  cout << " Chip Erase";              break;
            case SYS_CMD_MANF_DOWNGRADE_CALIBRATION:    manfUnlock = true;  cout << " Downgrade Calibration";   break;
            case SYS_CMD_MANF_ENABLE_ROM_BOOTLOADER:    manfUnlock = true;  cout << " Enable ROM Bootloader";   break;
        }
        cout << endl;
        system_command_t cfg;

        if (manfUnlock)
        {
            cfg.command = SYS_CMD_MANF_UNLOCK;
            cfg.invCommand = ~cfg.command;
            inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
        }

        cfg.command = g_commandLineOptions.sysCommand;
        cfg.invCommand = ~cfg.command;
        inertialSenseInterface.SendRawData(DID_SYS_CMD, (uint8_t*)&cfg, sizeof(system_command_t), 0);
        SLEEP_MS(XMIT_CLOSE_DELAY_MS);      // Delay to allow transmit time before port closes
        return false;
    }

    return true;
}


static int cltool_dataStreaming()
{
    InertialSense inertialSenseInterface(NULL);

    // [C++ COMM INSTRUCTION] STEP 2: Open serial port
    if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate, false))
    {
        cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
        return -1;    // Failed to open serial port
    }

    int exitCode = 0;

    s_serialPort = inertialSenseInterface.SerialPort();

    // [C++ COMM INSTRUCTION] STEP 3: Enable data broadcasting)
    if (cltool_setupCommunications(inertialSenseInterface))
    {
            // Main loop. Could be in separate thread if desired.

            // yield to allow comms
            SLEEP_MS(1);

            uint32_t display_updateMs = 0;
            uint32_t rxBytesLast = 0;

            // [C++ COMM INSTRUCTION] STEP 4: Read data
            while (1)
            {
#if 0
                if (!inertialSenseInterface.Update())
                {   // device disconnected, exit
                    exitCode = -2;
                    break;
                }
#else
                unsigned char line[8192];
                serialPortRead(s_serialPort, line, sizeof(line));
#endif
                uint32_t timeMs = current_timeMs();
                if (timeMs - display_updateMs >= 1000)
                {
                    display_updateMs = timeMs;
                    printf("Rx bytes %d\n", s_serialPort->rxBytes - rxBytesLast);
                    rxBytesLast = s_serialPort->rxBytes;
                }
                
                // Prevent processor overload
                SLEEP_MS(1);
            }
    }

    // [C++ COMM INSTRUCTION] STEP 6: Close interface
    SLEEP_MS(100);
    inertialSenseInterface.Close();
    SLEEP_MS(100);

    return exitCode;
}

static int inertialSenseMain()
{
    return cltool_dataStreaming();
}


int main(int argc, char* argv[])
{
    // Parse command line options
    if (!cltool_parseCommandLine(argc, argv))
    {
        // parsing failed
        return -1;
    }

    // InertialSense class example using command line options
    int result = inertialSenseMain();
    if (result == -1)
    {
        cltool_outputHelp();

        // Pause so user can read error
        SLEEP_MS(2000);
    }

    return result;
}
