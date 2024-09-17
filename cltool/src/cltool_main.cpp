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

static void display_logger_status(InertialSense* i, bool refreshDisplay=false)
{
	if (!i || !refreshDisplay)
	{
		return;
	}

	cISLogger &logger = *(i->Logger());

	if (!logger.Enabled())
	{
		return;
	}

    float logSize = logger.LogSizeAll() * 0.001;
    if (logSize < 5000.0)
        printf("\nLogging %.1f MB to: %s\n", logSize, logger.LogDirectory().c_str());
    else
        printf("\nLogging %.2f KB to: %s\n", logSize * 0.001f, logger.LogDirectory().c_str());
}

// [C++ COMM INSTRUCTION] STEP 5: Handle received data 
static void cltool_dataCallback(InertialSense* i, p_data_t* data, int pHandle)
{
    if (!g_enableDataCallback)
    {   // Receive disabled
        return;
    }

    if (g_commandLineOptions.outputOnceDid && g_commandLineOptions.outputOnceDid != data->hdr.id)
    {   // ignore all other received data, except the "onceDid"
        return; 
    }

    (void)i;
    (void)pHandle;

    // track which DIDs we've received and when, and how frequently
    for (stream_did_t& did : g_commandLineOptions.datasets) {
        if (did.did == data->hdr.id) {
            did.rxStats.lastRxTime = current_timeMs();
            did.rxStats.rxCount++;
        }
    }

    // Print data to terminal - but only if we aren't doing a firmware update...
    if (g_devicesUpdating)
        cout.flush();
    // else if (!g_inertialSenseDisplay.ExitProgram()) // don't process any additional data once we've been told to exit
    //     g_inertialSenseDisplay.ProcessData(data);
}


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
    inertialSenseInterface.StopBroadcasts();
    SLEEP_MS(100);
    g_enableDataCallback = true;

    // Point display to serial port and is_comm_instance to print debug info
    // g_inertialSenseDisplay.SetSerialPort(inertialSenseInterface.SerialPort());
    com_manager_t* cm = (com_manager_t*)comManagerGetGlobal();
    if (cm != NULL && cm->numPorts > 0 && cm->ports)
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

std::vector<ISBootloader::cISBootloaderBase*> firmwareProgressContexts;

is_operation_result bootloadUpdateCallback(void* obj, float percent);
is_operation_result bootloadVerifyCallback(void* obj, float percent);

static int cltool_updateFirmware()
{
    // [BOOTLOADER INSTRUCTION] Update firmware
    if (g_commandLineOptions.updateBootloaderFilename.size() > 0)
    {
        cout << "Checking bootloader firmware:  " << g_commandLineOptions.updateBootloaderFilename << endl;
    }
    cout << "Updating application firmware: " << g_commandLineOptions.updateAppFirmwareFilename << endl;

    firmwareProgressContexts.clear();
    if(InertialSense::BootloadFile(
            g_commandLineOptions.comPort,
            0,
            g_commandLineOptions.updateAppFirmwareFilename,
            g_commandLineOptions.updateBootloaderFilename,
            g_commandLineOptions.forceBootloaderUpdate,
            g_commandLineOptions.baudRate,
            bootloadUpdateCallback,
            (g_commandLineOptions.bootloaderVerify ? bootloadVerifyCallback : 0),
            cltool_bootloadUpdateInfo,
            cltool_firmwareUpdateWaiter
    ) == IS_OP_OK) return 0;
    return -1;
}

std::mutex print_mutex;

void printProgress()
{
    print_mutex.lock();

    int divisor = 0;
    float total = 0.0f;

    cISBootloaderThread::m_ctx_mutex.lock();

    for (size_t i = 0; i < cISBootloaderThread::ctx.size(); i++)
    {
        if (cISBootloaderThread::ctx[i] && cISBootloaderThread::ctx[i]->m_use_progress)
        {
            divisor++;

            if (!cISBootloaderThread::ctx[i]->m_verify)
            {
                total += cISBootloaderThread::ctx[i]->m_update_progress;
            }
            else
            {
                total += cISBootloaderThread::ctx[i]->m_update_progress * 0.5f;
                total += cISBootloaderThread::ctx[i]->m_verify_progress * 0.5f;
            }
        }
    }

    cISBootloaderThread::m_ctx_mutex.unlock();

    if (divisor)
    {
        total /= divisor;
        int display = (int)(total * 100);

        // Print progress in condensed format.
        static int displayLast = 0;
#define DISPLAY_RES    5
        if (display == displayLast && display!=0)
        {
            printf("%d%% ", display);
        }
        fflush(stdout);

        while (display < displayLast)
        {   // Decrement
            displayLast -= DISPLAY_RES;
        }
        while (display >= displayLast)
        {   // Increment
            displayLast += DISPLAY_RES;
        }
    }

    print_mutex.unlock();
}

is_operation_result bootloadUpdateCallback(void* obj, float percent)
{
    if(obj)
    {
        ISBootloader::cISBootloaderBase* ctx = (ISBootloader::cISBootloaderBase*)obj;
        ctx->m_update_progress = percent;
    }
    return g_killThreadsNow ? IS_OP_CANCELLED : IS_OP_OK;
}

is_operation_result bootloadVerifyCallback(void* obj, float percent)
{
    if(obj)
    {
        ISBootloader::cISBootloaderBase* ctx = (ISBootloader::cISBootloaderBase*)obj;
        ctx->m_verify_progress = percent;
    }

    return g_killThreadsNow ? IS_OP_CANCELLED : IS_OP_OK;
}

void cltool_bootloadUpdateInfo(void* obj, ISBootloader::eLogLevel level, const char* str, ...)
{
    print_mutex.lock();
    static char buffer[256];

    va_list ap;
    va_start(ap, str);
    vsnprintf(buffer, sizeof(buffer) - 1, str, ap);
    va_end(ap);

    if(obj == NULL)
    {
        cout << buffer << endl;
        print_mutex.unlock();
        return;
    }

    ISBootloader::cISBootloaderBase* ctx = (ISBootloader::cISBootloaderBase *)obj;

    if (ctx->m_sn != 0 && ctx->m_port_name.size() != 0)
    {
        printf("    | %s (SN%d):", ctx->m_port_name.c_str(), ctx->m_sn);
    }
    else if(ctx->m_sn != 0)
    {
        printf("    | (SN%d):", ctx->m_sn);
    }
    else if (ctx->m_port_name.size() != 0)
    {
        printf("    | %s:", ctx->m_port_name.c_str());
    }
    else
    {
        printf("    | SN?:");
    }

    if (buffer[0])
        printf(" %s", buffer);

    printf("\r\n");

    print_mutex.unlock();
}

void cltool_firmwareUpdateInfo(void* obj, ISBootloader::eLogLevel level, const char* str, ...)
{
    print_mutex.lock();
    static char buffer[256];

    memset(buffer, 0, sizeof(buffer));
    if (str) {
        va_list ap;
        va_start(ap, str);
        vsnprintf(buffer, sizeof(buffer) - 1, str, ap);
        va_end(ap);
    }

    if(obj == NULL) {
        cout << buffer << endl;
    } else {
        ISFirmwareUpdater *fwCtx = (ISFirmwareUpdater *) obj;
        if (buffer[0] || (((g_commandLineOptions.displayMode != cInertialSenseDisplay::DMODE_QUIET) && (fwCtx->fwUpdate_getSessionStatus() == fwUpdate::IN_PROGRESS)))) {
            printf("[%5.2f] [%s:SN%07d > %s]", current_timeMs() / 1000.0f, fwCtx->portName, fwCtx->devInfo->serialNumber, fwCtx->fwUpdate_getSessionTargetName());
            if (fwCtx->fwUpdate_getSessionStatus() == fwUpdate::IN_PROGRESS) {
                int tot = fwCtx->fwUpdate_getTotalChunks();
                int num = fwCtx->fwUpdate_getNextChunkID();
                float percent = num / (float) (tot) * 100.f;
                printf(" :: Progress %d/%d (%0.1f%%)", num, tot, percent);
            }
            if (buffer[0])
                printf(" :: %s", buffer);
            printf("\n");
        }
    }

    print_mutex.unlock();
}

void cltool_firmwareUpdateWaiter()
{
    printProgress();
}

static int cltool_dataStreaming()
{
    InertialSense inertialSenseInterface(cltool_dataCallback);

    // [C++ COMM INSTRUCTION] STEP 2: Open serial port
    if (!inertialSenseInterface.Open(g_commandLineOptions.comPort.c_str(), g_commandLineOptions.baudRate, false))
    {
        cout << "Failed to open serial port at " << g_commandLineOptions.comPort.c_str() << endl;
        return -1;    // Failed to open serial port
    }

    int exitCode = 0;

    // [C++ COMM INSTRUCTION] STEP 3: Enable data broadcasting)
    if (cltool_setupCommunications(inertialSenseInterface))
    {
        try
        {
            if ((g_commandLineOptions.updateFirmwareTarget != fwUpdate::TARGET_HOST) && !g_commandLineOptions.fwUpdateCmds.empty()) {
                if(inertialSenseInterface.updateFirmware(
                        g_commandLineOptions.comPort,
                        g_commandLineOptions.baudRate,
                        g_commandLineOptions.updateFirmwareTarget,
                        g_commandLineOptions.fwUpdateCmds,
                        bootloadUpdateCallback,
                        (g_commandLineOptions.bootloaderVerify ? bootloadVerifyCallback : 0),
                        cltool_firmwareUpdateInfo,
                        cltool_firmwareUpdateWaiter
                ) != IS_OP_OK) {
                    // No need to Close() the InertialSense class interface; It will be closed when destroyed.
                    return -1;
                };
            }

            // before we start, if we are doing a run-once, set a default runDuration, so we don't hang indefinitely
            if (g_commandLineOptions.outputOnceDid && !g_commandLineOptions.runDuration)
                g_commandLineOptions.runDuration = 30000; // 30 second timeout, if none is specified

            // Main loop. Could be in separate thread if desired.
            uint32_t exitTime = current_timeMs() + g_commandLineOptions.runDuration;
            uint32_t requestDataSetsTimeMs = 0;

            // yield to allow comms
            SLEEP_MS(1);

            // [C++ COMM INSTRUCTION] STEP 4: Read data
            while ((!g_commandLineOptions.runDuration || (current_timeMs() < exitTime)))
            {

                if (!inertialSenseInterface.Update())
                {   // device disconnected, exit
                    exitCode = -2;
                    break;
                }

               // If updating firmware, and all devices have finished, Exit
                if (g_commandLineOptions.updateFirmwareTarget != fwUpdate::TARGET_HOST) {
                    if (inertialSenseInterface.isFirmwareUpdateFinished()) {
                        exitCode = 0;
                        break;
                    }
                }

                if ((current_timeMs() - requestDataSetsTimeMs) > 1000) {
                    // Re-request data every 1s
                    requestDataSetsTimeMs = current_timeMs(); 
                    cltool_requestDataSets(inertialSenseInterface, g_commandLineOptions.datasets);
                }
                
                // Prevent processor overload
                SLEEP_MS(1);
            }
        }
        catch (...)
        {
            cout << "Unknown exception..." << endl;
        }
    }

    //If Firmware Update is specified return an error code based on the Status of the Firmware Update
    if ((g_commandLineOptions.updateFirmwareTarget != fwUpdate::TARGET_HOST) && !g_commandLineOptions.updateAppFirmwareFilename.empty()) {
        for (auto& device : inertialSenseInterface.getDevices()) {
            if (device.fwUpdate.hasError) {
                exitCode = -3;
                break;
            }
        }
    }

    // [C++ COMM INSTRUCTION] STEP 6: Close interface
    SLEEP_MS(100);
    inertialSenseInterface.Close();
    SLEEP_MS(100);

    return exitCode;
}

static void sigint_cb(int sig)
{
    g_killThreadsNow = true;
    cltool_bootloadUpdateInfo(NULL, ISBootloader::eLogLevel::IS_LOG_LEVEL_ERROR, "Update cancelled, killing threads and exiting...");
    signal(SIGINT, SIG_DFL);
}

static int inertialSenseMain()
{
	if ((g_commandLineOptions.updateFirmwareTarget == fwUpdate::TARGET_HOST) && (g_commandLineOptions.updateAppFirmwareFilename.length() != 0))
    {
        // FIXME: {{ DEPRECATED }} -- This is the legacy update method (still required by the uINS3 and IMX-5, but will go away with the IMX-5.1)
        signal(SIGINT, sigint_cb);
        return cltool_updateFirmware();
    }
    else if (g_commandLineOptions.updateBootloaderFilename.length() != 0)
    {
        cout << "option -uf [FILENAME] must be used with option -ub [FILENAME] " << endl;
        return -1;
    }
    else
    {   // open the device, start streaming data and logging if needed
	    return cltool_dataStreaming();
	}
}


int main(int argc, char* argv[])
{
    // Parse command line options
    if (!cltool_parseCommandLine(argc, argv))
    {
        // parsing failed
        return -1;
    }

    g_inertialSenseDisplay.setOutputOnceDid(g_commandLineOptions.outputOnceDid);

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
