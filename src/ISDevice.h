/**
 * @file ISDevice.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 2/24/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef INERTIALSENSESDK_ISDEVICE_H
#define INERTIALSENSESDK_ISDEVICE_H

#include <memory>

#include "DeviceLog.h"
#include "protocol/FirmwareUpdate.h"

extern "C"
{
    // [C COMM INSTRUCTION]  Include data_sets.h and com_manager.h
    #include "data_sets.h"
    #include "com_manager.h"
    #include "serialPortPlatform.h"
}

class ISFirmwareUpdater;

class ISDeviceUpdater {
public:
    ISFirmwareUpdater* fwUpdater;
    float percent;
    bool hasError;
    uint16_t lastSlot;
    fwUpdate::target_t lastTarget;
    fwUpdate::update_status_e lastStatus;
    std::string lastMessage;

    std::vector<std::string> target_idents;
    std::vector<std::string> target_messages;

    bool inProgress();
    void update();
};

class ISDevice {
public:
    static ISDevice invalidRef;

    port_handle_t port = 0;
    // libusb_device* usbDevice = nullptr; // reference to the USB device (if using a USB connection), otherwise should be nullptr.

    dev_info_t devInfo = { };
    sys_params_t sysParams = { };
    nvm_flash_cfg_t flashCfg = { };
    unsigned int flashCfgUploadTimeMs = 0;		// (ms) non-zero time indicates an upload is in progress and local flashCfg should not be overwritten
    uint32_t flashCfgUploadChecksum = 0;
    evb_flash_cfg_t evbFlashCfg = { };
    system_command_t sysCmd = { };

    std::shared_ptr<cDeviceLog> devLogger;
    fwUpdate::update_status_e closeStatus = { };
    ISDeviceUpdater fwUpdate = { };

    bool operator==(const ISDevice& a) const { return (a.devInfo.serialNumber == devInfo.serialNumber) && (a.devInfo.hardwareType == devInfo.hardwareType); };

    ISDevice(const ISDevice& src) {
        port = src.port;
        devInfo = src.devInfo;
        flashCfg = src.flashCfg;
        flashCfgUploadTimeMs = src.flashCfgUploadTimeMs;
        flashCfgUploadChecksum = src.flashCfgUploadChecksum;
        evbFlashCfg = src.evbFlashCfg;
        sysCmd = src.sysCmd;
        devLogger = src.devLogger;
        closeStatus = src.closeStatus;
        fwUpdate = src.fwUpdate;
    }
    ISDevice() {
        flashCfg.checksum = (uint32_t)-1;
    };
};



#endif //INERTIALSENSESDK_ISDEVICE_H
