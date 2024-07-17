/**
 * @file types.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 7/3/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CORE_TYPES_H
#define IS_CORE_TYPES_H

#include <inttypes.h>

/**
 * Port definitions used across the entire product line & SDK.
 */
#define PORT_TYPE__UART             0x01
#define PORT_TYPE__USB              0x02
#define PORT_TYPE__SPI              0x03
#define PORT_TYPE__CAN              0x04
#define PORT_TYPE__LOOPBACK         0x0F

#define PORT_TYPE__GNSS             0x20    //! bit indicates that this port is a GNSS receiver port
#define PORT_TYPE__COMM             0x40    //! bit indicates that this port has an ISComm associated with it
#define PORT_TYPE__HDW              0x80    //! bit indicates that this port is static/hardware-defined

#define PORT_ERROR__NONE            0x00
#define PORT_ERROR__NOT_SUPPORTED   0x01

typedef void* port_handle_t;

typedef int(*pfnPortRead)(port_handle_t port, uint8_t* buf, int len);
typedef int(*pfnPortWrite)(port_handle_t port, const uint8_t* buf, int len);
typedef int(*pfnPortFree)(port_handle_t port);
typedef int(*pfnPortAvailable)(port_handle_t port);
typedef const char*(*pfnPortName)(port_handle_t port);

typedef struct base_port_s {
    uint16_t pnum;              //! an identifier for a specific port that belongs to this device
    uint16_t ptype;             //! an indicator of the type of port

    pfnPortRead portRead;
    pfnPortWrite portWrite;
    pfnPortFree portFree;
    pfnPortAvailable portAvailable;
    pfnPortName portName;
} base_port_t;
// typedef base_port_t* port_handle_t;

static inline uint16_t portId(port_handle_t port) { return (port) ? ((base_port_t*)port)->pnum : 0xFFFF; }
static inline uint16_t portType(port_handle_t port) { return (port) ? ((base_port_t*)port)->ptype : 0xFFFF; }
static inline int portRead(port_handle_t port, uint8_t* buf, int len) { return (port && ((base_port_t*)port)->portRead) ? ((base_port_t*)port)->portRead(port, buf, len) : PORT_ERROR__NOT_SUPPORTED; }
static inline int portWrite(port_handle_t port, const uint8_t* buf, int len) { return (port && ((base_port_t*)port)->portWrite) ? ((base_port_t*)port)->portWrite(port, buf, len) : PORT_ERROR__NOT_SUPPORTED; }
static inline int portFree(port_handle_t port) { return (port && ((base_port_t*)port)->portFree) ? ((base_port_t*)port)->portFree(port) : PORT_ERROR__NOT_SUPPORTED; }
static inline int portAvailable(port_handle_t port) { return (port && ((base_port_t*)port)->portAvailable) ? ((base_port_t*)port)->portAvailable(port) : PORT_ERROR__NOT_SUPPORTED; }
static inline const char *portName(port_handle_t port) { return (port && ((base_port_t*)port)->portName) ? ((base_port_t*)port)->portName(port) : (const char *)0; }

#endif //IS_CORE_TYPES_H
