/*
 * SolarHSK_protocol.cpp
 *
 * Declares the interface protocol for cross device communication.
 *
 * Housekeeping packet consists of header
 * 0-255 payload bytes
 * 1 byte CRCS (or checksum)
 */

/*****************************************************************************
 * Defines
 ****************************************************************************/
#pragma once
#include <stdint.h>


/*******************************************************************************
 * Typedef enums
 *******************************************************************************/

/* Command definitions */
typedef enum SolarHSK_cmd {
  // 2-248 are board-specific: these are test commands
	eIntSensorRead = 0x02,
        ePacketCount = 0x03,
        eVoltageCurrentInput = 0x04,
        eVoltageCurrentOutput = 0x05,
        ePanelTemp = 0x06,
        eBoardTemp = 0x07,
        eInductorTemp = 0x08,
        eISR=0xA0,
} SolarHSK_cmd;
