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
        ePower1 = 0x04,
        ePower2 = 0x05,
        eSense1 = 0x06,
        eSense2 = 0x07,
        eCurrent1 = 0x08,
        eCurrent2 = 0x09,
        eGPIO1 = 0x0A,  //10
        eGPIO2 = 0x0B,  //11
        eGPIO3 = 0x0C,  //12
        eGPIO4 = 0x0D,  //13
        eISR=0xA0,
} SolarHSK_cmd;
