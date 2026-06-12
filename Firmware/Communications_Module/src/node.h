#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>

// Node-level identity and interface configuration lives in this file.
#define NODE_ORIGIN AIM_ORG_COMMS
#define NODE_NAME "COMMS_MODULE"

#define NODE_CAN_BAUD 500000U

#define NODE_CAN_BUS CAN1

#define NODE_SERIAL_RX_PIN PB10
#define NODE_SERIAL_TX_PIN PB11

#define NODE_SERIAL_BAUD 38400U

// Flash debug storage configuration (SPI flash on PB12-PB15).
#define NODE_FLASH_CS_PIN PB12
#define NODE_FLASH_SCK_PIN PB13
#define NODE_FLASH_MISO_PIN PB14
#define NODE_FLASH_MOSI_PIN PB15

// Number of columns stored in each flash table row.
#define NODE_FLASH_TABLE_COLS 1U
// Interval, in rows, between origin/metadata refresh operations.
#define NODE_FLASH_ORIGIN_REFRESH_INT 64U
// Total size, in bytes, reserved for the flash table.
#define NODE_FLASH_TABLE_SIZE 65536U
// Flash table instance index used by this node.
#define NODE_FLASH_TABLE_NUM 0U
// Scratch/data buffer size, in bytes, allocated in STM32 MCU RAM for flash table operations.
#define NODE_MCU_BUFFER_SIZE 256U

enum NodeState : uint8_t {
  INIT = 0U,
  OPERATIONAL = 1U,
  DEBUG_CONSOLE = 2U,
  FLASH_DUMP = 3U,
  FLASH_ERASE = 4U,
  SAFE_MODE = 5U,
  LOW_POWER = 6U,
  FAULT = 7U
};

// Add node-specific periodic behavior in nodeUpdate().
void nodeUpdate(uint32_t schedulerNowMs);

#endif  // NODE_H
