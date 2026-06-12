#ifndef CONSOLE_H
#define CONSOLE_H

#ifndef FLIGHT_BUILD

#include <Arduino.h>
#include <cstdint>

class AimNetwork;
class Logger;

enum ConsoleAction : uint8_t {
  CONSOLE_ACTION_NONE        = 0U,
  CONSOLE_ACTION_ENTER       = 1U,
  CONSOLE_ACTION_EXIT        = 2U,
  CONSOLE_ACTION_FLASH_INFO  = 3U,
  CONSOLE_ACTION_FLASH_DUMP  = 4U,
  CONSOLE_ACTION_FLASH_ERASE = 5U,
  CONSOLE_ACTION_READ_BOARD_HEARTBEATS = 6U
  // add more console actions here as needed
};

void consoleInit(Stream& serial,
                 AimNetwork& aim,
                 Logger& log);

ConsoleAction consoleCheckEntry(void);
ConsoleAction consoleService(uint8_t currentState, uint32_t networkNowMs);
void consoleResume(void);

// Functions to retrieve node health data from main
struct AimNodeHealth;
const AimNodeHealth* getNodeHealth(uint8_t origin);
uint8_t getTrackedNodesCount(void);
const uint8_t* getTrackedOrigins(void);

#endif // FLIGHT_BUILD

#endif  // CONSOLE_H
