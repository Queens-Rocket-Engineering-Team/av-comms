#pragma once

#include <Arduino.h>
#include <WiFi.h>

#define EARTH_RADIUS_FEET 20925524.9

inline const char FIRMWARE_VERSION[] = "Jun.11.2025, V1.1.0C";

// Pure constants => constexpr
inline constexpr uint8_t  NUM_RGB_LEDS   = 5;
inline constexpr uint32_t USB_BAUD       = 115200;
inline constexpr uint32_t GPS_BAUD       = 9600;
inline constexpr uint32_t EPD_SPI_CLOCK  = 1000000; // 1MHz
inline constexpr uint32_t EPD_BAUD       = 115200;
inline constexpr uint32_t RFM_SPI_CLOCK  = 2000000; // 2MHz
inline constexpr uint32_t I2C_SPEED      = 100000;  // 100kHz

// Strings: use inline constexpr char[]
inline constexpr const char AP_SSID[]     = "QRET Kuhglocke";
inline constexpr const char AP_PASSWORD[] = "BessieRocks";

inline constexpr uint8_t WIFI_CHANNEL = 10;
inline constexpr wifi_power_t WIFI_TX_POWER = WIFI_POWER_7dBm;

inline constexpr double   BATT_VDIV = 3.003579098067;
inline constexpr uint16_t SENSOR_SAMPLE_PERIOD = 250;

inline constexpr uint8_t  DEFAULT_LED_BRIGHTNESS = 15; // 0-255
inline constexpr uint16_t IND_RF_FLASH_TIME = 130;     // ms
inline constexpr uint16_t IND_BATTERY_FLASH_GAP = 750; // ms

inline constexpr uint16_t RFM_CONNECTED_TIMEOUT = 2000; // ms
inline constexpr uint8_t  RFM_PACKET_SIZE = 10;

inline constexpr uint32_t LOCAL_GPS_LOG_RATE = 1000;
inline constexpr uint32_t ALT_CORE_STACKS_SIZE = 10000;
inline constexpr uint16_t EPD_UPDATE_INT = 800;
