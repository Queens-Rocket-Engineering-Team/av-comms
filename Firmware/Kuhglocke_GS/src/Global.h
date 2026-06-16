#pragma once


#include "libraries.h"
#include <Arduino.h>
#include "pinouts.h"   

// ================================================================================================
// ================================= \/ GLOBAL CONSTANTS \/ =======================================
// ================================================================================================

#define EARTH_RADIUS_FEET 20925524.9

const char FIRMWARE_VERSION[] = "Jun.11.2025, V1.1.0C";

// Pure constants => constexpr (no multiple definition)
inline constexpr uint8_t  NUM_RGB_LEDS   = 5;
inline constexpr uint32_t USB_BAUD       = 115200;
inline constexpr uint32_t GPS_BAUD       = 9600;//supports up to 115200?
inline constexpr uint32_t EPD_SPI_CLOCK  = 1000000; // 1MHz
inline constexpr uint32_t EPD_BAUD       = 115200; //TODO: what does this baud actually do electrically?
inline constexpr uint32_t RFM_SPI_CLOCK  = 2000000; // 2MHz
inline constexpr uint32_t I2C_SPEED      = 100000;  // 100kHz

// Strings: use inline constexpr char[] (safe + no multiple-definition)
inline constexpr const char AP_SSID[]     = "QRET Kuhglocke";
inline constexpr const char AP_PASSWORD[] = "BessieRocks";

// These are also constants
inline constexpr uint8_t WIFI_CHANNEL = 10;
inline constexpr wifi_power_t WIFI_TX_POWER = WIFI_POWER_7dBm;//WIFI_POWER_19_5dBm

inline constexpr double   BATT_VDIV = 3.003579098067;
inline constexpr uint16_t SENSOR_SAMPLE_PERIOD = 250;//How often (ms) to sample onboard sensors (temp, batt volt, current, etc)


inline constexpr uint8_t  DEFAULT_LED_BRIGHTNESS = 15;//0-255
inline constexpr uint16_t IND_RF_FLASH_TIME = 130; //Duration (ms) of Ping LED flash time
inline constexpr uint16_t IND_BATTERY_FLASH_GAP = 750;//How long (ms) between LED flashes when battery is charging

inline constexpr uint16_t RFM_CONNECTED_TIMEOUT = 2000; //wait period after a ping to consider radio disconnected
inline constexpr uint8_t  RFM_PACKET_SIZE = 20;// expected num bytes in rocket packet

inline constexpr uint32_t LOCAL_GPS_LOG_RATE = 1000;
inline constexpr uint32_t ALT_CORE_STACKS_SIZE = 10000;// how frequently to update values on EPD (partial update)
inline constexpr uint16_t EPD_UPDATE_INT = 800;



// ================================================================================================
// ================================== \/ GLOBAL OBJECTS \/ ========================================
// ================================================================================================

// Second core task handle
extern TaskHandle_t Core0Task;

// Sensor median filters
extern MedianFilter<uint32_t> vccVoltFilter;
extern MedianFilter<uint32_t> battVoltFilter;
extern MedianFilter<uint32_t> sysCurrentFilter; //TODO: CHANGE THIS TO AN AVG FILTER
extern MedianFilter<uint32_t> ambTempFilter;

// Core loop median filter TODO: CHANGE THIS TO AN AVG FILTER
extern MedianFilter<uint16_t> core1LoopFilter;

// The TinyGPSPlus object
extern TinyGPSPlus gps;

// UART1 Serial Object
extern HardwareSerial gpsSerial;

// RGB LEDs object
extern Adafruit_NeoPixel rgbLEDs;
extern SPIClass epdSPI;
extern GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT> display;

// RFM95 Objects
extern SPIClass rfmSPI;
extern SPISettings rfmSPISettings;
extern RFM95 radio;

// NAU7802 ADC object
extern NAU7802 nau;

// AsyncWebServer object on port 80
extern AsyncWebServer webServer;

// Global log file handle
extern File logFile;




// for RFM_PACKET_SIZE (or wherever it is)

// ===== Radio menu options =====
extern double freqOpts[];
extern uint8_t numFreqOpts;

extern double bandwidthOpts[];
extern uint8_t numBandwidthOpts;

extern int32_t spreadOpts[];
extern uint8_t numSpreadOpts;

extern int32_t codingOpts[];
extern uint8_t numCodingOpts;

extern int32_t freqCorrectionOpts[];
extern uint8_t numFreqCorrectionOpts;

// ===== Selected indexes =====
extern uint8_t freqSelected;
extern uint8_t bandwidthSelected;
extern uint8_t spreadSelected;
extern uint8_t codingSelected;
extern uint8_t freqCorrectionSelected;

// ===== Incoming RFM95 packet data =====
extern volatile uint32_t rfmLastRFReceived;
extern volatile int16_t rfmLastRSSI;
extern volatile float rfmLastSNR;
extern volatile int32_t rfmLastFreqErr;
extern byte rfmLastPacket[RFM_PACKET_SIZE];
extern volatile bool rfmLastPacketValid;

// ===== Decoded rocket packet data =====
extern volatile int32_t rocketGPSLat;
extern volatile int32_t rocketGPSLon;
extern volatile uint8_t rocketGPSSats;
extern volatile int32_t rocketAltitude;
extern volatile uint8_t rocketStatus;
extern volatile double rocketVelocity;
extern volatile char rocketCallsign[7];

extern uint32_t lastLocalGPSLog;
extern int32_t curFreqOffset;

extern uint32_t lastEPDUpdate;

// ===== Core timing / flags =====
extern volatile uint32_t core0FreeStack;
extern volatile uint32_t core0LoopTime;
extern volatile uint32_t maxCore0LoopTime;
extern uint32_t maxCore1LoopTime;

extern volatile bool rfmReceivedFlag;

// ===== Sensors / UI =====
extern uint32_t lastSensorRead;
extern bool nauChannel;
extern bool usbDataOnly;

// ===== Indicators =====
extern uint32_t indRFFlashStart;
extern bool indBattFlashState;
extern uint32_t indBattLastToggle;

// ===== Logging =====
extern uint16_t logFileNumber;
