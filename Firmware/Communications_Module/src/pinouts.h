#pragma once

#include <Arduino.h>  // STM32 variant pin macros (PA0, PB10, ...)
#include <cstdint>

// pinouts.h - Communications Module (STM32F103CBT6) pin map.
// Values are MCU-native STM32 Arduino pin macros.
namespace pins {

// --- USB-UART bridge (USART3, names are from the MCU perspective) ---
// The schematic net names are reversed at the bridge:
//   PB10 / USART3_TX -> USB_RX
//   PB11 / USART3_RX <- USB_TX
constexpr uint8_t kSerialTx = PB10;
constexpr uint8_t kSerialRx = PB11;

// --- LoRa radio (SPI) ---
constexpr uint8_t kRfBusy  = PA0;
constexpr uint8_t kRfDio1  = PA1;
constexpr uint8_t kRfDio3  = PA2;
constexpr uint8_t kRfReset = PA3;
constexpr uint8_t kRfCs    = PA4;
constexpr uint8_t kRfSclk  = PA5;
constexpr uint8_t kRfMiso  = PA6;
constexpr uint8_t kRfMosi  = PA7;

// --- Flash (SPI) ---
constexpr uint8_t kFlashReset = PA8;
constexpr uint8_t kFlashCs    = PB12;
constexpr uint8_t kSpiSclk    = PB13;
constexpr uint8_t kSpiMiso    = PB14;
constexpr uint8_t kSpiMosi    = PB15;

// --- CAN bus ---
constexpr uint8_t kCanRx = PB8;
constexpr uint8_t kCanTx = PB9;

// --- Buzzer (TIM1 complementary PWM pair) ---
constexpr uint8_t kBuzzerA = PA10;  // TIM1_CH3
constexpr uint8_t kBuzzerB = PB1;   // TIM1_CH3N

// --- LEDs ---
constexpr uint8_t kRgbData  = PB3;
constexpr uint8_t kDebugLed = PA15;

// --- Analogue / power monitoring ---
constexpr uint8_t kBatterySense = PB0;

// --- Boot and SWD debug ---
constexpr uint8_t kBoot1 = PB2;
constexpr uint8_t kSwdio = PA13;
constexpr uint8_t kSwclk = PA14;

// --- Test points ---
constexpr uint8_t kTp4 = PA9;
constexpr uint8_t kTp6 = PB6;
constexpr uint8_t kTp7 = PB7;

}  // namespace pins
