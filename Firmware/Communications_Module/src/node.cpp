#include "node.h"
#include <Adafruit_NeoPixel.h>
#include <aim_job.h>
#include <logger.h>
#include <SPI.h>
#include <RadioLib.h>
#include <lora_link.h>

static lora::LivenessTracker s_liveness;

// LoRa SPI & Radio Objects
static SPIClass s_loraSpi(pins::kRfMosi, pins::kRfMiso, pins::kRfSclk);
static Module s_radioModule(pins::kRfCs, pins::kRfDio1, pins::kRfReset, pins::kRfBusy, s_loraSpi);
static SX1262 s_radio(&s_radioModule);

static bool s_transmitting = false;
static volatile bool s_transmittedFlag = false;
static bool s_loraInitOk = false;
static bool s_lowPower = false;

static Adafruit_NeoPixel s_rgbLeds(1U, pins::kRgbData, NEO_GRB + NEO_KHZ800);

struct StateSnapshot {
  lora::FastFrame fast;
  lora::SlowFrame slow;
};

static StateSnapshot s_snapshot = {};
static uint8_t s_seqCnt = 0;

static aim::Job s_fastTxJob{50U, 0U};   // 20 Hz
static aim::Job s_slowTxJob{1000U, 0U}; // 1 Hz

static void setTxFlag(void) {
  s_transmittedFlag = true;
}

static void updateLed(aim::NodeState state) {
  static aim::NodeState s_lastState = static_cast<aim::NodeState>(0xFF);
  if (state == s_lastState) return;
  s_lastState = state;
  uint8_t r = 0, g = 0, b = 0;
  switch (state) {
    case aim::NodeState::Nominal: g = 255; break;
    case aim::NodeState::Fault:   r = 255; break;
    default:                      b = 255; break;
  }
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(r, g, b));
  s_rgbLeds.show();
}

void nodeInit() {
  s_rgbLeds.begin();
  s_rgbLeds.setPixelColor(0, s_rgbLeds.Color(0, 0, 0));
  s_rgbLeds.show();

  s_loraSpi.begin();
  s_radio.reset();
  delay(100);
  int state = s_radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    s_loraInitOk = true;
    LOG_INFO("LoRa SX1262 init success");
  } else {
    LOG_ERROR("LoRa SX1262 init failed, code %d", state);
  }

  s_radio.setDio1Action(setTxFlag);

  // Configure explicit header mode
  s_radio.setFrequency(904.5);
  s_radio.setBandwidth(250.0);
  s_radio.setSpreadingFactor(7);
  s_radio.setCodingRate(5); // CR 4/5
  s_radio.setOutputPower(20);
  s_radio.setSyncWord(0x12);
  s_radio.setPreambleLength(8);
  s_radio.setCRC(true);
  s_radio.explicitHeader();
}

void nodeUpdate(uint32_t nowMs) {
  updateLed(nodeCurrentState());

  if (s_transmitting) {
    if (s_transmittedFlag) {
      s_transmittedFlag = false;
      s_radio.finishTransmit();
      s_transmitting = false;
    } else if (nowMs - s_slowTxJob.lastMs > 50U && nowMs - s_fastTxJob.lastMs > 50U) {
      s_radio.finishTransmit();
      s_transmitting = false;
      LOG_WARN("LoRa TX timeout recovered");
    } else {
      return;
    }
  }

  if (!s_transmitting && s_loraInitOk) {
    if (s_slowTxJob.due(nowMs)) {
      s_snapshot.slow.header.frame_type = 1;
      s_snapshot.slow.header.seq_cnt    = s_seqCnt;
      s_snapshot.slow.setLivenessMask(s_liveness.getMask(nowMs));

      uint8_t buf[lora::kSlowPacketSize];
      lora::encodeSlow(s_snapshot.slow, buf);

      int state = s_radio.startTransmit(buf, lora::kSlowPacketSize);
      if (state == RADIOLIB_ERR_NONE) {
        s_transmitting = true;
        s_seqCnt = (s_seqCnt + 1U) % 16U;
      }
    } else if (s_fastTxJob.due(nowMs)) {
      s_snapshot.fast.header.frame_type = 0;
      s_snapshot.fast.header.seq_cnt    = s_seqCnt;

      uint8_t buf[lora::kFastPacketSize];
      lora::encodeFast(s_snapshot.fast, buf);

      int state = s_radio.startTransmit(buf, lora::kFastPacketSize);
      if (state == RADIOLIB_ERR_NONE) {
        s_transmitting = true;
        s_seqCnt = (s_seqCnt + 1U) % 16U;
      }
    }
  }
}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
  (void)nowMs;
  (void)aim;
}

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  s_liveness.recordRx(m.source, nowMs);

  if (m.cls == aim::Class::Event) {
    if (m.subject == aim::subject::LowPower) {
      s_lowPower = (m.b[0] == 1U);
      LOG_INFO("Comms low power state updated: %d", s_lowPower);
    } else if (m.subject == aim::subject::LaunchDetect) {
      if (m.b[0] == 1U) {
        s_snapshot.fast.header.flight_state = static_cast<uint8_t>(aim::FlightPhase::Boost);
        s_snapshot.slow.header.flight_state = static_cast<uint8_t>(aim::FlightPhase::Boost);
      }
    }
  } else if (m.cls == aim::Class::State) {
    if (m.subject == aim::subject::Av203) {
      s_snapshot.slow.setSolenoidState(0, m.b[0] == 1U);
    } else if (m.subject == aim::subject::Av205) {
      s_snapshot.slow.setSolenoidState(1, m.b[0] == 1U);
    } else if (m.subject == aim::subject::Av204) {
      s_snapshot.slow.setSolenoidState(2, m.b[0] == 1U);
    } else {
      s_snapshot.slow.setSolenoidState(3, m.b[0] == 1U);
    }
  } else if (m.cls == aim::Class::Sensor) {
    if (m.subject == aim::subject::Altitude) {
      s_snapshot.fast.setAltitudeFromWire(m.sensorValue());
    } else if (m.subject == aim::subject::Acceleration) {
      s_snapshot.fast.setAccelFromWire(m.sensorValue());
    } else if (m.subject == aim::subject::Pt204) {
      s_snapshot.fast.setPressureFromWire(m.sensorValue());
    } else if (m.subject == aim::subject::GpsPosition) {
      int32_t lat = 0, lon = 0;
      m.getGpsPosition(lon, lat);
      s_snapshot.slow.setGpsPosition(lat, lon, s_snapshot.slow.getSatellites(), true);
    } else if (m.subject == aim::subject::GpsNumSats) {
      s_snapshot.slow.gps_sats = m.b[0] & 0x0F;
    } else if (m.subject == aim::subject::BattVolt) {
      s_snapshot.slow.setBatteryVolts(static_cast<float>(m.sensorValue()) / 1000.0f);
    }
  }
}

aim::NodeState nodeCurrentState() {
  if (!s_loraInitOk) {
    return aim::NodeState::Fault;
  }
  return aim::NodeState::Nominal;
}

uint16_t nodeErrorBits() {
  return 0U;
}

#ifndef FLIGHT_BUILD
static const char* sourceName(aim::Source src) {
  switch (src) {
    case aim::Source::Comms:     return "COMMS";
    case aim::Source::Ucm:       return "UCM";
    case aim::Source::Lcm:       return "LCM";
    case aim::Source::Altimeter: return "ALT";
    case aim::Source::Gps:       return "GPS";
    case aim::Source::Power:     return "PWR";
    default:                     return "?";
  }
}

static void hookLiveness(Stream& out) {
  const uint32_t nowMs = millis();
  out.println("Node liveness:");
  for (uint8_t i = 0U; i < lora::kTrackedNodeCount; i++) {
    out.print("  ");
    out.print(lora::sourceName(static_cast<aim::Source>(i + 1)));
    out.print(": ");
    if (!s_liveness.everHeard(i)) {
      out.println("never heard");
    } else {
      const uint32_t ageMs = nowMs - s_liveness.lastHeardMs(i);
      out.print(s_liveness.isAlive(i, nowMs) ? "ALIVE" : "DEAD");
      out.print(" ageMs=");
      out.println(static_cast<unsigned long>(ageMs));
    }
  }
}

static const AimConsoleHook s_consoleHooks[] = {
  {'n', "node liveness", hookLiveness},
};

const AimConsoleHook* nodeConsoleHooks(uint8_t& count) {
  count = sizeof(s_consoleHooks) / sizeof(s_consoleHooks[0]);
  return s_consoleHooks;
}
#endif
