#include "node.h"
#include <aim_job.h>
#include <logger.h>

// The six AIM nodes Comms tracks for liveness. Comms itself is included so the
// console table is complete; its own row simply never goes stale.
static constexpr uint8_t kTrackedNodes = 6U;
static NodeLiveness s_liveness[kTrackedNodes];

void nodeInit(uint32_t nowMs) {
  nodeLivenessInit(nowMs);
}

void nodeUpdate(uint32_t nowMs) {
  // Stub: LoRa RX processing will go here once the radio driver is wired in.
  (void)nowMs;
}

void nodeServiceCanTx(uint32_t nowMs, AimNetwork& aim) {
#ifdef AIM_COMMS_TIME_MASTER
  // No-UCM configuration: Comms is the sole TimeSync source from startup,
  // broadcasting at 1 Hz from its local clock (send() stamps syncedMillis(),
  // which equals millis() because Comms never receives TimeSync here).
  static aim::Job s_timeSyncJob{1000U};
  if (s_timeSyncJob.due(nowMs)) {
    aim::Msg m = {};
    m.cls = aim::Class::Time;
    m.subject = aim::subject::TimeSync;
    if (!aim.send(m)) {
      LOG_ERROR("TimeSync TX failed");
    }
  }
#else
  (void)nowMs;
  (void)aim;
#endif
}

void nodeLivenessInit(uint32_t nowMs) {
  static const aim::Source kSources[kTrackedNodes] = {
    aim::Source::Comms,
    aim::Source::Ucm,
    aim::Source::Lcm,
    aim::Source::Altimeter,
    aim::Source::Gps,
    aim::Source::Power,
  };

  for (uint8_t i = 0U; i < kTrackedNodes; i++) {
    s_liveness[i].source = kSources[i];
    s_liveness[i].lastHeardMs = nowMs;
    s_liveness[i].everHeard = false;
  }
}

void nodeLivenessOnRx(aim::Source source, uint32_t nowMs) {
  for (uint8_t i = 0U; i < kTrackedNodes; i++) {
    if (s_liveness[i].source == source) {
      s_liveness[i].lastHeardMs = nowMs;
      s_liveness[i].everHeard = true;
      return;
    }
  }
}

const NodeLiveness* nodeLivenessTable(uint8_t* countOut) {
  if (countOut != nullptr) {
    *countOut = kTrackedNodes;
  }
  return s_liveness;
}

static bool s_lowPower = false;

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  nodeLivenessOnRx(m.source, nowMs);
  
  if (m.cls == aim::Class::Event && m.subject == aim::subject::LowPower) {
    s_lowPower = (m.b[0] == 1U);
    LOG_INFO("Comms low power state updated: %d", s_lowPower);
  }
}

aim::NodeState nodeCurrentState() {
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
  uint8_t count = 0U;
  const NodeLiveness* table = nodeLivenessTable(&count);
  const uint32_t nowMs = millis();
  static constexpr uint32_t kLivenessTimeoutMs = 10000U;

  out.println("Node liveness:");
  for (uint8_t i = 0U; i < count; i++) {
    const uint32_t ageMs = nowMs - table[i].lastHeardMs;
    const bool alive = table[i].everHeard && (ageMs < kLivenessTimeoutMs);
    out.print("  ");
    out.print(sourceName(table[i].source));
    out.print(" (0x");
    out.print(static_cast<unsigned>(table[i].source), HEX);
    out.print("): ");
    if (!table[i].everHeard) {
      out.println("never heard");
    } else {
      out.print(alive ? "ALIVE" : "DEAD");
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
