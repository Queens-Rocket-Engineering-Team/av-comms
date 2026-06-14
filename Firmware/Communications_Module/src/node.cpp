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

void nodeOnRx(const aim::Msg& m, uint32_t nowMs) {
  nodeLivenessOnRx(m.source, nowMs);
  // TODO: forward frame over LoRa when the radio driver is wired in.
}
