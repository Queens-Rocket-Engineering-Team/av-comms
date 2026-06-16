#include "qlcp_uplink.h"
#include "global.h"
#include "radio_control.h"
#include "power_sensors.h"
#include "gps.h"
extern "C" {
#include "wifi_tools.h"
#include <qlcp_lib.h>
}
#include <WiFi.h>
#include <esp_netif.h>

constexpr char kBoardQlcpConfigJson[] = R"json({
  "device_name": "KUHGLOCKE-GS",
  "device_type": "Ground Station",
  "sensor_info": {
    "gps": {
      "RocketLat": {
        "sensor_index": "RocketLat",
        "unit": "deg"
      },
      "RocketLon": {
        "sensor_index": "RocketLon",
        "unit": "deg"
      },
      "RocketAlt": {
        "sensor_index": "RocketAlt",
        "unit": "m"
      },
      "RocketSats": {
        "sensor_index": "RocketSats",
        "unit": ""
      }
    },
    "velocity": {
      "RocketVel": {
        "sensor_index": "RocketVel",
        "unit": "m/s"
      }
    },
    "radio": {
      "RadioRssi": {
        "sensor_index": "RadioRssi",
        "unit": "dBm"
      },
      "RadioSnr": {
        "sensor_index": "RadioSnr",
        "unit": "dB"
      }
    },
    "voltage_sense": {
      "GsBattery": {
        "sensor_index": "GsBattery",
        "unit": "V"
      }
    },
    "current_sensor": {
      "GsCurrent": {
        "sensor_index": "GsCurrent",
        "unit": "A"
      }
    },
    "thermocouple": {
      "GsTemp": {
        "sensor_index": "GsTemp",
        "unit": "C"
      }
    }
  },
  "controls": {}
})json";

enum QlcpNetState : uint8_t {
  QLCP_NET_IDLE = 0U,
  QLCP_NET_DISCOVER,
  QLCP_NET_TCP_CONNECT,
  QLCP_NET_CONNECTED,
  QLCP_NET_BACKOFF
};

static QlcpNetState s_netState = QLCP_NET_IDLE;
static uint32_t s_stateEnteredMs = 0U;
static uint32_t s_lastRxMs = 0U;
static uint32_t s_backoffMs = 1000U;
static bool s_configSent = false;
static net_link_t s_netLink = {};

static uint16_t s_sequence = 0U;
static uint32_t s_tsOffset = 0U;
static uint16_t s_streamFrequencyHz = 0U;
static uint32_t s_lastStreamTxMs = 0U;

constexpr uint32_t kNetTcpConnectTimeoutMs = 5000U;
constexpr uint32_t kNetRxIdleTimeoutMs    = 15000U;
constexpr uint32_t kNetBackoffMinMs       = 1000U;
constexpr uint32_t kNetBackoffMaxMs       = 8000U;

static void fillHeader(qlcp_header& header) {
  header.sequence = static_cast<uint8_t>(s_sequence++);
  header.timestamp = s_tsOffset + millis();
}

static void netTransition(QlcpNetState next, uint32_t nowMs) {
  Serial.printf("QLCP net: %u -> %u\n", s_netState, next);
  s_netState = next;
  s_stateEnteredMs = nowMs;
}

static void netFail(uint32_t nowMs) {
  net_link_close_all(&s_netLink);
  s_streamFrequencyHz = 0U;
  s_configSent = false;
  netTransition(QLCP_NET_BACKOFF, nowMs);
}

static void sendTelemetry() {
  qlcp_sensor_data readings[10] = {};

  // 0. Rocket Lat
  readings[0].sensor_id = 0;
  readings[0].unit = QLCP_UNIT_UNITLESS;
  readings[0].value = static_cast<float>(getRocketLatDeg());

  // 1. Rocket Lon
  readings[1].sensor_id = 1;
  readings[1].unit = QLCP_UNIT_UNITLESS;
  readings[1].value = static_cast<float>(getRocketLonDeg());

  // 2. Rocket Alt
  readings[2].sensor_id = 2;
  readings[2].unit = QLCP_UNIT_UNITLESS;
  readings[2].value = static_cast<float>(getRocketAltitudeMeters());

  // 3. Rocket Sats
  readings[3].sensor_id = 3;
  readings[3].unit = QLCP_UNIT_UNITLESS;
  readings[3].value = static_cast<float>(getRocketGPSSats());

  // 4. Rocket Velocity
  readings[4].sensor_id = 4;
  readings[4].unit = QLCP_UNIT_UNITLESS;
  readings[4].value = static_cast<float>(getRocketVelocity());

  // 5. Radio RSSI
  readings[5].sensor_id = 5;
  readings[5].unit = QLCP_UNIT_UNITLESS;
  readings[5].value = static_cast<float>(getRfmLastRSSI());

  // 6. Radio SNR
  readings[6].sensor_id = 6;
  readings[6].unit = QLCP_UNIT_UNITLESS;
  readings[6].value = static_cast<float>(getRfmLastSNR());

  // 7. GS Battery Voltage
  readings[7].sensor_id = 7;
  readings[7].unit = QLCP_UNIT_VOLTS;
  readings[7].value = getBatteryVoltage() / 1000.0f;

  // 8. GS System Current
  readings[8].sensor_id = 8;
  readings[8].unit = QLCP_UNIT_AMPS;
  readings[8].value = getSystemCurrent() / 1000.0f;

  // 9. GS Ambient Temp
  readings[9].sensor_id = 9;
  readings[9].unit = QLCP_UNIT_CELSIUS;
  readings[9].value = getAmbTemperature() / 100.0f;

  qlcp_data_packet pkt = {};
  fillHeader(pkt.header);
  pkt.sensor_data = readings;
  pkt.sensor_count = 10;

  (void)udp_send_data(&s_netLink, &pkt);
}

static void qlcpTelemetryService(uint32_t nowMs) {
  if ((s_netState != QLCP_NET_CONNECTED) || (s_streamFrequencyHz == 0U)) {
    return;
  }
  const uint32_t periodMs = 1000U / s_streamFrequencyHz;
  if ((nowMs - s_lastStreamTxMs) < periodMs) {
    return;
  }
  s_lastStreamTxMs = nowMs;

  sendTelemetry();
}

static void sendAck(uint8_t ackType, uint16_t ackSeq) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_ACK;
  fillHeader(out.payload_data.ack.header);
  out.payload_data.ack.ack_packet_type = ackType;
  out.payload_data.ack.ack_sequence = ackSeq;
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    Serial.println("[WARN] ACK dropped - TX busy");
  }
}

static void sendNack(uint8_t nackType, uint16_t nackSeq, uint8_t errCode) {
  qlcp_server_payload out = {};
  out.packet_type = QLCP_PT_NACK;
  fillHeader(out.payload_data.nack.header);
  out.payload_data.nack.nack_packet_type = nackType;
  out.payload_data.nack.nack_sequence = nackSeq;
  out.payload_data.nack.nack_error_code = errCode;
  if (tcp_tx_payload(&s_netLink, &out) != 0) {
    Serial.println("[WARN] NACK dropped - TX busy");
  }
}

static void qlcpHandlePacket(const qlcp_client_payload& in) {
  switch (in.packet_type) {
    case QLCP_PT_TIMESYNC: {
      const uint32_t serverTime = in.payload_data.header_only.timestamp;
      s_tsOffset = serverTime - millis();
      Serial.printf("QLCP timesync completed. Offset: %u ms\n", s_tsOffset);
      sendAck(QLCP_PT_TIMESYNC, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_HEARTBEAT: {
      sendAck(QLCP_PT_HEARTBEAT, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_STREAM_START: {
      const uint16_t freq = in.payload_data.stream_start.stream_frequency;
      if (freq > 0U) {
        s_streamFrequencyHz = freq;
        s_lastStreamTxMs = millis();
        Serial.printf("QLCP Stream Start at %u Hz\n", freq);
      }
      sendAck(QLCP_PT_STREAM_START, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_STREAM_STOP: {
      s_streamFrequencyHz = 0U;
      Serial.println("QLCP Stream Stop");
      sendAck(QLCP_PT_STREAM_STOP, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_GET_SINGLE: {
      sendTelemetry();
      break;
    }
    case QLCP_PT_ESTOP: {
      Serial.println("[WARN] ESTOP received on Ground Station!");
      sendAck(QLCP_PT_ESTOP, in.payload_data.header_only.sequence);
      break;
    }
    case QLCP_PT_STATUS_REQUEST: {
      qlcp_server_payload out = {};
      out.packet_type = QLCP_PT_STATUS;
      fillHeader(out.payload_data.status.header);
      out.payload_data.status.control_data = nullptr;
      out.payload_data.status.control_count = 0;
      out.payload_data.status.device_status = QLCP_DS_ACTIVE;
      if (tcp_tx_payload(&s_netLink, &out) != 0) {
        Serial.println("[WARN] STATUS dropped - TX busy");
      }
      break;
    }
    case QLCP_PT_CONTROL: {
      const uint8_t cmdId = in.payload_data.control.command_id;
      const uint8_t state = in.payload_data.control.command_state;
      Serial.printf("Received control cmdId=%u state=%u\n", cmdId, state);
      
      bool success = false;
      switch (cmdId) {
        case 0: // FreqUp
          if (state == 1) {
            changeFreqOffset(1000);
            success = true;
          }
          break;
        case 1: // FreqDown
          if (state == 1) {
            changeFreqOffset(-1000);
            success = true;
          }
          break;
        case 2: // RadioReload
          if (state == 1) {
            rfmInit();
            success = true;
          }
          break;
        case 3: // SetBandwidth
          setRadioConfig("bandwidth", state);
          success = true;
          break;
        case 4: // SetSpreadingFactor
          setRadioConfig("spreadingfactor", state);
          success = true;
          break;
        case 5: // SetCodingRate
          setRadioConfig("codingrate", state);
          success = true;
          break;
        default:
          break;
      }
      
      if (success) {
        sendAck(QLCP_PT_CONTROL, in.payload_data.header_only.sequence);
      } else {
        sendNack(QLCP_PT_CONTROL, in.payload_data.header_only.sequence, QLCP_ERR_INVALID_PARAM);
      }
      break;
    }
    default: {
      sendNack(in.packet_type, in.payload_data.header_only.sequence, QLCP_ERR_UNKNOWN_TYPE);
      break;
    }
  }
}

void qlcpUplinkInit() {
  net_link_init(&s_netLink);
}

void qlcpUplinkService() {
  uint32_t nowMs = millis();
  
  if (WiFi.status() != WL_CONNECTED) {
    if (s_netState != QLCP_NET_IDLE && s_netState != QLCP_NET_BACKOFF) {
      netFail(nowMs);
    }
    if (s_netState == QLCP_NET_BACKOFF) {
      if (nowMs - s_stateEnteredMs >= s_backoffMs) {
        s_stateEnteredMs = nowMs;
      }
    }
    return;
  }
  
  switch (s_netState) {
    case QLCP_NET_IDLE: {
      s_netLink.netif_handle = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
      if (ssdp_listen_begin(&s_netLink) == ESP_OK) {
        netTransition(QLCP_NET_DISCOVER, nowMs);
      } else {
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_DISCOVER: {
      const int found = ssdp_listen_service(&s_netLink);
      if (found == 1) {
        ssdp_listen_end(&s_netLink);
        if (tcp_connect_begin(&s_netLink) == ESP_OK) {
          netTransition(QLCP_NET_TCP_CONNECT, nowMs);
        } else {
          netFail(nowMs);
        }
      } else if (found < 0) {
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_TCP_CONNECT: {
      const int conn = tcp_connect_service(&s_netLink);
      if (conn == 1) {
        if (udp_create_socket(&s_netLink) != ESP_OK) {
          netFail(nowMs);
          break;
        }
        s_configSent = false;
        s_lastRxMs = nowMs;
        s_backoffMs = kNetBackoffMinMs;
        netTransition(QLCP_NET_CONNECTED, nowMs);
      } else if ((conn < 0) || ((nowMs - s_stateEnteredMs) >= kNetTcpConnectTimeoutMs)) {
        netFail(nowMs);
      }
      break;
    }
    case QLCP_NET_CONNECTED: {
      if (!s_configSent) {
        qlcp_server_payload out = {};
        out.packet_type = QLCP_PT_CONFIG;
        fillHeader(out.payload_data.config.header);
        out.payload_data.config.config_data = kBoardQlcpConfigJson;
        out.payload_data.config.config_data_len = sizeof(kBoardQlcpConfigJson) - 1U;
        if (tcp_tx_payload(&s_netLink, &out) == 0) {
          s_configSent = true;
          Serial.println("Sent CONFIG packet to server");
        }
      }
      if (tcp_tx_service(&s_netLink) < 0) {
        netFail(nowMs);
        break;
      }
      qlcp_client_payload in = {};
      const int rx = tcp_rx_service(&s_netLink, &in);
      if (rx < 0) {
        netFail(nowMs);
        break;
      }
      if (rx == 1) {
        s_lastRxMs = nowMs;
        qlcpHandlePacket(in);
      }
      if ((nowMs - s_lastRxMs) >= kNetRxIdleTimeoutMs) {
        Serial.println("[WARN] QLCP server silent - reconnecting");
        netFail(nowMs);
        break;
      }
      
      qlcpTelemetryService(nowMs);
      break;
    }
    case QLCP_NET_BACKOFF: {
      if ((nowMs - s_stateEnteredMs) >= s_backoffMs) {
        s_backoffMs = (s_backoffMs >= (kNetBackoffMaxMs / 2U)) ? kNetBackoffMaxMs : (s_backoffMs * 2U);
        if (ssdp_listen_begin(&s_netLink) == ESP_OK) {
          netTransition(QLCP_NET_DISCOVER, nowMs);
        } else {
          netFail(nowMs);
        }
      }
      break;
    }
    default:
      break;
  }
}
