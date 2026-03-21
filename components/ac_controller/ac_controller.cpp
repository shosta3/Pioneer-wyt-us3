#include "ac_controller.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"

namespace esphome {
namespace ac_controller {

static const char *const TAG = "ac_controller";

// ─────────────────────────────────────────────────────────────────────────────
// CRC-16/XMODEM  poly=0x1021  init=0x0000  no reflection
// ─────────────────────────────────────────────────────────────────────────────
uint16_t AcController::crc16_xmodem(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t) data[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame building
// ─────────────────────────────────────────────────────────────────────────────
std::vector<uint8_t> AcController::build_frame(const std::vector<uint8_t> &payload,
                                                uint8_t dev_id) {
  uint8_t total_len = (uint8_t)(10 + payload.size());
  std::vector<uint8_t> frame;
  frame.reserve(total_len);
  frame.push_back(FRAME_HEADER);
  frame.push_back(BUS_ID);
  frame.push_back(dev_id);
  frame.push_back(TYPE_CMD);
  frame.push_back(tx_seq_);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(total_len);
  frame.push_back(0x00);  // CRC placeholder
  frame.push_back(0x00);
  for (auto b : payload) frame.push_back(b);

  std::vector<uint8_t> crc_input(frame.begin(), frame.begin() + 8);
  crc_input.insert(crc_input.end(), payload.begin(), payload.end());
  uint16_t crc = crc16_xmodem(crc_input.data(), crc_input.size());
  frame[8] = (crc >> 8) & 0xFF;
  frame[9] = crc & 0xFF;
  return frame;
}

std::vector<uint8_t> AcController::build_ack_frame(uint8_t seq, uint8_t dev_id) {
  std::vector<uint8_t> frame;
  frame.push_back(FRAME_HEADER);
  frame.push_back(BUS_ID);
  frame.push_back(dev_id);
  frame.push_back(TYPE_ACK);
  frame.push_back(0x00);
  frame.push_back(seq);
  frame.push_back(0x00);
  frame.push_back(12);
  frame.push_back(0x00);
  frame.push_back(0x00);
  // ACK payload mirrors the prefix: 0x80 + incoming prefix byte
  uint8_t mirror = (dev_id == DEV_HEARTBEAT) ? PREFIX_INDOOR_LO : PREFIX_INDOOR_LO;
  frame.push_back(0x80);
  frame.push_back(PREFIX_INDOOR_LO);  // 0x0C

  uint8_t crc_buf[10] = {
    frame[0], frame[1], frame[2], frame[3],
    frame[4], frame[5], frame[6], frame[7],
    0x80, PREFIX_INDOOR_LO
  };
  uint16_t crc = crc16_xmodem(crc_buf, 10);
  frame[8] = (crc >> 8) & 0xFF;
  frame[9] = crc & 0xFF;
  return frame;
}

// ─────────────────────────────────────────────────────────────────────────────
// TLV helpers
// ─────────────────────────────────────────────────────────────────────────────
bool AcController::is_4byte_special(uint8_t reg) {
  return reg == REG_OUTDOOR_COIL || reg == 0x64 ||
         reg == REG_INDOOR_COIL  || reg == REG_FAN_RPM;
}

std::vector<TlvEntry> AcController::parse_tlv(const uint8_t *data, size_t len) {
  std::vector<TlvEntry> entries;
  size_t i = 0;
  while (i + 1 < len) {
    uint8_t bank = data[i];
    uint8_t reg  = data[i + 1];
    TlvEntry e;
    e.bank = bank;
    e.reg  = reg;

    if (is_4byte_special(reg)) {
      if (i + 6 > len) break;
      e.value = ((uint32_t) data[i+2] << 24) | ((uint32_t) data[i+3] << 16) |
                ((uint32_t) data[i+4] <<  8) |  (uint32_t) data[i+5];
      e.size = 4; i += 6;
    } else if (bank == 0x00) {
      if (i + 3 > len) break;
      e.value = data[i + 2]; e.size = 1; i += 3;
    } else if (bank == 0x01) {
      if (i + 4 > len) break;
      e.value = ((uint32_t) data[i+2] << 8) | data[i+3];
      e.size = 2; i += 4;
    } else if (bank == 0x02) {
      if (i + 6 > len) break;
      e.value = ((uint32_t) data[i+2] << 24) | ((uint32_t) data[i+3] << 16) |
                ((uint32_t) data[i+4] <<  8) |  (uint32_t) data[i+5];
      e.size = 4; i += 6;
    } else {
      i += 2; continue;
    }
    entries.push_back(e);
  }
  return entries;
}

static std::vector<uint8_t> build_tlv_payload(const std::vector<TlvEntry> &entries) {
  std::vector<uint8_t> payload;
  payload.push_back(PREFIX_CTRL_HI);
  payload.push_back(PREFIX_CTRL_LO);
  for (size_t i = 0; i < entries.size(); i++) {
    const TlvEntry &e = entries[i];
    payload.push_back(e.bank);
    payload.push_back(e.reg);
    if (e.size == 1) {
      payload.push_back(e.value & 0xFF);
    } else if (e.size == 2) {
      payload.push_back((e.value >> 8) & 0xFF);
      payload.push_back(e.value & 0xFF);
    } else {
      payload.push_back((e.value >> 24) & 0xFF);
      payload.push_back((e.value >> 16) & 0xFF);
      payload.push_back((e.value >>  8) & 0xFF);
      payload.push_back(e.value & 0xFF);
    }
  }
  return payload;
}

// ─────────────────────────────────────────────────────────────────────────────
// TX queue
// ─────────────────────────────────────────────────────────────────────────────
void AcController::enqueue_command(const std::vector<uint8_t> &payload,
                                   uint8_t dev_id) {
  PendingCommand cmd;
  cmd.payload = payload;
  cmd.dev_id  = dev_id;
  tx_queue_.push(cmd);
}

void AcController::send_register(uint8_t bank, uint8_t reg, uint32_t value) {
  uint8_t size = (bank == 0x02 || is_4byte_special(reg)) ? 4 : (bank == 0x01 ? 2 : 1);
  TlvEntry e;
  e.bank = bank; e.reg = reg; e.value = value; e.size = size;
  std::vector<TlvEntry> v;
  v.push_back(e);
  send_registers(v);
}

void AcController::send_registers(const std::vector<TlvEntry> &entries) {
  enqueue_command(build_tlv_payload(entries), DEV_NORMAL);
}

void AcController::send_frame(const std::vector<uint8_t> &frame) {
  write_array(frame.data(), frame.size());
  ESP_LOGV(TAG, "TX [%d bytes] dev=%02X", (int) frame.size(), frame[2]);
}

void AcController::maybe_send_next_command() {
  if (waiting_for_ack_) {
    if (millis() - last_tx_ms_ > ACK_TIMEOUT_MS) {
      ESP_LOGW(TAG, "ACK timeout, retrying");
      waiting_for_ack_ = false;
    } else {
      return;
    }
  }
  if (tx_queue_.empty()) return;

  PendingCommand &cmd = tx_queue_.front();
  auto frame = build_frame(cmd.payload, cmd.dev_id);
  send_frame(frame);
  last_tx_ms_ = millis();
  waiting_for_ack_ = true;
  tx_seq_ = (tx_seq_ == 0xFF) ? 0x01 : tx_seq_ + 1;
  tx_queue_.pop();
}

// ─────────────────────────────────────────────────────────────────────────────
// Handshake state machine
//
// Mirrors the WiFi module startup sequence observed in captures:
//   1. [00 00 01] dev=00           — identity query (x4, ignored response)
//   2. [0B 0B FF FF] dev=01        — announce as controller (x2, wait [80 0B])
//   3. [0B 0B 03 05] dev=00 x2     — bus-level handshake
//   4. [28 28 00 01 ...] dev=00    — indoor sends this TO us (power state push), we ACK
//   5. [15 15 <ts> FC] dev=00      — timestamp frame (no ack expected)
//   After: indoor sends 26 26 / 10 10 / 0C 0C — respond to each
// ─────────────────────────────────────────────────────────────────────────────
void AcController::run_handshake() {
  uint32_t now = millis();

  // Build raw frames directly for handshake (bypasses normal TX queue)
  auto send_hs = [&](const std::vector<uint8_t> &payload, uint8_t dev_id) {
    uint8_t total_len = (uint8_t)(10 + payload.size());
    std::vector<uint8_t> frame;
    frame.push_back(FRAME_HEADER);
    frame.push_back(BUS_ID);
    frame.push_back(dev_id);
    frame.push_back(TYPE_CMD);
    frame.push_back(hs_seq_);
    frame.push_back(0x00);
    frame.push_back(0x00);
    frame.push_back(total_len);
    frame.push_back(0x00);
    frame.push_back(0x00);
    for (auto b : payload) frame.push_back(b);
    std::vector<uint8_t> ci(frame.begin(), frame.begin() + 8);
    ci.insert(ci.end(), payload.begin(), payload.end());
    uint16_t crc = crc16_xmodem(ci.data(), ci.size());
    frame[8] = (crc >> 8) & 0xFF;
    frame[9] = crc & 0xFF;
    send_frame(frame);
    hs_seq_ = (hs_seq_ == 0xFF) ? 0x01 : hs_seq_ + 1;
    hs_step_ms_ = now;
    hs_waiting_ack_ = true;
  };

  switch (hs_state_) {

    case HS_IDLE:
      ESP_LOGI(TAG, "Starting handshake sequence");
      send_hs({0x00, 0x00, 0x01}, DEV_HEARTBEAT);
      hs_state_ = HS_IDENTITY_1;
      break;

    case HS_IDENTITY_1:
      if (!hs_waiting_ack_) {
        // Got identity response — now announce as controller
        send_hs({0x0B, 0x0B, 0xFF, 0xFF}, DEV_NORMAL);
        hs_state_ = HS_ANNOUNCE_1;
      } else if (now - hs_step_ms_ > ACK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "HS identity_1 timeout, retrying");
        hs_state_ = HS_IDLE;
      }
      break;

    case HS_ANNOUNCE_1:
      // [0B 0B FF FF] sent — indoor may or may not send [80 0B].
      // Advance on ACK OR after a short timeout — CtrlCount increment confirms registration.
      if (!hs_waiting_ack_ || (now - hs_step_ms_ > ACK_TIMEOUT_MS)) {
        if (hs_waiting_ack_) {
          ESP_LOGD(TAG, "HS announce_1: no ACK but advancing (indoor may not reply)");
        }
        hs_waiting_ack_ = false;
        send_hs({0x00, 0x00, 0x01}, DEV_HEARTBEAT);
        hs_state_ = HS_IDENTITY_2;
      }
      break;

    case HS_IDENTITY_2:
      if (!hs_waiting_ack_) {
        send_hs({0x0B, 0x0B, 0x03, 0x05}, DEV_HEARTBEAT);
        hs_state_ = HS_BUS_HANDSHAKE_A;
      } else if (now - hs_step_ms_ > ACK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "HS identity_2 timeout, retrying");
        hs_state_ = HS_IDLE;
      }
      break;

    case HS_BUS_HANDSHAKE_A:
      if (!hs_waiting_ack_) {
        send_hs({0x0B, 0x0B, 0x03, 0x05}, DEV_HEARTBEAT);
        hs_state_ = HS_BUS_HANDSHAKE_B;
      } else if (now - hs_step_ms_ > ACK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "HS bus_hs_a timeout, retrying");
        hs_state_ = HS_IDLE;
      }
      break;

    case HS_BUS_HANDSHAKE_B:
      if (!hs_waiting_ack_) {
        send_hs({0x00, 0x00, 0x01}, DEV_HEARTBEAT);
        hs_state_ = HS_IDENTITY_3;
      } else if (now - hs_step_ms_ > ACK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "HS bus_hs_b timeout, retrying");
        hs_state_ = HS_IDLE;
      }
      break;

    case HS_IDENTITY_3:
      if (!hs_waiting_ack_) {
        // Wait ~5s before second controller announce
        hs_state_ = HS_WAIT_REANNOUNCE;
        hs_step_ms_ = now;
        hs_waiting_ack_ = false;
      } else if (now - hs_step_ms_ > ACK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "HS identity_3 timeout, retrying");
        hs_state_ = HS_IDLE;
      }
      break;

    case HS_WAIT_REANNOUNCE:
      if (now - hs_step_ms_ > HS_REANNOUNCE_MS) {
        send_hs({0x0B, 0x0B, 0xFF, 0xFF}, DEV_NORMAL);
        hs_state_ = HS_ANNOUNCE_2;
      }
      break;

    case HS_ANNOUNCE_2:
      // [0B 0B FF FF] sent second time — advance on ACK or timeout.
      if (!hs_waiting_ack_ || (now - hs_step_ms_ > ACK_TIMEOUT_MS)) {
        hs_waiting_ack_ = false;
        send_hs({0x00, 0x00, 0x01}, DEV_HEARTBEAT);
        hs_state_ = HS_IDENTITY_4;
      }
      break;

    case HS_POWER_SYNC:
      // Unused — falls through to identity_4 immediately
      hs_state_ = HS_IDENTITY_4;
      hs_waiting_ack_ = false;
      break;

    case HS_IDENTITY_4:
      if (!hs_waiting_ack_) {
        // Send timestamp frame: [15 15 <4-byte millis/0> FC]
        // We don't have a real clock so use 0x00000000 for the timestamp.
        // The indoor unit doesn't validate it — it just stores it.
        send_hs({0x15, 0x15, 0x00, 0x00, 0x00, 0x00, 0xFC}, DEV_HEARTBEAT);
        hs_state_ = HS_TIMESTAMP;
        // 15 15 frame: no ack expected from indoor, just advance after short delay
        hs_waiting_ack_ = false;
        hs_step_ms_ = now;
      } else if (now - hs_step_ms_ > ACK_TIMEOUT_MS) {
        ESP_LOGW(TAG, "HS identity_4 timeout, retrying");
        hs_state_ = HS_IDLE;
      }
      break;

    case HS_TIMESTAMP:
      // No ACK expected — wait a short moment then declare handshake complete.
      // The indoor will now send 26 26 / 10 10 / 0C 0C which handle_indoor_frame
      // will respond to automatically.
      if (now - hs_step_ms_ > 500) {
        ESP_LOGI(TAG, "Handshake complete — waiting for indoor bus sequence");
        hs_state_ = HS_COMPLETE;
        last_poll_ms_ = now;  // reset poll timer
      }
      break;

    case HS_COMPLETE:
      // Nothing to do here — periodic poll is handled in loop()
      break;
  }
}

// Called from handle_ack_frame to advance the handshake
void AcController::advance_handshake(bool ack_received,
                                     const std::vector<uint8_t> &frame) {
  if (hs_state_ == HS_COMPLETE) return;
  if (ack_received) {
    hs_waiting_ack_ = false;
    ESP_LOGD(TAG, "HS ACK received in state %d", (int) hs_state_);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// RX parsing
// ─────────────────────────────────────────────────────────────────────────────
void AcController::process_rx_byte(uint8_t byte) {
  if (rx_buf_.empty() && byte != FRAME_HEADER) return;
  rx_buf_.push_back(byte);
  if (rx_buf_.size() < 8) return;

  uint8_t expected_len = rx_buf_[7];
  if (expected_len < 10 || expected_len > 250) {
    rx_buf_.clear(); return;
  }
  if (rx_buf_.size() < (size_t) expected_len) return;

  if (try_parse_frame()) {
    rx_buf_.clear();
  } else {
    rx_buf_.erase(rx_buf_.begin());
  }
}

bool AcController::try_parse_frame() {
  if (rx_buf_.size() < 10) return false;
  if (rx_buf_[0] != FRAME_HEADER) return false;

  uint8_t len = rx_buf_[7];
  if (rx_buf_.size() < (size_t) len) return false;

  std::vector<uint8_t> crc_input(rx_buf_.begin(), rx_buf_.begin() + 8);
  crc_input.insert(crc_input.end(), rx_buf_.begin() + 10, rx_buf_.begin() + len);
  uint16_t calc  = crc16_xmodem(crc_input.data(), crc_input.size());
  uint16_t given = ((uint16_t) rx_buf_[8] << 8) | rx_buf_[9];

  if (calc != given) {
    ESP_LOGW(TAG, "CRC mismatch calc=0x%04X given=0x%04X", calc, given);
    return false;
  }

  std::vector<uint8_t> frame(rx_buf_.begin(), rx_buf_.begin() + len);
  uint8_t type   = frame[3];
  uint8_t dev_id = frame[2];

  if (type == TYPE_CMD) {
    handle_indoor_frame(frame);
  } else if (type == TYPE_ACK) {
    handle_ack_frame(frame);
  }
  return true;
}

void AcController::handle_ack_frame(const std::vector<uint8_t> &frame) {
  uint8_t seq = frame[5];
  uint8_t dev = frame[2];
  ESP_LOGV(TAG, "RX ACK seq=%02X dev=%02X", seq, dev);

  // Release normal TX queue wait
  if (waiting_for_ack_) {
    waiting_for_ack_ = false;
  }

  // Advance handshake if in progress
  if (hs_state_ != HS_COMPLETE) {
    advance_handshake(true, frame);
  }
}

void AcController::handle_indoor_frame(const std::vector<uint8_t> &frame) {
  if (frame.size() < 12) return;

  uint8_t seq    = frame[4];
  uint8_t dev_id = frame[2];
  const uint8_t *payload = frame.data() + 10;
  size_t plen = frame.size() - 10;

  ESP_LOGV(TAG, "RX CMD seq=%02X dev=%02X len=%d", seq, dev_id, (int) frame.size());

  // ── Heartbeat bus frames (dev=00) ─────────────────────────────────────────
  if (dev_id == DEV_HEARTBEAT && plen >= 2) {
    uint8_t p0 = payload[0], p1 = payload[1];

    if (p0 == 0x26 && p1 == 0x26) {
      // [26 26] — no response needed
      ESP_LOGD(TAG, "RX 26 26 bus announce");
      return;
    }

    if (p0 == 0x28 && p1 == 0x28) {
      // [28 28] power state push from indoor — ACK with [80 28]
      ESP_LOGD(TAG, "RX 28 28 power sync from indoor");
      uint8_t ack_buf[12];
      ack_buf[0] = FRAME_HEADER; ack_buf[1] = BUS_ID; ack_buf[2] = DEV_HEARTBEAT;
      ack_buf[3] = TYPE_ACK; ack_buf[4] = 0x00; ack_buf[5] = seq;
      ack_buf[6] = 0x00; ack_buf[7] = 12;
      ack_buf[8] = 0x00; ack_buf[9] = 0x00;
      ack_buf[10] = 0x80; ack_buf[11] = 0x28;
      uint8_t ci[10] = {ack_buf[0],ack_buf[1],ack_buf[2],ack_buf[3],
                        ack_buf[4],ack_buf[5],ack_buf[6],ack_buf[7],0x80,0x28};
      uint16_t crc = crc16_xmodem(ci, 10);
      ack_buf[8] = (crc >> 8) & 0xFF; ack_buf[9] = crc & 0xFF;
      write_array(ack_buf, 12);
      // Also advance handshake if we're waiting for this in ANNOUNCE_2 state
      if (hs_state_ != HS_COMPLETE) {
        hs_waiting_ack_ = false;
      }
      return;
    }

    if (p0 == 0x10 && p1 == 0x10) {
      // [10 10] — respond with [80 10 <timestamp 4 bytes> FC]
      // Mirror format: use 0x00000000 as timestamp (no RTC available)
      ESP_LOGD(TAG, "RX 10 10 — sending timestamp response");
      uint8_t resp_buf[17];
      resp_buf[0] = FRAME_HEADER;
      resp_buf[1] = BUS_ID;
      resp_buf[2] = DEV_HEARTBEAT;
      resp_buf[3] = TYPE_ACK;
      resp_buf[4] = 0x00;
      resp_buf[5] = seq;
      resp_buf[6] = 0x00;
      resp_buf[7] = 17;  // total length
      resp_buf[8] = 0x00; resp_buf[9] = 0x00;  // CRC placeholders
      resp_buf[10] = 0x80;
      resp_buf[11] = 0x10;
      resp_buf[12] = 0x00; resp_buf[13] = 0x00;
      resp_buf[14] = 0x00; resp_buf[15] = 0x00;  // 4-byte timestamp = 0
      resp_buf[16] = 0xFC;
      uint8_t ci[15] = {resp_buf[0],resp_buf[1],resp_buf[2],resp_buf[3],
                        resp_buf[4],resp_buf[5],resp_buf[6],resp_buf[7],
                        0x80,0x10,0x00,0x00,0x00,0x00,0xFC};
      uint16_t crc = crc16_xmodem(ci, 15);
      resp_buf[8] = (crc >> 8) & 0xFF;
      resp_buf[9] = crc & 0xFF;
      write_array(resp_buf, 17);
      return;
    }

    if (p0 == 0x0C && p1 == 0x0C) {
      // [0C 0C] heartbeat — respond with sensor poll request.
      // The real WiFi module sends [80 0C 02 64 FF FF FF C0] rather than just
      // [80 0C]. The extra bytes are a TLV read request (bank=0x02, reg=0x64)
      // which causes the indoor unit to reply with a full sensor data frame
      // including reg 0x60 (outdoor coil), reg 0x65 (indoor coil), reg 0x64
      // etc. Without these bytes the indoor sends no sensor data at all.
      ESP_LOGV(TAG, "RX 0C 0C heartbeat — sending sensor poll");

      static const uint8_t POLL_PAYLOAD[8] = {0x80, 0x0C, 0x02, 0x64, 0xFF, 0xFF, 0xFF, 0xC0};
      uint8_t resp_frame[18];
      resp_frame[0] = FRAME_HEADER; resp_frame[1] = BUS_ID;
      resp_frame[2] = DEV_HEARTBEAT; resp_frame[3] = TYPE_ACK;
      resp_frame[4] = 0x00; resp_frame[5] = seq;
      resp_frame[6] = 0x00; resp_frame[7] = 18;
      resp_frame[8] = 0x00; resp_frame[9] = 0x00;
      for (int i = 0; i < 8; i++) resp_frame[10+i] = POLL_PAYLOAD[i];
      uint8_t ci[16] = {resp_frame[0],resp_frame[1],resp_frame[2],resp_frame[3],
                        resp_frame[4],resp_frame[5],resp_frame[6],resp_frame[7],
                        0x80,0x0C,0x02,0x64,0xFF,0xFF,0xFF,0xC0};
      uint16_t crc = crc16_xmodem(ci, 16);
      resp_frame[8] = (crc >> 8) & 0xFF;
      resp_frame[9] = crc & 0xFF;
      write_array(resp_frame, 18);
      return;
    }

    // Unknown heartbeat frame — just log it
    ESP_LOGD(TAG, "RX unknown heartbeat frame %02X %02X", p0, p1);
    return;
  }

  // ── Normal addressed frames (dev=01) ─────────────────────────────────────
  // ACK immediately, mirroring the device ID
  send_frame(build_ack_frame(seq, dev_id));

  if (plen < 2) return;
  if (payload[0] != PREFIX_INDOOR_HI || payload[1] != PREFIX_INDOOR_LO) return;

  // Detect sensor scan: look for REG_SENSOR_MARKER in TLV data
  const uint8_t *tlv = payload + 2;
  size_t tlvlen = plen - 2;
  bool is_sensor_scan = false;
  for (size_t i = 0; i + 1 < tlvlen; i++) {
    if (tlv[i] == 0x00 && tlv[i + 1] == REG_SENSOR_MARKER) {
      is_sensor_scan = true; break;
    }
  }

  apply_tlv_entries(parse_tlv(tlv, tlvlen), is_sensor_scan);
}

// ─────────────────────────────────────────────────────────────────────────────
// State application
// ─────────────────────────────────────────────────────────────────────────────
void AcController::apply_tlv_entries(const std::vector<TlvEntry> &entries,
                                     bool is_sensor_scan) {
  bool changed = false;

  for (size_t i = 0; i < entries.size(); i++) {
    const TlvEntry &e = entries[i];

    if (is_sensor_scan) {
      if (e.bank != 0x00) { continue; }
      if (e.reg == SENSOR_ROOM_TEMP) {
        // reg 0x0D — room temperature in °F
        // Only reported during active operation; 0 = not available
        float f = (float) e.value;
        if (f > 32.0f && f < 120.0f) {
          room_temp_f_ = f;
          current_temperature = (f - 32.0f) / 1.8f;
          if (room_temp_sensor_) room_temp_sensor_->publish_state(f);
          changed = true;
        }
      } else if (e.reg == REG_COMP_FREQ) {
        // reg 0x09 — pipe/refrigerant temperature in °F (also appears in scan frames)
        comp_freq_ = (uint8_t) e.value;
        if (comp_freq_sensor_) comp_freq_sensor_->publish_state(comp_freq_);
      }
      // All other sensor scan registers are informational only (motor positions,
      // refrigerant pressures etc.) — no sensor exposed for them yet.
      continue;
    }

    switch (e.reg) {
      case REG_POWER:
        power_ = (e.value != 0); changed = true; break;
      case REG_MODE:
        mode_ = (uint8_t) e.value; changed = true; break;
      case REG_SET_TEMP:
        if (e.bank == 0x02 || e.size == 4) {
          set_temp_f_ = (float) e.value;
          target_temperature = (set_temp_f_ - 32.0f) / 1.8f;
          changed = true;
        }
        break;
      case REG_FAN_SPEED:
        fan_speed_ = (uint8_t) e.value; changed = true; break;
      case REG_FAN_AUTO_MODE:
        fan_auto_ = (e.value != 0); changed = true; break;
      case REG_V_SWING:
        v_swing_ = (uint8_t) e.value;
        if (v_swing_select_)
          v_swing_select_->publish_state(v_swing_val_to_str(v_swing_));
        break;
      case REG_H_SWING:
        h_swing_ = (uint8_t) e.value;
        if (h_swing_select_)
          h_swing_select_->publish_state(h_swing_val_to_str(h_swing_));
        break;
      case REG_ECO:
        eco_ = (e.value != 0);
        if (eco_switch_) eco_switch_->publish_state(eco_);
        break;
      case REG_DISPLAY:
        disp_ = (e.value != 0);
        if (display_switch_) display_switch_->publish_state(disp_);
        break;
      case REG_BEEP:
        beep_ = (e.value != 0);
        if (beep_switch_) beep_switch_->publish_state(beep_);
        break;
      case REG_SLEEP_MODE: {
        sleep_mode_ = (uint8_t) e.value;
        if (sleep_select_) {
          const char *s = "Off";
          if      (sleep_mode_ == 1) s = "Standard";
          else if (sleep_mode_ == 2) s = "Aged";
          else if (sleep_mode_ == 3) s = "Child";
          sleep_select_->publish_state(s);
        }
        break;
      }
      case REG_COMP_FREQ:
        // reg 0x09 — pipe/refrigerant temperature in °F, not compressor Hz
        // Values: ~96-106°F at rest, varies during operation
        comp_freq_ = (uint8_t) e.value;
        if (comp_freq_sensor_) comp_freq_sensor_->publish_state(comp_freq_);
        break;
      case REG_FAN_RPM:
        fan_rpm_pct_ = (uint8_t)(e.value & 0xFF);
        if (fan_rpm_sensor_) fan_rpm_sensor_->publish_state(fan_rpm_pct_);
        break;
      case REG_INDOOR_COIL:
        // reg 0x65 values are whole °C integers (not °F as previously assumed)
        // 21=21°C, 46=46°C etc — confirmed against heat mode coil temperatures
        if (e.value > 0) {
          indoor_coil_f_ = (float) e.value;  // field name legacy, value is now °C
          if (indoor_coil_sensor_) indoor_coil_sensor_->publish_state(indoor_coil_f_);
        }
        break;
      case REG_OUTDOOR_COIL: {
        // reg 0x60 values are tenths of °F (900=90°F, 1000=100°F, 1100=110°F)
        // Published directly in °F to match IndoorCoil sensor units
        int32_t sv = (int32_t) e.value;
        outdoor_coil_c_ = sv / 10.0f;  // field name legacy; value is °F
        if (outdoor_coil_sensor_) outdoor_coil_sensor_->publish_state(outdoor_coil_c_);
        break;
      }
      default:
        ESP_LOGV(TAG, "Unhandled reg=0x%02X bank=0x%02X val=0x%08X",
                 e.reg, e.bank, e.value);
        break;
    }
  }
  if (changed) {
    state_received_ = true;
    publish_climate_state();
  }
}

void AcController::publish_climate_state() {
  if (!power_) {
    mode = climate::CLIMATE_MODE_OFF;
  } else {
    switch (mode_) {
      case MODE_COOL:    mode = climate::CLIMATE_MODE_COOL;     break;
      case MODE_HEAT:    mode = climate::CLIMATE_MODE_HEAT;     break;
      case MODE_DRY:     mode = climate::CLIMATE_MODE_DRY;      break;
      case MODE_FAN_ONLY:mode = climate::CLIMATE_MODE_FAN_ONLY; break;
      case MODE_AUTO:    mode = climate::CLIMATE_MODE_AUTO;     break;
      default:           mode = climate::CLIMATE_MODE_COOL;     break;
    }
  }

  if (fan_auto_) {
    fan_mode = climate::CLIMATE_FAN_AUTO;
  } else {
    switch (fan_speed_) {
      case FAN_MUTE:
      case FAN_LOW:     fan_mode = climate::CLIMATE_FAN_LOW;    break;
      case FAN_MID_LOW:
      case FAN_MID:     fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
      case FAN_MID_HIGH:
      case FAN_HIGH:
      case FAN_STRONG:  fan_mode = climate::CLIMATE_FAN_HIGH;   break;
      default:          fan_mode = climate::CLIMATE_FAN_AUTO;   break;
    }
  }

  bool v_swinging = (v_swing_ >= VSWING_UPDOWN && v_swing_ <= VSWING_DOWN);
  bool h_swinging = (h_swing_ >= HSWING_LR     && h_swing_ <= HSWING_RIGHT);

  if      (v_swinging && h_swinging) swing_mode = climate::CLIMATE_SWING_BOTH;
  else if (v_swinging)               swing_mode = climate::CLIMATE_SWING_VERTICAL;
  else if (h_swinging)               swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
  else                               swing_mode = climate::CLIMATE_SWING_OFF;

  publish_state();
}

// ─────────────────────────────────────────────────────────────────────────────
// Swing string helpers
// ─────────────────────────────────────────────────────────────────────────────
std::string v_swing_val_to_str(uint8_t val) {
  switch (val) {
    case VSWING_UPDOWN:      return "Swing Up-Down";
    case VSWING_UP:          return "Swing Up";
    case VSWING_DOWN:        return "Swing Down";
    case VSWING_FIX_UP:      return "Fixed Up";
    case VSWING_FIX_ABOVEUP: return "Fixed Above-Up";
    case VSWING_FIX_MID:     return "Fixed Middle";
    case VSWING_FIX_ABOVEDN: return "Fixed Above-Down";
    case VSWING_FIX_DOWN:    return "Fixed Down";
    default:                 return "Fixed Middle";
  }
}

std::string h_swing_val_to_str(uint8_t val) {
  switch (val) {
    case HSWING_LR:            return "Swing Left-Right";
    case HSWING_LEFT:          return "Swing Left";
    case HSWING_CENTER:        return "Swing Center";
    case HSWING_RIGHT:         return "Swing Right";
    case HSWING_FIX_LEFT:      return "Fixed Left";
    case HSWING_FIX_ABITLEFT:  return "Fixed A-bit-Left";
    case HSWING_FIX_CENTER:    return "Fixed Center";
    case HSWING_FIX_ABITRIGHT: return "Fixed A-bit-Right";
    case HSWING_FIX_RIGHT:     return "Fixed Right";
    default:                   return "Fixed Center";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Climate::control
// ─────────────────────────────────────────────────────────────────────────────
void AcController::control(const climate::ClimateCall &call) {
  std::vector<TlvEntry> entries;

  if (call.get_mode().has_value()) {
    auto nm = *call.get_mode();
    bool np = (nm != climate::CLIMATE_MODE_OFF);
    if (np != power_) {
      TlvEntry e; e.bank=0x00; e.reg=REG_POWER; e.value=(np?1:0); e.size=1;
      entries.push_back(e);
      power_ = np;
      if (np) {
        // Send Eco and VSwing with power-on frame
        TlvEntry e2; e2.bank=0x00; e2.reg=REG_ECO; e2.value=(eco_?1:0); e2.size=1;
        entries.push_back(e2);
        uint8_t target_vswing = (v_swing_ == 0x08) ? VSWING_FIX_MID : v_swing_;
        TlvEntry e3; e3.bank=0x00; e3.reg=REG_V_SWING; e3.value=target_vswing; e3.size=1;
        entries.push_back(e3);
        v_swing_ = target_vswing;
        // The indoor unit ignores Mode when bundled in the power-on frame —
        // it reverts to its stored mode. Queue a separate mode frame to follow.
        pending_mode_followup_ = true;
      } else {
        v_swing_ = 0x08;
        h_swing_ = 0x08;
      }
    }
    if (np) {
      uint8_t am = mode_;
      switch (nm) {
        case climate::CLIMATE_MODE_COOL:     am = MODE_COOL;     break;
        case climate::CLIMATE_MODE_HEAT:     am = MODE_HEAT;     break;
        case climate::CLIMATE_MODE_DRY:      am = MODE_DRY;      break;
        case climate::CLIMATE_MODE_FAN_ONLY: am = MODE_FAN_ONLY; break;
        case climate::CLIMATE_MODE_AUTO:     am = MODE_AUTO;     break;
        default: break;
      }
      if (am != mode_) {
        mode_ = am;
        if (!pending_mode_followup_) {
          // Not a fresh power-on — send mode change immediately
          TlvEntry e; e.bank=0x00; e.reg=REG_MODE; e.value=am; e.size=1;
          entries.push_back(e);
        }
        // If pending_mode_followup_ is set, mode will be sent as a follow-up frame
      }
    }
  }

  if (call.get_target_temperature().has_value()) {
    float tc = *call.get_target_temperature();
    uint32_t tf = (uint32_t)(tc * 1.8f + 32.0f + 0.5f);
    if (tf != (uint32_t) set_temp_f_) {
      TlvEntry e; e.bank=0x02; e.reg=REG_SET_TEMP; e.value=tf; e.size=4;
      entries.push_back(e);
      set_temp_f_ = (float) tf;
      target_temperature = tc;
    }
  }

  if (call.get_fan_mode().has_value()) {
    uint8_t nf = fan_speed_; bool na = false;
    switch (*call.get_fan_mode()) {
      case climate::CLIMATE_FAN_AUTO:   nf = FAN_AUTO; na = true;  break;
      case climate::CLIMATE_FAN_LOW:    nf = FAN_LOW;  na = false; break;
      case climate::CLIMATE_FAN_MEDIUM: nf = FAN_MID;  na = false; break;
      case climate::CLIMATE_FAN_HIGH:   nf = FAN_HIGH; na = false; break;
      default: break;
    }
    if (nf != fan_speed_ || na != fan_auto_) {
      TlvEntry e1; e1.bank=0x00; e1.reg=REG_FAN_SPEED;    e1.value=nf;    e1.size=1;
      TlvEntry e2; e2.bank=0x00; e2.reg=REG_FAN_AUTO_MODE; e2.value=(na?1:0); e2.size=1;
      entries.push_back(e1); entries.push_back(e2);
      fan_speed_ = nf; fan_auto_ = na;
    }
  }

  if (call.get_swing_mode().has_value()) {
    uint8_t nv = v_swing_, nh = h_swing_;
    switch (*call.get_swing_mode()) {
      case climate::CLIMATE_SWING_OFF:
        nv = VSWING_FIX_MID;  nh = HSWING_FIX_CENTER; break;
      case climate::CLIMATE_SWING_VERTICAL:
        nv = VSWING_UPDOWN;   nh = HSWING_FIX_CENTER; break;
      case climate::CLIMATE_SWING_HORIZONTAL:
        nv = VSWING_FIX_MID;  nh = HSWING_LR;         break;
      case climate::CLIMATE_SWING_BOTH:
        nv = VSWING_UPDOWN;   nh = HSWING_LR;         break;
      default: break;
    }
    if (nv != v_swing_) {
      TlvEntry e; e.bank=0x00; e.reg=REG_V_SWING; e.value=nv; e.size=1;
      entries.push_back(e); v_swing_ = nv;
    }
    if (nh != h_swing_) {
      TlvEntry e; e.bank=0x00; e.reg=REG_H_SWING; e.value=nh; e.size=1;
      entries.push_back(e); h_swing_ = nh;
    }
  }

  if (!entries.empty())
    send_registers(entries);
}

// ─────────────────────────────────────────────────────────────────────────────
// Climate traits
// ─────────────────────────────────────────────────────────────────────────────
climate::ClimateTraits AcController::traits() {
  auto t = climate::ClimateTraits();
  t.set_supports_current_temperature(true);
  t.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_FAN_ONLY,
    climate::CLIMATE_MODE_AUTO,
  });
  t.set_supported_fan_modes({
    climate::CLIMATE_FAN_AUTO,
    climate::CLIMATE_FAN_LOW,
    climate::CLIMATE_FAN_MEDIUM,
    climate::CLIMATE_FAN_HIGH,
  });
  t.set_supported_swing_modes({
    climate::CLIMATE_SWING_OFF,
    climate::CLIMATE_SWING_VERTICAL,
    climate::CLIMATE_SWING_HORIZONTAL,
    climate::CLIMATE_SWING_BOTH,
  });
  t.set_visual_min_temperature(15.5f);
  t.set_visual_max_temperature(32.0f);
  t.set_visual_temperature_step(0.5f);
  return t;
}

// ─────────────────────────────────────────────────────────────────────────────
// Periodic poll (after handshake)
// ─────────────────────────────────────────────────────────────────────────────
void AcController::send_poll_frame() {
  // Heartbeat poll: A5 01 00 21 00 00 00 0C CRC1 CRC2 0C 0C
  uint8_t frame[12];
  frame[0] = FRAME_HEADER; frame[1] = BUS_ID; frame[2] = DEV_HEARTBEAT;
  frame[3] = TYPE_CMD; frame[4] = 0x00; frame[5] = 0x00;
  frame[6] = 0x00; frame[7] = 12;
  frame[8] = 0x00; frame[9] = 0x00;
  frame[10] = PREFIX_INDOOR_HI; frame[11] = PREFIX_INDOOR_LO;
  uint8_t ci[10] = {frame[0],frame[1],frame[2],frame[3],
                    frame[4],frame[5],frame[6],frame[7],
                    PREFIX_INDOOR_HI, PREFIX_INDOOR_LO};
  uint16_t crc = crc16_xmodem(ci, 10);
  frame[8] = (crc >> 8) & 0xFF;
  frame[9] = crc & 0xFF;
  ESP_LOGD(TAG, "Sending poll frame");
  write_array(frame, 12);
  last_poll_ms_ = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// ESPHome lifecycle
// ─────────────────────────────────────────────────────────────────────────────
void AcController::setup() {
  ESP_LOGI(TAG, "AC Controller setup — will start handshake");
  target_temperature  = 22.0f;
  current_temperature = NAN;
  mode       = climate::CLIMATE_MODE_OFF;
  fan_mode   = climate::CLIMATE_FAN_AUTO;
  swing_mode = climate::CLIMATE_SWING_OFF;
  hs_state_  = HS_IDLE;
  hs_seq_    = 0x01;
}

void AcController::loop() {
  // Feed the watchdog — on ESP8266 the single-core architecture means
  // the API task can miss keepalive pings if loop() is too busy.
  App.feed_wdt();

  // Drain UART RX
  while (available()) {
    uint8_t b; read_byte(&b);
    process_rx_byte(b);
  }

  // Run handshake state machine until complete
  // NOTE: Do NOT return early here — the ESPHome API task must continue
  // running during handshake or HA will see no devices/entities.
  if (hs_state_ != HS_COMPLETE) {
    run_handshake();
    return;  // still return to avoid sending normal commands mid-handshake
  }

  // Normal operation
  maybe_send_next_command();

  // Send mode follow-up after power-on — indoor ignores mode bundled in power frame
  if (pending_mode_followup_ && tx_queue_.empty() && !waiting_for_ack_) {
    if (millis() - last_tx_ms_ > 300) {
      ESP_LOGD(TAG, "Sending mode follow-up: 0x%02X", mode_);
      TlvEntry e; e.bank=0x00; e.reg=REG_MODE; e.value=mode_; e.size=1;
      std::vector<TlvEntry> v; v.push_back(e);
      send_registers(v);
      pending_mode_followup_ = false;
    }
  }
  // NOTE: No self-initiated [0C 0C] poll here. The indoor unit sends [0C 0C]
  // heartbeats on its own schedule and we respond to those. Sending our own
  // duplicate polls was causing the indoor to enter degraded state.
}

void AcController::dump_config() {
  ESP_LOGCONFIG(TAG, "AC Controller:");
  ESP_LOGCONFIG(TAG, "  Handshake: %s", hs_state_ == HS_COMPLETE ? "complete" : "pending");
  if (room_temp_sensor_)    ESP_LOGCONFIG(TAG, "  Room temp sensor: YES");
  if (indoor_coil_sensor_)  ESP_LOGCONFIG(TAG, "  Indoor coil sensor: YES");
  if (outdoor_coil_sensor_) ESP_LOGCONFIG(TAG, "  Outdoor coil sensor: YES");
  if (comp_freq_sensor_)    ESP_LOGCONFIG(TAG, "  Compressor freq: YES");
  if (fan_rpm_sensor_)      ESP_LOGCONFIG(TAG, "  Fan RPM: YES");
  if (v_swing_select_)      ESP_LOGCONFIG(TAG, "  V-swing select: YES");
  if (h_swing_select_)      ESP_LOGCONFIG(TAG, "  H-swing select: YES");
  if (eco_switch_)          ESP_LOGCONFIG(TAG, "  Eco switch: YES");
  if (display_switch_)      ESP_LOGCONFIG(TAG, "  Display switch: YES");
  if (beep_switch_)         ESP_LOGCONFIG(TAG, "  Beep switch: YES");
  if (sleep_select_)        ESP_LOGCONFIG(TAG, "  Sleep mode: YES");
}

// ─────────────────────────────────────────────────────────────────────────────
// Sub-component implementations
// ─────────────────────────────────────────────────────────────────────────────
void AcSwingSelect::control(const std::string &value) {
  AcController *parent = (AcController *) parent_;
  uint8_t reg = is_vertical_ ? REG_V_SWING : REG_H_SWING;
  uint8_t val = is_vertical_ ? VSWING_FIX_MID : HSWING_FIX_CENTER;

  if (is_vertical_) {
    if      (value == "Swing Up-Down")    val = VSWING_UPDOWN;
    else if (value == "Swing Up")         val = VSWING_UP;
    else if (value == "Swing Down")       val = VSWING_DOWN;
    else if (value == "Fixed Up")         val = VSWING_FIX_UP;
    else if (value == "Fixed Above-Up")   val = VSWING_FIX_ABOVEUP;
    else if (value == "Fixed Middle")     val = VSWING_FIX_MID;
    else if (value == "Fixed Above-Down") val = VSWING_FIX_ABOVEDN;
    else if (value == "Fixed Down")       val = VSWING_FIX_DOWN;
  } else {
    if      (value == "Swing Left-Right")  val = HSWING_LR;
    else if (value == "Swing Left")        val = HSWING_LEFT;
    else if (value == "Swing Center")      val = HSWING_CENTER;
    else if (value == "Swing Right")       val = HSWING_RIGHT;
    else if (value == "Fixed Left")        val = HSWING_FIX_LEFT;
    else if (value == "Fixed A-bit-Left")  val = HSWING_FIX_ABITLEFT;
    else if (value == "Fixed Center")      val = HSWING_FIX_CENTER;
    else if (value == "Fixed A-bit-Right") val = HSWING_FIX_ABITRIGHT;
    else if (value == "Fixed Right")       val = HSWING_FIX_RIGHT;
  }
  parent->send_register(0x00, reg, val);
  publish_state(value);
}

void AcSwitch::write_state(bool state) {
  AcController *parent = (AcController *) parent_;
  uint8_t reg;
  switch (kind_) {
    case KIND_ECO:     reg = REG_ECO;     break;
    case KIND_DISPLAY: reg = REG_DISPLAY; break;
    case KIND_BEEP:    reg = REG_BEEP;    break;
    default: return;
  }
  parent->send_register(0x00, reg, state ? 1 : 0);
  publish_state(state);
}

void AcSleepSelect::control(const std::string &value) {
  AcController *parent = (AcController *) parent_;
  uint8_t val = 0;
  if      (value == "Standard") val = 1;
  else if (value == "Aged")     val = 2;
  else if (value == "Child")    val = 3;
  parent->send_register(0x00, REG_SLEEP_MODE, val);
  publish_state(value);
}

}  // namespace ac_controller
}  // namespace esphome
