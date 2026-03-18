#include "ac_controller.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ac_controller {

static const char *const TAG = "ac_controller";

// ─────────────────────────────────────────────────────────────────────────────
// CRC-16/XMODEM  poly=0x1021  init=0x0000  no reflection
// ─────────────────────────────────────────────────────────────────────────────
uint16_t AcController::crc16_xmodem(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame builder
// Header: A5 01 01 TYPE SEQ 00 00 LEN CRC1 CRC2 PAYLOAD
// ─────────────────────────────────────────────────────────────────────────────
std::vector<uint8_t> AcController::build_frame(const std::vector<uint8_t> &payload) {
  uint8_t total_len = 10 + payload.size();
  std::vector<uint8_t> frame;
  frame.reserve(total_len);

  frame.push_back(FRAME_HEADER);   // 0
  frame.push_back(BUS_ID);         // 1
  frame.push_back(DEV_ID);         // 2
  frame.push_back(TYPE_CMD);       // 3
  frame.push_back(tx_seq_);        // 4 - SEQ
  frame.push_back(0x00);           // 5
  frame.push_back(0x00);           // 6 - reserved
  frame.push_back(total_len);      // 7 - length
  frame.push_back(0x00);           // 8 - CRC1 placeholder
  frame.push_back(0x00);           // 9 - CRC2 placeholder
  for (auto b : payload) frame.push_back(b);

  // CRC over bytes 0-7 + payload (10+), skipping CRC bytes 8-9
  std::vector<uint8_t> crc_data(frame.begin(), frame.begin() + 8);
  crc_data.insert(crc_data.end(), payload.begin(), payload.end());
  uint16_t crc = crc16_xmodem(crc_data.data(), crc_data.size());
  frame[8] = (crc >> 8) & 0xFF;
  frame[9] = crc & 0xFF;

  return frame;
}

std::vector<uint8_t> AcController::build_ack_frame(uint8_t seq) {
  // ACK frame: type=0x23, seq2=echoed seq, payload=80 0A
  uint8_t total_len = 12;
  std::vector<uint8_t> frame;
  frame.push_back(FRAME_HEADER);
  frame.push_back(BUS_ID);
  frame.push_back(DEV_ID);
  frame.push_back(TYPE_ACK);
  frame.push_back(0x00);     // SEQ1 = 0 for ACK
  frame.push_back(seq);      // SEQ2 = echoed seq
  frame.push_back(0x00);     // reserved
  frame.push_back(total_len);
  frame.push_back(0x00);     // CRC1 placeholder
  frame.push_back(0x00);     // CRC2 placeholder
  frame.push_back(0x80);     // payload: 80 0A (acking indoor frame)
  frame.push_back(ACK_PAYLOAD_IN);

  std::vector<uint8_t> crc_data = {frame[0],frame[1],frame[2],frame[3],
                                    frame[4],frame[5],frame[6],frame[7],
                                    0x80, ACK_PAYLOAD_IN};
  uint16_t crc = crc16_xmodem(crc_data.data(), crc_data.size());
  frame[8] = (crc >> 8) & 0xFF;
  frame[9] = crc & 0xFF;

  return frame;
}

// ─────────────────────────────────────────────────────────────────────────────
// TLV helpers
// ─────────────────────────────────────────────────────────────────────────────
bool AcController::is_4byte_special(uint8_t reg) {
  return reg == REG_OUTDOOR_COIL || reg == 0x64 || reg == REG_INDOOR_COIL || reg == REG_FAN_RPM;
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
      e.value = ((uint32_t)data[i+2] << 24) | ((uint32_t)data[i+3] << 16) |
                ((uint32_t)data[i+4] << 8)  |  (uint32_t)data[i+5];
      e.size = 4;
      i += 6;
    } else if (bank == 0x00) {
      if (i + 3 > len) break;
      e.value = data[i + 2];
      e.size = 1;
      i += 3;
    } else if (bank == 0x01) {
      if (i + 4 > len) break;
      e.value = ((uint32_t)data[i+2] << 8) | data[i+3];
      e.size = 2;
      i += 4;
    } else if (bank == 0x02) {
      if (i + 6 > len) break;
      e.value = ((uint32_t)data[i+2] << 24) | ((uint32_t)data[i+3] << 16) |
                ((uint32_t)data[i+4] << 8)  |  (uint32_t)data[i+5];
      e.size = 4;
      i += 6;
    } else {
      // Unknown bank — skip 2 bytes to avoid infinite loop
      i += 2;
      continue;
    }
    entries.push_back(e);
  }
  return entries;
}

// Build a payload from a list of TLV entries (with 0A 0A prefix)
static std::vector<uint8_t> build_tlv_payload(const std::vector<TlvEntry> &entries) {
  std::vector<uint8_t> payload = {PREFIX_CTRL_HI, PREFIX_CTRL_LO};
  for (const auto &e : entries) {
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
void AcController::enqueue_command(const std::vector<uint8_t> &payload) {
  PendingCommand cmd;
  cmd.payload = payload;
  tx_queue_.push(cmd);
}

void AcController::send_register(uint8_t bank, uint8_t reg, uint32_t value) {
  uint8_t size = (bank == 0x02 || is_4byte_special(reg)) ? 4 : (bank == 0x01 ? 2 : 1);
  TlvEntry e{bank, reg, value, size};
  send_registers({e});
}

void AcController::send_registers(const std::vector<TlvEntry> &entries) {
  auto payload = build_tlv_payload(entries);
  enqueue_command(payload);
}

void AcController::send_frame(const std::vector<uint8_t> &frame) {
  write_array(frame.data(), frame.size());
  ESP_LOGV(TAG, "TX [%d bytes]: %s", (int)frame.size(),
           format_hex_pretty(frame.data(), frame.size()).c_str());
}

void AcController::maybe_send_next_command() {
  if (waiting_for_ack_) {
    // Check for timeout
    if (millis() - last_tx_ms_ > ACK_TIMEOUT_MS) {
      ESP_LOGW(TAG, "ACK timeout — retrying");
      waiting_for_ack_ = false;
    } else {
      return;
    }
  }
  if (tx_queue_.empty()) return;

  auto &cmd = tx_queue_.front();
  auto frame = build_frame(cmd.payload);
  send_frame(frame);
  last_tx_ms_ = millis();
  waiting_for_ack_ = true;
  tx_seq_ = (tx_seq_ == 0xFF) ? 0x01 : tx_seq_ + 1;
  tx_queue_.pop();
}

// ─────────────────────────────────────────────────────────────────────────────
// RX parsing
// ─────────────────────────────────────────────────────────────────────────────
void AcController::process_rx_byte(uint8_t byte) {
  // If buffer empty, wait for start of frame
  if (rx_buf_.empty() && byte != FRAME_HEADER) return;
  rx_buf_.push_back(byte);

  // Need at least 8 bytes to read length
  if (rx_buf_.size() < 8) return;

  uint8_t expected_len = rx_buf_[7];
  if (expected_len < 10 || expected_len > 128) {
    // Invalid length — reset
    rx_buf_.clear();
    return;
  }

  if (rx_buf_.size() < expected_len) return;

  // We have a complete frame
  if (try_parse_frame()) {
    rx_buf_.clear();
  } else {
    // Bad CRC or malformed — drop first byte and retry
    rx_buf_.erase(rx_buf_.begin());
  }
}

bool AcController::try_parse_frame() {
  if (rx_buf_.size() < 10) return false;
  if (rx_buf_[0] != FRAME_HEADER) return false;

  uint8_t len = rx_buf_[7];
  if (rx_buf_.size() < len) return false;

  // Verify CRC
  std::vector<uint8_t> crc_data(rx_buf_.begin(), rx_buf_.begin() + 8);
  crc_data.insert(crc_data.end(), rx_buf_.begin() + 10, rx_buf_.begin() + len);
  uint16_t calc_crc = crc16_xmodem(crc_data.data(), crc_data.size());
  uint16_t frame_crc = ((uint16_t)rx_buf_[8] << 8) | rx_buf_[9];

  if (calc_crc != frame_crc) {
    ESP_LOGW(TAG, "CRC mismatch: calc=0x%04X frame=0x%04X", calc_crc, frame_crc);
    return false;
  }

  std::vector<uint8_t> frame(rx_buf_.begin(), rx_buf_.begin() + len);
  uint8_t type = frame[3];

  if (type == TYPE_CMD) {
    handle_indoor_frame(frame);
  } else if (type == TYPE_ACK) {
    handle_ack_frame(frame);
  }

  return true;
}

void AcController::handle_ack_frame(const std::vector<uint8_t> &frame) {
  ESP_LOGV(TAG, "RX ACK seq=%02X", frame[5]);
  waiting_for_ack_ = false;
}

void AcController::handle_indoor_frame(const std::vector<uint8_t> &frame) {
  if (frame.size() < 12) return;

  uint8_t seq = frame[4];
  ESP_LOGV(TAG, "RX indoor CMD seq=%02X [%d bytes]", seq, (int)frame.size());

  // Send ACK immediately
  auto ack = build_ack_frame(seq);
  send_frame(ack);

  // Parse payload (skip 2-byte prefix)
  const uint8_t *payload = frame.data() + 10;
  size_t payload_len = frame.size() - 10;
  if (payload_len < 2) return;

  // Check prefix
  if (payload[0] != PREFIX_INDOOR_HI || payload[1] != PREFIX_INDOOR_LO) return;

  // Detect sensor scan frame: short frame with REG_SENSOR_MARKER present
  bool is_sensor_scan = false;
  const uint8_t *tlv_data = payload + 2;
  size_t tlv_len = payload_len - 2;
  for (size_t i = 0; i + 2 < tlv_len; i++) {
    if (tlv_data[i] == 0x00 && tlv_data[i+1] == REG_SENSOR_MARKER) {
      is_sensor_scan = true;
      break;
    }
  }

  auto entries = parse_tlv(tlv_data, tlv_len);
  apply_tlv_entries(entries, is_sensor_scan);
}

// ─────────────────────────────────────────────────────────────────────────────
// State application
// ─────────────────────────────────────────────────────────────────────────────
void AcController::apply_tlv_entries(const std::vector<TlvEntry> &entries, bool is_sensor_scan) {
  bool climate_changed = false;

  for (const auto &e : entries) {
    if (is_sensor_scan) {
      // In sensor scan frames, reg numbers are sensor IDs, not control regs
      if (e.reg == SENSOR_ROOM_TEMP && e.bank == 0x00) {
        float temp_f = (float)e.value;
        if (temp_f > 32.0f && temp_f < 120.0f) {
          room_temp_f_ = temp_f;
          float temp_c = (temp_f - 32.0f) / 1.8f;
          if (room_temp_sensor_ != nullptr)
            room_temp_sensor_->publish_state(temp_f);
          // Update climate current temperature
          current_temperature = temp_c;
          climate_changed = true;
        }
      }
      continue;  // Don't interpret other sensor scan regs as control regs
    }

    // Normal control/status registers
    switch (e.reg) {
      case REG_POWER:
        power_ = (e.value != 0);
        climate_changed = true;
        break;

      case REG_MODE:
        mode_ = e.value & 0xFF;
        climate_changed = true;
        break;

      case REG_SET_TEMP:
        // bank=0x02, 4-byte, °F integer
        if (e.bank == 0x02 || e.size == 4) {
          set_temp_f_ = (float)e.value;
          target_temperature = (set_temp_f_ - 32.0f) / 1.8f;
          climate_changed = true;
        }
        break;

      case REG_FAN_SPEED:
        fan_speed_ = e.value & 0xFF;
        climate_changed = true;
        break;

      case REG_FAN_AUTO_MODE:
        fan_auto_ = (e.value != 0);
        climate_changed = true;
        break;

      case REG_V_SWING:
        v_swing_ = e.value & 0xFF;
        if (v_swing_select_ != nullptr) {
          v_swing_select_->publish_state(v_swing_val_to_str(v_swing_));
        }
        break;

      case REG_H_SWING:
        h_swing_ = e.value & 0xFF;
        if (h_swing_select_ != nullptr) {
          h_swing_select_->publish_state(h_swing_val_to_str(h_swing_));
        }
        break;

      case REG_ECO:
        eco_ = (e.value != 0);
        if (eco_switch_ != nullptr)
          eco_switch_->publish_state(eco_);
        break;

      case REG_DISPLAY:
        display_ = (e.value != 0);
        if (display_switch_ != nullptr)
          display_switch_->publish_state(display_);
        break;

      case REG_BEEP:
        beep_ = (e.value != 0);
        if (beep_switch_ != nullptr)
          beep_switch_->publish_state(beep_);
        break;

      case REG_SLEEP_MODE:
        sleep_mode_ = e.value & 0xFF;
        if (sleep_select_ != nullptr) {
          std::string s;
          switch (sleep_mode_) {
            case 0: s = "Off"; break;
            case 1: s = "Standard"; break;
            case 2: s = "Aged"; break;
            case 3: s = "Child"; break;
            default: s = "Off";
          }
          sleep_select_->publish_state(s);
        }
        break;

      case REG_COMP_FREQ:
        comp_freq_ = e.value & 0xFF;
        if (comp_freq_sensor_ != nullptr)
          comp_freq_sensor_->publish_state(comp_freq_);
        break;

      case REG_FAN_RPM:
        // Special 4-byte
        fan_rpm_pct_ = e.value & 0xFF;
        if (fan_rpm_sensor_ != nullptr)
          fan_rpm_sensor_->publish_state(fan_rpm_pct_);
        break;

      case REG_INDOOR_COIL:
        // Special 4-byte, °F
        if (e.value > 0) {
          indoor_coil_f_ = (float)e.value;
          if (indoor_coil_sensor_ != nullptr)
            indoor_coil_sensor_->publish_state(indoor_coil_f_);
        }
        break;

      case REG_OUTDOOR_COIL: {
        // Signed 32-bit, tenths of °C
        int32_t signed_val = (int32_t)e.value;
        outdoor_coil_c_ = signed_val / 10.0f;
        if (outdoor_coil_sensor_ != nullptr)
          outdoor_coil_sensor_->publish_state(outdoor_coil_c_);
        break;
      }

      default:
        ESP_LOGV(TAG, "Unhandled reg=0x%02X bank=0x%02X val=0x%08X",
                 e.reg, e.bank, e.value);
        break;
    }
  }

  if (climate_changed) {
    state_received_ = true;
    publish_climate_state();
  }
}

void AcController::publish_climate_state() {
  // Power / mode mapping
  if (!power_) {
    mode = climate::CLIMATE_MODE_OFF;
  } else {
    switch (mode_) {
      case MODE_COOL:    mode = climate::CLIMATE_MODE_COOL; break;
      case MODE_HEAT:    mode = climate::CLIMATE_MODE_HEAT; break;
      case MODE_DRY:     mode = climate::CLIMATE_MODE_DRY;  break;
      case MODE_FAN_ONLY:mode = climate::CLIMATE_MODE_FAN_ONLY; break;
      case MODE_AUTO:    mode = climate::CLIMATE_MODE_AUTO; break;
      default:           mode = climate::CLIMATE_MODE_COOL; break;
    }
  }

  // Fan mode mapping
  if (fan_auto_) {
    fan_mode = climate::CLIMATE_FAN_AUTO;
  } else {
    switch (fan_speed_) {
      case FAN_MUTE:
      case FAN_LOW:      fan_mode = climate::CLIMATE_FAN_LOW;    break;
      case FAN_MID_LOW:
      case FAN_MID:      fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
      case FAN_MID_HIGH:
      case FAN_HIGH:
      case FAN_STRONG:   fan_mode = climate::CLIMATE_FAN_HIGH;   break;
      default:           fan_mode = climate::CLIMATE_FAN_AUTO;   break;
    }
  }

  // Swing mapping — use V-swing for the standard swing field
  bool v_is_swing = (v_swing_ >= VSWING_UPDOWN && v_swing_ <= VSWING_DOWN);
  bool h_is_swing = (h_swing_ >= HSWING_LR     && h_swing_ <= HSWING_RIGHT);

  if (v_is_swing && h_is_swing) {
    swing_mode = climate::CLIMATE_SWING_BOTH;
  } else if (v_is_swing) {
    swing_mode = climate::CLIMATE_SWING_VERTICAL;
  } else if (h_is_swing) {
    swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
  } else {
    swing_mode = climate::CLIMATE_SWING_OFF;
  }

  publish_state();
}

// ─────────────────────────────────────────────────────────────────────────────
// Swing string helpers
// ─────────────────────────────────────────────────────────────────────────────
std::string v_swing_val_to_str(uint8_t val) {
  switch (val) {
    case VSWING_UPDOWN:       return "Swing Up-Down";
    case VSWING_UP:           return "Swing Up";
    case VSWING_DOWN:         return "Swing Down";
    case VSWING_FIX_UP:       return "Fixed Up";
    case VSWING_FIX_ABOVEUP:  return "Fixed Above-Up";
    case VSWING_FIX_MID:      return "Fixed Middle";
    case VSWING_FIX_ABOVEDN:  return "Fixed Above-Down";
    case VSWING_FIX_DOWN:     return "Fixed Down";
    default: return "Fixed Middle";
  }
}

std::string h_swing_val_to_str(uint8_t val) {
  switch (val) {
    case HSWING_LR:              return "Swing Left-Right";
    case HSWING_LEFT:            return "Swing Left";
    case HSWING_CENTER:          return "Swing Center";
    case HSWING_RIGHT:           return "Swing Right";
    case HSWING_FIX_LEFT:        return "Fixed Left";
    case HSWING_FIX_ABITLEFT:    return "Fixed A-bit-Left";
    case HSWING_FIX_CENTER:      return "Fixed Center";
    case HSWING_FIX_ABITRIGHT:   return "Fixed A-bit-Right";
    case HSWING_FIX_RIGHT:       return "Fixed Right";
    default: return "Fixed Center";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Climate::control — called when HA sends a command
// ─────────────────────────────────────────────────────────────────────────────
void AcController::control(const climate::ClimateCall &call) {
  std::vector<TlvEntry> entries;

  // ── Power / Mode ──────────────────────────────────────────────────────────
  if (call.get_mode().has_value()) {
    auto new_mode = *call.get_mode();
    bool new_power = (new_mode != climate::CLIMATE_MODE_OFF);

    if (new_power != power_) {
      entries.push_back({0x00, REG_POWER, (uint32_t)(new_power ? 1 : 0), 1});
      power_ = new_power;
    }

    if (new_power) {
      uint8_t ac_mode = mode_;
      switch (new_mode) {
        case climate::CLIMATE_MODE_COOL:     ac_mode = MODE_COOL;     break;
        case climate::CLIMATE_MODE_HEAT:     ac_mode = MODE_HEAT;     break;
        case climate::CLIMATE_MODE_DRY:      ac_mode = MODE_DRY;      break;
        case climate::CLIMATE_MODE_FAN_ONLY: ac_mode = MODE_FAN_ONLY; break;
        case climate::CLIMATE_MODE_AUTO:     ac_mode = MODE_AUTO;     break;
        default: break;
      }
      if (ac_mode != mode_) {
        entries.push_back({0x00, REG_MODE, (uint32_t)ac_mode, 1});
        mode_ = ac_mode;
      }
    }
  }

  // ── Temperature setpoint ─────────────────────────────────────────────────
  if (call.get_target_temperature().has_value()) {
    float temp_c = *call.get_target_temperature();
    float temp_f = temp_c * 1.8f + 32.0f;
    uint32_t temp_f_int = (uint32_t)roundf(temp_f);
    if (temp_f_int != (uint32_t)set_temp_f_) {
      entries.push_back({0x02, REG_SET_TEMP, temp_f_int, 4});
      set_temp_f_ = (float)temp_f_int;
      target_temperature = temp_c;
    }
  }

  // ── Fan mode ──────────────────────────────────────────────────────────────
  if (call.get_fan_mode().has_value()) {
    auto fm = *call.get_fan_mode();
    uint8_t new_fan = fan_speed_;
    bool new_auto = false;

    switch (fm) {
      case climate::CLIMATE_FAN_AUTO:   new_fan = FAN_AUTO;   new_auto = true;  break;
      case climate::CLIMATE_FAN_LOW:    new_fan = FAN_LOW;    new_auto = false; break;
      case climate::CLIMATE_FAN_MEDIUM: new_fan = FAN_MID;    new_auto = false; break;
      case climate::CLIMATE_FAN_HIGH:   new_fan = FAN_HIGH;   new_auto = false; break;
      default: break;
    }

    bool fan_changed = (new_fan != fan_speed_ || new_auto != fan_auto_);
    if (fan_changed) {
      entries.push_back({0x00, REG_FAN_SPEED,    (uint32_t)new_fan,         1});
      entries.push_back({0x00, REG_FAN_AUTO_MODE, (uint32_t)(new_auto ? 1 : 0), 1});
      fan_speed_ = new_fan;
      fan_auto_  = new_auto;
    }
  }

  // ── Swing ─────────────────────────────────────────────────────────────────
  if (call.get_swing_mode().has_value()) {
    auto sm = *call.get_swing_mode();
    uint8_t new_v = v_swing_;
    uint8_t new_h = h_swing_;

    switch (sm) {
      case climate::CLIMATE_SWING_OFF:
        new_v = VSWING_FIX_MID;
        new_h = HSWING_FIX_CENTER;
        break;
      case climate::CLIMATE_SWING_VERTICAL:
        new_v = VSWING_UPDOWN;
        new_h = HSWING_FIX_CENTER;
        break;
      case climate::CLIMATE_SWING_HORIZONTAL:
        new_v = VSWING_FIX_MID;
        new_h = HSWING_LR;
        break;
      case climate::CLIMATE_SWING_BOTH:
        new_v = VSWING_UPDOWN;
        new_h = HSWING_LR;
        break;
      default: break;
    }

    if (new_v != v_swing_) {
      entries.push_back({0x00, REG_V_SWING, (uint32_t)new_v, 1});
      v_swing_ = new_v;
    }
    if (new_h != h_swing_) {
      entries.push_back({0x00, REG_H_SWING, (uint32_t)new_h, 1});
      h_swing_ = new_h;
    }
  }

  if (!entries.empty()) {
    send_registers(entries);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Climate traits
// ─────────────────────────────────────────────────────────────────────────────
climate::ClimateTraits AcController::traits() {
  auto traits = climate::ClimateTraits();

  traits.set_supports_current_temperature(true);
  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_FAN_ONLY,
    climate::CLIMATE_MODE_AUTO,
  });

  traits.set_supported_fan_modes({
    climate::CLIMATE_FAN_AUTO,
    climate::CLIMATE_FAN_LOW,
    climate::CLIMATE_FAN_MEDIUM,
    climate::CLIMATE_FAN_HIGH,
  });

  traits.set_supported_swing_modes({
    climate::CLIMATE_SWING_OFF,
    climate::CLIMATE_SWING_VERTICAL,
    climate::CLIMATE_SWING_HORIZONTAL,
    climate::CLIMATE_SWING_BOTH,
  });

  // Temperature range: 60–90°F → 15.6–32.2°C
  traits.set_visual_min_temperature(15.6f);
  traits.set_visual_max_temperature(32.2f);
  traits.set_visual_temperature_step(0.5f);

  return traits;
}

// ─────────────────────────────────────────────────────────────────────────────
// ESPHome lifecycle
// ─────────────────────────────────────────────────────────────────────────────
void AcController::setup() {
  ESP_LOGI(TAG, "AC Controller starting up");
  // Set sensible defaults for climate entity
  target_temperature = 22.0f;   // 72°F
  current_temperature = NAN;
  mode = climate::CLIMATE_MODE_OFF;
  fan_mode = climate::CLIMATE_FAN_AUTO;
  swing_mode = climate::CLIMATE_SWING_OFF;
}

void AcController::loop() {
  // Drain UART RX buffer
  while (available()) {
    uint8_t byte;
    read_byte(&byte);
    process_rx_byte(byte);
  }

  // Send next queued command if ready
  maybe_send_next_command();
}

void AcController::dump_config() {
  ESP_LOGCONFIG(TAG, "AC Controller:");
  ESP_LOGCONFIG(TAG, "  UART configured");
  if (room_temp_sensor_)    ESP_LOGCONFIG(TAG, "  Room temp sensor: YES");
  if (indoor_coil_sensor_)  ESP_LOGCONFIG(TAG, "  Indoor coil sensor: YES");
  if (outdoor_coil_sensor_) ESP_LOGCONFIG(TAG, "  Outdoor coil sensor: YES");
  if (comp_freq_sensor_)    ESP_LOGCONFIG(TAG, "  Compressor freq sensor: YES");
  if (fan_rpm_sensor_)      ESP_LOGCONFIG(TAG, "  Fan RPM sensor: YES");
  if (v_swing_select_)      ESP_LOGCONFIG(TAG, "  V-swing select: YES");
  if (h_swing_select_)      ESP_LOGCONFIG(TAG, "  H-swing select: YES");
  if (eco_switch_)          ESP_LOGCONFIG(TAG, "  Eco switch: YES");
  if (display_switch_)      ESP_LOGCONFIG(TAG, "  Display switch: YES");
  if (beep_switch_)         ESP_LOGCONFIG(TAG, "  Beep switch: YES");
  if (sleep_select_)        ESP_LOGCONFIG(TAG, "  Sleep mode select: YES");
}

// ─────────────────────────────────────────────────────────────────────────────
// Sub-component implementations
// ─────────────────────────────────────────────────────────────────────────────
void AcSwingSelect::control(const std::string &value) {
  auto *parent = (AcController *)parent_;
  uint8_t reg = is_vertical_ ? REG_V_SWING : REG_H_SWING;
  uint8_t val = 0x0B;  // default: fixed middle/center

  if (is_vertical_) {
    if (value == "Swing Up-Down")     val = VSWING_UPDOWN;
    else if (value == "Swing Up")     val = VSWING_UP;
    else if (value == "Swing Down")   val = VSWING_DOWN;
    else if (value == "Fixed Up")          val = VSWING_FIX_UP;
    else if (value == "Fixed Above-Up")    val = VSWING_FIX_ABOVEUP;
    else if (value == "Fixed Middle")      val = VSWING_FIX_MID;
    else if (value == "Fixed Above-Down")  val = VSWING_FIX_ABOVEDN;
    else if (value == "Fixed Down")        val = VSWING_FIX_DOWN;
  } else {
    if (value == "Swing Left-Right")       val = HSWING_LR;
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
  auto *parent = (AcController *)parent_;
  uint8_t reg;
  switch (type_) {
    case ECO:     reg = REG_ECO;     break;
    case DISPLAY: reg = REG_DISPLAY; break;
    case BEEP:    reg = REG_BEEP;    break;
    default: return;
  }
  parent->send_register(0x00, reg, state ? 1 : 0);
  publish_state(state);
}

void AcSleepSelect::control(const std::string &value) {
  auto *parent = (AcController *)parent_;
  uint8_t val = 0;
  if      (value == "Standard") val = 1;
  else if (value == "Aged")     val = 2;
  else if (value == "Child")    val = 3;
  parent->send_register(0x00, REG_SLEEP_MODE, val);
  publish_state(value);
}

}  // namespace ac_controller
}  // namespace esphome
