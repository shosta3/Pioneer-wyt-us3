#include "ac_controller.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ac_controller {

static const char *const TAG = "ac_controller";

// ── CRC-16/XMODEM ─────────────────────────────────────────────────────────────
uint16_t AcController::crc16_xmodem(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t) data[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

// ── Frame building ────────────────────────────────────────────────────────────
std::vector<uint8_t> AcController::build_frame(const std::vector<uint8_t> &payload) {
  uint8_t total_len = (uint8_t)(10 + payload.size());
  std::vector<uint8_t> frame;
  frame.reserve(total_len);
  frame.push_back(FRAME_HEADER);
  frame.push_back(BUS_ID);
  frame.push_back(DEV_ID);
  frame.push_back(TYPE_CMD);
  frame.push_back(tx_seq_);
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(total_len);
  frame.push_back(0x00);  // CRC1 placeholder
  frame.push_back(0x00);  // CRC2 placeholder
  for (auto b : payload) frame.push_back(b);

  // CRC over bytes 0-7 + payload (skipping CRC bytes 8-9)
  std::vector<uint8_t> crc_input(frame.begin(), frame.begin() + 8);
  crc_input.insert(crc_input.end(), payload.begin(), payload.end());
  uint16_t crc = crc16_xmodem(crc_input.data(), crc_input.size());
  frame[8] = (crc >> 8) & 0xFF;
  frame[9] = crc & 0xFF;
  return frame;
}

std::vector<uint8_t> AcController::build_ack_frame(uint8_t seq, uint8_t dev_id) {
  // ACK frame type=0x23, seq1=0x00, seq2=echoed seq, payload=0x80 0x0A
  std::vector<uint8_t> frame;
  frame.push_back(FRAME_HEADER);
  frame.push_back(BUS_ID);
  frame.push_back(dev_id);
  frame.push_back(TYPE_ACK);
  frame.push_back(0x00);
  frame.push_back(seq);
  frame.push_back(0x00);
  frame.push_back(12);   // total length
  frame.push_back(0x00); // CRC1
  frame.push_back(0x00); // CRC2
  frame.push_back(0x80);
  frame.push_back(ACK_PAYLOAD_IN);

  // CRC over header bytes + payload
  uint8_t crc_buf[10] = {
    frame[0], frame[1], frame[2], frame[3],
    frame[4], frame[5], frame[6], frame[7],
    0x80, ACK_PAYLOAD_IN
  };
  uint16_t crc = crc16_xmodem(crc_buf, 10);
  frame[8] = (crc >> 8) & 0xFF;
  frame[9] = crc & 0xFF;
  return frame;
}

// ── TLV helpers ───────────────────────────────────────────────────────────────
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
      e.size = 4;
      i += 6;
    } else if (bank == 0x00) {
      if (i + 3 > len) break;
      e.value = data[i + 2];
      e.size  = 1;
      i += 3;
    } else if (bank == 0x01) {
      if (i + 4 > len) break;
      e.value = ((uint32_t) data[i+2] << 8) | data[i+3];
      e.size  = 2;
      i += 4;
    } else if (bank == 0x02) {
      if (i + 6 > len) break;
      e.value = ((uint32_t) data[i+2] << 24) | ((uint32_t) data[i+3] << 16) |
                ((uint32_t) data[i+4] <<  8) |  (uint32_t) data[i+5];
      e.size = 4;
      i += 6;
    } else {
      i += 2;
      continue;
    }
    entries.push_back(e);
  }
  return entries;
}

static std::vector<uint8_t> build_tlv_payload(const std::vector<TlvEntry> &entries) {
  std::vector<uint8_t> payload;
  payload.push_back(PREFIX_CTRL_HI);
  payload.push_back(PREFIX_CTRL_LO);
  for (const TlvEntry &e : entries) {
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

// ── TX queue ──────────────────────────────────────────────────────────────────
void AcController::enqueue_command(const std::vector<uint8_t> &payload) {
  PendingCommand cmd;
  cmd.payload = payload;
  tx_queue_.push(cmd);
}

void AcController::send_register(uint8_t bank, uint8_t reg, uint32_t value) {
  uint8_t size = (bank == 0x02 || is_4byte_special(reg)) ? 4 : (bank == 0x01 ? 2 : 1);
  TlvEntry e;
  e.bank  = bank;
  e.reg   = reg;
  e.value = value;
  e.size  = size;
  std::vector<TlvEntry> v;
  v.push_back(e);
  send_registers(v);
}

void AcController::send_registers(const std::vector<TlvEntry> &entries) {
  enqueue_command(build_tlv_payload(entries));
}

void AcController::send_frame(const std::vector<uint8_t> &frame) {
  write_array(frame.data(), frame.size());
  ESP_LOGV(TAG, "TX [%d bytes]", (int) frame.size());
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
  auto frame = build_frame(cmd.payload);
  send_frame(frame);
  last_tx_ms_ = millis();
  waiting_for_ack_ = true;
  tx_seq_ = (tx_seq_ == 0xFF) ? 0x01 : tx_seq_ + 1;
  tx_queue_.pop();
}

// ── RX parsing ────────────────────────────────────────────────────────────────
void AcController::process_rx_byte(uint8_t byte) {
  if (rx_buf_.empty() && byte != FRAME_HEADER) return;
  rx_buf_.push_back(byte);
  if (rx_buf_.size() < 8) return;

  uint8_t expected_len = rx_buf_[7];
  if (expected_len < 10 || expected_len > 128) {
    rx_buf_.clear();
    return;
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

  // Build CRC input: header bytes + payload
  std::vector<uint8_t> crc_input(rx_buf_.begin(), rx_buf_.begin() + 8);
  crc_input.insert(crc_input.end(), rx_buf_.begin() + 10, rx_buf_.begin() + len);
  uint16_t calc  = crc16_xmodem(crc_input.data(), crc_input.size());
  uint16_t given = ((uint16_t) rx_buf_[8] << 8) | rx_buf_[9];

  if (calc != given) {
    ESP_LOGW(TAG, "CRC mismatch calc=0x%04X given=0x%04X", calc, given);
    return false;
  }

  std::vector<uint8_t> frame(rx_buf_.begin(), rx_buf_.begin() + len);
  if (frame[3] == TYPE_CMD) {
    handle_indoor_frame(frame);
  } else if (frame[3] == TYPE_ACK) {
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
  ESP_LOGV(TAG, "RX CMD seq=%02X len=%d", seq, (int) frame.size());

  // ACK immediately — mirror the device ID from the incoming frame
  send_frame(build_ack_frame(seq, frame[2]));

  const uint8_t *payload = frame.data() + 10;
  size_t plen = frame.size() - 10;
  if (plen < 2) return;
  if (payload[0] != PREFIX_INDOOR_HI || payload[1] != PREFIX_INDOOR_LO) return;

  // Detect sensor scan: look for REG_SENSOR_MARKER in TLV data
  const uint8_t *tlv = payload + 2;
  size_t tlvlen = plen - 2;
  bool is_sensor_scan = false;
  for (size_t i = 0; i + 1 < tlvlen; i++) {
    if (tlv[i] == 0x00 && tlv[i + 1] == REG_SENSOR_MARKER) {
      is_sensor_scan = true;
      break;
    }
  }

  apply_tlv_entries(parse_tlv(tlv, tlvlen), is_sensor_scan);
}

// ── State application ─────────────────────────────────────────────────────────
void AcController::apply_tlv_entries(const std::vector<TlvEntry> &entries, bool is_sensor_scan) {
  bool changed = false;

  for (size_t i = 0; i < entries.size(); i++) {
    const TlvEntry &e = entries[i];

    if (is_sensor_scan) {
      if (e.reg == SENSOR_ROOM_TEMP && e.bank == 0x00) {
        float f = (float) e.value;
        if (f > 32.0f && f < 120.0f) {
          room_temp_f_ = f;
          current_temperature = (f - 32.0f) / 1.8f;
          if (room_temp_sensor_) room_temp_sensor_->publish_state(f);
          changed = true;
        }
      }
      continue;
    }

    switch (e.reg) {
      case REG_POWER:
        power_ = (e.value != 0);
        changed = true;
        break;
      case REG_MODE:
        mode_ = (uint8_t) e.value;
        changed = true;
        break;
      case REG_SET_TEMP:
        if (e.bank == 0x02 || e.size == 4) {
          set_temp_f_ = (float) e.value;
          target_temperature = (set_temp_f_ - 32.0f) / 1.8f;
          changed = true;
        }
        break;
      case REG_FAN_SPEED:
        fan_speed_ = (uint8_t) e.value;
        changed = true;
        break;
      case REG_FAN_AUTO_MODE:
        fan_auto_ = (e.value != 0);
        changed = true;
        break;
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
        comp_freq_ = (uint8_t) e.value;
        if (comp_freq_sensor_) comp_freq_sensor_->publish_state(comp_freq_);
        break;
      case REG_FAN_RPM:
        fan_rpm_pct_ = (uint8_t)(e.value & 0xFF);
        if (fan_rpm_sensor_) fan_rpm_sensor_->publish_state(fan_rpm_pct_);
        break;
      case REG_INDOOR_COIL:
        if (e.value > 0) {
          indoor_coil_f_ = (float) e.value;
          if (indoor_coil_sensor_) indoor_coil_sensor_->publish_state(indoor_coil_f_);
        }
        break;
      case REG_OUTDOOR_COIL: {
        int32_t sv = (int32_t) e.value;
        outdoor_coil_c_ = sv / 10.0f;
        if (outdoor_coil_sensor_) outdoor_coil_sensor_->publish_state(outdoor_coil_c_);
        break;
      }
      default:
        ESP_LOGV(TAG, "Unhandled reg=0x%02X bank=0x%02X val=0x%08X", e.reg, e.bank, e.value);
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

// ── String helpers ────────────────────────────────────────────────────────────
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

// ── Climate::control ──────────────────────────────────────────────────────────
void AcController::control(const climate::ClimateCall &call) {
  std::vector<TlvEntry> entries;

  if (call.get_mode().has_value()) {
    auto nm = *call.get_mode();
    bool np = (nm != climate::CLIMATE_MODE_OFF);
    if (np != power_) {
      TlvEntry e; e.bank=0x00; e.reg=REG_POWER; e.value=(np?1:0); e.size=1;
      entries.push_back(e);
      power_ = np;
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
        TlvEntry e; e.bank=0x00; e.reg=REG_MODE; e.value=am; e.size=1;
        entries.push_back(e);
        mode_ = am;
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
    uint8_t nf = fan_speed_;
    bool na = false;
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
      entries.push_back(e1);
      entries.push_back(e2);
      fan_speed_ = nf;
      fan_auto_  = na;
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
      entries.push_back(e);
      v_swing_ = nv;
    }
    if (nh != h_swing_) {
      TlvEntry e; e.bank=0x00; e.reg=REG_H_SWING; e.value=nh; e.size=1;
      entries.push_back(e);
      h_swing_ = nh;
    }
  }

  if (!entries.empty())
    send_registers(entries);
}

// ── Climate traits ────────────────────────────────────────────────────────────
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
  // Temperature range: ~60-90°F in °C
  t.set_visual_min_temperature(15.5f);
  t.set_visual_max_temperature(32.0f);
  t.set_visual_temperature_step(0.5f);
  return t;
}


// ── Periodic poll frame ───────────────────────────────────────────────────────
// Mimics the real controller's ~30s heartbeat poll to keep the indoor unit
// in its normal low-traffic broadcast rhythm rather than "no controller" mode.
void AcController::send_poll_frame() {
  // A5 01 00 21 00 00 00 0C CRC1 CRC2 0C 0C
  std::vector<uint8_t> payload = {0x0C, 0x0C};
  uint8_t total_len = 12;
  std::vector<uint8_t> frame;
  frame.push_back(FRAME_HEADER);
  frame.push_back(BUS_ID);
  frame.push_back(0x00);       // dev_id = 0x00 (bus broadcast)
  frame.push_back(TYPE_CMD);
  frame.push_back(0x00);       // seq = 0x00
  frame.push_back(0x00);
  frame.push_back(0x00);
  frame.push_back(total_len);
  frame.push_back(0x00);       // CRC placeholder
  frame.push_back(0x00);
  frame.push_back(0x0C);
  frame.push_back(0x0C);

  uint8_t crc_buf[10] = {
    frame[0], frame[1], frame[2], frame[3],
    frame[4], frame[5], frame[6], frame[7],
    0x0C, 0x0C
  };
  uint16_t crc = crc16_xmodem(crc_buf, 10);
  frame[8] = (crc >> 8) & 0xFF;
  frame[9] = crc & 0xFF;

  ESP_LOGD(TAG, "Sending poll frame");
  send_frame(frame);
  last_poll_ms_ = millis();
}

// ── ESPHome lifecycle ─────────────────────────────────────────────────────────
void AcController::setup() {
  ESP_LOGI(TAG, "AC Controller setup");
  target_temperature  = 22.0f;
  current_temperature = NAN;
  mode       = climate::CLIMATE_MODE_OFF;
  fan_mode   = climate::CLIMATE_FAN_AUTO;
  swing_mode = climate::CLIMATE_SWING_OFF;
}

void AcController::loop() {
  while (available()) {
    uint8_t b;
    read_byte(&b);
    process_rx_byte(b);
  }
  maybe_send_next_command();

  // Send periodic poll to keep indoor unit in normal broadcast mode
  if (millis() - last_poll_ms_ > POLL_INTERVAL_MS) {
    send_poll_frame();
  }
}

void AcController::dump_config() {
  ESP_LOGCONFIG(TAG, "AC Controller:");
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

// ── Sub-component implementations ─────────────────────────────────────────────
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
