#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"

#include <vector>
#include <queue>
#include <string>

// Undefine Arduino ESP8266 macros that collide with our names
#ifdef DISPLAY
#undef DISPLAY
#endif
#ifdef ECO
#undef ECO
#endif

namespace esphome {
namespace ac_controller {

// ── Register addresses ────────────────────────────────────────────────────────
static const uint8_t REG_POWER         = 0x01;
static const uint8_t REG_FAN_SPEED     = 0x05;
static const uint8_t REG_V_SWING_MOTOR = 0x0C;
static const uint8_t REG_H_SWING_MOTOR = 0x0D;
static const uint8_t REG_H_SWING       = 0x0E;
static const uint8_t REG_V_SWING       = 0x11;
static const uint8_t REG_MODE          = 0x12;
static const uint8_t REG_DISPLAY       = 0x1E;
static const uint8_t REG_SLEEP_MODE    = 0x22;
static const uint8_t REG_BEEP          = 0x25;
static const uint8_t REG_SET_TEMP      = 0x27;  // bank 0x02, 4-byte, °F
static const uint8_t REG_STATUS_A      = 0x02;
static const uint8_t REG_STATUS_B      = 0x03;
static const uint8_t REG_COMP_FREQ     = 0x09;
static const uint8_t REG_OUTDOOR_COIL  = 0x60;  // special 4-byte, tenths °C signed
static const uint8_t REG_INDOOR_COIL   = 0x65;  // special 4-byte, °F
static const uint8_t REG_FAN_RPM       = 0x72;  // special 4-byte, %
static const uint8_t REG_FAN_AUTO_MODE = 0x73;
static const uint8_t REG_ECO           = 0xDF;
static const uint8_t REG_RUN_STATE     = 0x38;

static const uint8_t SENSOR_ROOM_TEMP  = 0x0D;
static const uint8_t REG_SENSOR_MARKER = 0x5C;

// ── Protocol constants ────────────────────────────────────────────────────────
static const uint8_t FRAME_HEADER     = 0xA5;
static const uint8_t BUS_ID           = 0x01;
static const uint8_t DEV_NORMAL       = 0x01;  // normal addressed
static const uint8_t DEV_HEARTBEAT    = 0x00;  // bus-level heartbeat/broadcast
static const uint8_t TYPE_CMD         = 0x21;
static const uint8_t TYPE_ACK         = 0x23;
static const uint8_t PREFIX_CTRL_HI   = 0x0A;
static const uint8_t PREFIX_CTRL_LO   = 0x0A;
static const uint8_t PREFIX_INDOOR_HI = 0x0C;
static const uint8_t PREFIX_INDOOR_LO = 0x0C;

// ── Fan speed values ──────────────────────────────────────────────────────────
static const uint8_t FAN_AUTO     = 0x00;
static const uint8_t FAN_MUTE     = 0x01;
static const uint8_t FAN_LOW      = 0x02;
static const uint8_t FAN_MID_LOW  = 0x03;
static const uint8_t FAN_MID      = 0x04;
static const uint8_t FAN_MID_HIGH = 0x05;
static const uint8_t FAN_HIGH     = 0x06;
static const uint8_t FAN_STRONG   = 0x07;

// ── Mode values ───────────────────────────────────────────────────────────────
static const uint8_t MODE_AUTO     = 0x00;
static const uint8_t MODE_COOL     = 0x01;
static const uint8_t MODE_DRY      = 0x02;
static const uint8_t MODE_FAN_ONLY = 0x03;
static const uint8_t MODE_HEAT     = 0x04;

// ── V-Swing values ────────────────────────────────────────────────────────────
static const uint8_t VSWING_UPDOWN      = 0x01;
static const uint8_t VSWING_UP          = 0x02;
static const uint8_t VSWING_DOWN        = 0x03;
static const uint8_t VSWING_FIX_UP      = 0x09;
static const uint8_t VSWING_FIX_ABOVEUP = 0x0A;
static const uint8_t VSWING_FIX_MID     = 0x0B;
static const uint8_t VSWING_FIX_ABOVEDN = 0x0C;
static const uint8_t VSWING_FIX_DOWN    = 0x0D;

// ── H-Swing values ────────────────────────────────────────────────────────────
static const uint8_t HSWING_LR            = 0x01;
static const uint8_t HSWING_LEFT          = 0x02;
static const uint8_t HSWING_CENTER        = 0x03;
static const uint8_t HSWING_RIGHT         = 0x04;
static const uint8_t HSWING_FIX_LEFT      = 0x09;
static const uint8_t HSWING_FIX_ABITLEFT  = 0x0A;
static const uint8_t HSWING_FIX_CENTER    = 0x0B;
static const uint8_t HSWING_FIX_ABITRIGHT = 0x0C;
static const uint8_t HSWING_FIX_RIGHT     = 0x0D;

// ── Handshake states ──────────────────────────────────────────────────────────
enum HandshakeState {
  HS_IDLE,            // not yet started
  HS_IDENTITY_1,      // sent [00 00 01], waiting for response
  HS_ANNOUNCE_1,      // sent [0B 0B FF FF] dev=01, waiting for [80 0B]
  HS_IDENTITY_2,      // sent [00 00 01] again
  HS_BUS_HANDSHAKE_A, // sent [0B 0B 03 05] dev=00 first time
  HS_BUS_HANDSHAKE_B, // sent [0B 0B 03 05] dev=00 second time
  HS_IDENTITY_3,      // sent [00 00 01] third time
  HS_WAIT_REANNOUNCE, // waiting ~5s before second announce
  HS_ANNOUNCE_2,      // sent [0B 0B FF FF] dev=01 second time
  HS_POWER_SYNC,      // unused — [28 28] is sent BY indoor to us, not the other way
  HS_IDENTITY_4,      // sent [00 00 01] fourth time
  HS_TIMESTAMP,       // sent [15 15 ...] timestamp frame
  HS_COMPLETE,        // handshake done, normal operation
};

// ── String helpers ────────────────────────────────────────────────────────────
std::string v_swing_val_to_str(uint8_t val);
std::string h_swing_val_to_str(uint8_t val);

// ── TLV entry ─────────────────────────────────────────────────────────────────
struct TlvEntry {
  uint8_t  bank;
  uint8_t  reg;
  uint32_t value;
  uint8_t  size;
};

// ── Pending command ───────────────────────────────────────────────────────────
struct PendingCommand {
  std::vector<uint8_t> payload;
  uint8_t dev_id;   // DEV_NORMAL or DEV_HEARTBEAT
};

// ── SwingSelect ───────────────────────────────────────────────────────────────
class AcSwingSelect : public select::Select, public Component {
 public:
  void set_is_vertical(bool v) { is_vertical_ = v; }
  void set_parent(void *p)     { parent_ = p; }
  void control(const std::string &value) override;
 protected:
  bool  is_vertical_{true};
  void *parent_{nullptr};
};

// ── Generic switch ────────────────────────────────────────────────────────────
class AcSwitch : public switch_::Switch, public Component {
 public:
  enum SwitchKind { KIND_ECO, KIND_DISPLAY, KIND_BEEP };
  void set_kind(SwitchKind k) { kind_ = k; }
  void set_parent(void *p)    { parent_ = p; }
  void write_state(bool state) override;
 protected:
  SwitchKind kind_{KIND_ECO};
  void      *parent_{nullptr};
};

// ── Sleep mode select ─────────────────────────────────────────────────────────
class AcSleepSelect : public select::Select, public Component {
 public:
  void set_parent(void *p) { parent_ = p; }
  void control(const std::string &value) override;
 protected:
  void *parent_{nullptr};
};

// ── Main component ────────────────────────────────────────────────────────────
class AcController : public climate::Climate, public uart::UARTDevice, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  // Sub-component setters
  void set_room_temp_sensor(sensor::Sensor *s)    { room_temp_sensor_    = s; }
  void set_indoor_coil_sensor(sensor::Sensor *s)  { indoor_coil_sensor_  = s; }
  void set_outdoor_coil_sensor(sensor::Sensor *s) { outdoor_coil_sensor_ = s; }
  void set_comp_freq_sensor(sensor::Sensor *s)    { comp_freq_sensor_    = s; }
  void set_fan_rpm_sensor(sensor::Sensor *s)      { fan_rpm_sensor_      = s; }
  void set_v_swing_select(AcSwingSelect *s)       { v_swing_select_      = s; }
  void set_h_swing_select(AcSwingSelect *s)       { h_swing_select_      = s; }
  void set_eco_switch(AcSwitch *s)                { eco_switch_          = s; }
  void set_display_switch(AcSwitch *s)            { display_switch_      = s; }
  void set_beep_switch(AcSwitch *s)               { beep_switch_         = s; }
  void set_sleep_select(AcSleepSelect *s)         { sleep_select_        = s; }

  // Called by sub-components
  void send_register(uint8_t bank, uint8_t reg, uint32_t value);
  void send_registers(const std::vector<TlvEntry> &entries);

 protected:
  // ── CRC ───────────────────────────────────────────────────────────────────
  uint16_t crc16_xmodem(const uint8_t *data, size_t len);

  // ── Frame building ────────────────────────────────────────────────────────
  std::vector<uint8_t> build_frame(const std::vector<uint8_t> &payload,
                                   uint8_t dev_id = DEV_NORMAL);
  std::vector<uint8_t> build_ack_frame(uint8_t seq, uint8_t dev_id = DEV_NORMAL);
  void enqueue_command(const std::vector<uint8_t> &payload,
                       uint8_t dev_id = DEV_NORMAL);

  // ── Handshake ─────────────────────────────────────────────────────────────
  void run_handshake();
  void send_handshake_frame(const std::vector<uint8_t> &payload,
                            uint8_t dev_id, HandshakeState next_state,
                            bool wait_ack = true);
  void advance_handshake(bool ack_received, const std::vector<uint8_t> &frame);

  // ── Frame parsing ─────────────────────────────────────────────────────────
  void process_rx_byte(uint8_t byte);
  bool try_parse_frame();
  void handle_indoor_frame(const std::vector<uint8_t> &frame);
  void handle_ack_frame(const std::vector<uint8_t> &frame);
  std::vector<TlvEntry> parse_tlv(const uint8_t *data, size_t len);
  bool is_4byte_special(uint8_t reg);

  // ── State application ─────────────────────────────────────────────────────
  void apply_tlv_entries(const std::vector<TlvEntry> &entries, bool is_sensor_scan);
  void publish_climate_state();

  // ── TX ────────────────────────────────────────────────────────────────────
  void send_frame(const std::vector<uint8_t> &frame);
  void maybe_send_next_command();
  void send_poll_frame();

  // ── Timing constants ──────────────────────────────────────────────────────
  static const uint32_t ACK_TIMEOUT_MS   = 500;
  static const uint32_t HS_STEP_DELAY_MS = 200;   // between handshake steps
  static const uint32_t HS_REANNOUNCE_MS = 5000;  // wait before second announce
  static const uint32_t POLL_INTERVAL_MS = 28000;

  // ── TX state ──────────────────────────────────────────────────────────────
  uint8_t  tx_seq_{0x01};
  bool     waiting_for_ack_{false};
  uint32_t last_tx_ms_{0};
  uint32_t last_poll_ms_{0};

  std::queue<PendingCommand> tx_queue_;
  std::vector<uint8_t>       rx_buf_;

  // ── Handshake state ───────────────────────────────────────────────────────
  HandshakeState hs_state_{HS_IDLE};
  uint32_t       hs_step_ms_{0};       // time of last hs step sent
  bool           hs_waiting_ack_{false};
  uint8_t        hs_seq_{0x01};        // dedicated seq counter for handshake frames

  // ── Unit state ────────────────────────────────────────────────────────────
  bool    power_{false};
  uint8_t mode_{MODE_COOL};
  float   set_temp_f_{72.0f};
  uint8_t fan_speed_{FAN_AUTO};
  bool    fan_auto_{true};
  uint8_t v_swing_{VSWING_FIX_MID};
  uint8_t h_swing_{HSWING_FIX_CENTER};
  bool    eco_{false};
  bool    disp_{true};
  bool    beep_{true};
  uint8_t sleep_mode_{0};

  float   room_temp_f_{0.0f};
  float   indoor_coil_f_{0.0f};
  float   outdoor_coil_c_{0.0f};
  uint8_t comp_freq_{0};
  uint8_t fan_rpm_pct_{0};

  bool    state_received_{false};
  bool    pending_mode_followup_{false};

  // ── Sub-components ────────────────────────────────────────────────────────
  sensor::Sensor  *room_temp_sensor_{nullptr};
  sensor::Sensor  *indoor_coil_sensor_{nullptr};
  sensor::Sensor  *outdoor_coil_sensor_{nullptr};
  sensor::Sensor  *comp_freq_sensor_{nullptr};
  sensor::Sensor  *fan_rpm_sensor_{nullptr};
  AcSwingSelect   *v_swing_select_{nullptr};
  AcSwingSelect   *h_swing_select_{nullptr};
  AcSwitch        *eco_switch_{nullptr};
  AcSwitch        *display_switch_{nullptr};
  AcSwitch        *beep_switch_{nullptr};
  AcSleepSelect   *sleep_select_{nullptr};
};

}  // namespace ac_controller
}  // namespace esphome
