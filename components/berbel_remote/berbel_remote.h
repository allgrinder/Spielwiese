#pragma once

#ifdef USE_ESP32

#include <cstdint>
#include <deque>
#include <functional>

#include "esphome/core/component.h"
#include "esphome/core/automation.h"

namespace esphome {
namespace berbel_remote {

// Decoded hood status (parsed from the 9-byte packet on f004f001).
struct BerbelStatus {
  bool fan_1{false};
  bool fan_2{false};
  bool fan_3{false};
  bool fan_p{false};
  bool light_up{false};       // Oberlicht
  bool light_down{false};     // Unterlicht / Effektlicht
  bool cover_moving_up{false};
  bool cover_moving_down{false};
  bool afterrun{false};       // Nachlauf
  uint8_t raw[9]{};
};

class BerbelRemote : public Component {
 public:
  void set_mac(uint8_t o1, uint8_t o2, uint8_t o3,
               uint8_t s1, uint8_t s2, uint8_t s3) {
    mac_[0] = o1; mac_[1] = o2; mac_[2] = o3;
    mac_[3] = s1; mac_[4] = s2; mac_[5] = s3;
  }

  void setup() override;
  void loop() override;
  void dump_config() override;
  // Has to run before WiFi so esp_base_mac_addr_set() takes effect.
  float get_setup_priority() const override {
    return setup_priority::BUS + 100.0f;
  }

  // Enqueue a button press; release frame is sent 100 ms later.
  void send_button(uint8_t code);

  bool is_connected() const { return conn_handle_ != 0xFFFF; }
  const BerbelStatus &status() const { return status_; }

  void add_on_connect_callback(std::function<void()> cb) {
    on_connect_.add(std::move(cb));
  }
  void add_on_disconnect_callback(std::function<void()> cb) {
    on_disconnect_.add(std::move(cb));
  }
  void add_on_status_callback(std::function<void(const BerbelStatus &)> cb) {
    on_status_.add(std::move(cb));
  }

  // -- called from C-style NimBLE callbacks --
  void on_button_subscribe(uint16_t conn_handle, bool notify);
  void on_status_write(const uint8_t *data, size_t len);
  void on_gap_connect(uint16_t conn_handle);
  void on_gap_disconnect();
  void start_advertising();  // public so the host sync callback can call it

 protected:
  void apply_mac_();
  void start_nimble_();
  void send_notification_(uint8_t b0, uint8_t b1);
  void decode_status_(const uint8_t *data, size_t len);

  uint8_t mac_[6]{0x88, 0x01, 0xF9, 0xAA, 0xBB, 0xCC};
  uint16_t conn_handle_{0xFFFF};
  uint16_t button_val_handle_{0};
  uint16_t status_val_handle_{0};
  bool button_subscribed_{false};

  BerbelStatus status_{};

  std::deque<uint8_t> tx_queue_;
  uint32_t last_send_ms_{0};
  uint32_t release_at_ms_{0};
  bool waiting_release_{false};

  CallbackManager<void()> on_connect_;
  CallbackManager<void()> on_disconnect_;
  CallbackManager<void(const BerbelStatus &)> on_status_;
};

// --------- Automations --------------------------------------------------

template<typename... Ts>
class SendButtonAction : public Action<Ts...> {
 public:
  explicit SendButtonAction(BerbelRemote *parent) : parent_(parent) {}
  void set_code(uint8_t code) { code_ = code; }
  void play(Ts... x) override { parent_->send_button(code_); }

 protected:
  BerbelRemote *parent_;
  uint8_t code_{0};
};

class ConnectTrigger : public Trigger<> {
 public:
  explicit ConnectTrigger(BerbelRemote *parent) {
    parent->add_on_connect_callback([this]() { this->trigger(); });
  }
};

class DisconnectTrigger : public Trigger<> {
 public:
  explicit DisconnectTrigger(BerbelRemote *parent) {
    parent->add_on_disconnect_callback([this]() { this->trigger(); });
  }
};

class StatusTrigger : public Trigger<const BerbelStatus &> {
 public:
  explicit StatusTrigger(BerbelRemote *parent) {
    parent->add_on_status_callback(
        [this](const BerbelStatus &s) { this->trigger(s); });
  }
};

}  // namespace berbel_remote
}  // namespace esphome

#endif  // USE_ESP32
