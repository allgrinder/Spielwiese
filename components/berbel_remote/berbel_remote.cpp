#ifdef USE_ESP32

#include "berbel_remote.h"

#include <cstring>

#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include "esp_mac.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_id.h"
#include "host/ble_store.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace esphome {
namespace berbel_remote {

static const char *const TAG = "berbel_remote";

// Singleton pointer so C callbacks can reach the component.
static BerbelRemote *g_instance = nullptr;

// -- UUIDs (NimBLE wants 128-bit UUIDs in little-endian byte order) --
//
// Service:           f004f000-5745-4053-8043-62657262656c
// Status char (W):   f004f001-5745-4053-8043-62657262656c
// Button char (N):   f004f002-5745-4053-8043-62657262656c
static const ble_uuid128_t SVC_BERBEL_UUID = BLE_UUID128_INIT(
    0x6c, 0x65, 0x62, 0x72, 0x65, 0x62, 0x43, 0x80,
    0x53, 0x40, 0x45, 0x57, 0x00, 0xf0, 0x04, 0xf0);
static const ble_uuid128_t CHR_STATUS_UUID = BLE_UUID128_INIT(
    0x6c, 0x65, 0x62, 0x72, 0x65, 0x62, 0x43, 0x80,
    0x53, 0x40, 0x45, 0x57, 0x01, 0xf0, 0x04, 0xf0);
static const ble_uuid128_t CHR_BUTTON_UUID = BLE_UUID128_INIT(
    0x6c, 0x65, 0x62, 0x72, 0x65, 0x62, 0x43, 0x80,
    0x53, 0x40, 0x45, 0x57, 0x02, 0xf0, 0x04, 0xf0);

// -- Static characteristic payloads --
static const char DIS_MANUFACTURER[] = "Texas Instruments";
// PnP ID: vendor source SIG (0x01), vendor 0x000D, product 0x0000, version 0x0010
static const uint8_t DIS_PNP_ID[7] = {0x01, 0x0D, 0x00, 0x00, 0x00, 0x10, 0x00};
// Battery level (the original firmware reports a fixed 90 %).
static uint8_t BATTERY_LEVEL = 90;
// HID Information: bcdHID 1.11, country 0, flags = RemoteWake+NormallyConnectable.
static const uint8_t HID_INFO[4] = {0x11, 0x01, 0x00, 0x03};
// Minimal Consumer-Control HID report map (21 bytes).
static const uint8_t HID_REPORT_MAP[] = {
    0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x01, 0x09, 0xE0,
    0x15, 0xE8, 0x25, 0x18, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06, 0xC0};
// Report reference: report ID 1, input.
static const uint8_t HID_REPORT_REF[2] = {0x01, 0x01};
static uint8_t HID_PROTOCOL_MODE = 0x01;  // Report Protocol
static const uint8_t HID_REPORT_VALUE[1] = {0x00};

// Returned through `arg` to the generic static-read access callback.
struct StaticBuf {
  const uint8_t *data;
  size_t len;
};
static const StaticBuf BUF_MANUFACTURER{
    reinterpret_cast<const uint8_t *>(DIS_MANUFACTURER),
    sizeof(DIS_MANUFACTURER) - 1};
static const StaticBuf BUF_PNP_ID{DIS_PNP_ID, sizeof(DIS_PNP_ID)};
static const StaticBuf BUF_HID_INFO{HID_INFO, sizeof(HID_INFO)};
static const StaticBuf BUF_HID_REPORT_MAP{HID_REPORT_MAP, sizeof(HID_REPORT_MAP)};
static const StaticBuf BUF_HID_REPORT_REF{HID_REPORT_REF, sizeof(HID_REPORT_REF)};
static const StaticBuf BUF_HID_REPORT_VALUE{HID_REPORT_VALUE, sizeof(HID_REPORT_VALUE)};

// Forward declarations for the access callbacks.
static int berbel_static_read_cb(uint16_t conn, uint16_t attr,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);
static int berbel_battery_cb(uint16_t conn, uint16_t attr,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);
static int berbel_protocol_mode_cb(uint16_t conn, uint16_t attr,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg);
static int berbel_status_cb(uint16_t conn, uint16_t attr,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);
static int berbel_button_cb(uint16_t conn, uint16_t attr,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);
static int berbel_dsc_report_ref_cb(uint16_t conn, uint16_t attr,
                                    struct ble_gatt_access_ctxt *ctxt,
                                    void *arg);

// Storage for the discovered attribute handles.
static uint16_t s_button_val_handle = 0;
static uint16_t s_status_val_handle = 0;
static uint16_t s_battery_val_handle = 0;

// -- GATT service table ---------------------------------------------------

static const struct ble_gatt_dsc_def s_hid_report_descriptors[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2908),  // Report Reference
        .att_flags = BLE_ATT_F_READ,
        .access_cb = berbel_dsc_report_ref_cb,
        .arg = const_cast<void *>(static_cast<const void *>(&BUF_HID_REPORT_REF)),
    },
    {0},
};

static const struct ble_gatt_chr_def s_dis_chrs[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2A29),  // Manufacturer Name
        .access_cb = berbel_static_read_cb,
        .arg = const_cast<void *>(static_cast<const void *>(&BUF_MANUFACTURER)),
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        .uuid = BLE_UUID16_DECLARE(0x2A50),  // PnP ID
        .access_cb = berbel_static_read_cb,
        .arg = const_cast<void *>(static_cast<const void *>(&BUF_PNP_ID)),
        .flags = BLE_GATT_CHR_F_READ,
    },
    {0},
};

static const struct ble_gatt_chr_def s_bas_chrs[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2A19),  // Battery Level
        .access_cb = berbel_battery_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_battery_val_handle,
    },
    {0},
};

static const struct ble_gatt_chr_def s_hid_chrs[] = {
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4A),  // HID Information
        .access_cb = berbel_static_read_cb,
        .arg = const_cast<void *>(static_cast<const void *>(&BUF_HID_INFO)),
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4B),  // Report Map
        .access_cb = berbel_static_read_cb,
        .arg = const_cast<void *>(static_cast<const void *>(&BUF_HID_REPORT_MAP)),
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4D),  // Report
        .access_cb = berbel_static_read_cb,
        .arg = const_cast<void *>(static_cast<const void *>(&BUF_HID_REPORT_VALUE)),
        .descriptors = const_cast<struct ble_gatt_dsc_def *>(s_hid_report_descriptors),
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4E),  // Protocol Mode
        .access_cb = berbel_protocol_mode_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = BLE_UUID16_DECLARE(0x2A4C),  // HID Control Point
        .access_cb = berbel_protocol_mode_cb,
        .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {0},
};

static const struct ble_gatt_chr_def s_berbel_chrs[] = {
    {
        .uuid = reinterpret_cast<const ble_uuid_t *>(&CHR_STATUS_UUID),
        .access_cb = berbel_status_cb,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY |
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC,
        .val_handle = &s_status_val_handle,
    },
    {
        .uuid = reinterpret_cast<const ble_uuid_t *>(&CHR_BUTTON_UUID),
        .access_cb = berbel_button_cb,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ |
                 BLE_GATT_CHR_F_READ_ENC,
        .val_handle = &s_button_val_handle,
    },
    {0},
};

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),  // Device Information
        .characteristics = s_dis_chrs,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F),  // Battery
        .characteristics = s_bas_chrs,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1812),  // HID
        .characteristics = s_hid_chrs,
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = reinterpret_cast<const ble_uuid_t *>(&SVC_BERBEL_UUID),
        .characteristics = s_berbel_chrs,
    },
    {0},
};

// -- Access callbacks -----------------------------------------------------

static int berbel_static_read_cb(uint16_t, uint16_t,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
  auto *buf = static_cast<const StaticBuf *>(arg);
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
  int rc = os_mbuf_append(ctxt->om, buf->data, buf->len);
  return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int berbel_dsc_report_ref_cb(uint16_t, uint16_t,
                                    struct ble_gatt_access_ctxt *ctxt,
                                    void *arg) {
  auto *buf = static_cast<const StaticBuf *>(arg);
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) return BLE_ATT_ERR_UNLIKELY;
  int rc = os_mbuf_append(ctxt->om, buf->data, buf->len);
  return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int berbel_battery_cb(uint16_t, uint16_t,
                             struct ble_gatt_access_ctxt *ctxt, void *) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
  int rc = os_mbuf_append(ctxt->om, &BATTERY_LEVEL, sizeof(BATTERY_LEVEL));
  return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int berbel_protocol_mode_cb(uint16_t, uint16_t,
                                   struct ble_gatt_access_ctxt *ctxt, void *) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    return os_mbuf_append(ctxt->om, &HID_PROTOCOL_MODE, 1) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    uint16_t got = OS_MBUF_PKTLEN(ctxt->om);
    if (got >= 1) {
      ble_hs_mbuf_to_flat(ctxt->om, &HID_PROTOCOL_MODE, 1, nullptr);
    }
    return 0;
  }
  return BLE_ATT_ERR_UNLIKELY;
}

static int berbel_status_cb(uint16_t conn_handle, uint16_t,
                            struct ble_gatt_access_ctxt *ctxt, void *) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    uint8_t buf[9];
    uint16_t out_len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &out_len);
    if (rc == 0 && out_len > 0 && g_instance != nullptr) {
      g_instance->on_status_write(buf, out_len);
    }
    return 0;
  }
  return BLE_ATT_ERR_UNLIKELY;
}

static int berbel_button_cb(uint16_t, uint16_t,
                            struct ble_gatt_access_ctxt *ctxt, void *) {
  // The button characteristic does not have a meaningful read payload;
  // we expose READ to keep some central stacks happy.
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    uint8_t zero[2] = {0, 0};
    return os_mbuf_append(ctxt->om, zero, 2) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  return BLE_ATT_ERR_UNLIKELY;
}

// -- GAP / host event handling -------------------------------------------

static int berbel_gap_event(struct ble_gap_event *event, void *) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        if (g_instance != nullptr)
          g_instance->on_gap_connect(event->connect.conn_handle);
      } else {
        ESP_LOGW(TAG, "Connect failed (status=%d), re-advertising",
                 event->connect.status);
        if (g_instance != nullptr) g_instance->on_gap_disconnect();
      }
      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      ESP_LOGI(TAG, "Disconnected (reason=0x%02x)",
               event->disconnect.reason);
      if (g_instance != nullptr) g_instance->on_gap_disconnect();
      return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
      if (event->subscribe.attr_handle == s_button_val_handle &&
          g_instance != nullptr) {
        g_instance->on_button_subscribe(event->subscribe.conn_handle,
                                        event->subscribe.cur_notify != 0);
      }
      return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
      ESP_LOGI(TAG, "Encryption change: status=%d",
               event->enc_change.status);
      return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
      // Hood is re-pairing -- drop the old bond and accept the new one.
      {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0)
          ble_store_util_delete_peer(&desc.peer_id_addr);
      }
      return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
      return 0;
  }
}

static void berbel_on_reset(int reason) {
  ESP_LOGW(TAG, "NimBLE host reset, reason=%d", reason);
}

static void berbel_on_sync() {
  // Once host & controller are in sync, configure address and advertise.
  ble_hs_util_ensure_addr(0);
  if (g_instance != nullptr) {
    g_instance->start_advertising();
  }
}

static void berbel_host_task(void *) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

// -- BerbelRemote member functions ---------------------------------------

void BerbelRemote::apply_mac_() {
  // Must be called before WiFi/BT init.
  esp_err_t rc = esp_base_mac_addr_set(mac_);
  if (rc != ESP_OK) {
    ESP_LOGW(TAG, "esp_base_mac_addr_set failed: %d", rc);
  } else {
    ESP_LOGCONFIG(TAG, "Base MAC set to %02X:%02X:%02X:%02X:%02X:%02X",
                  mac_[0], mac_[1], mac_[2], mac_[3], mac_[4], mac_[5]);
  }
}

void BerbelRemote::setup() {
  g_instance = this;
  apply_mac_();
  start_nimble_();
}

void BerbelRemote::start_nimble_() {
  esp_err_t err = nimble_port_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init failed: %d", err);
    this->mark_failed();
    return;
  }

  // Security: Legacy pairing, Just Works, bond, encryption keys only.
  ble_hs_cfg.reset_cb = berbel_on_reset;
  ble_hs_cfg.sync_cb = berbel_on_sync;
  ble_hs_cfg.gatts_register_cb = nullptr;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 0;
  ble_hs_cfg.sm_sc = 0;
  ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
  ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;

  ble_svc_gap_init();
  ble_svc_gatt_init();

  int rc = ble_gatts_count_cfg(s_services);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
    this->mark_failed();
    return;
  }
  rc = ble_gatts_add_svcs(s_services);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
    this->mark_failed();
    return;
  }
  ble_svc_gap_device_name_set("Berbel");

  nimble_port_freertos_init(berbel_host_task);
  ESP_LOGCONFIG(TAG, "NimBLE host started");
}

void BerbelRemote::start_advertising() {
  // Manually crafted AD payload: Flags + Service Data (128-bit) with the
  // hood's expected UUID and the ACTIVE state byte.
  static const uint8_t adv_data[] = {
      0x02, 0x01, 0x05,  // Flags: LE General Discoverable, BR/EDR not supp.
      0x12, 0x21,        // length=0x12, type=Service Data 128-bit
      0x6c, 0x65, 0x62, 0x72, 0x65, 0x62, 0x43, 0x80,
      0x53, 0x40, 0x45, 0x57, 0x00, 0xf0, 0x04, 0xf0,
      0x01               // ACTIVE
  };

  int rc = ble_gap_adv_set_data(adv_data, sizeof(adv_data));
  if (rc != 0) {
    ESP_LOGW(TAG, "adv_set_data failed: %d", rc);
    return;
  }

  struct ble_gap_adv_params advp;
  std::memset(&advp, 0, sizeof(advp));
  advp.conn_mode = BLE_GAP_CONN_MODE_UND;
  advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
  advp.itvl_min = 0x20;
  advp.itvl_max = 0x40;

  rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER, &advp,
                         berbel_gap_event, nullptr);
  if (rc != 0 && rc != BLE_HS_EALREADY) {
    ESP_LOGW(TAG, "adv_start failed: %d", rc);
  } else {
    ESP_LOGD(TAG, "Advertising started");
  }
}

void BerbelRemote::on_gap_connect(uint16_t conn_handle) {
  conn_handle_ = conn_handle;
  button_subscribed_ = false;
  ESP_LOGI(TAG, "Hood connected (conn_handle=%u)", conn_handle);
  on_connect_.call();
}

void BerbelRemote::on_gap_disconnect() {
  conn_handle_ = 0xFFFF;
  button_subscribed_ = false;
  tx_queue_.clear();
  waiting_release_ = false;
  on_disconnect_.call();
  start_advertising();
}

void BerbelRemote::on_button_subscribe(uint16_t, bool notify) {
  button_subscribed_ = notify;
  ESP_LOGD(TAG, "Button notify subscription: %s", notify ? "on" : "off");
}

void BerbelRemote::send_button(uint8_t code) {
  if (code == 0 || code > 0x0D) {
    ESP_LOGW(TAG, "Ignoring out-of-range button code 0x%02x", code);
    return;
  }
  if (!is_connected()) {
    ESP_LOGW(TAG, "Hood not connected, dropping button 0x%02x", code);
    return;
  }
  tx_queue_.push_back(code);
}

void BerbelRemote::send_notification_(uint8_t b0, uint8_t b1) {
  if (!is_connected() || button_val_handle_ == 0) return;
  uint8_t payload[2] = {b0, b1};
  struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
  if (om == nullptr) {
    ESP_LOGW(TAG, "mbuf alloc failed");
    return;
  }
  int rc = ble_gattc_notify_custom(conn_handle_, button_val_handle_, om);
  if (rc != 0) ESP_LOGW(TAG, "notify failed: %d", rc);
}

void BerbelRemote::loop() {
  // Pick up dynamic handles from the GATT registration.
  if (button_val_handle_ == 0) button_val_handle_ = s_button_val_handle;
  if (status_val_handle_ == 0) status_val_handle_ = s_status_val_handle;

  const uint32_t now = millis();

  // Send pending release 100 ms after the press.
  if (waiting_release_ && static_cast<int32_t>(now - release_at_ms_) >= 0) {
    send_notification_(0x00, 0x00);
    waiting_release_ = false;
    last_send_ms_ = now;
  }

  // Inter-press throttle of 300 ms to match the original firmware.
  if (!waiting_release_ && !tx_queue_.empty() &&
      (last_send_ms_ == 0 || now - last_send_ms_ >= 300)) {
    uint8_t code = tx_queue_.front();
    tx_queue_.pop_front();
    if (is_connected()) {
      send_notification_(code, 0x00);
      release_at_ms_ = now + 100;
      waiting_release_ = true;
    }
  }
}

void BerbelRemote::decode_status_(const uint8_t *data, size_t len) {
  if (len < 7) return;
  std::memset(status_.raw, 0, sizeof(status_.raw));
  std::memcpy(status_.raw, data, len < 9 ? len : 9);
  status_.fan_1 = (data[0] & 0x10) != 0;
  status_.fan_2 = (data[1] & 0x01) != 0;
  status_.fan_3 = (data[1] & 0x10) != 0;
  status_.fan_p = (data[2] & 0x09) != 0;
  status_.light_up = (data[2] & 0x10) != 0;
  status_.light_down = (data[4] & 0x10) != 0;
  status_.cover_moving_up = (data[4] & 0x01) != 0;
  status_.cover_moving_down = (data[6] & 0x01) != 0;
  status_.afterrun = (data[5] & 0x90) != 0;
}

void BerbelRemote::on_status_write(const uint8_t *data, size_t len) {
  // Hood sends a "sync" packet of all 0x11; ignore it.
  bool all_sync = true;
  for (size_t i = 0; i < len; i++) {
    if (data[i] != 0x11) { all_sync = false; break; }
  }
  if (all_sync) return;

  decode_status_(data, len);
  on_status_.call(status_);
}

void BerbelRemote::dump_config() {
  ESP_LOGCONFIG(TAG, "Berbel Remote:");
  ESP_LOGCONFIG(TAG, "  MAC: %02X:%02X:%02X:%02X:%02X:%02X", mac_[0], mac_[1],
                mac_[2], mac_[3], mac_[4], mac_[5]);
  ESP_LOGCONFIG(TAG, "  Status: %s",
                is_connected() ? "connected" : "advertising / waiting");
}

}  // namespace berbel_remote
}  // namespace esphome

#endif  // USE_ESP32
