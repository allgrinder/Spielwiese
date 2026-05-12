# esphome-berbel-remote

ESPHome external component that emulates a Berbel BFB 6bT kitchen-hood remote
control. It is a port of [tfohlmeister/berbel-remote](https://github.com/tfohlmeister/berbel-remote),
re-implemented as a native ESPHome component so the hood can be controlled
directly from Home Assistant via the ESPHome API – no MQTT broker required.

## How it works

The hood scans for BLE peripherals that

1. advertise its proprietary service UUID
   (`f004f000-5745-4053-8043-62657262656c`) with the *ACTIVE* service-data
   byte (`0x01`), and
2. use a MAC address from one of two Texas Instruments OUIs
   (`88:01:F9` or `30:AF:7E`).

When both conditions are met, the hood initiates a Legacy *Just-Works*
pairing and bonds with the peripheral. Afterwards it pushes 9-byte status
packets to characteristic `f004f001` (write) and accepts 2-byte button
commands as notifications on `f004f002`.

This component sets the ESP32's base MAC accordingly, brings up a NimBLE
peripheral with the required Device-Information / Battery / HID / custom
services, advertises the correct AD payload, decodes the status packet,
and serialises button commands (`[code, 0x00]` press / `[0x00, 0x00]`
release, 100 ms apart, throttled to ~300 ms between presses).

## Repository layout

```
components/
└── berbel_remote/         External component
    ├── __init__.py        Schema + codegen, action registration
    ├── berbel_remote.h    Public class + automations
    └── berbel_remote.cpp  NimBLE peripheral implementation
berbel-hood.yaml           Example device configuration
```

## Quick start

Reference the component straight from this repo in your ESPHome YAML –
no local checkout needed:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/allgrinder/Spielwiese
      ref: claude/esphome-berbel-remote-C0XhC
    components: [berbel_remote]
```

Then:

1. Use the bundled `berbel-hood.yaml` as a starting point. Important
   knobs:
   - `mac_oui`: `ti_1` (`88:01:F9`, default) or `ti_2` (`30:AF:7E`).
   - `mac_suffix`: three hex octets, pick something unique per device.
2. Flash the ESP32 and let it power up *before* the hood, so the hood
   discovers the spoofed remote on its first scan and pairs.
3. Reboot the hood once after the first flash if it was previously
   bonded to a real remote.

## Configuration reference

```yaml
berbel_remote:
  id: berbel
  mac_oui: ti_1                  # ti_1 | ti_2 | "XX:YY:ZZ"
  mac_suffix: "A1:B2:C3"         # required: 3 hex octets

  on_connect:                    # automations
    - logger.log: "..."
  on_disconnect:
    - logger.log: "..."
  on_status:
    - lambda: |-
        // x is `const esphome::berbel_remote::BerbelStatus &`
        // fields: fan_1, fan_2, fan_3, fan_p,
        //         light_up, light_down,
        //         cover_moving_up, cover_moving_down,
        //         afterrun, raw[9]
```

### Action `berbel_remote.send_button`

```yaml
- berbel_remote.send_button:
    id: berbel
    button: power     # power | fan_1 | fan_2 | fan_3 | fan_p |
                      # light_up | light_down | move_up | move_down |
                      # timer | play | reload | record
```

The action enqueues the press; the release frame is emitted automatically
100 ms later. Successive presses are throttled to 300 ms.

## Constraints

- **ESP-IDF framework only.** The component uses ESP-IDF NimBLE host APIs
  directly. Arduino framework on ESP32 also ships NimBLE, but the
  required `sdkconfig` options are managed through `add_idf_sdkconfig_option`,
  which is ESP-IDF specific.
- **Single BLE component.** Do not enable `esp32_ble_tracker`,
  `esp32_ble_server`, or `ble_client` alongside `berbel_remote` – they
  would re-initialise NimBLE in conflicting roles.
- **One hood per ESP32.** NimBLE is configured for a single connection.

## Credit

Original reverse-engineering and reference firmware:
[tfohlmeister/berbel-remote](https://github.com/tfohlmeister/berbel-remote)
(MIT). This port adapts the protocol layer to ESPHome.
