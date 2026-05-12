"""ESPHome external component: Berbel kitchen-hood BLE remote emulator.

Ports the firmware from https://github.com/tfohlmeister/berbel-remote
to ESPHome. The ESP32 acts as a BLE peripheral that the hood connects to
(it spoofs a Texas Instruments MAC OUI because the hood filters by it).
"""

from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import CONF_ID, CONF_ON_CONNECT, CONF_ON_DISCONNECT, CONF_TRIGGER_ID

CODEOWNERS = ["@allgrinder"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = []
MULTI_CONF = False

berbel_ns = cg.esphome_ns.namespace("berbel_remote")
BerbelRemote = berbel_ns.class_("BerbelRemote", cg.Component)
BerbelStatus = berbel_ns.struct("BerbelStatus")

SendButtonAction = berbel_ns.class_("SendButtonAction", automation.Action)
ConnectTrigger = berbel_ns.class_(
    "ConnectTrigger", automation.Trigger.template()
)
DisconnectTrigger = berbel_ns.class_(
    "DisconnectTrigger", automation.Trigger.template()
)
StatusTrigger = berbel_ns.class_(
    "StatusTrigger", automation.Trigger.template(BerbelStatus)
)

CONF_BERBEL_ID = "berbel_id"
CONF_MAC_SUFFIX = "mac_suffix"
CONF_MAC_OUI = "mac_oui"
CONF_ON_STATUS = "on_status"
CONF_BUTTON = "button"
CONF_CODE = "code"

# Texas Instruments OUIs accepted by the hood.
OUI_TI_1 = [0x88, 0x01, 0xF9]
OUI_TI_2 = [0x30, 0xAF, 0x7E]


def _validate_mac_suffix(value):
    value = cv.string_strict(value)
    parts = value.replace("-", ":").split(":")
    if len(parts) != 3:
        raise cv.Invalid("mac_suffix must have 3 hex octets, e.g. AA:BB:CC")
    out = []
    for p in parts:
        try:
            n = int(p, 16)
        except ValueError as e:
            raise cv.Invalid(f"invalid hex octet: {p}") from e
        if not 0 <= n <= 0xFF:
            raise cv.Invalid(f"octet out of range: {p}")
        out.append(n)
    return out


def _validate_oui(value):
    value = cv.string_strict(value).upper()
    if value in ("TI_1", "TI-1", "DEFAULT"):
        return OUI_TI_1
    if value in ("TI_2", "TI-2"):
        return OUI_TI_2
    parts = value.replace("-", ":").split(":")
    if len(parts) != 3:
        raise cv.Invalid("mac_oui must be ti_1, ti_2 or 3 hex octets")
    try:
        return [int(p, 16) for p in parts]
    except ValueError as e:
        raise cv.Invalid("mac_oui: invalid hex") from e


BUTTONS = {
    "power": 0x01,
    "fan_1": 0x02,
    "fan_2": 0x03,
    "fan_3": 0x04,
    "fan_p": 0x05,
    "light_up": 0x06,
    "play": 0x07,
    "reload": 0x08,
    "move_up": 0x09,
    "light_down": 0x0A,
    "record": 0x0B,
    "timer": 0x0C,
    "move_down": 0x0D,
}


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BerbelRemote),
        cv.Optional(CONF_MAC_OUI, default="ti_1"): _validate_oui,
        cv.Optional(CONF_MAC_SUFFIX, default="AA:BB:CC"): _validate_mac_suffix,
        cv.Optional(CONF_ON_CONNECT): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ConnectTrigger)}
        ),
        cv.Optional(CONF_ON_DISCONNECT): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(DisconnectTrigger)}
        ),
        cv.Optional(CONF_ON_STATUS): automation.validate_automation(
            {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(StatusTrigger)}
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    oui = config[CONF_MAC_OUI]
    sfx = config[CONF_MAC_SUFFIX]
    cg.add(var.set_mac(oui[0], oui[1], oui[2], sfx[0], sfx[1], sfx[2]))

    for conf in config.get(CONF_ON_CONNECT, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trig, [], conf)
    for conf in config.get(CONF_ON_DISCONNECT, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trig, [], conf)
    for conf in config.get(CONF_ON_STATUS, []):
        trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trig, [(BerbelStatus.operator("ref").operator("const"), "x")], conf
        )

    # NimBLE host stack: enable through ESP-IDF sdkconfig.
    esp32.add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ENABLED", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_PERIPHERAL", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_BROADCASTER", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SM_LEGACY", True)
    # Keep SC compiled in to satisfy ble_sm_deinit() (it unconditionally
    # calls ble_sm_sc_deinit()). We disable it at runtime via
    # ble_hs_cfg.sm_sc = 0 so pairing stays legacy Just-Works.
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SM_SC", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_NVS_PERSIST", True)
    esp32.add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_MAX_CONNECTIONS", 1)


# -------- berbel_remote.send_button action --------

SEND_BUTTON_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(BerbelRemote),
        cv.Required(CONF_BUTTON): cv.one_of(*BUTTONS.keys(), lower=True),
    }
)


@automation.register_action(
    "berbel_remote.send_button",
    SendButtonAction,
    SEND_BUTTON_SCHEMA,
    synchronous=True,
)
async def send_button_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cg.add(var.set_code(BUTTONS[config[CONF_BUTTON]]))
    return var
