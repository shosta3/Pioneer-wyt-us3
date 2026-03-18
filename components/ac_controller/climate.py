import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, climate, sensor, switch, select
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    CONF_NAME,
)

DEPENDENCIES = ["uart", "climate"]
AUTO_LOAD = ["sensor", "switch", "select"]


ac_ns = cg.esphome_ns.namespace("ac_controller")

AcController  = ac_ns.class_("AcController",  climate.Climate, uart.UARTDevice, cg.Component)
AcSwingSelect = ac_ns.class_("AcSwingSelect",  select.Select,  cg.Component)
AcSwitch      = ac_ns.class_("AcSwitch",       switch.Switch,  cg.Component)
AcSleepSelect = ac_ns.class_("AcSleepSelect",  select.Select,  cg.Component)

AcSwitchType = ac_ns.class_("AcSwitch").enum("SwitchKind")
SWITCH_TYPES = {
    "eco":     AcSwitchType.KIND_ECO,
    "display": AcSwitchType.KIND_DISPLAY,
    "beep":    AcSwitchType.KIND_BEEP,
}

# ── Config keys ───────────────────────────────────────────────────────────────
CONF_ROOM_TEMP_SENSOR    = "room_temperature"
CONF_INDOOR_COIL_SENSOR  = "indoor_coil_temperature"
CONF_OUTDOOR_COIL_SENSOR = "outdoor_coil_temperature"
CONF_COMP_FREQ_SENSOR    = "compressor_frequency"
CONF_FAN_RPM_SENSOR      = "fan_rpm_percent"
CONF_V_SWING_SELECT      = "v_swing"
CONF_H_SWING_SELECT      = "h_swing"
CONF_ECO_SWITCH          = "eco"
CONF_DISPLAY_SWITCH      = "display"
CONF_BEEP_SWITCH         = "beep"
CONF_SLEEP_SELECT        = "sleep_mode"

# ── Swing / sleep option lists ────────────────────────────────────────────────
V_SWING_OPTIONS = [
    "Swing Up-Down", "Swing Up", "Swing Down",
    "Fixed Up", "Fixed Above-Up", "Fixed Middle", "Fixed Above-Down", "Fixed Down",
]

H_SWING_OPTIONS = [
    "Swing Left-Right", "Swing Left", "Swing Center", "Swing Right",
    "Fixed Left", "Fixed A-bit-Left", "Fixed Center", "Fixed A-bit-Right", "Fixed Right",
]

SLEEP_OPTIONS = ["Off", "Standard", "Aged", "Child"]

# ── Sensor schemas ────────────────────────────────────────────────────────────
def _temp_f_sensor():
    return sensor.sensor_schema(
        unit_of_measurement="°F",
        icon="mdi:thermometer",
        accuracy_decimals=1,
        device_class="temperature",
        state_class="measurement",
    )

def _temp_c_sensor():
    return sensor.sensor_schema(
        unit_of_measurement="°C",
        icon="mdi:thermometer",
        accuracy_decimals=1,
        device_class="temperature",
        state_class="measurement",
    )

def _freq_sensor():
    return sensor.sensor_schema(
        unit_of_measurement="Hz",
        icon="mdi:fan",
        accuracy_decimals=0,
        state_class="measurement",
    )

def _pct_sensor():
    return sensor.sensor_schema(
        unit_of_measurement="%",
        icon="mdi:fan",
        accuracy_decimals=0,
        state_class="measurement",
    )

# ── Sub-entity schemas ────────────────────────────────────────────────────────
# Use basic cv.Schema with just name + ID rather than SELECT_SCHEMA/SWITCH_SCHEMA
# which have changed between ESPHome versions.
def _named_entity_schema(class_):
    return cv.Schema({
        cv.GenerateID(): cv.declare_id(class_),
        cv.Required(CONF_NAME): cv.string,
    })

# ── CONFIG_SCHEMA ─────────────────────────────────────────────────────────────
# climate.climate_schema(Class) is the correct modern API (replaces CLIMATE_SCHEMA)
CONFIG_SCHEMA = cv.All(
    climate.climate_schema(AcController)
    .extend({
        cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),

        # Optional sensors
        cv.Optional(CONF_ROOM_TEMP_SENSOR):    _temp_f_sensor(),
        cv.Optional(CONF_INDOOR_COIL_SENSOR):  _temp_f_sensor(),
        cv.Optional(CONF_OUTDOOR_COIL_SENSOR): _temp_c_sensor(),
        cv.Optional(CONF_COMP_FREQ_SENSOR):    _freq_sensor(),
        cv.Optional(CONF_FAN_RPM_SENSOR):      _pct_sensor(),

        # Optional swing selects
        cv.Optional(CONF_V_SWING_SELECT): _named_entity_schema(AcSwingSelect),
        cv.Optional(CONF_H_SWING_SELECT): _named_entity_schema(AcSwingSelect),

        # Optional sleep select
        cv.Optional(CONF_SLEEP_SELECT): _named_entity_schema(AcSleepSelect),

        # Optional switches
        cv.Optional(CONF_ECO_SWITCH):     _named_entity_schema(AcSwitch),
        cv.Optional(CONF_DISPLAY_SWITCH): _named_entity_schema(AcSwitch),
        cv.Optional(CONF_BEEP_SWITCH):    _named_entity_schema(AcSwitch),
    })
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await climate.register_climate(var, config)

    # ── Sensors ───────────────────────────────────────────────────────────────
    if CONF_ROOM_TEMP_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_ROOM_TEMP_SENSOR])
        cg.add(var.set_room_temp_sensor(sens))

    if CONF_INDOOR_COIL_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_INDOOR_COIL_SENSOR])
        cg.add(var.set_indoor_coil_sensor(sens))

    if CONF_OUTDOOR_COIL_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_COIL_SENSOR])
        cg.add(var.set_outdoor_coil_sensor(sens))

    if CONF_COMP_FREQ_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_COMP_FREQ_SENSOR])
        cg.add(var.set_comp_freq_sensor(sens))

    if CONF_FAN_RPM_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_FAN_RPM_SENSOR])
        cg.add(var.set_fan_rpm_sensor(sens))

    # ── Vertical swing select ─────────────────────────────────────────────────
    if CONF_V_SWING_SELECT in config:
        conf = config[CONF_V_SWING_SELECT]
        sel = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(sel, conf)
        cg.add(sel.set_name(conf[CONF_NAME]))
        cg.add(sel.traits.set_options(V_SWING_OPTIONS))
        cg.add(sel.set_is_vertical(True))
        cg.add(sel.set_parent(var))
        cg.add(var.set_v_swing_select(sel))

    # ── Horizontal swing select ───────────────────────────────────────────────
    if CONF_H_SWING_SELECT in config:
        conf = config[CONF_H_SWING_SELECT]
        sel = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(sel, conf)
        cg.add(sel.set_name(conf[CONF_NAME]))
        cg.add(sel.traits.set_options(H_SWING_OPTIONS))
        cg.add(sel.set_is_vertical(False))
        cg.add(sel.set_parent(var))
        cg.add(var.set_h_swing_select(sel))

    # ── Sleep mode select ─────────────────────────────────────────────────────
    if CONF_SLEEP_SELECT in config:
        conf = config[CONF_SLEEP_SELECT]
        sel = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(sel, conf)
        cg.add(sel.set_name(conf[CONF_NAME]))
        cg.add(sel.traits.set_options(SLEEP_OPTIONS))
        cg.add(sel.set_parent(var))
        cg.add(var.set_sleep_select(sel))

    # ── Switches ──────────────────────────────────────────────────────────────
    for conf_key, setter, sw_type in [
        (CONF_ECO_SWITCH,     "set_eco_switch",     "eco"),
        (CONF_DISPLAY_SWITCH, "set_display_switch", "display"),
        (CONF_BEEP_SWITCH,    "set_beep_switch",    "beep"),
    ]:
        if conf_key in config:
            conf = config[conf_key]
            sw = cg.new_Pvariable(conf[CONF_ID])
            await cg.register_component(sw, conf)
            cg.add(sw.set_name(conf[CONF_NAME]))
            cg.add(sw.set_kind(SWITCH_TYPES[sw_type]))
            cg.add(sw.set_parent(var))
            cg.add(getattr(var, setter)(sw))
