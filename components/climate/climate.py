import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, climate, sensor, switch, select
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    UNIT_CELSIUS,
    UNIT_FAHRENHEIT,
    UNIT_PERCENT,
    ICON_THERMOMETER,
    ICON_FAN,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
)

DEPENDENCIES = ["uart", "climate"]
AUTO_LOAD = ["sensor", "switch", "select"]

ac_controller_ns = cg.esphome_ns.namespace("ac_controller")
AcController    = ac_controller_ns.class_("AcController", climate.Climate, uart.UARTDevice, cg.Component)
AcSwingSelect   = ac_controller_ns.class_("AcSwingSelect",  select.Select,  cg.Component)
AcSwitch        = ac_controller_ns.class_("AcSwitch",       switch.Switch,  cg.Component)
AcSleepSelect   = ac_controller_ns.class_("AcSleepSelect",  select.Select,  cg.Component)

AcSwitchType = ac_controller_ns.enum("AcSwitch::SwitchType")
SWITCH_TYPES = {
    "eco":     AcSwitchType.ECO,
    "display": AcSwitchType.DISPLAY,
    "beep":    AcSwitchType.BEEP,
}

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

SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_FAHRENHEIT,
    icon=ICON_THERMOMETER,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

OUTDOOR_COIL_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    icon=ICON_THERMOMETER,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

FREQ_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement="Hz",
    icon=ICON_FAN,
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

RPM_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_PERCENT,
    icon=ICON_FAN,
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

V_SWING_OPTIONS = [
    "Swing Up-Down",
    "Swing Up",
    "Swing Down",
    "Fixed Up",
    "Fixed Above-Up",
    "Fixed Middle",
    "Fixed Above-Down",
    "Fixed Down",
]

H_SWING_OPTIONS = [
    "Swing Left-Right",
    "Swing Left",
    "Swing Center",
    "Swing Right",
    "Fixed Left",
    "Fixed A-bit-Left",
    "Fixed Center",
    "Fixed A-bit-Right",
    "Fixed Right",
]

SLEEP_OPTIONS = ["Off", "Standard", "Aged", "Child"]

CONFIG_SCHEMA = cv.ALL(climate.CLIMATE_SCHEMA).extend({
    cv.GenerateID(): cv.declare_id(AcController),
    cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),

    # Optional sensors
    cv.Optional(CONF_ROOM_TEMP_SENSOR):    SENSOR_SCHEMA,
    cv.Optional(CONF_INDOOR_COIL_SENSOR):  SENSOR_SCHEMA,
    cv.Optional(CONF_OUTDOOR_COIL_SENSOR): OUTDOOR_COIL_SCHEMA,
    cv.Optional(CONF_COMP_FREQ_SENSOR):    FREQ_SENSOR_SCHEMA,
    cv.Optional(CONF_FAN_RPM_SENSOR):      RPM_SENSOR_SCHEMA,

    # Optional selects
    cv.Optional(CONF_V_SWING_SELECT): select.SELECT_SCHEMA.extend({
        cv.GenerateID(): cv.declare_id(AcSwingSelect),
    }),
    cv.Optional(CONF_H_SWING_SELECT): select.SELECT_SCHEMA.extend({
        cv.GenerateID(): cv.declare_id(AcSwingSelect),
    }),
    cv.Optional(CONF_SLEEP_SELECT): select.SELECT_SCHEMA.extend({
        cv.GenerateID(): cv.declare_id(AcSleepSelect),
    }),

    # Optional switches
    cv.Optional(CONF_ECO_SWITCH): switch.SWITCH_SCHEMA.extend({
        cv.GenerateID(): cv.declare_id(AcSwitch),
    }),
    cv.Optional(CONF_DISPLAY_SWITCH): switch.SWITCH_SCHEMA.extend({
        cv.GenerateID(): cv.declare_id(AcSwitch),
    }),
    cv.Optional(CONF_BEEP_SWITCH): switch.SWITCH_SCHEMA.extend({
        cv.GenerateID(): cv.declare_id(AcSwitch),
    }),
}).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await climate.register_climate(var, config)

    # Sensors
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

    # Swing selects
    if CONF_V_SWING_SELECT in config:
        sel = cg.new_Pvariable(config[CONF_V_SWING_SELECT][CONF_ID])
        await cg.register_component(sel, config[CONF_V_SWING_SELECT])
        await select.register_select(sel, config[CONF_V_SWING_SELECT], options=V_SWING_OPTIONS)
        cg.add(sel.set_is_vertical(True))
        cg.add(sel.set_parent(var))
        cg.add(var.set_v_swing_select(sel))

    if CONF_H_SWING_SELECT in config:
        sel = cg.new_Pvariable(config[CONF_H_SWING_SELECT][CONF_ID])
        await cg.register_component(sel, config[CONF_H_SWING_SELECT])
        await select.register_select(sel, config[CONF_H_SWING_SELECT], options=H_SWING_OPTIONS)
        cg.add(sel.set_is_vertical(False))
        cg.add(sel.set_parent(var))
        cg.add(var.set_h_swing_select(sel))

    if CONF_SLEEP_SELECT in config:
        sel = cg.new_Pvariable(config[CONF_SLEEP_SELECT][CONF_ID])
        await cg.register_component(sel, config[CONF_SLEEP_SELECT])
        await select.register_select(sel, config[CONF_SLEEP_SELECT], options=SLEEP_OPTIONS)
        cg.add(sel.set_parent(var))
        cg.add(var.set_sleep_select(sel))

    # Switches
    for conf_key, setter, switch_type in [
        (CONF_ECO_SWITCH,     "set_eco_switch",     "eco"),
        (CONF_DISPLAY_SWITCH, "set_display_switch", "display"),
        (CONF_BEEP_SWITCH,    "set_beep_switch",    "beep"),
    ]:
        if conf_key in config:
            sw = cg.new_Pvariable(config[conf_key][CONF_ID])
            await cg.register_component(sw, config[conf_key])
            await switch.register_switch(sw, config[conf_key])
            cg.add(sw.set_type(SWITCH_TYPES[switch_type]))
            cg.add(sw.set_parent(var))
            cg.add(getattr(var, setter)(sw))
