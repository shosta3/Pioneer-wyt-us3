#import esphome.config_validation as cv
#COMPONENT_SCHEMA = cv.Schema({})

DEPENDENCIES = ["uart", "climate", "sensor", "switch", "select"]
AUTO_LOAD = ["climate", "sensor", "switch", "select"]
