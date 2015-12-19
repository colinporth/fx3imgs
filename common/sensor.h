// sensor.h
#pragma once

#include <cyu3types.h>

extern void I2C_Write (uint8_t hiAddr, uint8_t loAddr, uint8_t hiData, uint8_t loData);
extern void I2C_Read (uint8_t hiAddr, uint8_t loAddr, uint8_t *buf);

extern uint8_t sensorGetBrightness();
extern void sensorSetBrightness (uint8_t brightness);

extern void sensorScaling (int lines);
extern void sensorButton (int value);
extern void sensorFocus (int value);

extern void sensorInit();
