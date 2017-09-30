#pragma once

#include <cyu3types.h>

extern void drawRect (int16_t on, int16_t xorg, int16_t yorg, uint16_t xlen, uint16_t ylen);
extern void drawString (const char* str, int16_t xorg, int16_t yorg, uint16_t xlen, uint16_t ylen);

extern void line1 (const char* str);
extern void line2 (const char* str);
extern void line3 (const char* str, int32_t value);

extern void displayInit (const char* str);
