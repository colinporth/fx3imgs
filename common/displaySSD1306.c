// displaySSD1306.c - spi control
//{{{  includes
#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3types.h>
#include <cyu3gpio.h>
#include <cyu3utils.h>
#include <cyu3spi.h>
#include <cyu3dma.h>

#include "display.h"
#include "font.h"
//}}}
//{{{  ssd1306 command defines
// page mode addressing
#define SSD1306_CMD_COL_ADD_SET_LSB                 0x00
#define SSD1306_CMD_COL_ADD_SET_MSB                 0x10

// Fundamental Command defines
#define SSD1306_CMD_SET_MEMORY_ADDRESSING_MODE      0x20
#define SSD1306_CMD_SET_COLUMN_ADDRESS              0x21
#define SSD1306_CMD_SET_PAGE_ADDRESS                0x22

// Graphic Acceleration Command defines
#define SSD1306_CMD_SCROLL_H_RIGHT                  0x26
#define SSD1306_CMD_SCROLL_H_LEFT                   0x27
#define SSD1306_CMD_CONTINUOUS_SCROLL_V_AND_H_RIGHT 0x29
#define SSD1306_CMD_CONTINUOUS_SCROLL_V_AND_H_LEFT  0x2A
#define SSD1306_CMD_DEACTIVATE_SCROLL               0x2E
#define SSD1306_CMD_ACTIVATE_SCROLL                 0x2F

#define SSD1306_CMD_SET_DISPLAY_START_LINE          0x40

#define SSD1306_CMD_SET_CONTRAST_CONTROL_FOR_BANK0  0x81
#define SSD1306_CMD_SET_CHARGE_PUMP_SETTING         0x8D
#define SSD1306_CMD_SET_SEGMENT_RE_MAP_COL0_SEG0    0xA0
#define SSD1306_CMD_SET_SEGMENT_RE_MAP_COL127_SEG0  0xA1
#define SSD1306_CMD_SET_VERTICAL_SCROLL_AREA        0xA3
#define SSD1306_CMD_ENTIRE_DISPLAY_AND_GDDRAM_ON    0xA4
#define SSD1306_CMD_ENTIRE_DISPLAY_ON               0xA5
#define SSD1306_CMD_SET_NORMAL_DISPLAY              0xA6
#define SSD1306_CMD_SET_INVERSE_DISPLAY             0xA7
#define SSD1306_CMD_SET_MULTIPLEX_RATIO             0xA8
#define SSD1306_CMD_SET_DISPLAY_ON                  0xAF
#define SSD1306_CMD_SET_DISPLAY_OFF                 0xAE

// page mode addressing
#define SSD1306_CMD_SET_PAGE_START_ADDRESS          0xB0

#define SSD1306_CMD_SET_COM_OUTPUT_SCAN_UP          0xC0
#define SSD1306_CMD_SET_COM_OUTPUT_SCAN_DOWN        0xC8

#define SSD1306_CMD_SET_DISPLAY_OFFSET              0xD3
#define SSD1306_CMD_SET_DISPLAY_CLOCK_DIVIDE_RATIO  0xD5
#define SSD1306_CMD_SET_PRE_CHARGE_PERIOD           0xD9
#define SSD1306_CMD_SET_COM_PINS                    0xDA
#define SSD1306_CMD_SET_VCOMH_DESELECT_LEVEL        0xDB

#define SSD1306_CMD_NOP                             0xE3
//}}}
//{{{
static uint8_t ssd1306init[] = {
  SSD1306_CMD_SET_MULTIPLEX_RATIO, 0x3F,
  SSD1306_CMD_SET_DISPLAY_OFFSET, 0x00,
  SSD1306_CMD_SET_DISPLAY_START_LINE,
  SSD1306_CMD_SET_MEMORY_ADDRESSING_MODE, 0x00,
  SSD1306_CMD_SET_SEGMENT_RE_MAP_COL127_SEG0,
  SSD1306_CMD_SET_COM_OUTPUT_SCAN_UP,
  SSD1306_CMD_SET_COM_PINS, 0x12,
  SSD1306_CMD_SET_CONTRAST_CONTROL_FOR_BANK0, 0x5F,
  SSD1306_CMD_ENTIRE_DISPLAY_AND_GDDRAM_ON,
  SSD1306_CMD_SET_NORMAL_DISPLAY,
  SSD1306_CMD_SET_DISPLAY_CLOCK_DIVIDE_RATIO, 0x80,
  SSD1306_CMD_SET_CHARGE_PUMP_SETTING, 0x14,
  SSD1306_CMD_SET_VCOMH_DESELECT_LEVEL, 0x40,
  SSD1306_CMD_SET_PRE_CHARGE_PERIOD, 0xF1,
  SSD1306_CMD_SET_DISPLAY_ON
  };
//}}}

#define SPI_DC_GPIO 23  // CTL 6 pin
#define TFTWIDTH 128
#define TFTHEIGHT 64

static CyU3PDmaChannel spiDmaTxHandle;
static uint8_t frameBuf [TFTWIDTH * TFTHEIGHT / 8]; // 1024 bytes, 128 wide, 8 pages high

//{{{
static void spiInit() {

  // Start the SPI module and configure the master
  CyU3PSpiInit();

  // Start the SPI master block. Run the SPI clock at 8MHz
  // and configure the word length to 8 bits. Also configure
  // the slave select using FW
  CyU3PSpiConfig_t spiConfig;
  CyU3PMemSet ((uint8_t*)&spiConfig, 0, sizeof(spiConfig));
  spiConfig.isLsbFirst = CyFalse;
  spiConfig.cpol       = CyFalse;  // mode0
  spiConfig.cpha       = CyFalse;  // mode0
  spiConfig.ssnPol     = CyFalse;
  spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
  spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
  spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_HW_END_OF_XFER;
  spiConfig.clock      = 32000000;
  spiConfig.wordLen    = 8;
  CyU3PSpiSetConfig (&spiConfig, NULL);

  // Create the DMA channel for SPI write
  CyU3PDmaChannelConfig_t dmaConfig;
  CyU3PMemSet ((uint8_t*)&dmaConfig, 0, sizeof(dmaConfig));
  dmaConfig.size           = 0; //pagelen
  dmaConfig.count          = 0;
  dmaConfig.prodAvailCount = 0;
  dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
  dmaConfig.prodHeader     = 0;
  dmaConfig.prodFooter     = 0;
  dmaConfig.consHeader     = 0;
  dmaConfig.notification   = 0;
  dmaConfig.cb             = NULL;
  dmaConfig.prodSckId = CY_U3P_CPU_SOCKET_PROD;
  dmaConfig.consSckId = CY_U3P_LPP_SOCKET_SPI_CONS;
  CyU3PDmaChannelCreate (&spiDmaTxHandle, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaConfig);
  }
//}}}

//{{{
static void setPixel (int16_t on, int16_t x, int16_t y) {

  uint8_t* framePtr = frameBuf + ((y >> 3) * TFTWIDTH) + (TFTWIDTH - 1) - x;
  uint8_t yMask = 1 << (y & 0x07);
  if (on)
    *framePtr |= yMask;
  else
    *framePtr &= ~yMask;
  }
//}}}
//{{{
static void updateRect (uint8_t startPage, uint8_t endPage, int16_t xorg, uint16_t xend) {

  uint8_t cmd[6];
  cmd[0] = SSD1306_CMD_SET_COLUMN_ADDRESS;
  cmd[1] = (uint8_t)xorg;
  cmd[2] = (uint8_t)xend;
  cmd[3] = SSD1306_CMD_SET_PAGE_ADDRESS;
  cmd[4] = startPage;
  cmd[5] = endPage;
  CyU3PGpioSetValue (SPI_DC_GPIO, CyFalse);
  CyU3PSpiTransmitWords (cmd, 6);

  CyU3PGpioSetValue (SPI_DC_GPIO, CyTrue);
  for (uint16_t page = startPage; page <= endPage; page++)
    CyU3PSpiTransmitWords (frameBuf + (page * TFTWIDTH) + xorg, xend-xorg+1);
  }
//}}}

//{{{
void drawRect (int16_t on, int16_t xorg, int16_t yorg, uint16_t xlen, uint16_t ylen) {

  uint16_t xend = xorg + xlen - 1;
  uint16_t yend = yorg + ylen - 1;

  for (int16_t y = yorg; y <= yend; y++)
    for (int16_t x = xorg; x <= xend; x++)
      setPixel (on, x, y);

  uint8_t startPage = yorg >> 3;
  uint8_t endPage = yend >> 3;

  updateRect (startPage, endPage, xorg, xend);
  }
//}}}
//{{{
void drawString (const char* str, int16_t xorg, int16_t yorg, uint16_t xlen, uint16_t ylen) {

  uint16_t xend = xorg + xlen - 1;
  uint16_t yend = yorg + ylen - 1;

  for (int16_t y = yorg; y <= yend; y++)
    for (int16_t x = xorg; x <= xend; x++)
      setPixel (0, x, y);

  int16_t xChar = xorg;
  font_t* font = &font18;
  do {
    if (*str == ' ')
      xChar += font->spaceWidth;

    else if ((*str >= font->firstChar) && (*str <= font->lastChar)) {
      uint8_t* glyphData = (uint8_t*)(font->glyphsBase + font->glyphOffsets[*str - font->firstChar]);
      uint8_t width = (uint8_t)*glyphData++;
      uint8_t height = (uint8_t)*glyphData++;
      int8_t left = (int8_t)*glyphData++;
      uint8_t top = (uint8_t)*glyphData++;
      uint8_t advance = (uint8_t)*glyphData++;

      for (int16_t y = yorg+font->height-top; y < yorg+font->height-top+height; y++) {
        int16_t xGlyph = xChar + left;
        uint8_t glyphByte = *glyphData;
        for (int16_t i = 0; i < width; i++) {
          if (i % 8 == 0)
            glyphByte = *glyphData++;
          if (glyphByte & 0x80)
            setPixel (1, xGlyph + i, y);
          glyphByte <<= 1;
          }
        }
      xChar += advance;
      }
    } while (*(++str));

  uint8_t startPage = yorg >> 3;
  uint8_t endPage = yend >> 3;

  updateRect (startPage, endPage, xorg, xend);
  }
//}}}
//{{{
void line1 (const char* str) {

  drawString (str, 0, 0, 128, 21);
  }
//}}}
//{{{
void line2 (const char* str) {

  drawString (str, 0, 21, 128, 21);
  }
//}}}
//{{{
void line3 (const char* str, int32_t value) {

  char valueStr[30];

  int i = strlen (str);
  for (int j = 0; j < i; j++)
    valueStr[j] = str[j];

  valueStr[i] = ' ';
  valueStr[i+1] = ((value / 10000) % 10) + 0x30;
  valueStr[i+2] = ((value / 1000) % 10) + 0x30;
  valueStr[i+3] = ((value / 100) % 10) + 0x30;
  valueStr[i+4] = ((value / 10) % 10) + 0x30;
  valueStr[i+5] = (value % 10) + 0x30;
  valueStr[i+6] = 0;

  drawString (valueStr, 0, 42, 128, 21);
  }
//}}}

//{{{
void displayInit (const char* str) {

  spiInit();

  CyU3PDeviceGpioOverride (SPI_DC_GPIO, CyTrue);

  CyU3PGpioSimpleConfig_t gpioConfig;
  gpioConfig.outValue    = CyTrue;
  gpioConfig.driveLowEn  = CyTrue;
  gpioConfig.driveHighEn = CyTrue;
  gpioConfig.inputEn     = CyFalse;
  gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
  CyU3PGpioSetSimpleConfig (SPI_DC_GPIO, &gpioConfig);

  CyU3PGpioSetValue (SPI_DC_GPIO, CyFalse);
  CyU3PSpiTransmitWords (ssd1306init, sizeof(ssd1306init));

  drawRect (0, 0, 0, TFTWIDTH, TFTHEIGHT);
  //CyU3PThreadSleep (250);
  //drawRect (1, 0, 0, TFTWIDTH, TFTHEIGHT);
  //CyU3PThreadSleep (250);

  line1 (str);
  }
//}}}
