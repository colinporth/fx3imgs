// sensorMT9D.c - I2C control for mt9d111 & mt9d112
//{{{  includes
#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3error.h>
#include <cyu3uart.h>
#include <cyu3i2c.h>
#include <cyu3types.h>
#include <cyu3utils.h>

#include "sensor.h"
#include "display.h"
//}}}
#define SENSOR_ADDR_WR111 0x90
#define SENSOR_ADDR_RD111 0x91
#define SENSOR_ADDR_WR112 0x78
#define SENSOR_ADDR_RD112 0x79

static int mt9d111 = 1;
//{{{
static void I2Cinit (int speed) {

  CyU3PI2cInit();

  CyU3PI2cConfig_t i2cConfig;
  i2cConfig.bitRate = speed;
  i2cConfig.isDma = CyFalse;
  i2cConfig.busTimeout = 0xffffffffU;
  i2cConfig.dmaTimeout = 0xffff;
  CyU3PI2cSetConfig (&i2cConfig, 0);
  }
//}}}

//{{{
static uint16_t readReg111  (uint16_t address) {
// 1 byte address

  CyU3PI2cPreamble_t preamble;
  preamble.buffer[0] = SENSOR_ADDR_WR111;
  preamble.buffer[1] = address & 0xFF;
  preamble.buffer[2] = SENSOR_ADDR_RD111;
  preamble.length = 3;
  preamble.ctrlMask = 0x0002;  // Send start bit after second byte of preamble, 2nd bit mask

  uint8_t buf[2];
  buf[0] = 0;
  buf[1] = 0;
  if (CyU3PI2cReceiveBytes (&preamble, buf, 2, 0) == CY_U3P_SUCCESS)
    CyU3PBusyWait (10);

  return (buf[0] << 8) | buf[1];
  }
//}}}
//{{{
static void writeReg111 (uint16_t address, uint16_t value) {
// 1 byte address

  CyU3PI2cPreamble_t preamble;
  preamble.buffer[0] = SENSOR_ADDR_WR111;
  preamble.buffer[1] = address & 0xFF;
  preamble.length = 2;         // 2 byte preamble
  preamble.ctrlMask = 0x0000;  // No additional start and stop bits

  uint8_t buf[2];
  buf[0] = value >> 8;
  buf[1] = value & 0xFF;

  if (CyU3PI2cTransmitBytes (&preamble, buf, 2, 0) == CY_U3P_SUCCESS)
    CyU3PBusyWait (10);
  }
//}}}

//{{{
static uint16_t readReg112 (uint16_t address) {
// 2 byte address

  CyU3PI2cPreamble_t preamble;
  preamble.buffer[0] = SENSOR_ADDR_WR112;
  preamble.buffer[1] = address >> 8;
  preamble.buffer[2] = address & 0xFF;
  preamble.buffer[3] = SENSOR_ADDR_RD112;
  preamble.length = 4;
  preamble.ctrlMask = 0x0004;  // Send start bit after third byte of preamble, 3rd bit mask

  uint8_t buf[2];
  buf[0] = 0;
  buf[1] = 0;
  if (CyU3PI2cReceiveBytes (&preamble, buf, 2, 0) == CY_U3P_SUCCESS)
    CyU3PBusyWait (10);

  return (buf[0] << 8) | buf[1];
  }
//}}}
//{{{
static void writeReg112 (uint16_t address, uint16_t value) {
// 2 byte address

  CyU3PI2cPreamble_t preamble;
  preamble.buffer[0] = SENSOR_ADDR_WR112;
  preamble.buffer[1] = address >> 8;
  preamble.buffer[2] = address & 0xFF;
  preamble.length = 3;         //  Three byte preamble. */
  preamble.ctrlMask = 0x0000;  //  No additional start and stop bits. */

  uint8_t buf[2];
  buf[0] = value >> 8;
  buf[1] = value & 0xFF;

  if (CyU3PI2cTransmitBytes (&preamble, buf, 2, 0) == CY_U3P_SUCCESS)
    CyU3PBusyWait (10);
  }
//}}}
//
//{{{
void I2C_Write (uint8_t hiAddr, uint8_t loAddr, uint8_t hiData, uint8_t loData) {

  if (mt9d111)
    writeReg111 (loAddr, (hiData << 8) | loData);
  else
    writeReg112 ((hiAddr << 8) | loAddr, (hiData << 8) | loData);
  }
//}}}
//{{{
void I2C_Read (uint8_t hiAddr, uint8_t loAddr, uint8_t *buf) {

  uint16_t value = mt9d111 ? readReg111 (loAddr) : readReg112 ((hiAddr << 8) | loAddr);
  buf[0] = value >> 8;
  buf[1] = value & 0xFF;
  }
//}}}

//{{{
uint8_t sensorGetBrightness() {
  if (mt9d111)
    return 0x34;
  else
    return 0x34;
  }
//}}}
//{{{
void sensorSetBrightness (uint8_t brightness) {
  if (mt9d111)
    CyU3PDebugPrint (4, "SensorSetBrightness %d\r\n", brightness);
  else
    CyU3PDebugPrint (4, "SensorSetBrightness %d\r\n", brightness);
  }
//}}}

//{{{
void sensorScaling (int lines) {

  if (mt9d111) {
    if (lines == 1200) {
      line2 ("1600x1200x9");
      writeReg111 (0xC6, 0xA120); writeReg111 (0xC8, 0x02); // Sequencer.params.mode - capture video
      writeReg111 (0xC6, 0xA103); writeReg111 (0xC8, 0x02); // Sequencer goto capture B  - 1600x1200
      }
    else {
      line2 ("800x600x18");
      writeReg111 (0xC6, 0xA120); writeReg111 (0xC8, 0x00); // Sequencer.params.mode - none
      writeReg111 (0xC6, 0xA103); writeReg111 (0xC8, 0x01); // Sequencer goto preview A - 800x600
      }
    }

  else {
    if (lines == 1200) {
      line2 ("1600x1200x15");
      writeReg112 (0x338C, 0xA120); writeReg112 (0x3390, 0x0002); // sequencer.params.mode - capture video
      writeReg112 (0x338C, 0xA103); writeReg112 (0x3390, 0x0002); // sequencer.cmd - goto capture mode B
      }
    else{
      line2 ("640x480x30");
      writeReg112 (0x338C, 0xA120); writeReg112 (0x3390, 0x0000); // sequencer.params.mode - none
      writeReg112 (0x338C, 0xA103); writeReg112 (0x3390, 0x0001); // sequencer.cmd - goto preview mode A
      }
    }

  CyU3PThreadSleep (100);
  }
//}}}
//{{{
void sensorButton (int value) {

  if (mt9d111) {
    writeReg111 (0xF0, 1); // page 1
    if (value) {
      line2 ("cam AE 6500");
      // sequencer.mode - 5:0 - 6500K:AF:histogram:AWB:flicker:AE
      writeReg111 (0xC6, 0xA102); writeReg111 (0xC8, 0x21);
      }
    else  {
      line2 ("cam none");
      writeReg111 (0xC6, 0xA102); writeReg111 (0xC8, 0);
      }

    CyU3PThreadSleep (20);
    }

  else {
    if (value) {
      line2 ("cam AE 6500");
      // sequencer.mode - 5:0 - 6500K:AF:histogram:AWB:flicker:AE
      writeReg112 (0x338C, 0xA102); writeReg112 (0x3390, 0x21);
      }
    else  {
      line2 ("cam none");
      writeReg112 (0x338C, 0xA102); writeReg112 (0x3390, 0);
      }
    }
  }
//}}}
//{{{
void sensorFocus (int value) {

  if (mt9d111) {
    writeReg111 (0xF0, 1);

    if (value <= 1) {
      writeReg111 (0xC6, 0x9071); writeReg111 (0xC8, 0x00); // SFR GPIO data b1:0 = 0 - disable GPIO1
      writeReg111 (0xC6, 0x9081); writeReg111 (0xC8, 255);  // SFR GPIO wg_t00 = 255 initial off
      writeReg111 (0xC6, 0x9083); writeReg111 (0xC8, 0);    // SFR GPIO wg_t10 = 0 no on
      }

    else {
      if (value > 254)
        value = 254;

      writeReg111 (0xF0, 1);
      writeReg111 (0xC6, 0x9071); writeReg111 (0xC8, 0x02);        // SFR GPIO data b1:0 = enable GPIO1
      writeReg111 (0xC6, 0x9081); writeReg111 (0xC8, 255 - value); // SFR GPIO wg_t00 pwm off
      writeReg111 (0xC6, 0x9083); writeReg111 (0xC8, value);       // SFR GPIO wg_t10 pwm on
      }
    }
  }
//}}}

//{{{
void sensorInit111() {

  //  soft reset
  writeReg111 (0x65, 0xA000); // Bypass the PLL, R0x65:0 = 0xA000,
  writeReg111 (0xF0, 1); // page 1
  writeReg111 (0xC3, 0x0501); // Perform MCU reset by setting R0xC3:1 = 0x0501.
  writeReg111 (0xF0, 0); // page 0
  writeReg111 (0x0D, 0x0021); // Enable soft reset by setting R0x0D:0 = 0x0021. Bit 0 is used for the sensor core reset
  writeReg111 (0x0D, 0x0000); // Disable soft reset by setting R0x0D:0 = 0x0000.
  CyU3PThreadSleep (100);

  writeReg111 (0x05, 0x0204); // HBLANK B = 516
  writeReg111 (0x06, 0x0014); // VBLANK B = 20
  writeReg111 (0x07, 0x00FE); // HBLANK A = 254
  writeReg111 (0x08, 0x000C); // VBLANK A = 12
  writeReg111 (0x20, 0x0300); // Read Mode B = 9:showBorder 8:overSized
  writeReg111 (0x21, 0x8400); // Read Mode A = 15:binning 10:bothADC

  //  PLL
  writeReg111 (0x66, 0x1001);  // PLL Control 1    M:15:8,N:7:0 - M=16, N=1  (24mhz/(N+1))*M / 2*(P+1) = 48mhz
  writeReg111 (0x67, 0x0501);  // PLL Control 2 0x05:15:8,P:7:0 - P=1
  //writeReg111 (0x66, 0x8011);  // PLL Control 1    M:15:8,N:7:0 - M=79, N=2 (24mhz/(N+1))*M / 2*(P+1) = 80mhz
  //writeReg111 (0x67, 0x0500);  // PLL Control 2 0x05:15:8,P:7:0 - P=1
  writeReg111 (0x65, 0xA000);  // Clock CNTRL - PLL ON
  writeReg111 (0x65, 0x2000);  // Clock CNTRL - USE PLL
  CyU3PThreadSleep (100);

  // page 1
  writeReg111 (0xF0, 1);
  writeReg111 (0x97, 0x0002); // output format configuration luma:chroma swap

  //{{{  sequencer
  writeReg111 (0xC6, 0xA122); writeReg111 (0xC8, 0x01); // Enter Preview: Auto Exposure = 1
  writeReg111 (0xC6, 0xA123); writeReg111 (0xC8, 0x00); // Enter Preview: Flicker Detection = 0
  writeReg111 (0xC6, 0xA124); writeReg111 (0xC8, 0x01); // Enter Preview: Auto White Balance = 1
  writeReg111 (0xC6, 0xA125); writeReg111 (0xC8, 0x00); // Enter Preview: Auto Focus = 0
  writeReg111 (0xC6, 0xA126); writeReg111 (0xC8, 0x01); // Enter Preview: Histogram = 1
  writeReg111 (0xC6, 0xA127); writeReg111 (0xC8, 0x00); // Enter Preview: Strobe Control  = 0
  writeReg111 (0xC6, 0xA128); writeReg111 (0xC8, 0x00); // Enter Preview: Skip Control = 0

  writeReg111 (0xC6, 0xA129); writeReg111 (0xC8, 0x03); // In Preview: Auto Exposure = 3
  writeReg111 (0xC6, 0xA12A); writeReg111 (0xC8, 0x02); // In Preview: Flicker Detection = 2
  writeReg111 (0xC6, 0xA12B); writeReg111 (0xC8, 0x03); // In Preview: Auto White Balance = 3
  writeReg111 (0xC6, 0xA12C); writeReg111 (0xC8, 0x00); // In Preview: Auto Focus = 0
  writeReg111 (0xC6, 0xA12D); writeReg111 (0xC8, 0x03); // In Preview: Histogram  = 3
  writeReg111 (0xC6, 0xA12E); writeReg111 (0xC8, 0x00); // In Preview: Strobe Control = 0
  writeReg111 (0xC6, 0xA12F); writeReg111 (0xC8, 0x00); // In Preview: Skip Control = 0

  writeReg111 (0xC6, 0xA130); writeReg111 (0xC8, 0x04); // Exit Preview: Auto Exposure = 4
  writeReg111 (0xC6, 0xA131); writeReg111 (0xC8, 0x00); // Exit Preview: Flicker Detection = 0
  writeReg111 (0xC6, 0xA132); writeReg111 (0xC8, 0x01); // Exit Preview: Auto White Balance = 1
  writeReg111 (0xC6, 0xA133); writeReg111 (0xC8, 0x00); // Exit Preview: Auto Focus = 0
  writeReg111 (0xC6, 0xA134); writeReg111 (0xC8, 0x01); // Exit Preview: Histogram = 1
  writeReg111 (0xC6, 0xA135); writeReg111 (0xC8, 0x00); // Exit Preview: Strobe Control = 0
  writeReg111 (0xC6, 0xA136); writeReg111 (0xC8, 0x00); // Exit Preview: Skip Control = 0

  writeReg111 (0xC6, 0xA137); writeReg111 (0xC8, 0x00); // Capture: Auto Exposure = 0
  writeReg111 (0xC6, 0xA138); writeReg111 (0xC8, 0x00); // Capture: Flicker Detection = 0
  writeReg111 (0xC6, 0xA139); writeReg111 (0xC8, 0x00); // Capture: Auto White Balance  = 0
  writeReg111 (0xC6, 0xA13A); writeReg111 (0xC8, 0x00); // Capture: Auto Focus = 0
  writeReg111 (0xC6, 0xA13B); writeReg111 (0xC8, 0x00); // Capture: Histogram = 0
  writeReg111 (0xC6, 0xA13C); writeReg111 (0xC8, 0x00); // Capture: Strobe Control = 0
  writeReg111 (0xC6, 0xA13D); writeReg111 (0xC8, 0x00); // Capture: Skip Control = 0
  //}}}
  //{{{  mode a,b params
  writeReg111 (0xC6, 0x270B); writeReg111 (0xC8, 0x0030); // mode_config = disable jpeg A,B

  //{{{  [MODE A PARAMETERS]
  /*
  ; Max Frame Time: 33.3333 msec
  ; Max Frame Clocks: 1316666.6 clocks (39.500 MHz)
  ; No. of ADCs: 1
  ; Skip Mode: 1x cols, 1x rows, Bin Mode: Yes
  ; Active Sensor Columns: 808 pixels / 1616 clocks
  ; Active Sensor Rows: 608 rows
  ; Horiz Blanking: 254 pixels / 508 clocks
  ; Vert Blanking: 11 rows
  ; Extra Delay: 955 clocks
  ;
  ; Actual Frame Clocks: 1316666 clocks
  ; Row Time: 53.772 usec / 2124 clocks
  ; Frame time: 33.333316 msec
  ; Frames per Sec: 30 fps
  ;
  ; Max Shutter Delay: 402
  ; 50Hz Flicker Period: 185.97 lines
  ; 60Hz Flicker Period: 154.97 lines
  */
  //}}}
  writeReg111 (0xC6, 0x2703); writeReg111 (0xC8, 0x0320); // Output Width A  = 800
  writeReg111 (0xC6, 0x2705); writeReg111 (0xC8, 0x0258); // Output Height A = 600
  writeReg111 (0xC6, 0x270F); writeReg111 (0xC8, 0x001C); // Row Start A = 28
  writeReg111 (0xC6, 0x2711); writeReg111 (0xC8, 0x003C); // Column Start A = 60
  writeReg111 (0xC6, 0x2713); writeReg111 (0xC8, 0x04B0); // Row Height A = 1200
  writeReg111 (0xC6, 0x2715); writeReg111 (0xC8, 0x0640); // Column Width A = 1600
  writeReg111 (0xC6, 0x2717); writeReg111 (0xC8, 0x0384); // Extra Delay A = 900
  writeReg111 (0xC6, 0x2719); writeReg111 (0xC8, 0x0011); // Row Speed A = 17
  writeReg111 (0xC6, 0x2727); writeReg111 (0xC8, 0x0000); // Crop_X0 A = 0
  writeReg111 (0xC6, 0x2729); writeReg111 (0xC8, 0x0320); // Crop_X1 A = 800
  writeReg111 (0xC6, 0x272B); writeReg111 (0xC8, 0x0000); // Crop_Y0 A = 0
  writeReg111 (0xC6, 0x272D); writeReg111 (0xC8, 0x0258); // Crop_Y1 A = 600
  writeReg111 (0xC6, 0xA743); writeReg111 (0xC8, 0x02);   // Gamma and Contrast Settings A
  writeReg111 (0xC6, 0xA77D); writeReg111 (0xC8, 0x02);   // output format config A = 0x02 swap luma:chroma

  //{{{  [MODE B PARAMETERS]
  /*
  ; Max Frame Time: 66.6667 msec
  ; Max Frame Clocks: 2633333.3 clocks (39.500 MHz)
  ; No. of ADCs: 2
  ; Skip Mode: 1x cols, 1x rows, Bin Mode: No
  ; Active Sensor Columns: 1608 pixels
  ; Active Sensor Rows: 1208 rows
  ; Horiz Blanking: 516 pixels
  ; Vert Blanking: 31 rows
  ; Extra Delay: 1697 clocks
  ;
  ; Actual Frame Clocks: 2633333 clocks
  ; Row Time: 53.772 usec / 2124 clocks
  ; Frame time: 66.666658 msec
  ; Frames per Sec: 15 fps
  ;
  ; Max Shutter Delay: 1663
  ; 50Hz Flicker Period: 185.97 lines
  ; 60Hz Flicker Period: 154.97 lines
  */
  //}}}
  writeReg111 (0xC6, 0x2707); writeReg111 (0xC8, 0x0640); // Output Width B  = 1600
  writeReg111 (0xC6, 0x2709); writeReg111 (0xC8, 0x04B0); // Output Height B = 1200
  writeReg111 (0xC6, 0x271B); writeReg111 (0xC8, 0x001C); // Row Start B = 28
  writeReg111 (0xC6, 0x271D); writeReg111 (0xC8, 0x003C); // Column Start B = 60
  writeReg111 (0xC6, 0x271F); writeReg111 (0xC8, 0x04B0); // Row Height B = 1200
  writeReg111 (0xC6, 0x2721); writeReg111 (0xC8, 0x0640); // Column Width B = 1600
  writeReg111 (0xC6, 0x2723); writeReg111 (0xC8, 0x01A7); // Extra Delay B = 423
  writeReg111 (0xC6, 0x2725); writeReg111 (0xC8, 0x0011); // Row Speed B = 17
  writeReg111 (0xC6, 0x2735); writeReg111 (0xC8, 0x0000); // Crop_X0 B = 0
  writeReg111 (0xC6, 0x2737); writeReg111 (0xC8, 0x0640); // Crop_X1 B = 1600
  writeReg111 (0xC6, 0x2739); writeReg111 (0xC8, 0x0000); // Crop_Y0 B = 0
  writeReg111 (0xC6, 0x273B); writeReg111 (0xC8, 0x04B0); // Crop_Y1 B = 1200
  writeReg111 (0xC6, 0xA744); writeReg111 (0xC8, 0x02);   // Gamma and Contrast Settings B
  writeReg111 (0xC6, 0xA77E); writeReg111 (0xC8, 0x02);   // output format config B = 0x02 swap luma:chroma
  //}}}
  //{{{  Custom gamma tables...
  writeReg111 (0xC6, 0xA745);    //Gamma Table 0 A
  writeReg111 (0xC8, 0x00);  //      = 0
  writeReg111 (0xC6, 0xA746);    //Gamma Table 1 A
  writeReg111 (0xC8, 0x14);  //      = 20
  writeReg111 (0xC6, 0xA747);    //Gamma Table 2 A
  writeReg111 (0xC8, 0x23);  //      = 35
  writeReg111 (0xC6, 0xA748);    //Gamma Table 3 A
  writeReg111 (0xC8, 0x3A);  //      = 58
  writeReg111 (0xC6, 0xA749);    //Gamma Table 4 A
  writeReg111 (0xC8, 0x5E);  //      = 94
  writeReg111 (0xC6, 0xA74A);    //Gamma Table 5 A
  writeReg111 (0xC8, 0x76);  //      = 118
  writeReg111 (0xC6, 0xA74B);    //Gamma Table 6 A
  writeReg111 (0xC8, 0x88);  //      = 136
  writeReg111 (0xC6, 0xA74C);    //Gamma Table 7 A
  writeReg111 (0xC8, 0x96);  //      = 150
  writeReg111 (0xC6, 0xA74D);    //Gamma Table 8 A
  writeReg111 (0xC8, 0xA3);  //      = 163
  writeReg111 (0xC6, 0xA74E);    //Gamma Table 9 A
  writeReg111 (0xC8, 0xAF);  //      = 175
  writeReg111 (0xC6, 0xA74F);    //Gamma Table 10 A
  writeReg111 (0xC8, 0xBA);  //      = 186
  writeReg111 (0xC6, 0xA750);    //Gamma Table 11 A
  writeReg111 (0xC8, 0xC4);  //      = 196
  writeReg111 (0xC6, 0xA751);    //Gamma Table 12 A
  writeReg111 (0xC8, 0xCE);  //      = 206
  writeReg111 (0xC6, 0xA752);    //Gamma Table 13 A
  writeReg111 (0xC8, 0xD7);  //      = 215
  writeReg111 (0xC6, 0xA753);    //Gamma Table 14 A
  writeReg111 (0xC8, 0xE0);  //      = 224
  writeReg111 (0xC6, 0xA754);    //Gamma Table 15 A
  writeReg111 (0xC8, 0xE8);  //      = 232
  writeReg111 (0xC6, 0xA755);    //Gamma Table 16 A
  writeReg111 (0xC8, 0xF0);  //      = 240
  writeReg111 (0xC6, 0xA756);    //Gamma Table 17 A
  writeReg111 (0xC8, 0xF8);  //      = 248
  writeReg111 (0xC6, 0xA757);    //Gamma Table 18 A
  writeReg111 (0xC8, 0xFF);  //      = 255
  writeReg111 (0xC6, 0xA758);    //Gamma Table 0 B
  writeReg111 (0xC8, 0x00);  //      = 0
  writeReg111 (0xC6, 0xA759);    //Gamma Table 1 B
  writeReg111 (0xC8, 0x14);  //      = 20
  writeReg111 (0xC6, 0xA75A);    //Gamma Table 2 B
  writeReg111 (0xC8, 0x23);  //      = 35
  writeReg111 (0xC6, 0xA75B);    //Gamma Table 3 B
  writeReg111 (0xC8, 0x3A);  //      = 58
  writeReg111 (0xC6, 0xA75C);    //Gamma Table 4 B
  writeReg111 (0xC8, 0x5E);  //      = 94
  writeReg111 (0xC6, 0xA75D);    //Gamma Table 5 B
  writeReg111 (0xC8, 0x76);  //      = 118
  writeReg111 (0xC6, 0xA75E);    //Gamma Table 6 B
  writeReg111 (0xC8, 0x88);  //      = 136
  writeReg111 (0xC6, 0xA75F);    //Gamma Table 7 B
  writeReg111 (0xC8, 0x96);  //      = 150
  writeReg111 (0xC6, 0xA760);    //Gamma Table 8 B
  writeReg111 (0xC8, 0xA3);  //      = 163
  writeReg111 (0xC6, 0xA761);    //Gamma Table 9 B
  writeReg111 (0xC8, 0xAF);  //      = 175
  writeReg111 (0xC6, 0xA762);    //Gamma Table 10 B
  writeReg111 (0xC8, 0xBA);  //      = 186
  writeReg111 (0xC6, 0xA763);    //Gamma Table 11 B
  writeReg111 (0xC8, 0xC4);  //      = 196
  writeReg111 (0xC6, 0xA764);    //Gamma Table 12 B
  writeReg111 (0xC8, 0xCE);  //      = 206
  writeReg111 (0xC6, 0xA765);    //Gamma Table 13 B
  writeReg111 (0xC8, 0xD7);  //      = 215
  writeReg111 (0xC6, 0xA766);    //Gamma Table 14 B
  writeReg111 (0xC8, 0xE0);  //      = 224
  writeReg111 (0xC6, 0xA767);    //Gamma Table 15 B
  writeReg111 (0xC8, 0xE8);  //      = 232
  writeReg111 (0xC6, 0xA768);    //Gamma Table 16 B
  writeReg111 (0xC8, 0xF0);  //      = 240
  writeReg111 (0xC6, 0xA769);    //Gamma Table 17 B
  writeReg111 (0xC8, 0xF8);  //      = 248
  writeReg111 (0xC6, 0xA76A);    //Gamma Table 18 B
  writeReg111 (0xC8, 0xFF);  //      = 255
  //}}}
  //{{{  other config
  writeReg111 (0xC6, 0x276D); writeReg111 (0xC8, 0xE0E2); // FIFO_Conf1 A = 57570
  writeReg111 (0xC6, 0xA76F); writeReg111 (0xC8, 0xE1);   // FIFO_Conf2 A = 225
  writeReg111 (0xC6, 0x2774); writeReg111 (0xC8, 0xE0E1); // FIFO_Conf1 B = 57569
  writeReg111 (0xC6, 0xA776); writeReg111 (0xC8, 0xE1);   // FIFO_Conf2 B = 225

  writeReg111 (0xC6, 0x220B); writeReg111 (0xC8, 0x0192); // Max R12 B (Shutter Delay) = 402
  writeReg111 (0xC6, 0xA217); writeReg111 (0xC8, 0x08);   // IndexTH23 = 8
  writeReg111 (0xC6, 0x2228); writeReg111 (0xC8, 0x020F); // RowTime (msclk per)/4 = 527

  writeReg111 (0xC6, 0x222F); writeReg111 (0xC8, 0x009A); // R9 Step = 94
  writeReg111 (0xC6, 0xA408); writeReg111 (0xC8, 0x24);   // search_f1_50 = 21
  writeReg111 (0xC6, 0xA409); writeReg111 (0xC8, 0x26);   // search_f2_50 = 23
  writeReg111 (0xC6, 0xA40A); writeReg111 (0xC8, 0x1D);   // search_f1_60 = 17
  writeReg111 (0xC6, 0xA40B); writeReg111 (0xC8, 0x1F);   // search_f2_60 = 19
  writeReg111 (0xC6, 0x2411); writeReg111 (0xC8, 0x009A); // R9_Step_60 = 94
  writeReg111 (0xC6, 0x2413); writeReg111 (0xC8, 0x00B9); // R9_Step_50 = 112
  //}}}
  CyU3PThreadSleep (100);

  writeReg111 (0xC6, 0xA103); writeReg111 (0xC8, 0x06); // Sequencer Refresh Mode
  CyU3PThreadSleep (200);

  writeReg111 (0xC6, 0xA103); writeReg111 (0xC8, 0x05); // Sequencer Refresh
  CyU3PThreadSleep (200);

  //{{{  focus init
  writeReg111 (0xC6, 0x90B6); writeReg111 (0xC8, 0x01); // SFR GPIO suspend

  // enable GPIO0,1 as output, initial value 0
  writeReg111 (0xC6, 0x9079); writeReg111 (0xC8, 0xFC); // SFR GPIO data direction
  writeReg111 (0xC6, 0x9071); writeReg111 (0xC8, 0x00); // SFR GPIO data b1:0 = 0 GPIO0,1 initial 0

  // use 8bit counter clkdiv 2^(1+2)=8 -> 48mhz -> 6mhz ->> 23.7khz
  writeReg111 (0xC6, 0x90B0); writeReg111 (0xC8, 0x01); // SFR GPIO wg_config b0 = 1 8bit counter
  writeReg111 (0xC6, 0x90B2); writeReg111 (0xC8, 0x02); // SFR GPIO wg_clkdiv b0 = 2

  // GPIO0
  writeReg111 (0xC6, 0x908B); writeReg111 (0xC8, 0x00); // SFR GPIO wg_n0 = 0 infinite
  writeReg111 (0xC6, 0x9081); writeReg111 (0xC8, 255);  // SFR GPIO wg_t00 = 255 initial off
  writeReg111 (0xC6, 0x9083); writeReg111 (0xC8, 0);    // SFR GPIO wg_t10 = 0 no on

  // GPIO1
  writeReg111 (0xC6, 0x908A); writeReg111 (0xC8, 0x00); // SFR GPIO wg_n1 = 0 infinite
  writeReg111 (0xC6, 0x9080); writeReg111 (0xC8, 0xFF); // SFR GPIO wg_t01 = 255 max initial on
  writeReg111 (0xC6, 0x9082); writeReg111 (0xC8, 0x00); // SFR GPIO wg_t11 = 0 no off

  writeReg111 (0xC6, 0x90B5); writeReg111 (0xC8, 0x00); // SFR GPIO reset
  writeReg111 (0xC6, 0x90B6); writeReg111 (0xC8, 0x00); // SFR GPIO suspend
  //}}}
  sensorScaling (600);
  }
//}}}
//{{{
void sensorInit112() {

  //{{{  MCUBootMode - pulse reset
  writeReg112 (0x3386, 0x2501);
  writeReg112 (0x3386, 0x2500);
  CyU3PThreadSleep (100);
  //}}}
  //{{{  set parallel, standby, slew, PLL control
  // reset_register = parallel enable
  writeReg112 (0x301A, 0x0ACC);

  // standby_control
  writeReg112 (0x3202, 0x0008);
  CyU3PThreadSleep (100);

  // input powerDown 10:8:PIXCLKSlew=0, 7:inputPowerDown=0, 2:0:outputSlew=0 0=slow
  writeReg112 (0x3214, 0x0080);

  writeReg112 (0x341E, 0x8F09); // PLL,clk_in control BYPASS PLL
  writeReg112 (0x341C, 0x0150); // PLL 13:8:n=1, 7:0:m=85 clk = (10mhz/(n+1)) * (m/8) = 50mhz
  //writeReg112 (0x341C, 0x0180); // PLL 13:8:n=1, 7:0:m=128 clk = (10mhz/(n+1)) * (m/8) = 80mhz
  CyU3PThreadSleep (5);

  writeReg112 (0x341E, 0x8F09); // PLL,clk_in control: PLL ON, bypass PLL
  writeReg112 (0x341E, 0x8F08); // PLL,clk_in control: USE PLL
  //}}}

  //{{{  preview A
  // A output width = 640, height = 480
  writeReg112 (0x338C, 0x2703); writeReg112 (0x3390, 640);  // Output Width
  writeReg112 (0x338C, 0x2705); writeReg112 (0x3390, 480);  // Output Height

  // A row,column start,end = 120,160,1101,1461
  writeReg112 (0x338C, 0x270D); writeReg112 (0x3390, 0x0078);
  writeReg112 (0x338C, 0x270F); writeReg112 (0x3390, 0x00A0);
  writeReg112 (0x338C, 0x2711); writeReg112 (0x3390, 0x044d);
  writeReg112 (0x338C, 0x2713); writeReg112 (0x3390, 0x05b5);

  // A sensor_col_delay_A = 175
  writeReg112 (0x338C, 0x2715); writeReg112 (0x3390, 0x00AF);

  // A sensor_row_speed_A = 0x2111
  writeReg112 (0x338C, 0x2717); writeReg112 (0x3390, 0x2111);

  // A read mode = 0x046c default, x, xy binning, x,y odd increment
  writeReg112 (0x338C, 0x2719); writeReg112 (0x3390, 0x046c);

  // A sensor_x
  writeReg112 (0x338C, 0x271B); writeReg112 (0x3390, 0x024F); // sensor_sample_time_pck= 591
  writeReg112 (0x338C, 0x271D); writeReg112 (0x3390, 0x0102); // sensor_fine_correction = 258
  writeReg112 (0x338C, 0x271F); writeReg112 (0x3390, 0x0279); // sensor_fine_IT_min ) = 633
  writeReg112 (0x338C, 0x2721); writeReg112 (0x3390, 0x0155); // sensor_fine_IT_max_margin = 341

  // A frame lines = 480
  writeReg112 (0x338C, 0x2723); writeReg112 (0x3390, 0x01e0);

  // A line length = 768 + 64
  writeReg112 (0x338C, 0x2725); writeReg112 (0x3390, 0x0340);

  // A sensor_dac_id_x
  writeReg112 (0x338C, 0x2727); writeReg112 (0x3390, 0x2020); // sensor_dac_id_4_5 = 8224
  writeReg112 (0x338C, 0x2729); writeReg112 (0x3390, 0x2020); // sensor_dac_id_6_7 = 8224
  writeReg112 (0x338C, 0x272B); writeReg112 (0x3390, 0x1020); // sensor_dac_id_8_9 = 4128
  writeReg112 (0x338C, 0x272D); writeReg112 (0x3390, 0x2007); // sensor_dac_id_10_11 = 8199

  // A crop = 0,0,640,480
  writeReg112 (0x338C, 0x2751); writeReg112 (0x3390, 0x0000); // Crop_X0 = 0
  writeReg112 (0x338C, 0x2753); writeReg112 (0x3390, 0x0280); // Crop_X1 = 640
  writeReg112 (0x338C, 0x2755); writeReg112 (0x3390, 0x0000); // Crop_Y0 = 0
  writeReg112 (0x338C, 0x2757); writeReg112 (0x3390, 0x01E0); // Crop_Y1 = 480

  // A output_format
  writeReg112 (0x338c, 0x2795); writeReg112 (0x3390, 0x0002); // Natural, Swaps chrominance byte, yuv
  //}}}
  //{{{  capture B
  // B output width = 1600, height = 1200
  writeReg112 (0x338C, 0x2707); writeReg112 (0x3390, 0x0640);
  writeReg112 (0x338C, 0x2709); writeReg112 (0x3390, 0x04B0);

  // B row,column start,end = 4,4,1211,1611
  writeReg112 (0x338C, 0x272F); writeReg112 (0x3390, 0x0004); // Row Start (B)= 4
  writeReg112 (0x338C, 0x2731); writeReg112 (0x3390, 0x0004); // Column Start (B)= 4
  writeReg112 (0x338C, 0x2733); writeReg112 (0x3390, 0x04BB); // Row End (B)= 1211
  writeReg112 (0x338C, 0x2735); writeReg112 (0x3390, 0x064B); // Column End (B)= 1611

  // B extra delay = 124
  writeReg112 (0x338C, 0x2737); writeReg112 (0x3390, 0x007C);

  // B row speed  = 8465
  writeReg112 (0x338C, 0x2739); writeReg112 (0x3390, 0x2111);

  // B read mode = 0x0024 default, no binning, normal readout
  writeReg112 (0x338C, 0x273B); writeReg112 (0x3390, 0x0024);

  // B sensor_x
  writeReg112 (0x338C, 0x273D); writeReg112 (0x3390, 0x0120); // sensor_sample_time_pck (B)= 288
  writeReg112 (0x338C, 0x2741); writeReg112 (0x3390, 0x0169); // sensor_fine_IT_min (B)= 361

  // B frame lines = 1232
  writeReg112 (0x338C, 0x2745); writeReg112 (0x3390, 0x04D0);

  // B line length = 2284
  writeReg112 (0x338C, 0x2747); writeReg112 (0x3390, 0x08ec);

  // B crop = 0,0,1600,1200
  writeReg112 (0x338C, 0x275F); writeReg112 (0x3390, 0x0000); // Crop_X0 (B)= 0
  writeReg112 (0x338C, 0x2761); writeReg112 (0x3390, 0x0640); // Crop_X1 (B)= 1600
  writeReg112 (0x338C, 0x2763); writeReg112 (0x3390, 0x0000); // Crop_Y0 (B)= 0
  writeReg112 (0x338C, 0x2765); writeReg112 (0x3390, 0x04B0); // Crop_Y1 (B)= 1200

  // B output format
  writeReg112 (0x338c, 0x2797); writeReg112 (0x3390, 0x0002); // B - Natural, Swaps chrominance byte, yuv
  //}}}

  //{{{  AE
  writeReg112 (0x338C, 0xA215); writeReg112 (0x3390, 0x0006); // AE_maxADC
  writeReg112 (0x338C, 0xA206); writeReg112 (0x3390, 0x0036); // AE_TARGET brightness
  writeReg112 (0x338C, 0xA207); writeReg112 (0x3390, 0x0040); // AE_GATE sensitivity
  writeReg112 (0x338C, 0xA20C); writeReg112 (0x3390, 0x0008); // AE_maxIndex max zone num
  //}}}
  //{{{  black level, gain
  writeReg112 (0x3278, 0x0050); // first black level
  writeReg112 (0x327a, 0x0050); // first black level,red
  writeReg112 (0x327c, 0x0050); // green_1
  writeReg112 (0x327e, 0x0050); // green_2
  writeReg112 (0x3280, 0x0050); // blue

  CyU3PThreadSleep (10);
  //}}}
  writeReg112 (0x337e, 0x2000); // Y,RGB offset
  //{{{  AWB
  writeReg112 (0x338C, 0xA34A); writeReg112 (0x3390, 0x0059);     // AWB_GAIN_MIN
  writeReg112 (0x338C, 0xA34B); writeReg112 (0x3390, 0x00A6);     // AWB_GAIN_MAX

  writeReg112 (0x338C, 0x235F); writeReg112 (0x3390, 0x0040);     // AWB_CNT_PXL_TH

  writeReg112 (0x338C, 0xA361); writeReg112 (0x3390, 0x00D2);     // AWB_TG_MIN0
  writeReg112 (0x338C, 0xA362); writeReg112 (0x3390, 0x00E6);     // AWB_TG_MAX0
  writeReg112 (0x338C, 0xA363); writeReg112 (0x3390, 0x0010);     // AWB_X0

  writeReg112 (0x338C, 0xA364); writeReg112 (0x3390, 0x00A0);     // AWB_KR_L
  writeReg112 (0x338C, 0xA365); writeReg112 (0x3390, 0x0096);     // AWB_KG_L
  writeReg112 (0x338C, 0xA366); writeReg112 (0x3390, 0x0080);     // AWB_KB_L
  writeReg112 (0x338C, 0xA367); writeReg112 (0x3390, 0x0080);     // AWB_KR_R
  writeReg112 (0x338C, 0xA368); writeReg112 (0x3390, 0x0080);     // AWB_KG_R
  writeReg112 (0x338C, 0xA369); writeReg112 (0x3390, 0x0080);     // AWB_KB_R

  writeReg112 (0x32A2, 0x3640);     // RESERVED_SOC1_32A2  // fine tune color setting

  writeReg112 (0x338C, 0x2306); writeReg112 (0x3390, 0x02FF);     // AWB_CCM_L_0
  writeReg112 (0x338C, 0x2308); writeReg112 (0x3390, 0xFE6E);     // AWB_CCM_L_1
  writeReg112 (0x338C, 0x230A); writeReg112 (0x3390, 0xFFC2);     // AWB_CCM_L_2
  writeReg112 (0x338C, 0x230C); writeReg112 (0x3390, 0xFF4A);     // AWB_CCM_L_3
  writeReg112 (0x338C, 0x230E); writeReg112 (0x3390, 0x02D7);     // AWB_CCM_L_4
  writeReg112 (0x338C, 0x2310); writeReg112 (0x3390, 0xFF30);     // AWB_CCM_L_5
  writeReg112 (0x338C, 0x2312); writeReg112 (0x3390, 0xFF6E);     // AWB_CCM_L_6
  writeReg112 (0x338C, 0x2314); writeReg112 (0x3390, 0xFDEE);     // AWB_CCM_L_7
  writeReg112 (0x338C, 0x2316); writeReg112 (0x3390, 0x03CF);     // AWB_CCM_L_8
  writeReg112 (0x338C, 0x2318); writeReg112 (0x3390, 0x0020);     // AWB_CCM_L_9
  writeReg112 (0x338C, 0x231A); writeReg112 (0x3390, 0x003C);     // AWB_CCM_L_10

  writeReg112 (0x338C, 0x231C); writeReg112 (0x3390, 0x002C);     // AWB_CCM_RL_0
  writeReg112 (0x338C, 0x231E); writeReg112 (0x3390, 0xFFBC);     // AWB_CCM_RL_1
  writeReg112 (0x338C, 0x2320); writeReg112 (0x3390, 0x0016);     // AWB_CCM_RL_2
  writeReg112 (0x338C, 0x2322); writeReg112 (0x3390, 0x0037);     // AWB_CCM_RL_3
  writeReg112 (0x338C, 0x2324); writeReg112 (0x3390, 0xFFCD);     // AWB_CCM_RL_4
  writeReg112 (0x338C, 0x2326); writeReg112 (0x3390, 0xFFF3);     // AWB_CCM_RL_5
  writeReg112 (0x338C, 0x2328); writeReg112 (0x3390, 0x0077);     // AWB_CCM_RL_6
  writeReg112 (0x338C, 0x232A); writeReg112 (0x3390, 0x00F4);     // AWB_CCM_RL_7
  writeReg112 (0x338C, 0x232C); writeReg112 (0x3390, 0xFE95);     // AWB_CCM_RL_8
  writeReg112 (0x338C, 0x232E); writeReg112 (0x3390, 0x0014);     // AWB_CCM_RL_9
  writeReg112 (0x338C, 0x2330); writeReg112 (0x3390, 0xFFE8);     // AWB_CCM_RL_10  //end

  writeReg112 (0x338C, 0xA348); writeReg112 (0x3390, 0x0008);     // AWB_GAIN_BUFFER_SPEED
  writeReg112 (0x338C, 0xA349); writeReg112 (0x3390, 0x0002);     // AWB_JUMP_DIVISOR
  writeReg112 (0x338C, 0xA34A); writeReg112 (0x3390, 0x0059);     // AWB_GAIN_MIN
  writeReg112 (0x338C, 0xA34B); writeReg112 (0x3390, 0x00A6);     // AWB_GAIN_MAX
  writeReg112 (0x338C, 0xA34F); writeReg112 (0x3390, 0x0000);     // AWB_CCM_POSITION_MIN
  writeReg112 (0x338C, 0xA350); writeReg112 (0x3390, 0x007F);     // AWB_CCM_POSITION_MAX
  writeReg112 (0x338C, 0xA352); writeReg112 (0x3390, 0x001E);     // AWB_SATURATION
  writeReg112 (0x338C, 0xA353); writeReg112 (0x3390, 0x0002);     // AWB_MODE

  writeReg112 (0x338C, 0xA35B); writeReg112 (0x3390, 0x007E);     // AWB_STEADY_BGAIN_OUT_MIN
  writeReg112 (0x338C, 0xA35C); writeReg112 (0x3390, 0x0086);     // AWB_STEADY_BGAIN_OUT_MAX
  writeReg112 (0x338C, 0xA35D); writeReg112 (0x3390, 0x007F);     // AWB_STEADY_BGAIN_IN_MIN
  writeReg112 (0x338C, 0xA35E); writeReg112 (0x3390, 0x0082);     // AWB_STEADY_BGAIN_IN_MAX

  writeReg112 (0x338C, 0xA302); writeReg112 (0x3390, 0x0000);     // AWB_WINDOW_POS
  writeReg112 (0x338C, 0xA303); writeReg112 (0x3390, 0x00EF);     // AWB_WINDOW_SIZE
  writeReg112 (0x338C, 0xAB05); writeReg112 (0x3390, 0x0000);     // HG_PERCENT
  //}}}
  writeReg112 (0x35A4, 0x0596); // BRIGHT_COLOR_KILL_CONTROLS

  //{{{  SEQ_LL
  writeReg112 (0x338C, 0xA118); writeReg112 (0x3390, 0x001E); // SEQ_LLSAT1
  //writeReg112 (0x338c, 0xa118); writeReg112 (0x3390, 0x0026); // sequencer.saturation = 26
  writeReg112 (0x338C, 0xA119); writeReg112 (0x3390, 0x0004); // SEQ_LLSAT2

  writeReg112 (0x338C, 0xA11A); writeReg112 (0x3390, 0x000A); // SEQ_LLINTERPTHRESH1
  writeReg112 (0x338C, 0xA11B); writeReg112 (0x3390, 0x0020); // SEQ_LLINTERPTHRESH2
  //}}}
  //{{{  SEQ_NR
  writeReg112 (0x338C, 0xA13E); writeReg112 (0x3390, 0x0004); // SEQ_NR_TH1_R
  writeReg112 (0x338C, 0xA13F); writeReg112 (0x3390, 0x000E); // SEQ_NR_TH1_G
  writeReg112 (0x338C, 0xA140); writeReg112 (0x3390, 0x0004); // SEQ_NR_TH1_B
  writeReg112 (0x338C, 0xA141); writeReg112 (0x3390, 0x0004); // SEQ_NR_TH1_OL
  writeReg112 (0x338C, 0xA142); writeReg112 (0x3390, 0x0032); // SEQ_NR_TH2_R
  writeReg112 (0x338C, 0xA143); writeReg112 (0x3390, 0x000F); // SEQ_NR_TH2_G
  writeReg112 (0x338C, 0xA144); writeReg112 (0x3390, 0x0032); // SEQ_NR_TH2_B
  writeReg112 (0x338C, 0xA145); writeReg112 (0x3390, 0x0032); // SEQ_NR_TH2_OL

  writeReg112 (0x338C, 0xA146); writeReg112 (0x3390, 0x0005); // SEQ_NR_GAINTH1
  writeReg112 (0x338C, 0xA147); writeReg112 (0x3390, 0x003A); // SEQ_NR_GAINTH2
  //}}}
  //{{{  flicker R9 step
  writeReg112 (0x338C, 0x222E); writeReg112 (0x3390, 0x0090); // R9 Step = 144

  writeReg112 (0x338C, 0xA408); writeReg112 (0x3390, 0x001A); // search_f1_50 = 26
  writeReg112 (0x338C, 0xA409); writeReg112 (0x3390, 0x001D); // search_f2_50 = 29
  writeReg112 (0x338C, 0xA40A); writeReg112 (0x3390, 0x0020); // search_f1_60 = 32
  writeReg112 (0x338C, 0xA40B); writeReg112 (0x3390, 0x0023); // search_f2_60 = 35

  writeReg112 (0x338C, 0xA40D); writeReg112 (0x3390, 0x0002); // Stat_min = 2
  writeReg112 (0x338C, 0xA410); writeReg112 (0x3390, 0x0001); // Min_amplitude = 1

  writeReg112 (0x338C, 0x2411); writeReg112 (0x3390, 0x0090); // R9_Step_60_A = 144
  writeReg112 (0x338C, 0x2413); writeReg112 (0x3390, 0x00AD); // R9_Step_50_A = 173
  writeReg112 (0x338C, 0x2415); writeReg112 (0x3390, 0x0055); // R9_Step_60_B = 85
  writeReg112 (0x338C, 0x2417); writeReg112 (0x3390, 0x0066); // R9_Step_50_B = 102
  //}}}

  writeReg112 (0x338C, 0xA103); writeReg112 (0x3390, 0x0006); // sequencer.cmd = 6 = refresh mode
  CyU3PThreadSleep (100);

  writeReg112 (0x338C, 0xA103); writeReg112 (0x3390, 0x0005); // sequencer.cmd = 5 = refresh
  CyU3PThreadSleep (100);

  writeReg112 (0x33f4, 0x031d); // defect - undocumented
  }
//}}}
//{{{
void sensorInit() {

  I2Cinit (400000);

  //uint16_t value = readReg112 (0x3000);
  //mt9d111 = (value == 0x1519);

  writeReg111 (0xF0, 0);
  uint16_t value = readReg111 (0);
  line3 ("try 111", value);

  mt9d111 = (value == 0x1519);
  //mt9d111 = 1;
  if (mt9d111) {
    line3 ("9d111.48.", value);
    sensorInit111();
    }
  else {
    uint16_t value = readReg112 (0x3000);
    line3 ("try 112", value);
    if (value == 0x1580) {
      line3 ("9d112.50.", value);
      sensorInit112();
      }
    }
  }
//}}}
