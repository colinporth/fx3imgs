// sensorMT9D111.c - I2C control
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
//{{{  defines
#define SENSOR_ADDR_WR 0x90
#define SENSOR_ADDR_RD 0x91
//}}}

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
static uint16_t readReg (uint16_t address) {

  CyU3PI2cPreamble_t preamble;
  preamble.buffer[0] = SENSOR_ADDR_WR;
  preamble.buffer[1] = address & 0xFF;
  preamble.buffer[2] = SENSOR_ADDR_RD;
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
static void writeReg (uint16_t address, uint16_t value) {

  CyU3PI2cPreamble_t preamble;
  preamble.buffer[0] = SENSOR_ADDR_WR;
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
void I2C_Write (uint8_t hiAddr, uint8_t loAddr, uint8_t hiData, uint8_t loData) {

  //line3 ("write", loAddr);
  writeReg (loAddr, (hiData << 8) | loData);
  }
//}}}
//{{{
void I2C_Read (uint8_t hiAddr, uint8_t loAddr, uint8_t* buf) {

  //line3 ("read", loAddr);

  uint16_t value = readReg (loAddr);
  buf[0] = value >> 8;
  buf[1] = value & 0xFF;
  }
//}}}

//{{{
uint8_t sensorGetBrightness() {

  return 0x34;
  }
//}}}
//{{{
void sensorSetBrightness (uint8_t brightness) {

  CyU3PDebugPrint (4, "SensorSetBrightness %d\r\n", brightness);
  }
//}}}

//{{{
void sensorScaling (int lines) {

  if (lines == 1200) {
    line2 ("1600x1200x9");
    writeReg (0xC6, 0xA120); writeReg (0xC8, 0x02); // Sequencer.params.mode - capture video
    writeReg (0xC6, 0xA103); writeReg (0xC8, 0x02); // Sequencer goto capture B  - 1600x1200
    }
  else {
    line2 ("800x600x18");
    writeReg (0xC6, 0xA120); writeReg (0xC8, 0x00); // Sequencer.params.mode - none
    writeReg (0xC6, 0xA103); writeReg (0xC8, 0x01); // Sequencer goto preview A - 800x600
    }

  CyU3PThreadSleep (100);
  }
//}}}
//{{{
void sensorButton (int value) {

  writeReg (0xF0, 1); // page 1
  if (value) {
    line2 ("cam AE 6500");
    // sequencer.mode - 5:0 - 6500K:AF:histogram:AWB:flicker:AE
    writeReg (0xC6, 0xA102); writeReg (0xC8, 0x21);
    }
  else  {
    line2 ("cam none");
    writeReg (0xC6, 0xA102); writeReg (0xC8, 0);
    }

  CyU3PThreadSleep (20);
  }
//}}}
//{{{
void sensorFocus (int value) {

  writeReg (0xF0, 1);

  if (value <= 1) {
    writeReg (0xC6, 0x9071); writeReg (0xC8, 0x00); // SFR GPIO data b1:0 = 0 - disable GPIO1
    writeReg (0xC6, 0x9081); writeReg (0xC8, 255);  // SFR GPIO wg_t00 = 255 initial off
    writeReg (0xC6, 0x9083); writeReg (0xC8, 0);    // SFR GPIO wg_t10 = 0 no on
    }

  else {
    if (value > 254)
      value = 254;

    writeReg (0xF0, 1);
    writeReg (0xC6, 0x9071); writeReg (0xC8, 0x02);        // SFR GPIO data b1:0 = enable GPIO1
    writeReg (0xC6, 0x9081); writeReg (0xC8, 255 - value); // SFR GPIO wg_t00 pwm off
    writeReg (0xC6, 0x9083); writeReg (0xC8, value);       // SFR GPIO wg_t10 pwm on
    }
  }
//}}}

//{{{
void sensorInit() {

  I2Cinit (400000);

  // page 0
  writeReg (0xF0, 0);

  // get sensorId
  uint16_t value = readReg (0);
  line3 ("9d111.48.", value);

  //  soft reset
  writeReg (0x65, 0xA000); // Bypass the PLL, R0x65:0 = 0xA000,
  writeReg (0xF0, 1); // page 1
  writeReg (0xC3, 0x0501); // Perform MCU reset by setting R0xC3:1 = 0x0501.
  writeReg (0xF0, 0); // page 0
  writeReg (0x0D, 0x0021); // Enable soft reset by setting R0x0D:0 = 0x0021. Bit 0 is used for the sensor core reset
  writeReg (0x0D, 0x0000); // Disable soft reset by setting R0x0D:0 = 0x0000.
  CyU3PThreadSleep (100);

  writeReg (0x05, 0x0204); // HBLANK B = 516
  writeReg (0x06, 0x0014); // VBLANK B = 20
  writeReg (0x07, 0x00FE); // HBLANK A = 254
  writeReg (0x08, 0x000C); // VBLANK A = 12
  writeReg (0x20, 0x0300); // Read Mode B = 9:showBorder 8:overSized
  writeReg (0x21, 0x8400); // Read Mode A = 15:binning 10:bothADC

  //  PLL
  writeReg (0x66, 0x1001);  // PLL Control 1    M:15:8,N:7:0 - M=16, N=1  (24mhz/(N+1))*M / 2*(P+1) = 48mhz
  writeReg (0x67, 0x0501);  // PLL Control 2 0x05:15:8,P:7:0 - P=1
  //writeReg (0x66, 0x8011);  // PLL Control 1    M:15:8,N:7:0 - M=79, N=2 (24mhz/(N+1))*M / 2*(P+1) = 80mhz
  //writeReg (0x67, 0x0500);  // PLL Control 2 0x05:15:8,P:7:0 - P=1
  writeReg (0x65, 0xA000);  // Clock CNTRL - PLL ON
  writeReg (0x65, 0x2000);  // Clock CNTRL - USE PLL
  CyU3PThreadSleep (100);

  // page 1
  writeReg (0xF0, 1);
  writeReg (0x97, 0x0002); // output format configuration luma:chroma swap

  //{{{  sequencer
  writeReg (0xC6, 0xA122); writeReg (0xC8, 0x01); // Enter Preview: Auto Exposure = 1
  writeReg (0xC6, 0xA123); writeReg (0xC8, 0x00); // Enter Preview: Flicker Detection = 0
  writeReg (0xC6, 0xA124); writeReg (0xC8, 0x01); // Enter Preview: Auto White Balance = 1
  writeReg (0xC6, 0xA125); writeReg (0xC8, 0x00); // Enter Preview: Auto Focus = 0
  writeReg (0xC6, 0xA126); writeReg (0xC8, 0x01); // Enter Preview: Histogram = 1
  writeReg (0xC6, 0xA127); writeReg (0xC8, 0x00); // Enter Preview: Strobe Control  = 0
  writeReg (0xC6, 0xA128); writeReg (0xC8, 0x00); // Enter Preview: Skip Control = 0

  writeReg (0xC6, 0xA129); writeReg (0xC8, 0x03); // In Preview: Auto Exposure = 3
  writeReg (0xC6, 0xA12A); writeReg (0xC8, 0x02); // In Preview: Flicker Detection = 2
  writeReg (0xC6, 0xA12B); writeReg (0xC8, 0x03); // In Preview: Auto White Balance = 3
  writeReg (0xC6, 0xA12C); writeReg (0xC8, 0x00); // In Preview: Auto Focus = 0
  writeReg (0xC6, 0xA12D); writeReg (0xC8, 0x03); // In Preview: Histogram  = 3
  writeReg (0xC6, 0xA12E); writeReg (0xC8, 0x00); // In Preview: Strobe Control = 0
  writeReg (0xC6, 0xA12F); writeReg (0xC8, 0x00); // In Preview: Skip Control = 0

  writeReg (0xC6, 0xA130); writeReg (0xC8, 0x04); // Exit Preview: Auto Exposure = 4
  writeReg (0xC6, 0xA131); writeReg (0xC8, 0x00); // Exit Preview: Flicker Detection = 0
  writeReg (0xC6, 0xA132); writeReg (0xC8, 0x01); // Exit Preview: Auto White Balance = 1
  writeReg (0xC6, 0xA133); writeReg (0xC8, 0x00); // Exit Preview: Auto Focus = 0
  writeReg (0xC6, 0xA134); writeReg (0xC8, 0x01); // Exit Preview: Histogram = 1
  writeReg (0xC6, 0xA135); writeReg (0xC8, 0x00); // Exit Preview: Strobe Control = 0
  writeReg (0xC6, 0xA136); writeReg (0xC8, 0x00); // Exit Preview: Skip Control = 0

  writeReg (0xC6, 0xA137); writeReg (0xC8, 0x00); // Capture: Auto Exposure = 0
  writeReg (0xC6, 0xA138); writeReg (0xC8, 0x00); // Capture: Flicker Detection = 0
  writeReg (0xC6, 0xA139); writeReg (0xC8, 0x00); // Capture: Auto White Balance  = 0
  writeReg (0xC6, 0xA13A); writeReg (0xC8, 0x00); // Capture: Auto Focus = 0
  writeReg (0xC6, 0xA13B); writeReg (0xC8, 0x00); // Capture: Histogram = 0
  writeReg (0xC6, 0xA13C); writeReg (0xC8, 0x00); // Capture: Strobe Control = 0
  writeReg (0xC6, 0xA13D); writeReg (0xC8, 0x00); // Capture: Skip Control = 0
  //}}}
  //{{{  mode a,b params
  writeReg (0xC6, 0x270B); writeReg (0xC8, 0x0030); // mode_config = disable jpeg A,B

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
  writeReg (0xC6, 0x2703); writeReg (0xC8, 0x0320); // Output Width A  = 800
  writeReg (0xC6, 0x2705); writeReg (0xC8, 0x0258); // Output Height A = 600
  writeReg (0xC6, 0x270F); writeReg (0xC8, 0x001C); // Row Start A = 28
  writeReg (0xC6, 0x2711); writeReg (0xC8, 0x003C); // Column Start A = 60
  writeReg (0xC6, 0x2713); writeReg (0xC8, 0x04B0); // Row Height A = 1200
  writeReg (0xC6, 0x2715); writeReg (0xC8, 0x0640); // Column Width A = 1600
  writeReg (0xC6, 0x2717); writeReg (0xC8, 0x0384); // Extra Delay A = 900
  writeReg (0xC6, 0x2719); writeReg (0xC8, 0x0011); // Row Speed A = 17
  writeReg (0xC6, 0x2727); writeReg (0xC8, 0x0000); // Crop_X0 A = 0
  writeReg (0xC6, 0x2729); writeReg (0xC8, 0x0320); // Crop_X1 A = 800
  writeReg (0xC6, 0x272B); writeReg (0xC8, 0x0000); // Crop_Y0 A = 0
  writeReg (0xC6, 0x272D); writeReg (0xC8, 0x0258); // Crop_Y1 A = 600
  writeReg (0xC6, 0xA743); writeReg (0xC8, 0x02);   // Gamma and Contrast Settings A
  writeReg (0xC6, 0xA77D); writeReg (0xC8, 0x02);   // output format config A = 0x02 swap luma:chroma

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
  writeReg (0xC6, 0x2707); writeReg (0xC8, 0x0640); // Output Width B  = 1600
  writeReg (0xC6, 0x2709); writeReg (0xC8, 0x04B0); // Output Height B = 1200
  writeReg (0xC6, 0x271B); writeReg (0xC8, 0x001C); // Row Start B = 28
  writeReg (0xC6, 0x271D); writeReg (0xC8, 0x003C); // Column Start B = 60
  writeReg (0xC6, 0x271F); writeReg (0xC8, 0x04B0); // Row Height B = 1200
  writeReg (0xC6, 0x2721); writeReg (0xC8, 0x0640); // Column Width B = 1600
  writeReg (0xC6, 0x2723); writeReg (0xC8, 0x01A7); // Extra Delay B = 423
  writeReg (0xC6, 0x2725); writeReg (0xC8, 0x0011); // Row Speed B = 17
  writeReg (0xC6, 0x2735); writeReg (0xC8, 0x0000); // Crop_X0 B = 0
  writeReg (0xC6, 0x2737); writeReg (0xC8, 0x0640); // Crop_X1 B = 1600
  writeReg (0xC6, 0x2739); writeReg (0xC8, 0x0000); // Crop_Y0 B = 0
  writeReg (0xC6, 0x273B); writeReg (0xC8, 0x04B0); // Crop_Y1 B = 1200
  writeReg (0xC6, 0xA744); writeReg (0xC8, 0x02);   // Gamma and Contrast Settings B
  writeReg (0xC6, 0xA77E); writeReg (0xC8, 0x02);   // output format config B = 0x02 swap luma:chroma
  //}}}
  //{{{  Custom gamma tables...
  writeReg (0xC6, 0xA745);    //Gamma Table 0 A
  writeReg (0xC8, 0x00);  //      = 0
  writeReg (0xC6, 0xA746);    //Gamma Table 1 A
  writeReg (0xC8, 0x14);  //      = 20
  writeReg (0xC6, 0xA747);    //Gamma Table 2 A
  writeReg (0xC8, 0x23);  //      = 35
  writeReg (0xC6, 0xA748);    //Gamma Table 3 A
  writeReg (0xC8, 0x3A);  //      = 58
  writeReg (0xC6, 0xA749);    //Gamma Table 4 A
  writeReg (0xC8, 0x5E);  //      = 94
  writeReg (0xC6, 0xA74A);    //Gamma Table 5 A
  writeReg (0xC8, 0x76);  //      = 118
  writeReg (0xC6, 0xA74B);    //Gamma Table 6 A
  writeReg (0xC8, 0x88);  //      = 136
  writeReg (0xC6, 0xA74C);    //Gamma Table 7 A
  writeReg (0xC8, 0x96);  //      = 150
  writeReg (0xC6, 0xA74D);    //Gamma Table 8 A
  writeReg (0xC8, 0xA3);  //      = 163
  writeReg (0xC6, 0xA74E);    //Gamma Table 9 A
  writeReg (0xC8, 0xAF);  //      = 175
  writeReg (0xC6, 0xA74F);    //Gamma Table 10 A
  writeReg (0xC8, 0xBA);  //      = 186
  writeReg (0xC6, 0xA750);    //Gamma Table 11 A
  writeReg (0xC8, 0xC4);  //      = 196
  writeReg (0xC6, 0xA751);    //Gamma Table 12 A
  writeReg (0xC8, 0xCE);  //      = 206
  writeReg (0xC6, 0xA752);    //Gamma Table 13 A
  writeReg (0xC8, 0xD7);  //      = 215
  writeReg (0xC6, 0xA753);    //Gamma Table 14 A
  writeReg (0xC8, 0xE0);  //      = 224
  writeReg (0xC6, 0xA754);    //Gamma Table 15 A
  writeReg (0xC8, 0xE8);  //      = 232
  writeReg (0xC6, 0xA755);    //Gamma Table 16 A
  writeReg (0xC8, 0xF0);  //      = 240
  writeReg (0xC6, 0xA756);    //Gamma Table 17 A
  writeReg (0xC8, 0xF8);  //      = 248
  writeReg (0xC6, 0xA757);    //Gamma Table 18 A
  writeReg (0xC8, 0xFF);  //      = 255
  writeReg (0xC6, 0xA758);    //Gamma Table 0 B
  writeReg (0xC8, 0x00);  //      = 0
  writeReg (0xC6, 0xA759);    //Gamma Table 1 B
  writeReg (0xC8, 0x14);  //      = 20
  writeReg (0xC6, 0xA75A);    //Gamma Table 2 B
  writeReg (0xC8, 0x23);  //      = 35
  writeReg (0xC6, 0xA75B);    //Gamma Table 3 B
  writeReg (0xC8, 0x3A);  //      = 58
  writeReg (0xC6, 0xA75C);    //Gamma Table 4 B
  writeReg (0xC8, 0x5E);  //      = 94
  writeReg (0xC6, 0xA75D);    //Gamma Table 5 B
  writeReg (0xC8, 0x76);  //      = 118
  writeReg (0xC6, 0xA75E);    //Gamma Table 6 B
  writeReg (0xC8, 0x88);  //      = 136
  writeReg (0xC6, 0xA75F);    //Gamma Table 7 B
  writeReg (0xC8, 0x96);  //      = 150
  writeReg (0xC6, 0xA760);    //Gamma Table 8 B
  writeReg (0xC8, 0xA3);  //      = 163
  writeReg (0xC6, 0xA761);    //Gamma Table 9 B
  writeReg (0xC8, 0xAF);  //      = 175
  writeReg (0xC6, 0xA762);    //Gamma Table 10 B
  writeReg (0xC8, 0xBA);  //      = 186
  writeReg (0xC6, 0xA763);    //Gamma Table 11 B
  writeReg (0xC8, 0xC4);  //      = 196
  writeReg (0xC6, 0xA764);    //Gamma Table 12 B
  writeReg (0xC8, 0xCE);  //      = 206
  writeReg (0xC6, 0xA765);    //Gamma Table 13 B
  writeReg (0xC8, 0xD7);  //      = 215
  writeReg (0xC6, 0xA766);    //Gamma Table 14 B
  writeReg (0xC8, 0xE0);  //      = 224
  writeReg (0xC6, 0xA767);    //Gamma Table 15 B
  writeReg (0xC8, 0xE8);  //      = 232
  writeReg (0xC6, 0xA768);    //Gamma Table 16 B
  writeReg (0xC8, 0xF0);  //      = 240
  writeReg (0xC6, 0xA769);    //Gamma Table 17 B
  writeReg (0xC8, 0xF8);  //      = 248
  writeReg (0xC6, 0xA76A);    //Gamma Table 18 B
  writeReg (0xC8, 0xFF);  //      = 255
  //}}}
  //{{{  other config
  writeReg (0xC6, 0x276D); writeReg (0xC8, 0xE0E2); // FIFO_Conf1 A = 57570
  writeReg (0xC6, 0xA76F); writeReg (0xC8, 0xE1);   // FIFO_Conf2 A = 225
  writeReg (0xC6, 0x2774); writeReg (0xC8, 0xE0E1); // FIFO_Conf1 B = 57569
  writeReg (0xC6, 0xA776); writeReg (0xC8, 0xE1);   // FIFO_Conf2 B = 225
  writeReg (0xC6, 0x220B); writeReg (0xC8, 0x0192); // Max R12 B (Shutter Delay) = 402
  writeReg (0xC6, 0xA217); writeReg (0xC8, 0x08);   // IndexTH23 = 8
  writeReg (0xC6, 0x2228); writeReg (0xC8, 0x020F); // RowTime (msclk per)/4 = 527
  writeReg (0xC6, 0x222F); writeReg (0xC8, 0x009A); // R9 Step = 94
  writeReg (0xC6, 0xA408); writeReg (0xC8, 0x24);   // search_f1_50 = 21
  writeReg (0xC6, 0xA409); writeReg (0xC8, 0x26);   // search_f2_50 = 23
  writeReg (0xC6, 0xA40A); writeReg (0xC8, 0x1D);   // search_f1_60 = 17
  writeReg (0xC6, 0xA40B); writeReg (0xC8, 0x1F);   // search_f2_60 = 19
  writeReg (0xC6, 0x2411); writeReg (0xC8, 0x009A); // R9_Step_60 = 94
  writeReg (0xC6, 0x2413); writeReg (0xC8, 0x00B9); // R9_Step_50 = 112
  //}}}
  CyU3PThreadSleep (100);

  writeReg (0xC6, 0xA103); writeReg (0xC8, 0x06); // Sequencer Refresh Mode
  CyU3PThreadSleep (200);

  writeReg (0xC6, 0xA103); writeReg (0xC8, 0x05); // Sequencer Refresh
  CyU3PThreadSleep (200);

  //{{{  focus init
  writeReg (0xC6, 0x90B6); writeReg (0xC8, 0x01); // SFR GPIO suspend

  // enable GPIO0,1 as output, initial value 0
  writeReg (0xC6, 0x9079); writeReg (0xC8, 0xFC); // SFR GPIO data direction
  writeReg (0xC6, 0x9071); writeReg (0xC8, 0x00); // SFR GPIO data b1:0 = 0 GPIO0,1 initial 0

  // use 8bit counter clkdiv 2^(1+2)=8 -> 48mhz -> 6mhz ->> 23.7khz
  writeReg (0xC6, 0x90B0); writeReg (0xC8, 0x01); // SFR GPIO wg_config b0 = 1 8bit counter
  writeReg (0xC6, 0x90B2); writeReg (0xC8, 0x02); // SFR GPIO wg_clkdiv b0 = 2

  // GPIO0
  writeReg (0xC6, 0x908B); writeReg (0xC8, 0x00); // SFR GPIO wg_n0 = 0 infinite
  writeReg (0xC6, 0x9081); writeReg (0xC8, 255);  // SFR GPIO wg_t00 = 255 initial off
  writeReg (0xC6, 0x9083); writeReg (0xC8, 0);    // SFR GPIO wg_t10 = 0 no on

  // GPIO1
  writeReg (0xC6, 0x908A); writeReg (0xC8, 0x00); // SFR GPIO wg_n1 = 0 infinite
  writeReg (0xC6, 0x9080); writeReg (0xC8, 0xFF); // SFR GPIO wg_t01 = 255 max initial on
  writeReg (0xC6, 0x9082); writeReg (0xC8, 0x00); // SFR GPIO wg_t11 = 0 no off

  writeReg (0xC6, 0x90B5); writeReg (0xC8, 0x00); // SFR GPIO reset
  writeReg (0xC6, 0x90B6); writeReg (0xC8, 0x00); // SFR GPIO suspend
  //}}}
  sensorScaling (600);
  }
//}}}
