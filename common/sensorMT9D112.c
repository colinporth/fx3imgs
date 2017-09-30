// sensorMT9D112.c - I2C control
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
#define SENSOR_ADDR_WR 0x78
#define SENSOR_ADDR_RD 0x79
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
  preamble.buffer[1] = address >> 8;
  preamble.buffer[2] = address & 0xFF;
  preamble.buffer[3] = SENSOR_ADDR_RD;
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
static void writeReg (uint16_t address, uint16_t value) {

  CyU3PI2cPreamble_t preamble;
  preamble.buffer[0] = SENSOR_ADDR_WR;
  preamble.buffer[1] = address >> 8;
  preamble.buffer[2] = address & 0xFF;
  preamble.length = 3;         //  Three byte preamble. */
  preamble.ctrlMask = 0x0000;  //  No additional start and stop bits. */

  uint8_t buf[2];
  buf[0] = value >> 8;
  buf[1] = value & 0xFF;

  if (CyU3PI2cTransmitBytes (&preamble, buf, 2, 0) == CY_U3P_SUCCESS)
    CyU3PBusyWait(10);
  }
//}}}

//{{{
void I2C_Write (uint8_t hiAddr, uint8_t loAddr, uint8_t hiData, uint8_t loData) {

  //line3 ("write", loAddr);
  writeReg ((hiAddr << 8) | loAddr, (hiData << 8) | loData);
  }
//}}}
//{{{
void I2C_Read (uint8_t hiAddr, uint8_t loAddr, uint8_t* buf) {

  //line3 ("read", loAddr);
  uint16_t value = readReg ((hiAddr << 8) | loAddr);
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
    line2 ("1600x1200x15");
    writeReg (0x338C, 0xA120); writeReg (0x3390, 0x0002); // sequencer.params.mode - capture video
    writeReg (0x338C, 0xA103); writeReg (0x3390, 0x0002); // sequencer.cmd - goto capture mode B
    }
  else{
    line2 ("640x480x30");
    writeReg (0x338C, 0xA120); writeReg (0x3390, 0x0000); // sequencer.params.mode - none
    writeReg (0x338C, 0xA103); writeReg (0x3390, 0x0001); // sequencer.cmd - goto preview mode A
    }

  CyU3PThreadSleep (100);
  }
//}}}
//{{{
void sensorButton (int value) {

  if (value) {
    line2 ("cam AE 6500");
    // sequencer.mode - 5:0 - 6500K:AF:histogram:AWB:flicker:AE
    writeReg (0x338C, 0xA102); writeReg (0x3390, 0x21);
    }
  else  {
    line2 ("cam none");
    writeReg (0x338C, 0xA102); writeReg (0x3390, 0);
    }
  }
//}}}
//{{{
void sensorFocus (int value) {
  // ntohing
  }
//}}}

//{{{
void sensorInit() {

  I2Cinit (100000);

  uint16_t value = readReg (0x3000);
  line3 ("9d112.50.", value);

  //{{{  MCUBootMode - pulse reset
  writeReg (0x3386, 0x2501);
  writeReg (0x3386, 0x2500);
  CyU3PThreadSleep (100);
  //}}}
  //{{{  set parallel, standby, slew, PLL control
  // reset_register = parallel enable
  writeReg (0x301A, 0x0ACC);

  // standby_control
  writeReg (0x3202, 0x0008);
  CyU3PThreadSleep (100);

  // input powerDown 10:8:PIXCLKSlew=0, 7:inputPowerDown=0, 2:0:outputSlew=0 0=slow
  writeReg (0x3214, 0x0080);

  writeReg (0x341E, 0x8F09); // PLL,clk_in control BYPASS PLL
  writeReg (0x341C, 0x0150); // PLL 13:8:n=1, 7:0:m=85 clk = (10mhz/(n+1)) * (m/8) = 50mhz
  //writeReg (0x341C, 0x0180); // PLL 13:8:n=1, 7:0:m=128 clk = (10mhz/(n+1)) * (m/8) = 80mhz
  CyU3PThreadSleep (5);

  writeReg (0x341E, 0x8F09); // PLL,clk_in control: PLL ON, bypass PLL
  writeReg (0x341E, 0x8F08); // PLL,clk_in control: USE PLL
  //}}}

  //{{{  preview A
  // A output width = 640, height = 480
  writeReg (0x338C, 0x2703); writeReg (0x3390, 640);  // Output Width
  writeReg (0x338C, 0x2705); writeReg (0x3390, 480);  // Output Height

  // A row,column start,end = 120,160,1101,1461
  writeReg (0x338C, 0x270D); writeReg (0x3390, 0x0078);
  writeReg (0x338C, 0x270F); writeReg (0x3390, 0x00A0);
  writeReg (0x338C, 0x2711); writeReg (0x3390, 0x044d);
  writeReg (0x338C, 0x2713); writeReg (0x3390, 0x05b5);

  // A sensor_col_delay_A = 175
  writeReg (0x338C, 0x2715); writeReg (0x3390, 0x00AF);

  // A sensor_row_speed_A = 0x2111
  writeReg (0x338C, 0x2717); writeReg (0x3390, 0x2111);

  // A read mode = 0x046c default, x, xy binning, x,y odd increment
  writeReg (0x338C, 0x2719); writeReg (0x3390, 0x046c);

  // A sensor_x
  writeReg (0x338C, 0x271B); writeReg (0x3390, 0x024F); // sensor_sample_time_pck= 591
  writeReg (0x338C, 0x271D); writeReg (0x3390, 0x0102); // sensor_fine_correction = 258
  writeReg (0x338C, 0x271F); writeReg (0x3390, 0x0279); // sensor_fine_IT_min ) = 633
  writeReg (0x338C, 0x2721); writeReg (0x3390, 0x0155); // sensor_fine_IT_max_margin = 341

  // A frame lines = 480
  writeReg (0x338C, 0x2723); writeReg (0x3390, 0x01e0);

  // A line length = 768 + 64
  writeReg (0x338C, 0x2725); writeReg (0x3390, 0x0340);

  // A sensor_dac_id_x
  writeReg (0x338C, 0x2727); writeReg (0x3390, 0x2020); // sensor_dac_id_4_5 = 8224
  writeReg (0x338C, 0x2729); writeReg (0x3390, 0x2020); // sensor_dac_id_6_7 = 8224
  writeReg (0x338C, 0x272B); writeReg (0x3390, 0x1020); // sensor_dac_id_8_9 = 4128
  writeReg (0x338C, 0x272D); writeReg (0x3390, 0x2007); // sensor_dac_id_10_11 = 8199

  // A crop = 0,0,640,480
  writeReg (0x338C, 0x2751); writeReg (0x3390, 0x0000); // Crop_X0 = 0
  writeReg (0x338C, 0x2753); writeReg (0x3390, 0x0280); // Crop_X1 = 640
  writeReg (0x338C, 0x2755); writeReg (0x3390, 0x0000); // Crop_Y0 = 0
  writeReg (0x338C, 0x2757); writeReg (0x3390, 0x01E0); // Crop_Y1 = 480

  // A output_format
  writeReg (0x338c, 0x2795); writeReg (0x3390, 0x0002); // Natural, Swaps chrominance byte, yuv
  //}}}
  //{{{  capture B
  // B output width = 1600, height = 1200
  writeReg (0x338C, 0x2707); writeReg (0x3390, 0x0640);
  writeReg (0x338C, 0x2709); writeReg (0x3390, 0x04B0);

  // B row,column start,end = 4,4,1211,1611
  writeReg (0x338C, 0x272F); writeReg (0x3390, 0x0004); // Row Start (B)= 4
  writeReg (0x338C, 0x2731); writeReg (0x3390, 0x0004); // Column Start (B)= 4
  writeReg (0x338C, 0x2733); writeReg (0x3390, 0x04BB); // Row End (B)= 1211
  writeReg (0x338C, 0x2735); writeReg (0x3390, 0x064B); // Column End (B)= 1611

  // B extra delay = 124
  writeReg (0x338C, 0x2737); writeReg (0x3390, 0x007C);

  // B row speed  = 8465
  writeReg (0x338C, 0x2739); writeReg (0x3390, 0x2111);

  // B read mode = 0x0024 default, no binning, normal readout
  writeReg (0x338C, 0x273B); writeReg (0x3390, 0x0024);

  // B sensor_x
  writeReg (0x338C, 0x273D); writeReg (0x3390, 0x0120); // sensor_sample_time_pck (B)= 288
  writeReg (0x338C, 0x2741); writeReg (0x3390, 0x0169); // sensor_fine_IT_min (B)= 361

  // B frame lines = 1232
  writeReg (0x338C, 0x2745); writeReg (0x3390, 0x04D0);

  // B line length = 2284
  writeReg (0x338C, 0x2747); writeReg (0x3390, 0x08ec);

  // B crop = 0,0,1600,1200
  writeReg (0x338C, 0x275F); writeReg (0x3390, 0x0000); // Crop_X0 (B)= 0
  writeReg (0x338C, 0x2761); writeReg (0x3390, 0x0640); // Crop_X1 (B)= 1600
  writeReg (0x338C, 0x2763); writeReg (0x3390, 0x0000); // Crop_Y0 (B)= 0
  writeReg (0x338C, 0x2765); writeReg (0x3390, 0x04B0); // Crop_Y1 (B)= 1200

  // B output format
  writeReg (0x338c, 0x2797); writeReg (0x3390, 0x0002); // B - Natural, Swaps chrominance byte, yuv
  //}}}

  //{{{  AE
  writeReg (0x338C, 0xA215); writeReg (0x3390, 0x0006); // AE_maxADC
  writeReg (0x338C, 0xA206); writeReg (0x3390, 0x0036); // AE_TARGET brightness
  writeReg (0x338C, 0xA207); writeReg (0x3390, 0x0040); // AE_GATE sensitivity
  writeReg (0x338C, 0xA20C); writeReg (0x3390, 0x0008); // AE_maxIndex max zone num
  //}}}
  //{{{  black level, gain
  writeReg (0x3278, 0x0050); // first black level
  writeReg (0x327a, 0x0050); // first black level,red
  writeReg (0x327c, 0x0050); // green_1
  writeReg (0x327e, 0x0050); // green_2
  writeReg (0x3280, 0x0050); // blue

  CyU3PThreadSleep (10);
  //}}}
  writeReg (0x337e, 0x2000); // Y,RGB offset
  //{{{  AWB
  writeReg (0x338C, 0xA34A); writeReg (0x3390, 0x0059);     // AWB_GAIN_MIN
  writeReg (0x338C, 0xA34B); writeReg (0x3390, 0x00A6);     // AWB_GAIN_MAX

  writeReg (0x338C, 0x235F); writeReg (0x3390, 0x0040);     // AWB_CNT_PXL_TH

  writeReg (0x338C, 0xA361); writeReg (0x3390, 0x00D2);     // AWB_TG_MIN0
  writeReg (0x338C, 0xA362); writeReg (0x3390, 0x00E6);     // AWB_TG_MAX0
  writeReg (0x338C, 0xA363); writeReg (0x3390, 0x0010);     // AWB_X0

  writeReg (0x338C, 0xA364); writeReg (0x3390, 0x00A0);     // AWB_KR_L
  writeReg (0x338C, 0xA365); writeReg (0x3390, 0x0096);     // AWB_KG_L
  writeReg (0x338C, 0xA366); writeReg (0x3390, 0x0080);     // AWB_KB_L
  writeReg (0x338C, 0xA367); writeReg (0x3390, 0x0080);     // AWB_KR_R
  writeReg (0x338C, 0xA368); writeReg (0x3390, 0x0080);     // AWB_KG_R
  writeReg (0x338C, 0xA369); writeReg (0x3390, 0x0080);     // AWB_KB_R

  writeReg (0x32A2, 0x3640);     // RESERVED_SOC1_32A2  // fine tune color setting

  writeReg (0x338C, 0x2306); writeReg (0x3390, 0x02FF);     // AWB_CCM_L_0
  writeReg (0x338C, 0x2308); writeReg (0x3390, 0xFE6E);     // AWB_CCM_L_1
  writeReg (0x338C, 0x230A); writeReg (0x3390, 0xFFC2);     // AWB_CCM_L_2
  writeReg (0x338C, 0x230C); writeReg (0x3390, 0xFF4A);     // AWB_CCM_L_3
  writeReg (0x338C, 0x230E); writeReg (0x3390, 0x02D7);     // AWB_CCM_L_4
  writeReg (0x338C, 0x2310); writeReg (0x3390, 0xFF30);     // AWB_CCM_L_5
  writeReg (0x338C, 0x2312); writeReg (0x3390, 0xFF6E);     // AWB_CCM_L_6
  writeReg (0x338C, 0x2314); writeReg (0x3390, 0xFDEE);     // AWB_CCM_L_7
  writeReg (0x338C, 0x2316); writeReg (0x3390, 0x03CF);     // AWB_CCM_L_8
  writeReg (0x338C, 0x2318); writeReg (0x3390, 0x0020);     // AWB_CCM_L_9
  writeReg (0x338C, 0x231A); writeReg (0x3390, 0x003C);     // AWB_CCM_L_10

  writeReg (0x338C, 0x231C); writeReg (0x3390, 0x002C);     // AWB_CCM_RL_0
  writeReg (0x338C, 0x231E); writeReg (0x3390, 0xFFBC);     // AWB_CCM_RL_1
  writeReg (0x338C, 0x2320); writeReg (0x3390, 0x0016);     // AWB_CCM_RL_2
  writeReg (0x338C, 0x2322); writeReg (0x3390, 0x0037);     // AWB_CCM_RL_3
  writeReg (0x338C, 0x2324); writeReg (0x3390, 0xFFCD);     // AWB_CCM_RL_4
  writeReg (0x338C, 0x2326); writeReg (0x3390, 0xFFF3);     // AWB_CCM_RL_5
  writeReg (0x338C, 0x2328); writeReg (0x3390, 0x0077);     // AWB_CCM_RL_6
  writeReg (0x338C, 0x232A); writeReg (0x3390, 0x00F4);     // AWB_CCM_RL_7
  writeReg (0x338C, 0x232C); writeReg (0x3390, 0xFE95);     // AWB_CCM_RL_8
  writeReg (0x338C, 0x232E); writeReg (0x3390, 0x0014);     // AWB_CCM_RL_9
  writeReg (0x338C, 0x2330); writeReg (0x3390, 0xFFE8);     // AWB_CCM_RL_10  //end

  writeReg (0x338C, 0xA348); writeReg (0x3390, 0x0008);     // AWB_GAIN_BUFFER_SPEED
  writeReg (0x338C, 0xA349); writeReg (0x3390, 0x0002);     // AWB_JUMP_DIVISOR
  writeReg (0x338C, 0xA34A); writeReg (0x3390, 0x0059);     // AWB_GAIN_MIN
  writeReg (0x338C, 0xA34B); writeReg (0x3390, 0x00A6);     // AWB_GAIN_MAX
  writeReg (0x338C, 0xA34F); writeReg (0x3390, 0x0000);     // AWB_CCM_POSITION_MIN
  writeReg (0x338C, 0xA350); writeReg (0x3390, 0x007F);     // AWB_CCM_POSITION_MAX
  writeReg (0x338C, 0xA352); writeReg (0x3390, 0x001E);     // AWB_SATURATION
  writeReg (0x338C, 0xA353); writeReg (0x3390, 0x0002);     // AWB_MODE

  writeReg (0x338C, 0xA35B); writeReg (0x3390, 0x007E);     // AWB_STEADY_BGAIN_OUT_MIN
  writeReg (0x338C, 0xA35C); writeReg (0x3390, 0x0086);     // AWB_STEADY_BGAIN_OUT_MAX
  writeReg (0x338C, 0xA35D); writeReg (0x3390, 0x007F);     // AWB_STEADY_BGAIN_IN_MIN
  writeReg (0x338C, 0xA35E); writeReg (0x3390, 0x0082);     // AWB_STEADY_BGAIN_IN_MAX

  writeReg (0x338C, 0xA302); writeReg (0x3390, 0x0000);     // AWB_WINDOW_POS
  writeReg (0x338C, 0xA303); writeReg (0x3390, 0x00EF);     // AWB_WINDOW_SIZE
  writeReg (0x338C, 0xAB05); writeReg (0x3390, 0x0000);     // HG_PERCENT
  //}}}
  writeReg (0x35A4, 0x0596); // BRIGHT_COLOR_KILL_CONTROLS

  //{{{  SEQ_LL
  writeReg (0x338C, 0xA118); writeReg (0x3390, 0x001E);     // SEQ_LLSAT1
  //writeReg (0x338c, 0xa118); writeReg (0x3390, 0x0026); // sequencer.saturation = 26
  writeReg (0x338C, 0xA119); writeReg (0x3390, 0x0004);     // SEQ_LLSAT2

  writeReg (0x338C, 0xA11A); writeReg (0x3390, 0x000A);     // SEQ_LLINTERPTHRESH1
  writeReg (0x338C, 0xA11B); writeReg (0x3390, 0x0020);     // SEQ_LLINTERPTHRESH2
  //}}}
  //{{{  SEQ_NR
  writeReg (0x338C, 0xA13E); writeReg (0x3390, 0x0004);     // SEQ_NR_TH1_R
  writeReg (0x338C, 0xA13F); writeReg (0x3390, 0x000E);     // SEQ_NR_TH1_G
  writeReg (0x338C, 0xA140); writeReg (0x3390, 0x0004);     // SEQ_NR_TH1_B
  writeReg (0x338C, 0xA141); writeReg (0x3390, 0x0004);     // SEQ_NR_TH1_OL
  writeReg (0x338C, 0xA142); writeReg (0x3390, 0x0032);     // SEQ_NR_TH2_R
  writeReg (0x338C, 0xA143); writeReg (0x3390, 0x000F);     // SEQ_NR_TH2_G
  writeReg (0x338C, 0xA144); writeReg (0x3390, 0x0032);     // SEQ_NR_TH2_B
  writeReg (0x338C, 0xA145); writeReg (0x3390, 0x0032);     // SEQ_NR_TH2_OL

  writeReg (0x338C, 0xA146); writeReg (0x3390, 0x0005);     // SEQ_NR_GAINTH1
  writeReg (0x338C, 0xA147); writeReg (0x3390, 0x003A);     // SEQ_NR_GAINTH2
  //}}}
  //{{{  flicker R9 step
  writeReg (0x338C, 0x222E); writeReg (0x3390, 0x0090); // R9 Step = 144

  writeReg (0x338C, 0xA408); writeReg (0x3390, 0x001A); // search_f1_50 = 26
  writeReg (0x338C, 0xA409); writeReg (0x3390, 0x001D); // search_f2_50 = 29
  writeReg (0x338C, 0xA40A); writeReg (0x3390, 0x0020); // search_f1_60 = 32
  writeReg (0x338C, 0xA40B); writeReg (0x3390, 0x0023); // search_f2_60 = 35

  writeReg (0x338C, 0xA40D); writeReg (0x3390, 0x0002); // Stat_min = 2
  writeReg (0x338C, 0xA410); writeReg (0x3390, 0x0001); // Min_amplitude = 1

  writeReg (0x338C, 0x2411); writeReg (0x3390, 0x0090); // R9_Step_60_A = 144
  writeReg (0x338C, 0x2413); writeReg (0x3390, 0x00AD); // R9_Step_50_A = 173
  writeReg (0x338C, 0x2415); writeReg (0x3390, 0x0055); // R9_Step_60_B = 85
  writeReg (0x338C, 0x2417); writeReg (0x3390, 0x0066); // R9_Step_50_B = 102
  //}}}

  writeReg (0x338C, 0xA103); writeReg (0x3390, 0x0006); // sequencer.cmd = 6 = refresh mode
  CyU3PThreadSleep (100);
  writeReg (0x338C, 0xA103); writeReg (0x3390, 0x0005); // sequencer.cmd = 5 = refresh
  CyU3PThreadSleep (100);

  writeReg (0x33f4, 0x031d); // defect - undocumented
  }
//}}}
