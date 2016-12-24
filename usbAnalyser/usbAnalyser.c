// usbDevWare.c
/*{{{  includes*/
#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyu3gpio.h"
#include "cyu3utils.h"
#include "cyu3pib.h"
#include "cyu3gpif.h"

#include "cyfxgpif2config.h"

#include "../common/display.h"
#include "../common/sensor.h"
/*}}}*/
/*{{{  defines*/
#define RESET_GPIO 22  // CTL 5 pin
#define BUTTON_GPIO 45

#define CY_FX_EP_CONSUMER  0x81 // EP1 in

#define CY_FX_USB_BUTTON_DOWN_EVENT   (1 << 0)
#define CY_FX_USB_BUTTON_UP_EVENT     (1 << 1)
/*}}}*/
/*{{{  descriptors*/
/*{{{*/
const uint8_t CyFxUSB30DeviceDscr[] __attribute__ ((aligned (32))) = {
  0x12,                           /* Descriptor size */
  CY_U3P_USB_DEVICE_DESCR,        /* Device descriptor type */
  0x00,0x03,                      /* USB 3.0 */
  0x00,                           /* Device class */
  0x00,                           /* Device sub-class */
  0x00,                           /* Device protocol */
  0x09,                           /* Maxpacket size for EP0 : 2^9 */
  0xB4,0x04,                      /* Vendor ID */
  0xF1,0x00,                      /* Product ID */
  0x00,0x00,                      /* Device release number */
  0x01,                           /* Manufacture string index */
  0x02,                           /* Product string index */
  0x00,                           /* Serial number string index */
  0x01                            /* Number of configurations */
  };
/*}}}*/
/*{{{*/
const uint8_t CyFxUSB20DeviceDscr[] __attribute__ ((aligned (32))) = {
  0x12,                           /* Descriptor size */
  CY_U3P_USB_DEVICE_DESCR,        /* Device descriptor type */
  0x10,0x02,                      /* USB 2.10 */
  0x00,                           /* Device class */
  0x00,                           /* Device sub-class */
  0x00,                           /* Device protocol */
  0x40,                           /* Maxpacket size for EP0 : 64 bytes */
  0xB4,0x04,                      /* Vendor ID */
  0xF1,0x00,                      /* Product ID */
  0x00,0x00,                      /* Device release number */
  0x01,                           /* Manufacture string index */
  0x02,                           /* Product string index */
  0x00,                           /* Serial number string index */
  0x01                            /* Number of configurations */
  };
/*}}}*/

/*{{{*/
const uint8_t CyFxUSBBOSDscr[] __attribute__ ((aligned (32))) = {
  0x05,                           /* Descriptor size */
  CY_U3P_BOS_DESCR,               /* Device descriptor type */
  0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
  0x02,                           /* Number of device capability descriptors */

  /* USB 2.0 extension */
  0x07,                           /* Descriptor size */
  CY_U3P_DEVICE_CAPB_DESCR,       /* Device capability type descriptor */
  CY_U3P_USB2_EXTN_CAPB_TYPE,     /* USB 2.0 extension capability type */
  0x02,0x00,0x00,0x00,            /* Supported device level features: LPM support  */

  /* SuperSpeed device capability */
  0x0A,                           /* Descriptor size */
  CY_U3P_DEVICE_CAPB_DESCR,       /* Device capability type descriptor */
  CY_U3P_SS_USB_CAPB_TYPE,        /* SuperSpeed device capability type */
  0x00,                           /* Supported device level features  */
  0x0E,0x00,                      /* Speeds supported by the device : SS, HS and FS */
  0x03,                           /* Functionality support */
  0x0A,                           /* U1 Device Exit latency */
  0xFF,0x07                       /* U2 Device Exit latency */
  };
/*}}}*/
/*{{{*/
const uint8_t CyFxUSBDeviceQualDscr[] __attribute__ ((aligned (32))) = {
  0x0A,                           /* Descriptor size */
  CY_U3P_USB_DEVQUAL_DESCR,       /* Device qualifier descriptor type */
  0x00,0x02,                      /* USB 2.0 */
  0x00,                           /* Device class */
  0x00,                           /* Device sub-class */
  0x00,                           /* Device protocol */
  0x40,                           /* Maxpacket size for EP0 : 64 bytes */
  0x01,                           /* Number of configurations */
  0x00                            /* Reserved */
  };
/*}}}*/

/*{{{*/
const uint8_t CyFxUSBFSConfigDscr[] __attribute__ ((aligned (32))) = {
  /* Configuration descriptor */
  0x09,                           /* Descriptor size */
  CY_U3P_USB_CONFIG_DESCR,        /* Configuration descriptor type */
  0x19,0x00,                      /* Length of this descriptor and all sub descriptors */
  0x01,                           /* Number of interfaces */
  0x01,                           /* Configuration number */
  0x00,                           /* COnfiguration string index */
  0x80,                           /* Config characteristics - bus powered */
  0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

  /* Interface descriptor */
  0x09,                           /* Descriptor size */
  CY_U3P_USB_INTRFC_DESCR,        /* Interface descriptor type */
  0x00,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x01,                           /* Number of endpoints */
  0xFF,                           /* Interface class */
  0x00,                           /* Interface sub class */
  0x00,                           /* Interface protocol code */
  0x00,                           /* Interface descriptor string index */

  /* Endpoint descriptor for consumer EP */
  0x07,                           /* Descriptor size */
  CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
  CY_FX_EP_CONSUMER,              /* Endpoint address and description */
  CY_U3P_USB_EP_BULK,             /* Bulk endpoint type */
  0x40,0x00,                      /* Max packet size = 64 bytes */
  0x00                            /* Servicing interval for data transfers : 0 for bulk */
  };
/*}}}*/
/*{{{*/
const uint8_t CyFxUSBHSConfigDscr[] __attribute__ ((aligned (32))) = {
  /* Configuration descriptor */
  0x09,                           /* Descriptor size */
  CY_U3P_USB_CONFIG_DESCR,        /* Configuration descriptor type */
  0x19,0x00,                      /* Length of this descriptor and all sub descriptors */
  0x01,                           /* Number of interfaces */
  0x01,                           /* Configuration number */
  0x00,                           /* COnfiguration string index */
  0x80,                           /* Config characteristics - bus powered */
  0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

  /* Interface descriptor */
  0x09,                           /* Descriptor size */
  CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
  0x00,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x01,                           /* Number of endpoints */
  0xFF,                           /* Interface class */
  0x00,                           /* Interface sub class */
  0x00,                           /* Interface protocol code */
  0x00,                           /* Interface descriptor string index */

  /* Endpoint descriptor for consumer EP */
  0x07,                           /* Descriptor size */
  CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
  CY_FX_EP_CONSUMER,              /* Endpoint address and description */
  CY_U3P_USB_EP_BULK,             /* Bulk endpoint type */
  0x00,0x02,                      /* Max packet size = 512 bytes */
  0x00                            /* Servicing interval for data transfers : 0 for bulk */
  };
/*}}}*/
/*{{{*/
const uint8_t CyFxUSBSSConfigDscr[] __attribute__ ((aligned (32))) = {
  /* Configuration descriptor */
  0x09,                           /* Descriptor size */
  CY_U3P_USB_CONFIG_DESCR,        /* Configuration descriptor type */
  0x1F,0x00,                      /* Length of this descriptor and all sub descriptors */
  0x01,                           /* Number of interfaces */
  0x01,                           /* Configuration number */
  0x00,                           /* Configuration string index */
  0x80,                           /* Config characteristics - Bus powered */
  0x32,                           /* Max power consumption of device (in 8mA unit) : 400mA */

  /* Interface descriptor */
  0x09,                           /* Descriptor size */
  CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
  0x00,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x01,                           /* Number of end points */
  0xFF,                           /* Interface class */
  0x00,                           /* Interface sub class */
  0x00,                           /* Interface protocol code */
  0x00,                           /* Interface descriptor string index */

  // Endpoint descriptor for consumer in EP1
  0x07,                           /* Descriptor size */
  CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
  CY_FX_EP_CONSUMER,              /* Endpoint address and description */
  CY_U3P_USB_EP_BULK,             /* Bulk endpoint type */
  0x00,0x04,                      /* Max packet size = 1024 bytes */
  0x01,                           /* Servicing interval for data transfers : 0 for Bulk */

  // Super speed endpoint companion descriptor for consumer in EP4
  0x06,                           /* Descriptor size */
  CY_U3P_SS_EP_COMPN_DESCR,       /* SS endpoint companion descriptor type */
  15,                             /* Max no. of packets in a burst(0-15) - 0: burst 1 packet at a time */
  0x00,                           /* Max streams for bulk EP = 0 (No streams) */
  0x00,0x00,                      /* Service interval for the EP : 0 for bulk */
  };
/*}}}*/

/*{{{*/
const uint8_t CyFxUSBStringLangIDDscr[] __attribute__ ((aligned (32))) = {
  0x04,                           /* Descriptor size */
  CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
  0x09,0x04                       /* Language ID supported */
  };
/*}}}*/
/*{{{*/
const uint8_t CyFxUSBManufactureDscr[] __attribute__ ((aligned (32))) = {
  0x10,                           /* Descriptor size */
  CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
  'C',0x00, 'y',0x00, 'p',0x00, 'r',0x00, 'e',0x00, 's',0x00, 's',0x00
  };
/*}}}*/
/*{{{*/
const uint8_t CyFxUSBProductDscr[] __attribute__ ((aligned (32))) = {
  0x08,                           /* Descriptor size */
  CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
  'F',0x00, 'X',0x00, '3',0x00
  };
/*}}}*/
/*{{{*/
const uint8_t CyFxUsbOSDscr[] __attribute__ ((aligned (32))) = {
  0x0E,
  CY_U3P_USB_STRING_DESCR,
  'O', 0x00, 'S', 0x00, ' ', 0x00, 'D', 0x00, 'e', 0x00, 's', 0x00, 'c', 0x00
  };
/*}}}*/

// align cache line
const uint8_t CyFxUsbDscrAlignBuffer[32] __attribute__ ((aligned (32)));
/*}}}*/
/*{{{  vars*/
static CyU3PThread appThread;
static CyU3PEvent appEvent;

static CyU3PDmaChannel dmaChannel;

static CyBool_t appActive = CyFalse;     // Whether the source sink application is active or not. */
static CyBool_t glForceLinkU2 = CyFalse; // Whether the device should try to initiate U2 mode. */

volatile static CyBool_t hitFV = CyFalse;       // Whether end of frame (FV) signal has been hit
volatile static CyBool_t gotPartial = CyFalse;  // track last partial buffer ensure committed to USB

uint8_t glEp0Buffer[4096] __attribute__ ((aligned (32)));
/*}}}*/

// button interrupt
/*{{{*/
static void gpioInterruptCallback (uint8_t gpioId) {

  CyBool_t gpioValue = CyFalse;
  if (gpioId == BUTTON_GPIO)
    if (CyU3PGpioGetValue (gpioId, &gpioValue) == CY_U3P_SUCCESS)
      CyU3PEventSet (&appEvent,
                     gpioValue ? CY_FX_USB_BUTTON_UP_EVENT : CY_FX_USB_BUTTON_DOWN_EVENT,
                     CYU3P_EVENT_OR);
  }
/*}}}*/

// vid thread
/*{{{*/
static void appStart() {

  line2 ("appStart");

  // setup consumer endpoint
  uint16_t size = 64;
  uint16_t burstLen = 1;
  switch (CyU3PUsbGetSpeed()) {
    case CY_U3P_FULL_SPEED:  size = 64; break;
    case CY_U3P_HIGH_SPEED:  size = 512; break;
    case CY_U3P_SUPER_SPEED: size = 1024; burstLen = 16; break;
    case CY_U3P_NOT_CONNECTED: break;
    }

  CyU3PEpConfig_t epConfig;
  CyU3PMemSet ((uint8_t*)&epConfig, 0, sizeof (epConfig));
  epConfig.enable = 1;
  epConfig.epType = CY_U3P_USB_EP_BULK;
  epConfig.burstLen = burstLen;
  epConfig.pcktSize = size;
  CyU3PSetEpConfig (CY_FX_EP_CONSUMER, &epConfig);

  CyU3PUsbFlushEp (CY_FX_EP_CONSUMER);

  CyU3PDmaChannelConfig_t dmaChannelConfig;
  CyU3PMemSet ((uint8_t*)&dmaChannelConfig, 0, sizeof(dmaChannelConfig));
  dmaChannelConfig.size  = 16384;
  dmaChannelConfig.count = 4;
  dmaChannelConfig.prodSckId = CY_U3P_PIB_SOCKET_0;      // gpif pib0 producer
  dmaChannelConfig.consSckId = CY_U3P_UIB_SOCKET_CONS_1; // ep1 consumer
  dmaChannelConfig.dmaMode = CY_U3P_DMA_MODE_BYTE;
  CyU3PDmaChannelCreate (&dmaChannel, CY_U3P_DMA_TYPE_AUTO, &dmaChannelConfig);

  CyU3PDmaChannelSetXfer (&dmaChannel, 0);

  CyU3PGpifLoad (&CyFxGpifConfig);
  CyU3PGpifSMStart (START, ALPHA_START);

  appActive = CyTrue;
  }
/*}}}*/
/*{{{*/
static void appStop() {
// RESET or DISCONNECT event is received from the USB host

  line2 ("appStop");
  appActive = CyFalse;

  CyU3PGpifDisable (CyTrue);
  CyU3PDmaChannelDestroy (&dmaChannel);

  CyU3PUsbFlushEp (CY_FX_EP_CONSUMER);

  // Disable endpoints
  CyU3PEpConfig_t epConfig;
  CyU3PMemSet ((uint8_t*)&epConfig, 0, sizeof (epConfig));
  epConfig.enable = CyFalse;
  CyU3PSetEpConfig (CY_FX_EP_CONSUMER, &epConfig);
  }
/*}}}*/
/*{{{*/
static void appClear() {
// CLEAR FEATURE

  line2 ("appClear");

  CyU3PDmaChannelReset (&dmaChannel);

  CyU3PUsbFlushEp (CY_FX_EP_CONSUMER);
  CyU3PUsbResetEp (CY_FX_EP_CONSUMER);

  CyU3PDmaChannelSetXfer (&dmaChannel, 0);
  }
/*}}}*/

/*{{{*/
static void USBEventCallback (CyU3PUsbEventType_t evtype, uint16_t evdata) {

  switch (evtype) {
    case CY_U3P_USB_EVENT_CONNECT:
      line2 ("CONNECT");
      break;

    case CY_U3P_USB_EVENT_SETCONF:
      line2 ("SETCONF");
      if (appActive)
        appStop();
      appStart();
      break;

    case CY_U3P_USB_EVENT_RESET:
    case CY_U3P_USB_EVENT_DISCONNECT:
      glForceLinkU2 = CyFalse;

      if (appActive)
        appStop();

      if (evtype == CY_U3P_USB_EVENT_DISCONNECT)
        line2 ("DISCONNECT");
      else
        line2 ("RESET");
      break;

    default:
      break;
    }
  }
/*}}}*/
/*{{{*/
static CyBool_t USBSetupCallback (uint32_t setupdat0, uint32_t setupdat1) {
// Fast enumeration is used. Only requests addressed to the interface, class,
// vendor and unknown control requests are received by this function.
// This application does not support any class or vendor requests

  uint8_t bReqType  =  setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK;
  uint8_t bType     =  bReqType  & CY_U3P_USB_TYPE_MASK;
  uint8_t bTarget   =  bReqType  & CY_U3P_USB_TARGET_MASK;
  uint8_t bRequest  = (setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS;
  uint16_t wValue   = (setupdat0 & CY_U3P_USB_VALUE_MASK) >> CY_U3P_USB_VALUE_POS;
  uint16_t wIndex   = (setupdat1 & CY_U3P_USB_INDEX_MASK) >> CY_U3P_USB_INDEX_POS;
  uint16_t wLength  = (setupdat1 & CY_U3P_USB_LENGTH_MASK) >> CY_U3P_USB_LENGTH_POS;

  CyBool_t isHandled = CyFalse;
  if (bType == CY_U3P_USB_VENDOR_RQT) {
    // vendor requests
    switch (bRequest) {
      case 0xA0: // streamer
        break;
      case 0xAD:
        /*{{{  readReg*/
        I2C_Read (wValue >> 8, wValue & 0xFF, glEp0Buffer);

        CyU3PUsbSendEP0Data (2, glEp0Buffer);

        isHandled = CyTrue;
        break;
        /*}}}*/
      case 0xAE:
        /*{{{  writeReg*/
        CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);

        I2C_Write (wValue >> 8, wValue & 0xFF, glEp0Buffer[0], glEp0Buffer[1]);

        isHandled = CyTrue;
        break;
        /*}}}*/
      case 0xAF:
        /*{{{  startStreaming*/
        line2 ("vStream");
        CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);

        isHandled = CyTrue;
        break;
        /*}}}*/
      default: // other vendor
        line3 ("vendor", bRequest);
        break;
      }
    }
  else if (bType == CY_U3P_USB_STANDARD_RQT) {
    if ((bTarget == CY_U3P_USB_TARGET_INTF) &&
        ((bRequest == CY_U3P_USB_SC_SET_FEATURE) || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) &&
        (wValue == 0)) {
      /*{{{  SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND) request*/
      line2 ("interface");
      if (appActive) {
        CyU3PUsbAckSetup();

        // one interface, the link can be pushed into U2 state as soon as this interface is suspended
        if (bRequest == CY_U3P_USB_SC_SET_FEATURE)
          glForceLinkU2 = CyTrue;
        else
          glForceLinkU2 = CyFalse;
        }
      else
        CyU3PUsbStall (0, CyTrue, CyFalse);

      isHandled = CyTrue;
      }
      /*}}}*/

    if ((bTarget == CY_U3P_USB_TARGET_ENDPT) &&
        (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE) &&
        (wValue == CY_U3P_USBX_FS_EP_HALT)) {
      /*{{{  clear feature request*/
      /* CLEAR_FEATURE request for endpoint is always passed to the setup callback
       * regardless of the enumeration model used. When a clear feature is received,
       * the previous transfer has to be flushed and cleaned up. This is done at the
       * protocol level. Since this is just a loopback operation, there is no higher
       * level protocol. So flush the EP memory and reset the DMA channel associated
       * with it. If there are more than one EP associated with the channel reset both
       * the EPs. The endpoint stall and toggle / sequence number is also expected to be
       * reset. Return CyFalse to make the library clear the stall and reset the endpoint
       * toggle. Or invoke the CyU3PUsbStall (ep, CyFalse, CyTrue) and return CyTrue.
       * Here we are clearing the stall. */

      line2 ("clear feature");

      if (appActive) {
        if (wIndex == CY_FX_EP_CONSUMER) {
          appClear();
          CyU3PUsbStall (wIndex, CyFalse, CyTrue);
          CyU3PUsbAckSetup();

          isHandled = CyTrue;
          }
        }
      }
      /*}}}*/
    }
  return isHandled;
  }
/*}}}*/
/*{{{*/
static CyBool_t LPMRqtCallback (CyU3PUsbLinkPowerMode link_mode) {
// Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
//   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
//   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
//   to trigger an exit back to U0.
//   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
//   the function always return CyTrue.

  return CyTrue;
  }
/*}}}*/

/*{{{*/
static void debugInit() {

  CyU3PUartInit();

  CyU3PUartConfig_t uartConfig;
  CyU3PMemSet ((uint8_t*)&uartConfig, 0, sizeof (uartConfig));
  uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
  uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
  uartConfig.parity = CY_U3P_UART_NO_PARITY;
  uartConfig.txEnable = CyTrue;
  uartConfig.rxEnable = CyFalse;
  uartConfig.flowCtrl = CyFalse;
  uartConfig.isDma = CyTrue;

  CyU3PUartSetConfig (&uartConfig, NULL);
  CyU3PUartTxSetBlockXfer (0xFFFFFFFF);

  CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);

  CyU3PDebugPreamble(CyFalse);
  }
/*}}}*/
/*{{{*/
static void gpioInit() {

  CyU3PGpioClock_t gpioClock;
  gpioClock.fastClkDiv = 2;
  gpioClock.slowClkDiv = 2;
  gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
  gpioClock.clkSrc     = CY_U3P_SYS_CLK;
  gpioClock.halfDiv    = 0;
  CyU3PGpioInit (&gpioClock, gpioInterruptCallback);

  // Confige BUTTON_GPIO to trigger interrupt on falling edge
  CyU3PDeviceGpioOverride (BUTTON_GPIO, CyTrue);
  CyU3PGpioSimpleConfig_t gpioConfig;
  gpioConfig.outValue    = CyFalse;
  gpioConfig.inputEn     = CyTrue;
  gpioConfig.driveLowEn  = CyFalse;
  gpioConfig.driveHighEn = CyFalse;
  gpioConfig.intrMode    = CY_U3P_GPIO_INTR_BOTH_EDGE;
  CyU3PGpioSetSimpleConfig (BUTTON_GPIO, &gpioConfig);

  // config RESET_GPIO CTL5 pin
  CyU3PDeviceGpioOverride (RESET_GPIO, CyTrue);
  gpioConfig.outValue    = CyTrue;
  gpioConfig.driveLowEn  = CyTrue;
  gpioConfig.driveHighEn = CyTrue;
  gpioConfig.inputEn     = CyFalse;
  gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
  CyU3PGpioSetSimpleConfig (RESET_GPIO, &gpioConfig);

  // pulse reset lo
  CyU3PGpioSetValue (RESET_GPIO, CyFalse);
  CyU3PThreadSleep (10);
  CyU3PGpioSetValue (RESET_GPIO, CyTrue);
  CyU3PThreadSleep (10);
  }
/*}}}*/
/*{{{*/
static void appInit() {

  CyU3PEventCreate (&appEvent);
  /*{{{  init P-port*/
  CyU3PPibClock_t pibClock;
  pibClock.clkDiv = 4;
  pibClock.clkSrc = CY_U3P_SYS_CLK;
  pibClock.isDllEnable = CyFalse;
  pibClock.isHalfDiv = CyFalse;
  CyU3PPibInit (CyTrue, &pibClock);
  /*}}}*/

  sensorInit();

  CyBool_t no_renum = CyFalse;
  CyU3PReturnStatus_t apiRetStatus = CyU3PUsbStart();
  if (apiRetStatus == CY_U3P_ERROR_NO_REENUM_REQUIRED)
    no_renum = CyTrue;

  CyU3PUsbRegisterSetupCallback (USBSetupCallback, CyTrue);
  CyU3PUsbRegisterEventCallback (USBEventCallback);
  CyU3PUsbRegisterLPMRequestCallback (LPMRqtCallback);

  //  Set the USB Enumeration descriptors
  CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t*)CyFxUSB30DeviceDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t*)CyFxUSB20DeviceDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t*)CyFxUSBBOSDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t*)CyFxUSBDeviceQualDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t*)CyFxUSBSSConfigDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t*)CyFxUSBHSConfigDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t*)CyFxUSBFSConfigDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t*)CyFxUSBStringLangIDDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t*)CyFxUSBManufactureDscr);
  CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t*)CyFxUSBProductDscr);

  //  Connect the USB Pins with super speed operation enabled
  if (no_renum) {
    // USB connection already active, config the endpoints and DMA channels
    if (appActive)
      appStop();
    appStart();
    }
  else
    CyU3PConnectState (CyTrue, CyTrue);
  }
/*}}}*/
/*{{{*/
static void appThreadFunc (uint32_t input) {

  CyU3PReturnStatus_t stat;
  CyU3PUsbLinkPowerMode curState;

  for (;;) {
    // Try to get the USB 3.0 link back to U0
    if (glForceLinkU2) {
      stat = CyU3PUsbGetLinkPowerState (&curState);
      while ((glForceLinkU2) && (stat == CY_U3P_SUCCESS) && (curState == CyU3PUsbLPM_U0)) {
        /*{{{  Repeatedly try to go into U2 state*/
        CyU3PUsbSetLinkPowerState (CyU3PUsbLPM_U2);
        CyU3PThreadSleep (5);
        stat = CyU3PUsbGetLinkPowerState (&curState);
        }
        /*}}}*/
      }
    else {
      // Once data transfer has started, we keep trying to get the USB link to stay in U0. If this is done
      // before data transfers have started, there is a likelihood of failing the TD 9.24 U1/U2 test
      if (CyU3PUsbGetSpeed() == CY_U3P_SUPER_SPEED) {
        /*{{{  If the link is in U1/U2 states, try to get back to U0*/
        stat = CyU3PUsbGetLinkPowerState (&curState);
        while ((stat == CY_U3P_SUCCESS) && (curState >= CyU3PUsbLPM_U1) && (curState <= CyU3PUsbLPM_U3)) {
          CyU3PUsbSetLinkPowerState (CyU3PUsbLPM_U0);
          CyU3PThreadSleep (1);
          stat = CyU3PUsbGetLinkPowerState (&curState);
          }
        }
        /*}}}*/
      }

    uint32_t eventFlag;
    if (CyU3PEventGet (&appEvent, CY_FX_USB_BUTTON_DOWN_EVENT | CY_FX_USB_BUTTON_UP_EVENT,
                       CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS) {
      if (eventFlag & CY_FX_USB_BUTTON_DOWN_EVENT)
        sensorButton (1);
      if (eventFlag & CY_FX_USB_BUTTON_UP_EVENT)
        sensorButton (0);
      }
    }
  }
/*}}}*/

/*{{{*/
void CyFxApplicationDefine() {

  debugInit();
  gpioInit();

  displayInit ("USB anal int");

  appInit();

  CyU3PThreadCreate (
    &appThread,              // App thread structure
    "31:Analyser",           // Thread ID and thread name
     appThreadFunc,          // App thread entry function
     0,                      // No input parameter to thread
     CyU3PMemAlloc (0x1000), // Pointer to the allocated thread stack
     0x1000,                 // App thread stack size
     8, 8,                   // App thread priority
     CYU3P_NO_TIME_SLICE,    // No time slice for the application thread
     CYU3P_AUTO_START        // Start the thread immediately
     );
  }
/*}}}*/

/*{{{*/
int main() {

  CyU3PSysClockConfig_t clockConfig;
  clockConfig.setSysClk400  = CyTrue;
  clockConfig.cpuClkDiv     = 2;
  clockConfig.dmaClkDiv     = 2;
  clockConfig.mmioClkDiv    = 2;
  clockConfig.useStandbyClk = CyFalse;
  clockConfig.clkSrc        = CY_U3P_SYS_CLK;
  CyU3PDeviceInit (&clockConfig);

  // Init caches, Enable the I-Cache and keep D-Cache disabled
  CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);

  CyU3PIoMatrixConfig_t io_cfg;
  io_cfg.isDQ32Bit        = CyFalse; // no 32bit
  io_cfg.s0Mode           = CY_U3P_SPORT_INACTIVE;
  io_cfg.s1Mode           = CY_U3P_SPORT_INACTIVE;
  io_cfg.useUart          = CyTrue; // uart debug - DQ30 because of spi
  io_cfg.useI2C           = CyTrue; // I2C sensor
  io_cfg.useI2S           = CyFalse;
  io_cfg.useSpi           = CyTrue; // spi display
  io_cfg.lppMode          = CY_U3P_IO_MATRIX_LPP_DEFAULT;
  io_cfg.gpioSimpleEn[0]  = 0;
  io_cfg.gpioSimpleEn[1]  = 0;
  io_cfg.gpioComplexEn[0] = 0;
  io_cfg.gpioComplexEn[1] = 0;
  CyU3PDeviceConfigureIOMatrix (&io_cfg);

  CyU3PKernelEntry();

  return 0;
  }
/*}}}*/
