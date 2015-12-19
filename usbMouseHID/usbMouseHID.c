// mouseHID.c
//{{{  includes
#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3uart.h>
#include <cyu3utils.h>
#include <cyu3gpio.h>

#include "display.h"
//}}}
//{{{  defines
// Endpoint and socket definitions for the HID application
// To change the Producer and Consumer EP enter the appropriate EP numbers for the #defines.
// In the case of IN endpoints enter EP number along with the direction bit.
// For eg. EP 6 IN endpoint is 0x86
//     and EP 6 OUT endpoint is 0x06.
// To change sockets mention the appropriate socket number in the #defines
// Note: For USB 2.0 the endpoints and corresponding sockets are one-to-one mapped
//       i.e. EP 1 is mapped to UIB socket 1 and EP 2 to socket 2 so on

/* Interrupt GPIO ID. */
#define CY_FX_BUTTON_GPIO      45
#define RESET_GPIO             22  // CTL 5 pin

#define CY_FX_HID_EP_INTR_IN                  (0x81)          /* EP 1 IN */

#define CY_FX_HID_THREAD_STACK                (0x1000)        /* HID application thread stack size */
#define CY_FX_HID_THREAD_PRIORITY             (8)             /* HID application thread priority */
#define CY_FX_HID_DMA_BUF_COUNT               (20)            /* DMA Buffer Count */

#define CY_FX_APP_GPIO_INTR_CB_EVENT_FLAG     (1 << 1)        /* MSC application Clear Stall IN Event Flag */

#define CY_FX_HID_SET_IDLE                    (0x0A)          /* HID Class Specific Setup Request */
#define CY_FX_GET_REPORT_DESC                 (0x22)          /* HID Standard Request */

#define CY_FX_USB_HID_INTF                    (0x00)          /* HID interface number */

/* Descriptor Types */
#define CY_FX_BOS_DSCR_TYPE                   (15)            /* BOS Descriptor */
#define CY_FX_DEVICE_CAPB_DSCR_TYPE           (16)            /* Device Capability */
#define CY_FX_SS_EP_COMPN_DSCR_TYPE           (48)            /* EP Companion */
#define CY_FX_USB_HID_DESC_TYPE               (0x21)          /* HID Descriptor */
//}}}
//{{{  vars
CyU3PThread            UsbHidAppThread;           /* HID application thread structure */
static CyU3PEvent      glHidAppEvent;             /* HID application Event group */
static CyU3PDmaChannel glChHandleInterruptCPU2U;  /* DMA Channel handle */
static uint32_t        glButtonPress = 0;         /* Button Press Counter */
static CyBool_t        glIsAppActive = CyFalse;   /* HID application Active Flag */
//}}}

//{{{
static const uint8_t CyFxUSB30DeviceDscr[] __attribute__ ((aligned (32))) =
{
    0x12,                           /* Descriptor Size */
    CY_U3P_USB_DEVICE_DESCR,        /* Device Descriptor Type */
    0x00,0x03,                      /* USB 3.0 */
    0x00,                           /* Device Class */
    0x00,                           /* Device Sub-class */
    0x00,                           /* Device protocol */
    0x09,                           /* Maxpacket size for EP0 : 2^9 */
    0xB4,0x04,                      /* Vendor ID */
    0x25,0x60,                      /* Product ID */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x03,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};
//}}}
//{{{
static const uint8_t CyFxUSB20DeviceDscr[] __attribute__ ((aligned (32))) =
{
    0x12,                           /* Descriptor Size */
    CY_U3P_USB_DEVICE_DESCR,        /* Device Descriptor Type */
    0x10,0x02,                      /* USB 2.1 */
    0x00,                           /* Device Class */
    0x00,                           /* Device Sub-class */
    0x00,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0xB4,0x04,                      /* Vendor ID */
    0x25,0x60,                      /* Product ID */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x03,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};
//}}}
//{{{
static const uint8_t CyFxUSBBOSDscr[] __attribute__ ((aligned (32))) =
{
    0x05,                           /* Descriptor Size */
    CY_FX_BOS_DSCR_TYPE,            /* Device Descriptor Type */
    0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of device capability descriptors */

    /* USB 2.0 Extension */
    0x07,                           /* Descriptor Size */
    CY_FX_DEVICE_CAPB_DSCR_TYPE,    /* Device Capability Type descriptor */
    CY_U3P_USB2_EXTN_CAPB_TYPE,     /* USB 2.0 Extension Capability Type */
    0x02,0x00,0x00,0x00,            /* Supported device level features  */

    /* SuperSpeed Device Capability */
    0x0A,                           /* Descriptor Size */
    CY_FX_DEVICE_CAPB_DSCR_TYPE,    /* Device Capability Type descriptor */
    CY_U3P_SS_USB_CAPB_TYPE,        /* SuperSpeed Device Capability Type */
    0x00,                           /* Supported device level features  */
    0x0E,0x00,                      /* Speeds Supported by the device : SS, HS and FS */
    0x03,                           /* Functionality support */
    0x00,                           /* U1 Device Exit Latency */
    0x00,0x00                       /* U2 Device Exit Latency */
};
//}}}
//{{{
static const uint8_t CyFxUSBDeviceQualDscr[] __attribute__ ((aligned (32))) =
{
    0x0A,                           /* Descriptor Size */
    CY_U3P_USB_DEVQUAL_DESCR,       /* Device Qualifier Descriptor Type */
    0x01,0x02,                      /* USB 2.1 */
    0x00,                           /* Device Class */
    0x00,                           /* Device Sub-class */
    0x00,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0x01,                           /* Number of configurations */
    0x00                            /* Reserved */
};
//}}}
//{{{
static const uint8_t CyFxUSBSSConfigDscr[] __attribute__ ((aligned (32))) =
{
    /* Configuration Descriptor Type */
    0x09,                           /* Descriptor Size */
    CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
    0x28,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x01,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
    0xC0,                           /* Self powered device. */
    0x0C,                           /* Max power consumption of device (in 8mA unit) : 96 mA */

    /* Interface Descriptor */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
    CY_FX_USB_HID_INTF,             /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of end points */
    0x03,                           /* Interface class */
    0x00,                           /* Interface sub class : None */
    0x02,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* HID Descriptor (Mouse) */
    0x09,                           /* Descriptor size */
    CY_FX_USB_HID_DESC_TYPE,        /* Descriptor Type */
    0x10,0x11,                      /* HID Class Spec 11.1 */
    0x00,                           /* Target Country */
    0x01,                           /* Total HID Class Descriptors */
    0x22,                           /* Report Descriptor Type */
    0x1C,0x00,                      /* Total Length of Report Descriptor */

    /* Endpoint Descriptor (Mouse) */
    0x07,                           /* Descriptor size */
    CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
    CY_FX_HID_EP_INTR_IN,           /* Endpoint address and description */
    CY_U3P_USB_EP_INTR,             /* Bulk End point Type */
    0x02,0x00,                      /* Max packet size = 2 bytes */
    0x05,                           /* Servicing interval is 2 ** (5 - 1) = 16 Intervals = 2 ms. */

    /* Super Speed Endpoint Companion Descriptor (Mouse) */
    0x06,                           /* Descriptor size */
    CY_FX_SS_EP_COMPN_DSCR_TYPE,    /* SS Endpoint Companion Descriptor Type */
    0x00,                           /* Max no. of packets in a Burst. */
    0x00,                           /* No streaming for Interrupt Endpoints. */
    0x02,0x00                       /* Number of bytes per interval = 2. */
};
//}}}
//{{{
static const uint8_t CyFxUSBHSConfigDscr[] __attribute__ ((aligned (32))) =
{
    /* Configuration Descriptor Type */
    0x09,                           /* Descriptor Size */
    CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
    0x22,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x01,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
    0xC0,                           /* Self powered device. */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

    /* Interface Descriptor */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
    CY_FX_USB_HID_INTF,             /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of end points */
    0x03,                           /* Interface class : HID Class */
    0x01,                           /* Interface sub class : None */
    0x02,                           /* Interface protocol code : Mouse */
    0x00,                           /* Interface descriptor string index */

    /* HID Descriptor (Mouse) */
    0x09,                           /* Descriptor size */
    CY_FX_USB_HID_DESC_TYPE,        /* Descriptor Type */
    0x10,0x11,                      /* HID Class Spec 11.1 */
    0x00,                           /* Target Country */
    0x01,                           /* Total HID Class Descriptors */
    0x22,                           /* Report Descriptor Type */
    0x1C,0x00,                      /* Total Length of Report Descriptor */

    /* Endpoint Descriptor (Mouse) */
    0x07,                           /* Descriptor size */
    CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
    CY_FX_HID_EP_INTR_IN,           /* Endpoint address and description */
    CY_U3P_USB_EP_INTR,             /* Interrupt Endpoint Type */
    0x02,0x00,                      /* Max packet size = 2 Bytes */
    0x05                            /* Polling interval : 2 ** (5-1) = 16 MicroFrames == 2 msec */
};
//}}}
//{{{
static const uint8_t CyFxUSBFSConfigDscr[] __attribute__ ((aligned (32))) =
{
    /* Configuration Descriptor Type */
    0x09,                           /* Descriptor Size */
    CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
    0x22,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x01,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
    0xC0,                           /* Self powered device. */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

    /* Interface Descriptor */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
    CY_FX_USB_HID_INTF,             /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of end points */
    0x03,                           /* Interface class : HID Class */
    0x00,                           /* Interface sub class : None */
    0x02,                           /* Interface protocol code : Mouse */
    0x00,                           /* Interface descriptor string index */

    /* HID Descriptor (Mouse) */
    0x09,                           /* Descriptor size */
    CY_FX_USB_HID_DESC_TYPE,        /* Descriptor Type */
    0x10,0x11,                      /* HID Class Spec 11.1 */
    0x00,                           /* Target Country */
    0x01,                           /* Total HID Class Descriptors */
    0x22,                           /* Report Descriptor Type */
    0x1C,0x00,                      /* Total Length of Report Descriptor */

    /* Endpoint Descriptor (Mouse) */
    0x07,                           /* Descriptor size */
    CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
    CY_FX_HID_EP_INTR_IN,           /* Endpoint address and description */
    CY_U3P_USB_EP_INTR,             /* Interrupt Endpoint Type */
    0x02,0x00,                      /* Max packet size = 2 bytes */
    0x02                            /* Servicing interval for data transfers : 2 msec */
};
//}}}
//{{{
static const uint8_t CyFxUSBStringLangIDDscr[] __attribute__ ((aligned (32))) =
{
    0x04,                           /* Descriptor Size */
    CY_U3P_USB_STRING_DESCR,        /* String Descriptor Type */
    0x09,0x04                       /* Language ID supported: US English */
};
//}}}
//{{{
static const uint8_t CyFxUSBManufactureDscr[] __attribute__ ((aligned (32))) =
{
    0x10,                           /* Descriptor Size */
    CY_U3P_USB_STRING_DESCR,        /* String Descriptor Type */
    'C',0x00, 'y',0x00, 'p',0x00, 'r',0x00, 'e',0x00, 's',0x00, 's',0x00
};
//}}}
//{{{
static const uint8_t CyFxUSBProductDscr[] __attribute__ ((aligned (32))) =
{
    0x08,                           /* Descriptor Size */
    CY_U3P_USB_STRING_DESCR,        /* String Descriptor Type */
    'F',0x00, 'X',0x00, '3',0x00
};
//}}}
//{{{
static const uint8_t CyFxUSBSerialNumberDscr[] __attribute__ ((aligned (32))) =
{
    0x1A,                           /* Descriptor Size */
    CY_U3P_USB_STRING_DESCR,        /* String Descriptor Type */
    0x30, 0x00, 0x31, 0x00, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00, 0x35, 0x00,
    0x36, 0x00, 0x37, 0x00, 0x38, 0x00, 0x39, 0x00, 0x30, 0x00, 0x31, 0x00
};
//}}}

//{{{
static const uint8_t CyFxUSBReportDscr[] __attribute__ ((aligned (32))) =
{
    0x05,0x01,                      /* Usage Page (Generic Desktop) */
    0x09,0x02,                      /* Usage (Mouse) */
    0xA1,0x01,                      /* Collection (Application) */
    0x09,0x01,                      /* Usage (Pointer) */
    0xA1,0x00,                      /* Collection (Physical) */
    0x05,0x01,                      /* Usage Page (Generic Desktop) */
    0x09,0x30,                      /* Usage (X) */
    0x09,0x31,                      /* Usage (Y) */
    0x15,0x81,                      /* Logical Minimum (-127) */
    0x25,0x7F,                      /* Logical Maximum (127) */
    0x75,0x08,                      /* Report Size (8) */
    0x95,0x02,                      /* Report Count (2) */
    0x81,0x06,                      /* Input (Data, Value, Relative, Bit Field) */
    0xC0,                           /* End Collection */
    0xC0                            /* End Collection */
};
//}}}
//{{{
/* Static table containing pre-computed co-ordinates that make the mouse cursor circumscribe a circle. */
static const uint8_t glMouseData[472] = {
    0x00, 0xF7,
    0x01, 0xFA,
    0x01, 0xFC,
    0x01, 0xFD,
    0x01, 0xFD,
    0x01, 0xFD,
    0x01, 0xFD,
    0x01, 0xFC,
    0x01, 0xFE,
    0x01, 0xFF,
    0x01, 0xFE,
    0x01, 0xFE,
    0x01, 0xFE,
    0x01, 0xFE,
    0x01, 0xFE,
    0x01, 0xFF,
    0x01, 0xFE,
    0x01, 0xFF,
    0x01, 0xFE,
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFE,
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFE,
    0x01, 0xFF,          //1st Arc
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFF,
    0x02, 0xFF,
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFF,
    0x01, 0xFF,
    0x02, 0xFF,
    0x01, 0xFF,
    0x02, 0xFF,
    0x02, 0xFF,
    0x01, 0xFF,
    0x02, 0xFF,
    0x02, 0xFF,
    0x02, 0xFF,
    0x02, 0xFF,
    0x02, 0xFF,
    0x02, 0xFF,
    0x03, 0xFF,
    0x03, 0xFF,
    0x03, 0xFF,
    0x04, 0xFF,
    0x03, 0xFF,
    0x04, 0xFF,
    0x07, 0xFF,
    0x09, 0xFF,          //2nd Arc, 1st Quarter
    0x09, 0x01,
    0x07, 0x01,
    0x04, 0x01,
    0x03, 0x01,
    0x04, 0x01,
    0x03, 0x01,
    0x03, 0x01,
    0x03, 0x01,
    0x02, 0x01,
    0x02, 0x01,
    0x02, 0x01,
    0x02, 0x01,
    0x02, 0x01,
    0x02, 0x01,
    0x01, 0x01,
    0x02, 0x01,
    0x02, 0x01,
    0x01, 0x01,
    0x02, 0x01,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x01,
    0x02, 0x01,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x01,          //3nd Arc
    0x01, 0x01,
    0x01, 0x02,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x02,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x01,
    0x01, 0x02,
    0x01, 0x01,
    0x01, 0x02,
    0x01, 0x01,
    0x01, 0x02,
    0x01, 0x02,
    0x01, 0x02,
    0x01, 0x02,
    0x01, 0x02,
    0x01, 0x01,
    0x01, 0x02,
    0x01, 0x04,
    0x01, 0x03,
    0x01, 0x03,
    0x01, 0x03,
    0x01, 0x03,
    0x01, 0x04,
    0x01, 0x05,
    0x01, 0x09,          //4rt Arc, 2nd Quarter, Semi Circle
    0xFF, 0x09,
    0xFF, 0x05,
    0xFF, 0x04,
    0xFF, 0x03,
    0xFF, 0x03,
    0xFF, 0x03,
    0xFF, 0x03,
    0xFF, 0x04,
    0xFF, 0x02,
    0xFF, 0x01,
    0xFF, 0x02,
    0xFF, 0x02,
    0xFF, 0x02,
    0xFF, 0x02,
    0xFF, 0x02,
    0xFF, 0x01,
    0xFF, 0x02,
    0xFF, 0x01,
    0xFF, 0x02,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x02,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x02,
    0xFF, 0x01,          //5th Arc
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFE, 0x01,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFF, 0x01,
    0xFE, 0x01,
    0xFF, 0x01,
    0xFE, 0x01,
    0xFE, 0x01,
    0xFF, 0x01,
    0xFE, 0x01,
    0xFE, 0x01,
    0xFE, 0x01,
    0xFE, 0x01,
    0xFE, 0x01,
    0xFE, 0x01,
    0xFD, 0x01,
    0xFD, 0x01,
    0xFD, 0x01,
    0xFC, 0x01,
    0xFD, 0x01,
    0xFC, 0x01,
    0xFA, 0x01,
    0xF7, 0x01,          //6th Arc, 3rd Quarter
    0xF7, 0x00,
    0xFA, 0xFF,
    0xFB, 0xFF,
    0xFD, 0xFF,
    0xFC, 0xFF,
    0xFD, 0xFF,
    0xFD, 0xFF,
    0xFD, 0xFF,
    0xFE, 0xFF,
    0xFE, 0xFF,
    0xFE, 0xFF,
    0xFE, 0xFF,
    0xFE, 0xFF,
    0xFE, 0xFF,
    0xFF, 0xFF,
    0xFE, 0xFF,
    0xFE, 0xFF,
    0xFF, 0xFF,
    0xFE, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFE, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,          //7th Arc
    0xFF, 0xFF,
    0xFF, 0xFE,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFE,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFF,
    0xFF, 0xFE,
    0xFF, 0xFF,
    0xFF, 0xFE,
    0xFF, 0xFF,
    0xFF, 0xFE,
    0xFF, 0xFE,
    0xFF, 0xFE,
    0xFF, 0xFE,
    0xFF, 0xFE,
    0xFF, 0xFF,
    0xFF, 0xFE,
    0xFF, 0xFC,
    0xFF, 0xFD,
    0xFF, 0xFD,
    0xFF, 0xFD,
    0xFF, 0xFD,
    0xFF, 0xFC,
    0xFF, 0xFB,
    0xFF, 0xF7          //8th Arc, 3rd Quarter, Circle
};
//}}}

//{{{
static void CyFxAppErrorHandler (CyU3PReturnStatus_t apiRetStatus) {

  for (;;)
    CyU3PThreadSleep (100);
  }
//}}}

//{{{
/* Send Reports containing pre-defined patterns to Host through interrupt EP */
static CyU3PReturnStatus_t hidSendReport() {

  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  CyU3PDmaBuffer_t outBuf;
  outBuf.buffer = 0;
  outBuf.status = 0;
  outBuf.size   = 2;
  outBuf.count  = 2;

  line2 ("button");
  CyU3PDebugPrint (4, "Input Report \r\n");

  uint16_t i = 0;
  do {
    /* Retrieve Free Buffer */
    status = CyU3PDmaChannelGetBuffer (&glChHandleInterruptCPU2U, &outBuf, 1000);
    if (status != CY_U3P_SUCCESS) {
      CyU3PDmaChannelReset (&glChHandleInterruptCPU2U);
      CyU3PDmaChannelSetXfer (&glChHandleInterruptCPU2U, 0);
      status = CyU3PDmaChannelGetBuffer (&glChHandleInterruptCPU2U, &outBuf, 1000);
      if (status != CY_U3P_SUCCESS)
        return status;
      }

    /* Copy Report Data into the output buffer */
    outBuf.buffer[0] = (uint8_t)(glMouseData[i]);
    outBuf.buffer[1] = (uint8_t)(glMouseData[i + 1]);
    status = CyU3PDmaChannelCommitBuffer (&glChHandleInterruptCPU2U, 2, 0);
    if (status != CY_U3P_SUCCESS) {
      CyU3PDmaChannelReset (&glChHandleInterruptCPU2U);
      CyU3PDmaChannelSetXfer (&glChHandleInterruptCPU2U, 0);
      }

    /* Wait for 2 msec after sending each packet */
    CyU3PDebugPrint (4, "Packet %d Status %d\r\n", i, status);
    CyU3PBusyWait (2000);

    line3 ("point", i/2);

    i += 2;
    } while (i != 472);

  line2 ("done");
  return status;
  }
//}}}

//{{{
/* Callback for GPIO related interrupts */
static void CyFxHidGpioInterruptCallback (uint8_t gpioId) {

  CyBool_t gpioValue = CyFalse;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  if (gpioId == CY_FX_BUTTON_GPIO) {
    apiRetStatus = CyU3PGpioGetValue (gpioId, &gpioValue);
    if (apiRetStatus == CY_U3P_SUCCESS) {
      /* Check status of the pin */
      if (gpioValue == CyFalse) {
        glButtonPress++;
        CyU3PEventSet (&glHidAppEvent, CY_FX_APP_GPIO_INTR_CB_EVENT_FLAG, CYU3P_EVENT_OR);
        }
      }
    }
  }
//}}}

//{{{
/* This function starts the HID application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
static void CyFxUsbHidAppStart() {

  line2 ("start");

  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  uint16_t size = 0x40;

  CyU3PEpConfig_t epCfg;
  CyU3PMemSet ((uint8_t*)&epCfg, 0, sizeof (epCfg));
  epCfg.enable   = CyTrue;
  epCfg.epType   = CY_U3P_USB_EP_INTR;
  epCfg.burstLen = 1;
  epCfg.streams  = 0;
  epCfg.pcktSize = size;
  apiRetStatus = CyU3PSetEpConfig (CY_FX_HID_EP_INTR_IN, &epCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
    CyFxAppErrorHandler (apiRetStatus);
    }

  CyU3PDmaChannelConfig_t dmaCfg;
  dmaCfg.size           = size;
  dmaCfg.count          = CY_FX_HID_DMA_BUF_COUNT;
  dmaCfg.prodSckId      = CY_U3P_CPU_SOCKET_PROD;
  dmaCfg.consSckId      = (CyU3PDmaSocketId_t) CY_U3P_UIB_SOCKET_CONS_1;
  dmaCfg.dmaMode        = CY_U3P_DMA_MODE_BYTE;
  dmaCfg.notification   = 0;
  dmaCfg.cb             = NULL;
  dmaCfg.prodHeader     = 0;
  dmaCfg.prodFooter     = 0;
  dmaCfg.consHeader     = 0;
  dmaCfg.prodAvailCount = 0;
  apiRetStatus = CyU3PDmaChannelCreate (&glChHandleInterruptCPU2U, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
    CyFxAppErrorHandler (apiRetStatus);
    }

  /* Set Channel for Infinite Transfer */
  CyU3PDmaChannelSetXfer (&glChHandleInterruptCPU2U, 0);

  /* Flush the Endpoint memory */
  CyU3PUsbFlushEp (CY_FX_HID_EP_INTR_IN);

  /* Update the status flag. */
  glIsAppActive = CyTrue;
  }
//}}}
//{{{
/* This function stops the HID application. This shall be called whenever
 * a RESET or DISCONNECT event is received from the USB host. The endpoints are
 * disabled and the DMA pipe is destroyed by this function. */
static void CyFxUsbHidAppStop() {

  line2 ("stop");

  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  CyU3PEpConfig_t epCfg;

  /* Update the flag. */
  glIsAppActive = CyFalse;

  /* Flush the endpoint memory */
  CyU3PUsbFlushEp (CY_FX_HID_EP_INTR_IN);

  /* Destroy the channel */
  CyU3PDmaChannelDestroy (&glChHandleInterruptCPU2U);

  /* Disable endpoints. */
  CyU3PMemSet ((uint8_t*)&epCfg, 0, sizeof (epCfg));
  epCfg.enable = CyFalse;

  /* Consumer Interrupt endpoint configuration. */
  apiRetStatus = CyU3PSetEpConfig (CY_FX_HID_EP_INTR_IN, &epCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
    CyFxAppErrorHandler (apiRetStatus);
    }

  /* De-Activate GPIO Block */
  apiRetStatus = CyU3PGpioDeInit();
  if (apiRetStatus != CY_U3P_SUCCESS) {
    CyU3PDebugPrint (4, "GPIO De-Init failed, Error code = %d\n", apiRetStatus);
    CyFxAppErrorHandler (apiRetStatus);
    }
  }
//}}}
//{{{
/* This is the callback function to handle the USB events. */
static void CyFxUsbHidAppUSBEventCallback (CyU3PUsbEventType_t evtype, uint16_t evdata) {

  switch (evtype) {
    case CY_U3P_USB_EVENT_SETCONF:
      /* Stop application before re-starting. */
      if (glIsAppActive)
        CyFxUsbHidAppStop();

      /* Start application. */
      CyFxUsbHidAppStart();
      break;

    case CY_U3P_USB_EVENT_RESET:
    case CY_U3P_USB_EVENT_DISCONNECT:
      /* Stop application. */
      if (glIsAppActive)
        CyFxUsbHidAppStop();
      break;

    default:
      break;
    }
  }
//}}}

//{{{
static CyBool_t CyFxUsbHidAppUSBSetupCallback (uint32_t setupdat0, uint32_t setupdat1) {

  CyBool_t isHandled = CyFalse;

  // Fast enumeration is used. Only requests addressed to the interface, class,
  // vendor and unknown control requests are received by this function.
  // This application does not support any class or vendor requests. */
  // Decode the fields from the setup request
  uint8_t bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
  uint8_t bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
  uint8_t bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
  uint8_t bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
  uint16_t wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
  uint16_t wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);

  if (bType == CY_U3P_USB_STANDARD_RQT) {
    // Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND) requests here.
    // It should be allowed to pass if the device is in configured state and failed otherwise
    if ((bTarget == CY_U3P_USB_TARGET_INTF) &&
        ((bRequest == CY_U3P_USB_SC_SET_FEATURE) || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) &&
        (wValue == 0)) {
      //{{{  handle
      if (glIsAppActive)
        CyU3PUsbAckSetup();
      else
        CyU3PUsbStall (0, CyTrue, CyFalse);
      isHandled = CyTrue;
      }
      //}}}

    if ((bTarget == CY_U3P_USB_TARGET_ENDPT) &&
        (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE) &&
        (wValue == CY_U3P_USBX_FS_EP_HALT)) {
      if (wIndex == CY_FX_HID_EP_INTR_IN) {
        //{{{  handle
        if (glIsAppActive) {
          CyU3PDmaChannelReset (&glChHandleInterruptCPU2U);
          CyU3PUsbFlushEp (CY_FX_HID_EP_INTR_IN);
          CyU3PUsbResetEp (CY_FX_HID_EP_INTR_IN);
          CyU3PUsbStall (wIndex, CyFalse, CyTrue);
          CyU3PUsbAckSetup();
          isHandled = CyTrue;
          }
        //}}}
        }
      }

    // Class specific descriptors such as HID Report descriptor need to handled by the callback
    bReqType = ((setupdat0 & CY_U3P_USB_VALUE_MASK) >> 24);
    if ((bRequest == CY_U3P_USB_SC_GET_DESCRIPTOR) && (bReqType == CY_FX_GET_REPORT_DESC)) {
      isHandled = CyTrue;
      if (CyU3PUsbSendEP0Data (0x1C, (uint8_t*)CyFxUSBReportDscr) != CY_U3P_SUCCESS)
        /* There was some error. We should try stalling EP0. */
        CyU3PUsbStall (0, CyTrue, CyFalse);
      }
    }
  else if (bType == CY_U3P_USB_CLASS_RQT) {
    /* Class Specific Request Handler */
    if (bRequest == CY_FX_HID_SET_IDLE) {
      CyU3PUsbAckSetup();
      glButtonPress = 0;
      isHandled = CyTrue;
      }
    }

  return isHandled;
  }
//}}}
//{{{
static CyBool_t CyFxUsbHidAppLPMRqtCallback (CyU3PUsbLinkPowerMode link_mode) {
// Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
//  whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
//  FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
//  to trigger an exit back to U0.
//  This application does not have any state in which we should not allow U1/U2 transitions; and therefore
//  the function always return CyTrue.

  return CyTrue;
  }
//}}}

//{{{
static void gpioInit() {

  CyU3PGpioClock_t gpioClock;
  gpioClock.fastClkDiv = 2;
  gpioClock.slowClkDiv = 16;
  gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
  gpioClock.clkSrc     = CY_U3P_SYS_CLK;
  gpioClock.halfDiv    = 0;
  CyU3PGpioInit (&gpioClock, CyFxHidGpioInterruptCallback);

  /* Configure GPIO CY_FX_BUTTON_GPIO to trigger interrupt on falling edge. */
  CyU3PDeviceGpioOverride (CY_FX_BUTTON_GPIO, CyTrue);
  CyU3PGpioSimpleConfig_t gpioConfig;
  gpioConfig.outValue    = CyFalse;
  gpioConfig.inputEn     = CyTrue;
  gpioConfig.driveLowEn  = CyFalse;
  gpioConfig.driveHighEn = CyFalse;
  gpioConfig.intrMode    = CY_U3P_GPIO_INTR_NEG_EDGE;
  CyU3PGpioSetSimpleConfig (CY_FX_BUTTON_GPIO, &gpioConfig);

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
//}}}
//{{{
static void debugInit() {

  CyU3PUartInit();

  CyU3PUartConfig_t uartConfig;
  CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
  uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
  uartConfig.stopBit  = CY_U3P_UART_ONE_STOP_BIT;
  uartConfig.parity   = CY_U3P_UART_NO_PARITY;
  uartConfig.txEnable = CyTrue;
  uartConfig.rxEnable = CyFalse;
  uartConfig.flowCtrl = CyFalse;
  uartConfig.isDma    = CyTrue;
  CyU3PUartSetConfig (&uartConfig, NULL);

  CyU3PUartTxSetBlockXfer (0xFFFFFFFF);

  CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);

  CyU3PDebugPreamble (CyFalse);
  }
//}}}
//{{{
// initializes USB Module, sets enumeration descriptors,
// does not start the bulk streaming and this is done only when * SET_CONF event is received
static void appInit() {

  CyU3PEventCreate (&glHidAppEvent);

  CyU3PUsbStart();

  CyU3PUsbRegisterSetupCallback (CyFxUsbHidAppUSBSetupCallback, CyTrue);
  CyU3PUsbRegisterLPMRequestCallback (CyFxUsbHidAppLPMRqtCallback);
  CyU3PUsbRegisterEventCallback (CyFxUsbHidAppUSBEventCallback);

  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t*)CyFxUSB30DeviceDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t*)CyFxUSB20DeviceDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t*)CyFxUSBBOSDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t*)CyFxUSBDeviceQualDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t*)CyFxUSBSSConfigDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t*)CyFxUSBHSConfigDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t*)CyFxUSBFSConfigDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t*)CyFxUSBStringLangIDDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t*)CyFxUSBManufactureDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t*)CyFxUSBProductDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 3, (uint8_t*)CyFxUSBSerialNumberDscr);

  // Connect the USB Pins with super speed operation enabled
  CyU3PConnectState (CyTrue, CyFalse);
  }
//}}}
//{{{
static void mouseThread (uint32_t input) {

  gpioInit();
  displayInit ("USB mouseHID");
  debugInit();
  appInit();

  uint32_t evMask = CY_FX_APP_GPIO_INTR_CB_EVENT_FLAG;
  for (;;) {
    uint32_t evStat;
    if (CyU3PEventGet (&glHidAppEvent, evMask, CYU3P_EVENT_OR_CLEAR, &evStat, CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS) {
      if ((evStat & CY_FX_APP_GPIO_INTR_CB_EVENT_FLAG) || glButtonPress) {
        while (glButtonPress) {
          /* Send Input Report */
          if (hidSendReport() == CY_U3P_SUCCESS)
            glButtonPress--;
          else
            CyU3PUsbStall (CY_FX_HID_EP_INTR_IN, CyTrue, CyFalse);
          }
        }
      }
    }
  }
//}}}

//{{{
/* Application define function which creates the threads. */
void CyFxApplicationDefine() {

  CyU3PThreadCreate (
            &UsbHidAppThread,                                   /* App Thread structure */
            "21:FX3_HID_MOUSE",                                 /* Thread ID and Thread name */
            mouseThread,                                        /* App Thread Entry function */
            0,                                                  /* No input parameter to thread */
            CyU3PMemAlloc (CY_FX_HID_THREAD_STACK),             /* Pointer to the allocated thread stack */
            CY_FX_HID_THREAD_STACK,                             /* App Thread stack size */
            CY_FX_HID_THREAD_PRIORITY,                          /* App Thread priority */
            CY_FX_HID_THREAD_PRIORITY,                          /* App Thread priority */
            CYU3P_NO_TIME_SLICE,                                /* No time slice for the application thread */
            CYU3P_AUTO_START                                    /* Start the Thread immediately */
            );
  }
//}}}

//{{{
int main() {

  CyU3PDeviceInit (0);

  // enable both Instruction and Data Caches
  CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);

  CyU3PIoMatrixConfig_t io_cfg;
  io_cfg.isDQ32Bit = CyFalse;
  io_cfg.s0Mode    = CY_U3P_SPORT_INACTIVE;
  io_cfg.s1Mode    = CY_U3P_SPORT_INACTIVE;
  io_cfg.useUart   = CyTrue;
  io_cfg.useI2C    = CyFalse;
  io_cfg.useI2S    = CyFalse;
  io_cfg.useSpi    = CyTrue;
  io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
  io_cfg.gpioSimpleEn[0]  = 0;
  io_cfg.gpioSimpleEn[1]  = 0;
  io_cfg.gpioComplexEn[0] = 0;
  io_cfg.gpioComplexEn[1] = 0;
  CyU3PDeviceConfigureIOMatrix (&io_cfg);

  CyU3PKernelEntry();

  return 0;
  }
//}}}
