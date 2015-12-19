//{{{
/* This file illustrates USB Audio Class application example (streaming audio data from the SPI Flash) */

/*
   This sample application implements a USB audio class Driver with the help of the appropriate USB
   enumeration descriptors. With these descriptors, the FX3 device enumerates as a USB Audio Class
   device on the USB host. The application implementes the USB Device Class Definition for Audio
   Devices Release 1.0

   The sample application illustrates USB Audio Class application example (streaming audio data from
   the SPI Flash). The application pulls out the audio data stored in the SPI Flash and streams it over
   the Isochronous endpoint to the USB Host. The application supports stereo channel, PCM data with a
   48 KHz sampling frequncy, and 16-bits per sample. The audio streaming is accomplished by having two
   separate DMA MANUAL channels. The SPI Flash to CPU DMA MANUAL_IN channel is used to read audio data
   from the SPI flash. The firmware then breaks up the data received such that it fits into the USB
   Isochronous packets. The CPU to USB DMA MANUAL_OUT channel is used to push audio data onto the USB
   Host.

   In the case of USB High Speed and Super Speed the isochronous service interval is set to 4 frames
   with 96 bytes of audio data transferred every 4 microframes.

   In the case of USB Full Speed 192 bytes of audio data are sent every frame.
 */
//}}}
//{{{  includes
#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3usbconst.h>
#include <cyu3uart.h>
#include <cyu3gpio.h>
#include <cyu3spi.h>
#include <cyu3utils.h>
//}}}
#define RESET_GPIO        22  // CTL 5 pin
//{{{  defines
/* Setup data field : Request */
#define CY_U3P_USB_REQUEST_MASK                         (0x0000FF00)
#define CY_U3P_USB_REQUEST_POS                          (8)

/* Iscochronous Transfer length */
#define CY_FX3_ISO_XFER_LEN                             (96)
/* Give a timeout value of 5s for any flash programming. */
#define CY_FX_USB_SPI_TIMEOUT                           (5000)
/* Size in pages of the Audio sample to be streamed from the SPI Flash. */
#define CY_FX3_AUDIO_SAMPLE_SIZE_IN_PAGES               (0x3CD)

#define CY_FX_UAC_APP_THREAD_STACK     (0x1000)        /* Thread stack size */
#define CY_FX_UAC_APP_THREAD_PRIORITY  (8)             /* Thread priority */

/* Endpoint definition for UAC application */
#define CY_FX_EP_ISO_AUDIO             (0x81)          /* EP 1 IN */
#define CY_FX_EP_AUDIO_CONS_SOCKET     (CY_U3P_UIB_SOCKET_CONS_1) /* Consumer socket 1 */

/* UAC streaming endpoint packet Count */
#define CY_FX_EP_ISO_AUDIO_PKTS_COUNT  (0x01)

/* UAC Buffer count */
#define CY_FX_UAC_STREAM_BUF_COUNT     (16)

#define CY_FX_UAC_STREAM_INTERFACE     (uint8_t)(1)   /* Streaming Interface : Alternate setting 1 */
//}}}
//{{{  descriptors
//{{{
/* Standard device descriptor for USB 3.0 */
const uint8_t CyFxUsb30DeviceDscr[] __attribute__ ((aligned (32))) =
{
    0x12,                           /* Descriptor size */
    CY_U3P_USB_DEVICE_DESCR,        /* Device descriptor type */
    0x00,0x03,                      /* USB 3.0 */
    0x00,                           /* Device class */
    0x00,                           /* Device Sub-class */
    0x00,                           /* Device protocol */
    0x09,                           /* Maxpacket size for EP0 : 2^9 */
    0xB4,0x04,                      /* Vendor ID  = 0x04B4 */
    0x22,0x47,                      /* Product ID = 0x4722 */
    0x01,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};
//}}}
//{{{
/* Standard device descriptor */
const uint8_t CyFxUsb20DeviceDscr[] __attribute__ ((aligned (32))) =
{
    0x12,                           /* Descriptor size */
    CY_U3P_USB_DEVICE_DESCR,        /* Device descriptor type */
    0x10,0x02,                      /* USB 2.10 */
    0x00,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0xB4,0x04,                      /* Vendor ID  = 0x04B4 */
    0x22,0x47,                      /* Product ID = 0x4722 */
    0x01,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};
//}}}
//{{{
/* Binary device object store descriptor */
const uint8_t CyFxUsbBOSDscr[] __attribute__ ((aligned (32))) =
{
    0x05,                           /* Descriptor size */
    CY_U3P_BOS_DESCR,               /* Device descriptor type */
    0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of device capability descriptors */

    /* USB 2.0 Extension */
    0x07,                           /* Descriptor size */
    CY_U3P_DEVICE_CAPB_DESCR,       /* Device capability type descriptor */
    CY_U3P_USB2_EXTN_CAPB_TYPE,     /* USB 2.0 extension capability type */
    0x02,0x00,0x00,0x00,            /* Supported device level features: LPM support  */

    /* SuperSpeed Device Capability */
    0x0A,                           /* Descriptor size */
    CY_U3P_DEVICE_CAPB_DESCR,       /* Device capability type descriptor */
    CY_U3P_SS_USB_CAPB_TYPE,        /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features  */
    0x0E,0x00,                      /* Speeds supported by the device : SS, HS and FS */
    0x03,                           /* Functionality support */
    0x00,                           /* U1 device exit latency */
    0x00,0x00                       /* U2 device exit latency */
};
//}}}
//{{{
/* Standard device qualifier descriptor */
const uint8_t CyFxUsbDeviceQualDscr[] __attribute__ ((aligned (32))) =
{
    0x0A,                           /* descriptor size */
    CY_U3P_USB_DEVQUAL_DESCR,       /* Device qualifier descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0xEF,                           /* Device class */
    0x02,                           /* Device sub-class */
    0x01,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0x01,                           /* Number of configurations */
    0x00                            /* Reserved */
};
//}}}

//{{{
/* Standard super speed configuration descriptor */
const uint8_t CyFxUsbSSConfigDscr[] __attribute__ ((aligned (32))) =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_CONFIG_DESCR,        /* Configuration descriptor type */
    0x6A,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
    0x80,                           /* Config characteristics - Bus powered */
    0x32,                           /* Max power consumption of device (in 8mA unit) : 400mA */

    /* Standard Audio Control Interface Descriptor */
    0x09,                           /* Descriptor Size */
    0x04,                           /* Interface Descriptor Type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting */
    0x00,                           /* Number of endpoints - 0 endpoints */
    0x01,                           /* Interface Class - Audio */
    0x01,                           /* Interface SubClass - Audio Control */
    0x00,                           /* Interface Protocol - Unused */
    0x00,                           /* Interface string index */

    /* Class Specific Audio Control Interface Descriptor */
    0x09,                           /* Descriptor Size */
    0x24,                           /* Descriptor Type - CS_INTERFACE */
    0x01,                           /* Descriptor SubType - Header */
    0x00, 0x01,                     /* Revision of class specification - 1.0 */
    0x1E, 0x00,                     /* Total size of class specific descriptors */
    0x01,                           /* Number of streaming Interfaces - 1 */
    0x01,                           /* Audio Streaming interface 1 belongs to this AudioControl Interface */

    /* Input terminal descriptor */
    0x0C,                           /* Descriptor size in bytes */
    0x24,                           /* CS Interface Descriptor */
    0x02,                           /* Input Terminal Descriptor subtype */
    0x01,                           /* ID Of the input terminal */
    0x01, 0x02,                     /* Microphone - terminal type */
    0x00,                           /* Association terminal - None */
    0x02,                           /* Number of channels - 2 */
    0x03, 0x00,                     /* spatial location of the logical channels - Front Left and Front Right */
    0x00,                           /* Channel names - Unused */
    0x00,                           /* String index for this descriptor - None */

    /* Output terminal descriptor */
    0x09,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x03,                           /* Output terminal descriptor type */
    0x02,                           /* ID of this terminal */
    0x01, 0x01,                     /* Output terminal type: USB Streaming */
    0x00,                           /* Association terminal - Unused */
    0x01,                           /* Id of the terminal/unit to which this is connected - Input terminal id */
    0x00,                           /* String desc index : Not used */

    /* Standard Audio Streaming Interface Descriptor (Alternate setting 0) */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface descriptor type */
    0x01,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x00,                           /* Number of end points : zero bandwidth */
    0x01,                           /* Interface class : Audio */
    0x02,                           /* Interface sub class : Audio Streaming */
    0x00,                           /* Interface protocol code : Unused */
    0x00,                           /* Interface descriptor string index */

    /* Standard Audio Streaming Interface descriptor (Alternate setting 1) */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface descriptor type */
    0x01,                           /* Interface number */
    0x01,                           /* Alternate setting number */
    0x01,                           /* Number of end points : 1 ISO EP */
    0x01,                           /* Interface Audio class */
    0x02,                           /* Interface Audio sub class - Audio Streaming */
    0x00,                           /* Interface protocol code : Unused */
    0x00,                           /* Interface descriptor string index */

    /* Class-specific Audio Streaming General Interface descriptor */
    0x07,                           /* Descriptor size */
    0x24,                           /* Class-specific AS i/f Type */
    0x01,                           /* Descriptotor subtype : AS General */
    0x02,                           /* Terminal Link - Output terminal id */
    0x01,                           /* Interface delay */
    0x01, 0x00,                     /* Audio data format - PCM */

    /* Class specific AS Format descriptor - Type I Format Descriptor */
    0x0B,                           /* Descriptor size */
    0x24,                           /* Class-specific Interface Descriptor Type */
    0x02,                           /* Format Type Descriptor subtype */
    0x01,                           /* PCM FORMAT_TYPE_I */
    0x02,                           /* Number of channels - 2 */
    0x02,                           /* Subframe size - 2 bytes per audio subframe */
    0x10,                           /* Bit resolution - 16 bits */
    0x01,                           /* Number of samping frequencies - 1 */
    0x80, 0xBB, 0x00,               /* Sampling frequency - 48000 Hz */

    /* Endpoint descriptor for ISO streaming Audio data */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
    CY_FX_EP_ISO_AUDIO,             /* Endpoint address and description */
    0x05,                           /* ISO End point : Async */
    0x60, 0x00,                     /* Transaction size - 96 bytes */
    0x03,                           /* Servicing interval for data transfers */
    0x00,                           /* bRefresh */
    0x00,                           /* bSynchAddress */

    /* Super speed endpoint companion descriptor */
    0x06,                           /* Descriptor size */
    CY_U3P_SS_EP_COMPN_DESCR,       /* SS endpoint companion descriptor type */
    0x00,                           /* Max no. of packets in a burst : 1 */
    0x00,                           /* Mult.: Max number of packets : 1 */
    0x60, 0x00,                     /* Bytes per interval : 1024 */

    /* Class Specific AS Isochronous Audio Data Endpoint Descriptor */
    0x07,                           /* Descriptor size in bytes */
    0x25,                           /* CS_ENDPOINT descriptor type */
    0x01,                           /* EP_GENERAL sub-descriptor type */
    0x00,                           /* bmAttributes - None  */
    0x00,                           /* bLockDelayUnits - Unused */
    0x00, 0x00                      /* wLockDelay - unused */
};
//}}}
//{{{
/* Standard High Speed Configuration Descriptor */
const uint8_t CyFxUsbHSConfigDscr[] __attribute__ ((aligned (32))) =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_CONFIG_DESCR,        /* Configuration descriptor type */
    0x64, 0x00,                     /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
    0x80,                           /* Config characteristics - bus powered */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

    /* Standard Audio Control Interface Descriptor */
    0x09,                           /* Descriptor Size */
    0x04,                           /* Interface Descriptor Type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting */
    0x00,                           /* Number of endpoints - 0 endpoints */
    0x01,                           /* Interface Class - Audio */
    0x01,                           /* Interface SubClass - Audio Control */
    0x00,                           /* Interface Protocol - Unused */
    0x00,                           /* Interface string index */

    /* Class Specific Audio Control Interface Descriptor */
    0x09,                           /* Descriptor Size */
    0x24,                           /* Descriptor Type - CS_INTERFACE */
    0x01,                           /* Descriptor SubType - Header */
    0x00, 0x01,                     /* Revision of class specification - 1.0 */
    0x1E, 0x00,                     /* Total size of class specific descriptors */
    0x01,                           /* Number of streaming Interfaces - 1 */
    0x01,                           /* Audio Streaming interface 1 belongs to this AudioControl Interface */

    /* Input terminal descriptor */
    0x0C,                           /* Descriptor size in bytes */
    0x24,                           /* CS Interface Descriptor */
    0x02,                           /* Input Terminal Descriptor subtype */
    0x01,                           /* ID Of the input terminal */
    0x01, 0x02,                     /* Microphone - terminal type */
    0x00,                           /* Association terminal - None */
    0x02,                           /* Number of channels - 2 */
    0x03, 0x00,                     /* spatial location of the logical channels - Front Left and Front Right */
    0x00,                           /* Channel names - Unused */
    0x00,                           /* String index for this descriptor - None */

    /* Output terminal descriptor */
    0x09,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x03,                           /* Output terminal descriptor type */
    0x02,                           /* ID of this terminal */
    0x01, 0x01,                     /* Output terminal type: USB Streaming */
    0x00,                           /* Association terminal - Unused */
    0x01,                           /* Id of the terminal/unit to which this is connected - Input terminal id */
    0x00,                           /* String desc index : Not used */

    /* Standard Audio Streaming Interface Descriptor (Alternate setting 0) */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface descriptor type */
    0x01,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x00,                           /* Number of end points : zero bandwidth */
    0x01,                           /* Interface class : Audio */
    0x02,                           /* Interface sub class : Audio Streaming */
    0x00,                           /* Interface protocol code : Unused */
    0x00,                           /* Interface descriptor string index */

    /* Standard Audio Streaming Interface descriptor (Alternate setting 1) */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface descriptor type */
    0x01,                           /* Interface number */
    0x01,                           /* Alternate setting number */
    0x01,                           /* Number of end points : 1 ISO EP */
    0x01,                           /* Interface Audio class */
    0x02,                           /* Interface Audio sub class - Audio Streaming */
    0x00,                           /* Interface protocol code : Unused */
    0x00,                           /* Interface descriptor string index */

    /* Class-specific Audio Streaming General Interface descriptor */
    0x07,                           /* Descriptor size */
    0x24,                           /* Class-specific AS i/f Type */
    0x01,                           /* Descriptotor subtype : AS General */
    0x02,                           /* Terminal Link - Output terminal id */
    0x01,                           /* Interface delay */
    0x01, 0x00,                     /* Audio data format - PCM */

    /* Class specific AS Format descriptor - Type I Format Descriptor */
    0x0B,                           /* Descriptor size */
    0x24,                           /* Class-specific Interface Descriptor Type */
    0x02,                           /* Format Type Descriptor subtype */
    0x01,                           /* PCM FORMAT_TYPE_I */
    0x02,                           /* Number of channels - 2 */
    0x02,                           /* Subframe size - 2 bytes per audio subframe */
    0x10,                           /* Bit resolution - 16 bits */
    0x01,                           /* Number of samping frequencies - 1 */
    0x80, 0xBB, 0x00,               /* Sampling frequency - 48000 Hz */

    /* Endpoint descriptor for ISO streaming Audio data */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
    0x81,                           /* Endpoint address and description */
    0x05,                           /* ISO End point : Async */
    0x60, 0x00,                     /* Transaction size - 96 bytes per frame */
    0x03,                           /* Servicing interval for data transfers */
    0x00,                           /* bRefresh */
    0x00,                           /* bSynchAddress */

    /* Class Specific AS Isochronous Audio Data Endpoint Descriptor */
    0x07,                           /* Descriptor size in bytes */
    0x25,                           /* CS_ENDPOINT descriptor type */
    0x01,                           /* EP_GENERAL sub-descriptor type */
    0x00,                           /* bmAttributes - None  */
    0x00,                           /* bLockDelayUnits - Unused */
    0x00, 0x00                      /* wLockDelay - unused */
};
//}}}
//{{{
/* Standard full speed configuration descriptor : full speed is not supported. */
const uint8_t CyFxUsbFSConfigDscr[] __attribute__ ((aligned (32))) =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_CONFIG_DESCR,        /* Configuration descriptor type */
    0x64,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
    0x80,                           /* Config characteristics - bus powered */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

    /* Standard Audio Control Interface Descriptor */
    0x09,                           /* Descriptor Size */
    0x04,                           /* Interface Descriptor Type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting */
    0x00,                           /* Number of endpoints - 0 endpoints */
    0x01,                           /* Interface Class - Audio */
    0x01,                           /* Interface SubClass - Audio Control */
    0x00,                           /* Interface Protocol - Unused */
    0x00,                           /* Interface string index */

    /* Class Specific Audio Control Interface Descriptor */
    0x09,                           /* Descriptor Size */
    0x24,                           /* Descriptor Type - CS_INTERFACE */
    0x01,                           /* Descriptor SubType - Header */
    0x00, 0x01,                     /* Revision of class specification - 1.0 */
    0x1E, 0x00,                     /* Total size of class specific descriptors */
    0x01,                           /* Number of streaming Interfaces - 1 */
    0x01,                           /* Audio Streaming interface 1 belongs to this AudioControl Interface */

    /* Input terminal descriptor */
    0x0C,                           /* Descriptor size in bytes */
    0x24,                           /* CS Interface Descriptor */
    0x02,                           /* Input Terminal Descriptor subtype */
    0x01,                           /* ID Of the input terminal */
    0x01, 0x02,                     /* Microphone - terminal type */
    0x00,                           /* Association terminal - None */
    0x02,                           /* Number of channels - 2 */
    0x03, 0x00,                     /* spatial location of the logical channels - Front Left and Front Right */
    0x00,                           /* Channel names - Unused */
    0x00,                           /* String index for this descriptor - None */

    /* Output terminal descriptor */
    0x09,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x03,                           /* Output terminal descriptor type */
    0x02,                           /* ID of this terminal */
    0x01, 0x01,                     /* Output terminal type: USB Streaming */
    0x00,                           /* Association terminal - Unused */
    0x01,                           /* Id of the terminal/unit to which this is connected - Input terminal id */
    0x00,                           /* String desc index : Not used */

    /* Standard Audio Streaming Interface Descriptor (Alternate setting 0) */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface descriptor type */
    0x01,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x00,                           /* Number of end points : zero bandwidth */
    0x01,                           /* Interface class : Audio */
    0x02,                           /* Interface sub class : Audio Streaming */
    0x00,                           /* Interface protocol code : Unused */
    0x00,                           /* Interface descriptor string index */

    /* Standard Audio Streaming Interface descriptor (Alternate setting 1) */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_INTRFC_DESCR,        /* Interface descriptor type */
    0x01,                           /* Interface number */
    0x01,                           /* Alternate setting number */
    0x01,                           /* Number of end points : 1 ISO EP */
    0x01,                           /* Interface Audio class */
    0x02,                           /* Interface Audio sub class - Audio Streaming */
    0x00,                           /* Interface protocol code : Unused */
    0x00,                           /* Interface descriptor string index */

    /* Class-specific Audio Streaming General Interface descriptor */
    0x07,                           /* Descriptor size */
    0x24,                           /* Class-specific AS i/f Type */
    0x01,                           /* Descriptotor subtype : AS General */
    0x02,                           /* Terminal Link - Output terminal id */
    0x01,                           /* Interface delay */
    0x01, 0x00,                     /* Audio data format - PCM */

    /* Class specific AS Format descriptor - Type I Format Descriptor */
    0x0B,                           /* Descriptor size */
    0x24,                           /* Class-specific Interface Descriptor Type */
    0x02,                           /* Format Type Descriptor subtype */
    0x01,                           /* PCM FORMAT_TYPE_I */
    0x02,                           /* Number of channels - 2 */
    0x02,                           /* Subframe size - 4 bytes per audio subframe */
    0x10,                           /* Bit resolution - 32 bits */
    0x01,                           /* Number of samping frequencies - 1 */
    0x80, 0xBB, 0x00,               /* Sampling frequency - 48000 Hz */

    /* Endpoint descriptor for ISO streaming Audio data */
    0x09,                           /* Descriptor size */
    CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
    CY_FX_EP_ISO_AUDIO,             /* Endpoint address and description */
    0x05,                           /* ISO End point : Async */
    0xC0, 0x00,                     /* Transaction size - 192 bytes per frame */
    0x01,                           /* Servicing interval for data transfers */
    0x00,                           /* bRefresh */
    0x00,                           /* bSynchAddress */

    /* Class Specific AS Isochronous Audio Data Endpoint Descriptor */
    0x07,                           /* Descriptor size in bytes */
    0x25,                           /* CS_ENDPOINT descriptor type */
    0x01,                           /* EP_GENERAL sub-descriptor type */
    0x00,                           /* bmAttributes - None  */
    0x00,                           /* bLockDelayUnits - Unused */
    0x00, 0x00                      /* wLockDelay - unused */
};
//}}}

//{{{
/* Standard language ID string descriptor */
const uint8_t CyFxUsbStringLangIDDscr[] __attribute__ ((aligned (32))) =
{
    0x04,                           /* Descriptor size */
    CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
    0x09,0x04                       /* Language ID supported */
};
//}}}
//{{{
/* Standard manufacturer string descriptor */
const uint8_t CyFxUsbManufactureDscr[] __attribute__ ((aligned (32))) =
{
    0x10,                           /* Descriptor size */
    CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
    'C',0x00, 'y',0x00, 'p',0x00, 'r',0x00, 'e',0x00, 's',0x00, 's',0x00
};
//}}}
//{{{
/* Standard product string descriptor */
const uint8_t CyFxUsbProductDscr[] __attribute__ ((aligned (32))) =
{
    0x08,                           /* Descriptor Size */
    CY_U3P_USB_STRING_DESCR,        /* Device descriptor type */
    'F',0x00, 'X',0x00, '3',0x00
};
//}}}

/* Place this buffer as the last buffer so that no other variable / code shares
 * the same cache line. Do not add any other variables / arrays in this file.
 * This will lead to variables sharing the same cache line. */
const uint8_t CyFxUsbDscrAlignBuffer[32] __attribute__ ((aligned (32)));
//}}}
//{{{  vars
CyU3PThread uacThread;                /* Thread structure */

CyBool_t glIsApplnActive = CyFalse;   /* Whether the loopback application is active or not. */
CyU3PDmaChannel glUacStreamHandle;    /* DMA Channel Handle  */
CyU3PDmaChannel glUacSpiRxHandle;     /* SPI Rx channel handle */

uint16_t glSpiPageSize = 0x100;      /* SPI Page size to be used for transfers. */
//}}}

//{{{
static void CyFxAppErrorHandler (CyU3PReturnStatus_t apiRetStatus) {

  for (;;)
    CyU3PThreadSleep (100);
  }
//}}}

//{{{
/* This function starts the audio streaming application. It is called
 * when there is a SET_INTERFACE event for alternate interface 1. */
static CyU3PReturnStatus_t CyFxUacApplnStart() {

  CyU3PEpConfig_t epCfg;
  CyU3PDmaChannelConfig_t dmaCfg;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  /* Audio streaming endpoint configuration */
  epCfg.enable = CyTrue;
  epCfg.epType = CY_U3P_USB_EP_ISO;
  epCfg.pcktSize = CY_FX3_ISO_XFER_LEN;
  epCfg.isoPkts = 1;
  epCfg.burstLen = 1;
  epCfg.streams = 0;

  apiRetStatus = CyU3PSetEpConfig (CY_FX_EP_ISO_AUDIO, &epCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error Code = %d\n", apiRetStatus);
    return apiRetStatus;
    }

  /* Create a DMA Manual OUT channel for streaming data */
  /* Audio streaming Channel is not active till a stream request is received */
  dmaCfg.size = CY_FX3_ISO_XFER_LEN;
  dmaCfg.count = CY_FX_UAC_STREAM_BUF_COUNT;
  dmaCfg.prodSckId = CY_U3P_CPU_SOCKET_PROD;
  dmaCfg.consSckId = CY_FX_EP_AUDIO_CONS_SOCKET;
  dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
  dmaCfg.cb = NULL;
  dmaCfg.prodHeader = 0;
  dmaCfg.prodFooter = 0;
  dmaCfg.consHeader = 0;
  dmaCfg.prodAvailCount = 0;
  apiRetStatus = CyU3PDmaChannelCreate (&glUacStreamHandle, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaCfg);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, error code = %d\n",apiRetStatus);
    return apiRetStatus;
    }

  /* Flush the endpoint memory */
  CyU3PUsbFlushEp (CY_FX_EP_ISO_AUDIO);

  apiRetStatus = CyU3PDmaChannelSetXfer (&glUacStreamHandle, 0);
  if (apiRetStatus != CY_U3P_SUCCESS) {
    CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer failed, error code = %d\n", apiRetStatus);
    return apiRetStatus;
    }

  /* Update the flag so that the application thread is notified of this. */
  glIsApplnActive = CyTrue;

  return CY_U3P_SUCCESS;
  }
//}}}
//{{{
/* This function stops the audio streaming. It is called from the USB event
 * handler, when there is a reset / disconnect or SET_INTERFACE for alternate
 * interface 0. */
static void CyFxUacApplnStop() {

  CyU3PEpConfig_t epCfg;

  /* Update the flag so that the application thread is notified of this. */
  glIsApplnActive = CyFalse;

  /* Reset the SPI Read Data channel Handle */
  CyU3PDmaChannelReset (&glUacSpiRxHandle);

  /* Abort and destroy the Audio streaming channel */
  CyU3PDmaChannelDestroy (&glUacStreamHandle);

  /* Flush the endpoint memory */
  CyU3PUsbFlushEp (CY_FX_EP_ISO_AUDIO);

  /* Disable the audio streaming endpoint. */
  CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
  epCfg.enable = CyFalse;

  CyU3PSetEpConfig (CY_FX_EP_ISO_AUDIO, &epCfg);
  }
//}}}

//{{{
/* This is the Callback function to handle the USB Events */
static void CyFxUacApplnUsbEventCbk (CyU3PUsbEventType_t evtype, uint16_t  evdata ) {

  uint8_t interface = 0, altSetting = 0;

  switch (evtype) {
    case CY_U3P_USB_EVENT_SETINTF:
      /* Start the audio streamer application if the nterface requested was 1. If not, stop the streamer. */
      interface = CY_U3P_GET_MSB (evdata);
      altSetting = CY_U3P_GET_LSB (evdata);

      if ((altSetting == CY_FX_UAC_STREAM_INTERFACE) && (interface == 1)) {
          /* Stop the application before re-starting. */
          if (glIsApplnActive)
              CyFxUacApplnStop ();
          CyFxUacApplnStart ();
          break;
      }
      /* Fall-through. */

    case CY_U3P_USB_EVENT_SETCONF:
    case CY_U3P_USB_EVENT_RESET:
    case CY_U3P_USB_EVENT_DISCONNECT:
      /* Stop the audio streamer application. */
      if (glIsApplnActive)
        CyFxUacApplnStop ();
      break;

    default:
        break;
    }
  }
//}}}
//{{{
/* Callback to handle the USB Setup Requests and UAC Class requests */
static CyBool_t CyFxUacApplnUSBSetupCB (uint32_t setupdat0, uint32_t setupdat1 ) {

  CyBool_t isHandled = CyFalse;

  /* Fast enumeration is used. Only requests addressed to the interface, class,
   * vendor and unknown control requests are received by this function */
  /* Decode the fields from the setup request. */
  uint8_t  bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
  uint8_t  bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
  uint8_t  bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
  uint8_t  bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
  uint16_t wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);

  if (bType == CY_U3P_USB_STANDARD_RQT) {
    /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
     * requests here. It should be allowed to pass if the device is in configured
     * state and failed otherwise. */
    if ((bTarget == CY_U3P_USB_TARGET_INTF) &&
        ((bRequest == CY_U3P_USB_SC_SET_FEATURE) || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) &&
        (wValue == 0)) {
      if (glIsApplnActive)
        CyU3PUsbAckSetup ();
      else
        CyU3PUsbStall (0, CyTrue, CyFalse);

      isHandled = CyTrue;
      }
    }

  /* Check for UAC Class Requests */
  if (bType == CY_U3P_USB_CLASS_RQT)
    while (bType == bType)
      CyU3PBusyWait (10);

  return isHandled;
  }
//}}}
//{{{
/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
 * whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function,
 * the FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately
 * tries to trigger an exit back to U0.
 * This application does not have any state in which we should not allow U1/U2 transitions; and therefore
 * the function always return CyTrue.
 * */
static CyBool_t CyFxUacApplnLPMRqtCbk (CyU3PUsbLinkPowerMode link_mode) {

  return CyTrue;
  }
//}}}

//{{{
/* Wait for the status response from the SPI flash. */
static CyU3PReturnStatus_t CyUacFxSpiWaitForStatus() {

  uint8_t buf[2], rd_buf[2];
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  /* Wait for status response from SPI flash device. */
  do {
    buf[0] = 0x06;  /* Write enable command. */

    CyU3PSpiSetSsnLine (CyFalse);
    status = CyU3PSpiTransmitWords (buf, 1);
    CyU3PSpiSetSsnLine (CyTrue);
    if (status != CY_U3P_SUCCESS) {
      CyU3PDebugPrint (2, "SPI WR_ENABLE command failed\n\r");
      return status;
      }

    buf[0] = 0x05;  /* Read status command */

    CyU3PSpiSetSsnLine (CyFalse);
    status = CyU3PSpiTransmitWords (buf, 1);
    if (status != CY_U3P_SUCCESS) {
      CyU3PDebugPrint (2, "SPI READ_STATUS command failed\n\r");
      CyU3PSpiSetSsnLine (CyTrue);
      return status;
      }

    status = CyU3PSpiReceiveWords (rd_buf, 2);
    CyU3PSpiSetSsnLine (CyTrue);
    if(status != CY_U3P_SUCCESS) {
      CyU3PDebugPrint (2, "SPI status read failed\n\r");
      return status;
      }

    } while ((rd_buf[0] & 1)|| (!(rd_buf[0] & 0x2)));

  return CY_U3P_SUCCESS;
  }
//}}}
//{{{
/* SPI read / write for programmer application. */
static CyU3PReturnStatus_t CyFxUacSpiTransfer (uint16_t pageAddress, uint16_t byteCount, uint8_t* buffer, CyBool_t  isRead) {

  CyU3PDmaBuffer_t buf_p;
  uint8_t location[4];
  uint32_t byteAddress = 0;
  uint16_t pageCount = (byteCount / glSpiPageSize);
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  if (byteCount == 0)
    return CY_U3P_SUCCESS;
  if ((byteCount % glSpiPageSize) != 0)
    pageCount ++;

  buf_p.buffer = buffer;
  buf_p.status = 0;

  byteAddress  = pageAddress * glSpiPageSize;
  CyU3PDebugPrint (2, "SPI access - addr: 0x%x, size: 0x%x, pages: 0x%x.\r\n", byteAddress, byteCount, pageCount);

  while (pageCount != 0) {
    location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
    location[2] = (byteAddress >> 8) & 0xFF;
    location[3] = byteAddress & 0xFF;               /* LS byte */

    if (isRead) {
      location[0] = 0x03; /* Read command. */
      buf_p.size  = glSpiPageSize;
      buf_p.count = glSpiPageSize;

      status = CyUacFxSpiWaitForStatus ();
      if (status != CY_U3P_SUCCESS)
        return status;

      CyU3PSpiSetSsnLine (CyFalse);
      status = CyU3PSpiTransmitWords (location, 4);
      if (status != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (2, "SPI READ command failed\r\n");
        CyU3PSpiSetSsnLine (CyTrue);
        return status;
        }

      CyU3PSpiSetBlockXfer (0, glSpiPageSize);

      status = CyU3PDmaChannelSetupRecvBuffer (&glUacSpiRxHandle, &buf_p);
      if (status != CY_U3P_SUCCESS) {
        CyU3PSpiSetSsnLine (CyTrue);
        return status;
        }
      status = CyU3PDmaChannelWaitForCompletion (&glUacSpiRxHandle, CY_FX_USB_SPI_TIMEOUT);
      if (status != CY_U3P_SUCCESS) {
        CyU3PSpiSetSsnLine (CyTrue);
        return status;
        }

      CyU3PSpiSetSsnLine (CyTrue);
      CyU3PSpiDisableBlockXfer (CyFalse, CyTrue);
      }

     /* Update the parameters */
    byteAddress  += glSpiPageSize;
    buf_p.buffer += glSpiPageSize;
    pageCount --;
    }

  return CY_U3P_SUCCESS;
  }
//}}}
//{{{
/* SPI initialization for application. */
static CyU3PReturnStatus_t CyFxUacSpiInit (uint16_t pageLen) {

  CyU3PSpiConfig_t spiConfig;
  CyU3PDmaChannelConfig_t dmaConfig;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  /* Start the SPI module and configure the master. */
  status = CyU3PSpiInit ();
  if (status != CY_U3P_SUCCESS)
      return status;

  /* Start the SPI master block. Run the SPI clock at 8MHz
   * and configure the word length to 8 bits. Also configure the slave select using FW. */
  CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
  spiConfig.isLsbFirst = CyFalse;
  spiConfig.cpol       = CyTrue;
  spiConfig.ssnPol     = CyFalse;
  spiConfig.cpha       = CyTrue;
  spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
  spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
  spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
  spiConfig.clock      = 10000000;
  spiConfig.wordLen    = 8;

  status = CyU3PSpiSetConfig (&spiConfig, NULL);
  if (status != CY_U3P_SUCCESS)
    return status;

  /* Create the DMA channels for SPI write and read. */
  CyU3PMemSet ((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));
  dmaConfig.size           = pageLen;
  /* No buffers need to be allocated as this channel * will be used only in override mode. */
  dmaConfig.count          = 0;
  dmaConfig.prodAvailCount = 0;
  dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
  dmaConfig.prodHeader     = 0;
  dmaConfig.prodFooter     = 0;
  dmaConfig.consHeader     = 0;
  dmaConfig.notification   = 0;
  dmaConfig.cb             = NULL;

  /* Channel to read from SPI flash. */
  dmaConfig.prodSckId = CY_U3P_LPP_SOCKET_SPI_PROD;
  dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
  status = CyU3PDmaChannelCreate (&glUacSpiRxHandle, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);

  if (status == CY_U3P_SUCCESS)
      glSpiPageSize = pageLen;
  return status;
  }
//}}}

//{{{
static void gpioInit() {
// define GPIO for reset

  // GPIO clock init
  CyU3PGpioClock_t gpioClock;
  gpioClock.fastClkDiv = 2;
  gpioClock.slowClkDiv = 2;
  gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
  gpioClock.clkSrc     = CY_U3P_SYS_CLK;
  gpioClock.halfDiv    = 0;
  CyU3PGpioInit (&gpioClock, NULL);

  // GPIO ctl pins config
  CyU3PDeviceGpioOverride (RESET_GPIO, CyTrue);
  CyU3PGpioSimpleConfig_t gpioConfig;
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
/* This function initializes the debug module for the UAC application */
static void debugInit() {

  CyU3PUartConfig_t uartConfig;

  CyU3PUartInit();

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
  }
//}}}
//{{{
static void appInit() {

  /* Initialize the SPI interface for flash of page size 256 bytes. */
  CyFxUacSpiInit (0x100);

  /* Start the USB functionality */
  CyU3PUsbStart ();

  /* The fast enumeration is the easiest way to setup a USB connection,
   * where all enumeration phase is handled by the library. Only the
   * class / vendor requests need to be handled by the application. */
  CyU3PUsbRegisterSetupCallback (CyFxUacApplnUSBSetupCB, CyTrue);

  /* Setup the callback to handle the USB events */
  CyU3PUsbRegisterEventCallback (CyFxUacApplnUsbEventCbk);

  /* Register a callback to handle LPM requests from the USB 3.0 host. */
  CyU3PUsbRegisterLPMRequestCallback (CyFxUacApplnLPMRqtCbk);

  /* Set the USB Enumeration descriptors */
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t*)CyFxUsb30DeviceDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t*)CyFxUsb20DeviceDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t*)CyFxUsbBOSDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t*)CyFxUsbDeviceQualDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t*)CyFxUsbSSConfigDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t*)CyFxUsbHSConfigDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t*)CyFxUsbFSConfigDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t*)CyFxUsbStringLangIDDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t*)CyFxUsbManufactureDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t*)CyFxUsbProductDscr);

  CyU3PConnectState (CyTrue, CyTrue);
  }
//}}}
//{{{
/* Entry function for the UAC application thread. */
static void uacAppThread (uint32_t input) {

    uint8_t spiBuffer [2048];
    uint16_t pageAddress = 0;
    CyU3PDmaBuffer_t dmaBuffer;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    CyBool_t partialBuf = CyFalse;
    uint16_t dataCount = 0;
    uint16_t dataCopyOffset = 0;
    uint16_t offset = 0;

    gpioInit();
    displayInit ("USB UAC");
    debugInit();
    appInit();

    for (;;) {
      /* Audio streaming logic */
      while (glIsApplnActive) {
        if (dataCount == 0) {
          //{{{  Read one page of data from SPI Flash
          dataCount = 8 * glSpiPageSize;
          status = CyFxUacSpiTransfer (pageAddress, dataCount, spiBuffer, CyTrue);
          if (status != CY_U3P_SUCCESS)
            CyFxAppErrorHandler (status);

          offset = 0;
          pageAddress += 8;
          }
          //}}}

        if (!partialBuf) {
          if (dataCount) {
            //{{{  If there is data to be copied then try and get a USB buffer
            status = CyU3PDmaChannelGetBuffer (&glUacStreamHandle, &dmaBuffer, 1000);
            if (status != CY_U3P_SUCCESS) {
              CyU3PThreadSleep (100);
              continue;
              }
            }
            //}}}

        if (dataCount / CY_FX3_ISO_XFER_LEN) {
          CyU3PMemCopy (dmaBuffer.buffer, &spiBuffer[offset], CY_FX3_ISO_XFER_LEN);
          dataCount -= CY_FX3_ISO_XFER_LEN;
          offset += CY_FX3_ISO_XFER_LEN;
          }
        else {
          if (dataCount) {
            partialBuf = CyTrue;
            dataCopyOffset = dataCount;
            dataCount = 0;
            /* Store the data. */
            CyU3PMemCopy (dmaBuffer.buffer, &spiBuffer[offset], dataCopyOffset);
            }
          continue;
          }
        }
      else {
        partialBuf = CyFalse;
        CyU3PMemCopy (&dmaBuffer.buffer[dataCopyOffset], &spiBuffer[offset], (CY_FX3_ISO_XFER_LEN - dataCopyOffset));

        /* Update the offset from which to start copying the data */
        offset = CY_FX3_ISO_XFER_LEN - dataCopyOffset;
        dataCount -= offset;
        }

      //{{{  Commit the buffer for transfer */
      status = CyU3PDmaChannelCommitBuffer (&glUacStreamHandle, CY_FX3_ISO_XFER_LEN, 0);
      if (status != CY_U3P_SUCCESS) {
        CyU3PThreadSleep (100);
        continue;
        }
      //}}}
      if (pageAddress >= CY_FX3_AUDIO_SAMPLE_SIZE_IN_PAGES)
        pageAddress = 0;
      }

    /* There is a streamer error. Flag it. */
    if ((status != CY_U3P_SUCCESS) && (glIsApplnActive)) {
      CyU3PDebugPrint (4, "UAC streamer error. Code %d.\n", status);
      CyFxAppErrorHandler (status);
      }

    /* Sleep for sometime as audio streamer is idle. */
    CyU3PThreadSleep (100);
    } /* End of for(;;) */
  }
//}}}

//{{{
/* Application define function which creates the threads. */
void CyFxApplicationDefine() {

  CyU3PThreadCreate (&uacThread,                    /* UAC Thread structure */
                     "31:UAC_App_Thread",           /* Thread Id and name */
                     uacAppThread,                  /* UAC Application Thread Entry function */
                     0,                             /* No input parameter to thread */
                     CyU3PMemAlloc (CY_FX_UAC_APP_THREAD_STACK), /* Pointer to the allocated thread stack */
                     CY_FX_UAC_APP_THREAD_STACK,    /* UAC Application Thread stack size */
                     CY_FX_UAC_APP_THREAD_PRIORITY, /* UAC Application Thread priority */
                     CY_FX_UAC_APP_THREAD_PRIORITY, /* Pre-emption threshold */
                     CYU3P_NO_TIME_SLICE,           /* No time slice for the application thread */
                     CYU3P_AUTO_START               /* Start the Thread immediately */
                     );
  }
//}}}

//{{{
int main() {

  CyU3PDeviceInit (NULL);

  /* Initialize the caches. Enable both Instruction and Data Caches. */
  CyU3PDeviceCacheControl (CyTrue, CyTrue, CyTrue);

  CyU3PIoMatrixConfig_t io_cfg;
  io_cfg.isDQ32Bit = CyFalse;
  io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
  io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
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

  CyU3PKernelEntry ();

  return 0;
  }
//}}}
