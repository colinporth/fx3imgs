//{{{  includes
#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3usbconst.h>
#include <cyu3uart.h>
#include <cyu3gpio.h>
#include <cyu3utils.h>

#include "display.h"
//}}}
#define CY_FX_MSC_CARD_CAPACITY  (256*1024)
#define RESET_GPIO        22  // CTL 5 pin
//{{{  defines
/* Endpoint and socket definitions for the MSC application */
/* To change the Producer and Consumer EP enter the appropriate EP numbers for the #defines.
 * In the case of IN endpoints enter EP number along with the direction bit.
 * For eg. EP 6 IN endpoint is 0x86
 *     and EP 6 OUT endpoint is 0x06.
 * To change sockets mention the appropriate socket number in the #defines. */
/* Note: For USB 2.0 the endpoints and corresponding sockets are one-to-one mapped
         i.e. EP 1 is mapped to UIB socket 1 and EP 2 to socket 2 so on */
#define CY_FX_MSC_EP_BULK_OUT              0x01    /* EP 1 OUT */
#define CY_FX_MSC_EP_BULK_IN               0x81    /* EP 1 IN */

#define CY_FX_MSC_EP_BULK_OUT_SOCKET       0x01    /* Socket 1 is producer */
#define CY_FX_MSC_EP_BULK_IN_SOCKET        0x01    /* Socket 1 is consumer */

#define CY_FX_MSC_DMA_BUF_COUNT      (3)                       /* MSC channel buffer count */
#define CY_FX_MSC_DMA_TX_SIZE        (0)                       /* DMA transfer size is set to infinite */
#define CY_FX_MSC_THREAD_STACK       (0x1000)                  /* MSC application thread stack size */
#define CY_FX_MSC_THREAD_PRIORITY    (8)                       /* MSC application thread priority */
#define CY_FX_MSC_CLR_STALL_IN_EVENT_FLAG  (1 << 0)            /* MSC application Clear Stall IN Event Flag */
#define CY_FX_MSC_BOT_RESET_EVENT_FLAG     (1 << 1)            /* MSC BOT Reset Event */
#define CY_FX_MSC_SET_CONFIG_EVENT         (1 << 2)            /* SET Config event */

#define CY_FX_MSC_USB_REQ_MASK             (0x60)              /* Request type mask */
#define CY_FX_MSC_USB_STANDARD_REQ         (0x00)              /* Standard request type */
#define CY_FX_MSC_USB_CLASS_REQ            (0x20)              /* Class request type */
#define CY_FX_MSC_GET_MAX_LUN_REQ          (0xFE)              /* MSC Get Max LUN request */
#define CY_FX_MSC_BOT_RESET_REQ            (0xFF)              /* MSC BOT Reset request */
#define CY_FX_MSC_USB_REQ_WINDEX_MASK      (0x0000FFFF)        /* USB Request wIndex mask */

/* Mass storage interface number */
#define CY_FX_USB_MSC_INTF                 (0x00)

//{{{  Descriptor Types
#define CY_FX_BOS_DSCR_TYPE             15
#define CY_FX_DEVICE_CAPB_DSCR_TYPE     16
#define CY_FX_SS_EP_COMPN_DSCR_TYPE     48
//}}}
//{{{  Device Capability Type Codes
#define CY_FX_WIRELESS_USB_CAPB_TYPE    1
#define CY_FX_USB2_EXTN_CAPB_TYPE       2
#define CY_FX_SS_USB_CAPB_TYPE          3
#define CY_FX_CONTAINER_ID_CAPBD_TYPE   4
//}}}

typedef enum {
  CY_FX_CBW_CMD_PASSED = 0,
  CY_FX_CBW_CMD_FAILED,
  CY_FX_CBW_CMD_PHASE_ERROR,
  CY_FX_CBW_CMD_MSC_RESET
  } CyFxMscCswReturnStatus_t;

#define CY_FX_MSC_CBW_MAX_COUNT             31
#define CY_FX_MSC_CSW_MAX_COUNT             13
#define CY_FX_MSC_REPONSE_DATA_MAX_COUNT    18

#define CY_FX_USB_SETUP_REQ_TYPE_MASK   (uint32_t)(0x000000FF)     /* Setup Request Type Mask */
#define CY_FX_USB_SETUP_REQ_MASK        (uint32_t)(0x0000FF00)     /* Setup Request Mask */
//}}}
//{{{  SCSI Commands
#define CY_FX_MSC_SCSI_TEST_UNIT_READY          0x00
#define CY_FX_MSC_SCSI_REQUEST_SENSE            0x03
#define CY_FX_MSC_SCSI_FORMAT_UNIT              0x04

#define CY_FX_MSC_SCSI_INQUIRY                  0x12
#define CY_FX_MSC_SCSI_MODE_SENSE_6             0x1A
#define CY_FX_MSC_SCSI_START_STOP_UNIT          0x1B
#define CY_FX_MSC_SCSI_PREVENT_ALLOW_MEDIUM     0x1E

#define CY_FX_MSC_SCSI_READ_FORMAT_CAPACITY     0x23
#define CY_FX_MSC_SCSI_READ_CAPACITY            0x25
#define CY_FX_MSC_SCSI_READ_10                  0x28
#define CY_FX_MSC_SCSI_WRITE_10                 0x2A
#define CY_FX_MSC_SCSI_VERIFY_10                0x2F
//}}}
//{{{  Sense Codes
#define CY_FX_MSC_SENSE_OK                      0x00
#define CY_FX_MSC_SENSE_CRC_ERROR               0x01
#define CY_FX_MSC_SENSE_INVALID_FIELD_IN_CBW    0x02
#define CY_FX_MSC_SENSE_NO_MEDIA                0x03
#define CY_FX_MSC_SENSE_WRITE_FAULT             0x04
#define CY_FX_MSC_SENSE_READ_ERROR              0x05
#define CY_FX_MSC_SENSE_ADDR_NOT_FOUND          0x06
#define CY_FX_MSC_SENSE_INVALID_OP_CODE         0x07
#define CY_FX_MSC_SENSE_INVALID_LBA             0x08
#define CY_FX_MSC_SENSE_INVALID_PARAMETER       0x09
#define CY_FX_MSC_SENSE_CANT_EJECT              0x0A
#define CY_FX_MSC_SENSE_MEDIA_CHANGED           0x0B
#define CY_FX_MSC_SENSE_DEVICE_RESET            0x0C
//}}}
//{{{  descriptors
//{{{
/* Standard Device Descriptor for 2.0 */
const uint8_t CyFxUSB20DeviceDscr[] __attribute__ ((aligned (32))) =
    {
        0x12,                           /* Descriptor Size */
        CY_U3P_USB_DEVICE_DESCR,        /* Device Descriptor Type */
        0x10,0x02,                      /* USB 2.1 */
        0x00,                           /* Device Class */
        0x00,                           /* Device Sub-class */
        0x00,                           /* Device protocol */
        0x40,                           /* Maxpacket size for EP0 : 64 bytes */
        0xB4,0x04,                      /* Vendor ID */
        0x21,0x47,                      /* Product ID */
        0x00,0x00,                      /* Device release number */
        0x01,                           /* Manufacture string index */
        0x02,                           /* Product string index */
        0x03,                           /* Serial number string index */
        0x01                            /* Number of configurations */
    };
//}}}
//{{{
/* Standard Device Descriptor for USB 3.0 */
const uint8_t CyFxUSB30DeviceDscr[] __attribute__ ((aligned (32))) =
    {
        0x12,                           /* Descriptor Size */
        CY_U3P_USB_DEVICE_DESCR,        /* Device Descriptor Type */
        0x00,0x03,                      /* USB 3.0 */
        0x00,                           /* Device Class */
        0x00,                           /* Device Sub-class */
        0x00,                           /* Device protocol */
        0x09,                           /* Maxpacket size for EP0 : 2^9 */
        0xB4,0x04,                      /* Vendor ID */
        0x21,0x47,                      /* Product ID */
        0x00,0x00,                      /* Device release number */
        0x01,                           /* Manufacture string index */
        0x02,                           /* Product string index */
        0x03,                           /* Serial number string index */
        0x01                            /* Number of configurations */
    };
//}}}
//{{{
/* Standard Device Qualifier Descriptor */
const uint8_t CyFxUSBDeviceQualDscr[] __attribute__ ((aligned (32))) =
    {
        0x0A,                           /* Descriptor Size */
        CY_U3P_USB_DEVQUAL_DESCR,       /* Device Qualifier Descriptor Type */
        0x00,0x02,                      /* USB 2.0 */
        0x00,                           /* Device Class */
        0x00,                           /* Device Sub-class */
        0x00,                           /* Device protocol */
        0x40,                           /* Maxpacket size for EP0 : 64 bytes */
        0x01,                           /* Number of configurations */
        0x00                            /* Reserved */
    };
//}}}

//{{{
/* Standard Full Speed Configuration Descriptor */
const uint8_t CyFxUSBFSConfigDscr[] __attribute__ ((aligned (32))) =
    {

        /* Configuration Descriptor Type */
        0x09,                           /* Descriptor Size */
        CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
        0x20,0x00,                      /* Length of this descriptor and all sub descriptors */
        0x01,                           /* Number of interfaces */
        0x01,                           /* Configuration number */
        0x00,                           /* COnfiguration string index */
        0x80,                           /* Config characteristics - D6: Self power; D5: Remote Wakeup */
        0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

        /* Interface Descriptor */
        0x09,                           /* Descriptor size */
        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
        CY_FX_USB_MSC_INTF,             /* Interface number */
        0x00,                           /* Alternate setting number */
        0x02,                           /* Number of end points */
        0x08,                           /* Interface class : Mass Storage Class */
        0x06,                           /* Interface sub class : SCSI Transparent Command Set */
        0x50,                           /* Interface protocol code : BOT */
        0x00,                           /* Interface descriptor string index */

        /* Endpoint Descriptor for Producer EP */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        CY_FX_MSC_EP_BULK_OUT,          /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* Bulk End point Type */
        0x40,0x00,                      /* Max packet size = 64 bytes */
        0x00,                           /* Servicing interval for data transfers : NA for Bulk */

        /* Endpoint Descriptor for Consumer EP */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        CY_FX_MSC_EP_BULK_IN,           /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* Bulk End point Type */
        0x40,0x00,                      /* Max packet size = 64 bytes */
        0x00                            /* Servicing interval for data transfers : NA for Bulk */

    };
//}}}
//{{{
/* Standard High Speed Configuration Descriptor */
const uint8_t CyFxUSBHSConfigDscr[] __attribute__ ((aligned (32))) =
    {

        /* Configuration Descriptor Type */
        0x09,                           /* Descriptor Size */
        CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
        0x20,0x00,                      /* Length of this descriptor and all sub descriptors */
        0x01,                           /* Number of interfaces */
        0x01,                           /* Configuration number */
        0x00,                           /* Configuration string index */
        0x80,                           /* Config characteristics - D6: Self power; D5: Remote Wakeup */
        0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

        /* Interface Descriptor */
        0x09,                           /* Descriptor size */
        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
        CY_FX_USB_MSC_INTF,             /* Interface number */
        0x00,                           /* Alternate setting number */
        0x02,                           /* Number of end points */
        0x08,                           /* Interface class : Mass Storage Class */
        0x06,                           /* Interface sub class : SCSI Transparent Command Set */
        0x50,                           /* Interface protocol code : BOT */
        0x00,                           /* Interface descriptor string index */

        /* Endpoint Descriptor for Producer EP */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        CY_FX_MSC_EP_BULK_OUT,          /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* Bulk End point Type */
        0x00,0x02,                      /* Max packet size = 512 bytes */
        0x00,                           /* Servicing interval for data transfers : NA for Bulk */

        /* Endpoint Descriptor for Consumer EP */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        CY_FX_MSC_EP_BULK_IN,           /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* Bulk End point Type */
        0x00,0x02,                      /* Max packet size = 512 bytes */
        0x00                            /* Servicing interval for data transfers : NA for Bulk */

    };
//}}}
//{{{
/* Binary Device Object Store Descriptor */
const uint8_t CyFxUSBBOSDscr[] __attribute__ ((aligned (32))) =
    {
        0x05,                           /* Descriptor Size */
        CY_FX_BOS_DSCR_TYPE,            /* Device Descriptor Type */
        0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
        0x02,                           /* Number of device capability descriptors */

        /* USB 2.0 Extension */
        0x07,                           /* Descriptor Size */
        CY_FX_DEVICE_CAPB_DSCR_TYPE,    /* Device Capability Type descriptor */
        CY_FX_USB2_EXTN_CAPB_TYPE,      /* USB 2.0 Extension Capability Type */
        0x02,0x00,0x00,0x00,            /* Supported device level features  */

        /* SuperSpeed Device Capability */
        0x0A,                           /* Descriptor Size */
        CY_FX_DEVICE_CAPB_DSCR_TYPE,    /* Device Capability Type descriptor */
        CY_FX_SS_USB_CAPB_TYPE,         /* SuperSpeed Device Capability Type */
        0x00,                           /* Supported device level features  */
        0x0E,0x00,                      /* Speeds Supported by the device : SS, HS and FS */
        0x03,                           /* Functionality support */
        0x00,                           /* U1 Device Exit Latency */
        0x00,0x00                       /* U2 Device Exit Latency */
    };
//}}}
//{{{
/* Standard Super Speed Configuration Descriptor */
const uint8_t CyFxUSBSSConfigDscr[] __attribute__ ((aligned (32))) =
    {

        /* Configuration Descriptor Type */
        0x09,                           /* Descriptor Size */
        CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
        0x2C,0x00,                      /* Length of this descriptor and all sub descriptors */
        0x01,                           /* Number of interfaces */
        0x01,                           /* Configuration number */
        0x00,                           /* Configuration string index */
        0x80,                           /* Config characteristics - D6: Self power; D5: Remote Wakeup */
        0x32,                           /* Max power consumption of device (in 8mA unit) : 400mA */

        /* Interface Descriptor */
        0x09,                           /* Descriptor size */
        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
        CY_FX_USB_MSC_INTF,             /* Interface number */
        0x00,                           /* Alternate setting number */
        0x02,                           /* Number of end points */
        0x08,                           /* Interface class */
        0x06,                           /* Interface sub class */
        0x50,                           /* Interface protocol code */
        0x00,                           /* Interface descriptor string index */

        /* Endpoint Descriptor for Producer EP */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        CY_FX_MSC_EP_BULK_OUT,          /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* Bulk End point Type */
        0x00,0x04,                      /* Max packet size = 1024 bytes */
        0x00,                           /* Servicing interval for data transfers : NA for Bulk */

        /* Super Speed Endpoint Companion Descriptor for Producer EP */
        0x06,                           /* Descriptor size */
        CY_FX_SS_EP_COMPN_DSCR_TYPE,    /* SS Endpoint Companion Descriptor Type */
        0x00,                           /* Max no. of packets in a Burst : 0: Burst 1 packet at a time */
        0x00,                           /* Max streams for Bulk EP = 0 (No streams)*/
        0x00,0x00,                      /* Service interval for the EP : NA for Bulk */

        /* Endpoint Descriptor for Consumer EP */
        0x07,                           /* Descriptor size */
        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
        CY_FX_MSC_EP_BULK_IN,              /* Endpoint address and description */
        CY_U3P_USB_EP_BULK,             /* Bulk End point Type */
        0x00,0x04,                      /* Max packet size = 1024 bytes */
        0x00,                           /* Servicing interval for data transfers : NA for Bulk */

        /* Super Speed Endpoint Companion Descriptor for Consumer EP */
        0x06,                           /* Descriptor size */
        CY_FX_SS_EP_COMPN_DSCR_TYPE,    /* SS Endpoint Companion Descriptor Type */
        0x00,                           /* Max no. of packets in a Burst : 0: Burst 1 packet at a time */
        0x00,                           /* Max streams for Bulk EP = 0 (No streams)*/
        0x00,0x00                       /* Service interval for the EP : NA for Bulk */

    };
//}}}

//{{{
/* Standard Language ID String Descriptor */
const uint8_t CyFxUSBStringLangIDDscr[] __attribute__ ((aligned (32))) =
    {
        0x04,                           /* Descriptor Size */
        CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
        0x09,0x04                       /* Language ID supported */
    };
//}}}
//{{{
/* Standard Manufacturer String Descriptor */
const uint8_t CyFxUSBManufactureDscr[] __attribute__ ((aligned (32))) =
    {
        0x10,                           /* Descriptor Size */
        CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
        'C',0x00,
        'y',0x00,
        'p',0x00,
        'r',0x00,
        'e',0x00,
        's',0x00,
        's',0x00
    };
//}}}
//{{{
/* Standard Product String Descriptor */
const uint8_t CyFxUSBProductDscr[] __attribute__ ((aligned (32))) =
    {
        0x08,                           /* Descriptor Size */
        CY_U3P_USB_STRING_DESCR,         /* Device Descriptor Type */
        'F',0x00,
        'X',0x00,
        '3',0x00
    };
//}}}
//{{{
/* Product Serial Number String Descriptor */
const uint8_t CyFxUSBSerialNumberDscr[] __attribute__ ((aligned (32))) =
    {
        0x1A,                           /* Descriptor Size */
        CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
        0x30, 0x00, 0x31, 0x00,
        0x32, 0x00, 0x33, 0x00,
        0x34, 0x00, 0x35, 0x00,
        0x36, 0x00, 0x37, 0x00,
        0x38, 0x00, 0x39, 0x00,
        0x30, 0x00, 0x31, 0x00
    };
//}}}
//}}}
//{{{
/* Request Sense Table */
static uint8_t glReqSenseCode[13][3] __attribute__ ((aligned (32))) = {
  /*SK,  ASC,  ASCQ*/
  {0x00, 0x00, 0x00},    /* senseOk                     0    */
  {0x0b, 0x08, 0x03},    /* senseCRCError               1    */
  {0x05, 0x24, 0x00},    /* senseInvalidFieldInCDB      2    */
  {0x02, 0x3a, 0x00},    /* senseNoMedia                3    */
  {0x03, 0x03, 0x00},    /* senseWriteFault             4    */
  {0x03, 0x11, 0x00},    /* senseReadError              5    */
  {0x03, 0x12, 0x00},    /* senseAddrNotFound           6    */
  {0x05, 0x20, 0x00},    /* senseInvalidOpcode          7    */
  {0x05, 0x21, 0x00},    /* senseInvalidLBA             8    */
  {0x05, 0x26, 0x00},    /* senseInvalidParameter       9    */
  {0x05, 0x53, 0x02},    /* senseCantEject              0xa  */
  {0x06, 0x28, 0x00},    /* senseMediaChanged           0xb  */
  {0x06, 0x29, 0x00}     /* senseDeviceReset            0xc  */
  };
//}}}
//{{{
/* Standard Inquiry Data */
static uint8_t CyFxMscScsiInquiryData[36] __attribute__ ((aligned (32))) = {
  0x00, /* PQ and PDT */
  0x80, /* RMB = 1 */
  0x00, /* Version */
  0x02, /* Response data format */
  0x1F, /* Addnl Length */
  0x00,
  0x00,
  0x00,
  'C', 'y', 'p', 'r', 'e', 's', 's', 0x00, /* Vendor Id */
  'F', 'X', '3', 0x00,
  'M', 'S', 'C', 0x00, /* Product Id */
  'D', 'E', 'M', 'O', 0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  '1'  /* Revision */
  };
//}}}
//{{{  vars
static CyU3PThread     MscAppThread;                          /* MSC application thread structure */
static CyU3PEvent      MscAppEvent;                             /* MSC application DMA Event group */

static uint32_t        glMscMaxSectors;                         /* Size of RAM Disk in 512 byte sectors. */
static uint16_t        glMscSectorSize;                         /* RAM Disk sector size: 512 bytes. */

/* Buffer for the MSC response data: 18 bytes is the maximum. */
static uint8_t         glMscOutBuffer[CY_FX_MSC_REPONSE_DATA_MAX_COUNT] __attribute__ ((aligned (32)));

/* Buffer to received the incoming MSC CBW packet: 31 bytes. */
static uint8_t glMscInBuffer[CY_FX_MSC_CBW_MAX_COUNT] __attribute__ ((aligned (32)));

/* Initialize the CSW signature. Other fields are 0 */
static uint8_t         glMscCswStatus[CY_FX_MSC_CSW_MAX_COUNT] __attribute__ ((aligned (32))) = {
    'U','S','B','S'
};

static CyU3PUSBSpeed_t glUsbSpeed;                              /* Current USB Bus speed */
static CyBool_t        glDevConfigured;                         /* Flag to indicate Set Config event handled */
static uint32_t        glCswDataResidue;                        /* Residue length for CSW */
static CyU3PDmaChannel glChHandleMscOut, glChHandleMscIn;       /* Channel handles */

/* Pointer for dynamic allocation of Storage device memory : 32K */
static uint8_t         *glMscStorageDeviceMemory;

static uint8_t           glCmdDirection;                        /* SCSI Command Direction */
static uint32_t          glDataTxLength;                        /* SCSI Data length */
uint8_t                  glInPhaseError = CyFalse;              /* Invalid command received flag. */
static volatile CyBool_t glMscChannelCreated = CyFalse;         /* Whether DMA channels have been created. */
static uint8_t           glReqSenseIndex = CY_FX_MSC_SENSE_DEVICE_RESET; /* Current sense data index. */
//}}}

//{{{
/* MSC application error handler */
static void CyFxAppErrorHandler (CyU3PReturnStatus_t apiRetStatus)
{
    for (;;)
    {
        /* Thread Sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}
//}}}

//{{{
/* Callback to handle the USB Setup Requests and Mass Storage Class requests */
static CyBool_t CyFxMscApplnUSBSetupCB (uint32_t setupdat0, uint32_t setupdat1) {

  CyBool_t mscHandleReq = CyFalse;
  uint8_t maxLun = 0;
  CyU3PReturnStatus_t apiRetStatus;
  uint32_t txApiRetStatus;
  CyU3PEpConfig_t endPointConfig;

  // Decode the fields from the setup request. */
  uint8_t  bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
  uint8_t  bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
  uint8_t  bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
  uint8_t  bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
  uint16_t wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
  uint16_t wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
  uint16_t wLength  = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);

  // Check for Set Configuration request
  if (bType == CY_U3P_USB_STANDARD_RQT) {
    // Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND) requests
    // should be allowed to pass if the device is in configured state and failed otherwise
    if ((bTarget == CY_U3P_USB_TARGET_INTF) &&
        ((bRequest == CY_U3P_USB_SC_SET_FEATURE) || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) &&
        (wValue == 0)) {
      if (glDevConfigured)
        CyU3PUsbAckSetup();
      else
        CyU3PUsbStall (0, CyTrue, CyFalse);
      mscHandleReq = CyTrue;
      }

    if (bRequest == CY_U3P_USB_SC_SET_CONFIGURATION) {
      //{{{  setConfig handler
      if (glDevConfigured == CyFalse ) {
        glDevConfigured = CyTrue;

        /* Configure the endpoints based on the USB speed */
        /* Get the Bus speed */
        glUsbSpeed = CyU3PUsbGetSpeed();

        /* Based on the Bus Speed configure the endpoint packet size */
        if (glUsbSpeed == CY_U3P_FULL_SPEED)
          endPointConfig.pcktSize = 64;
        else if (glUsbSpeed == CY_U3P_HIGH_SPEED)
          endPointConfig.pcktSize = 512;
        else if (glUsbSpeed == CY_U3P_SUPER_SPEED)
          endPointConfig.pcktSize = 1024;
        else
          CyU3PDebugPrint (4, "Error! USB Not connected\r\n");

        /* Set the Endpoint Configurations,  Producer Endpoint configuration */
        endPointConfig.enable = 1;
        endPointConfig.epType = CY_U3P_USB_EP_BULK;
        endPointConfig.streams = 0;
        endPointConfig.burstLen = 1;

        /* Configure the Endpoint */
        apiRetStatus = CyU3PSetEpConfig(CY_FX_MSC_EP_BULK_OUT,&endPointConfig);
        if (apiRetStatus != CY_U3P_SUCCESS) {
          CyU3PDebugPrint (4, "USB Set Endpoint config failed, Error Code = %d\r\n", apiRetStatus);
          CyFxAppErrorHandler(apiRetStatus);
          }

        /* Consumer Endpoint configuration */
        endPointConfig.enable = 1;
        endPointConfig.epType = CY_U3P_USB_EP_BULK;
        endPointConfig.streams = 0;
        endPointConfig.burstLen = 1;

        /* Configure the Endpoint */
        apiRetStatus = CyU3PSetEpConfig(CY_FX_MSC_EP_BULK_IN,&endPointConfig);
        if (apiRetStatus != CY_U3P_SUCCESS) {
          CyU3PDebugPrint (4, "USB Set Endpoint config failed, Error Code = %d\r\n", apiRetStatus);
          CyFxAppErrorHandler(apiRetStatus);
          }

        /* Set Event to indicate Set Configuration */
        txApiRetStatus = CyU3PEventSet(&MscAppEvent,CY_FX_MSC_SET_CONFIG_EVENT,CYU3P_EVENT_OR);
        if (txApiRetStatus != CY_U3P_SUCCESS)
          CyU3PDebugPrint (4, "Bulk Loop App Set Event Failed, Error Code = %d\r\n", txApiRetStatus);
        CyU3PDebugPrint (4, "SetConfig event set\r\n");
        }
      }
      //}}}
    else if (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE) {
      //{{{  clearFeature handler
      if (glInPhaseError) {
        CyU3PUsbAckSetup ();
        return CyTrue;
        }

      /* Check Clear Feature on IN EP */
      if (wIndex == CY_FX_MSC_EP_BULK_IN) {
          /* Clear stall */
          CyU3PUsbStall (CY_FX_MSC_EP_BULK_IN, CyFalse, CyTrue);
          mscHandleReq = CyTrue;
          CyU3PUsbAckSetup ();
          }

      /* Check Clear Feature on OUT EP */
      if (wIndex == CY_FX_MSC_EP_BULK_OUT) {
          /* Clear stall */
          CyU3PUsbStall (CY_FX_MSC_EP_BULK_OUT, CyFalse, CyTrue);
          mscHandleReq = CyTrue;
          CyU3PUsbAckSetup ();
          }
      }
      //}}}
    }

  // Check for MSC Requests
  else if (bType == CY_FX_MSC_USB_CLASS_REQ) {
    mscHandleReq = CyFalse;
    apiRetStatus = CY_U3P_SUCCESS;
    if ((bTarget == CY_U3P_USB_TARGET_INTF) && (wIndex == CY_FX_USB_MSC_INTF) && (wValue == 0)) {
      if (bRequest == CY_FX_MSC_GET_MAX_LUN_REQ) {
        //{{{  max lun request
        if (wLength == 1) {
          mscHandleReq = CyTrue;
          /* Send response */
          apiRetStatus = CyU3PUsbSendEP0Data(0x01, &maxLun);
          if (apiRetStatus != CY_U3P_SUCCESS)
            CyU3PDebugPrint (4, "Send EP0 Data Failed, Error Code = %d\r\n", apiRetStatus);
          }
        }
        //}}}
      else if (bRequest == CY_FX_MSC_BOT_RESET_REQ) {
        //{{{  bot reset request
        mscHandleReq = CyTrue;
        glInPhaseError = CyFalse;
        if (wLength == 0) {
            CyU3PUsbAckSetup ();

            if (glMscChannelCreated) {
              CyU3PDmaChannelReset(&glChHandleMscOut);
              CyU3PDmaChannelReset(&glChHandleMscIn);
              }

            CyU3PUsbFlushEp (CY_FX_MSC_EP_BULK_OUT);
            CyU3PUsbStall (CY_FX_MSC_EP_BULK_OUT,  CyFalse, CyTrue);

            CyU3PUsbFlushEp (CY_FX_MSC_EP_BULK_IN);
            CyU3PUsbStall (CY_FX_MSC_EP_BULK_IN,  CyFalse, CyTrue);

            /* Request Sense Index */
            glReqSenseIndex = CY_FX_MSC_SENSE_DEVICE_RESET;
            }
        else
           CyU3PUsbStall (0x00, CyTrue, CyFalse);
        }
        //}}}
      }
    if ((mscHandleReq == CyFalse) || (apiRetStatus != CY_U3P_SUCCESS))
      /* This is a failed handling. Stall the EP. */
      CyU3PUsbStall (0, CyTrue, CyFalse);
    }

  return mscHandleReq;
  }
//}}}
//{{{
static void CyFxMscApplnUSBEventCB (CyU3PUsbEventType_t evtype, uint16_t  evdata ) {

  CyU3PDebugPrint (4, "USB event %d %d\r\n", evtype, evdata);

  /* Check for Reset / Suspend / Disconnect / Connect Events */
  if ((evtype == CY_U3P_USB_EVENT_RESET) ||
      (evtype == CY_U3P_USB_EVENT_SUSPEND) ||
      (evtype == CY_U3P_USB_EVENT_DISCONNECT) ||
      (evtype == CY_U3P_USB_EVENT_CONNECT)) {

    if (glMscChannelCreated) {
      /* Abort the IN Channel */
      CyU3PDmaChannelAbort(&glChHandleMscIn);

      /* Abort the OUT Channel */
      CyU3PDmaChannelAbort(&glChHandleMscOut);
      }

    /* Request Sense Index */
    glReqSenseIndex = CY_FX_MSC_SENSE_DEVICE_RESET;

    /* Init the USB Speed */
    glUsbSpeed = CY_U3P_NOT_CONNECTED;

    /* Clear Flag */
    glDevConfigured = CyFalse;
    }
  }
//}}}
//{{{
/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
static CyBool_t CyFxMscApplnLPMRqtCB (CyU3PUsbLinkPowerMode link_mode) {

  return CyTrue;
  }
//}}}

//{{{
/* This is wrapper funtion to send the USB data to the host from the give data area */
static CyU3PReturnStatus_t CyFxMscSendUSBData (uint8_t* data, uint32_t length ) {

  CyU3PDmaBuffer_t dmaMscOutBuffer;
  dmaMscOutBuffer.buffer = data;
  dmaMscOutBuffer.status = 0;
  if (glUsbSpeed == CY_U3P_FULL_SPEED)
    dmaMscOutBuffer.size = 64;
  else if (glUsbSpeed == CY_U3P_HIGH_SPEED)
    dmaMscOutBuffer.size = 512;
  else if (glUsbSpeed == CY_U3P_SUPER_SPEED)
    dmaMscOutBuffer.size = 1024;
  else {
    CyU3PDebugPrint (4, "USB Not connected\r\n");
    return CY_U3P_ERROR_FAILURE;
    }
  dmaMscOutBuffer.count = length;

  /* Setup OUT buffer to send the data,  OUT buffer setup when there is no abort */
  CyU3PReturnStatus_t apiRetStatus = CyU3PDmaChannelSetupSendBuffer (&glChHandleMscOut, &dmaMscOutBuffer);
  if (apiRetStatus == CY_U3P_SUCCESS)
    apiRetStatus = CyU3PDmaChannelWaitForCompletion (&glChHandleMscOut,CYU3P_WAIT_FOREVER);

  return apiRetStatus;
  }
//}}}
//{{{
/* This is wrapper funtion to receive the USB data from the host and store into the give data area */
static CyU3PReturnStatus_t CyFxMscReceiveUSBData (uint8_t* data) {

  CyU3PDmaBuffer_t dmaMscInBuffer;
  dmaMscInBuffer.buffer = data;
  dmaMscInBuffer.status = 0;
  if (glUsbSpeed == CY_U3P_FULL_SPEED)
    dmaMscInBuffer.size = 64;
  else if (glUsbSpeed == CY_U3P_HIGH_SPEED)
    dmaMscInBuffer.size = 512;
  else if (glUsbSpeed == CY_U3P_SUPER_SPEED)
    dmaMscInBuffer.size = 1024;
  else {
    CyU3PDebugPrint (4, "USB Not connected\r\n");
    return CY_U3P_ERROR_FAILURE;
    }

  // Setup IN buffer to receive the data */
  CyU3PReturnStatus_t apiRetStatus  = CyU3PDmaChannelSetupRecvBuffer (&glChHandleMscIn, &dmaMscInBuffer);
  if (apiRetStatus == CY_U3P_SUCCESS)
    apiRetStatus  = CyU3PDmaChannelWaitForRecvBuffer (&glChHandleMscIn,&dmaMscInBuffer, CYU3P_WAIT_FOREVER);

  return apiRetStatus;
  }
//}}}

//{{{
/* This function validates the CBW received w.r.t signature and CBW length */
static CyFxMscCswReturnStatus_t CyFxMscCheckScsiCmd (uint8_t* buffer, uint16_t count) {

  CyFxMscCswReturnStatus_t  retStatus = CY_FX_CBW_CMD_PASSED;

  /* Verify signature */
  if (buffer[0] != 'U' || buffer[1] != 'S' || buffer[2] != 'B' || buffer[3] != 'C')
    retStatus = CY_FX_CBW_CMD_PHASE_ERROR;
  else
    /* Verify count */
    if (count != CY_FX_MSC_CBW_MAX_COUNT)
      retStatus = CY_FX_CBW_CMD_PHASE_ERROR;

  return retStatus;
  }
//}}}
//{{{
/* This function frames the CSW and sends it to the host */
static CyFxMscCswReturnStatus_t CyFxMscSendCsw (CyFxMscCswReturnStatus_t cswReturnStatus ) {

  if (cswReturnStatus != CY_FX_CBW_CMD_PASSED)
    CyU3PDebugPrint (4, "CSW status is %d\r\n", cswReturnStatus);

  /* Update the residue length in the CSW */
  glMscCswStatus[11] = (uint8_t)((glCswDataResidue & 0xFF000000) >> 24);
  glMscCswStatus[10] = (uint8_t)((glCswDataResidue & 0x00FF0000) >> 16);
  glMscCswStatus[9] = (uint8_t)((glCswDataResidue & 0x0000FF00) >> 8);
  glMscCswStatus[8] = (uint8_t)(glCswDataResidue & 0x000000FF);

  /* Update the status in the CSW */
  glMscCswStatus[12] = (uint8_t)(cswReturnStatus & 0x3);

  /* Check for phase error */
  if (cswReturnStatus == CY_FX_CBW_CMD_PHASE_ERROR) {
    //{{{  phase error
    glInPhaseError = CyTrue;

    CyU3PUsbSetEpNak (CY_FX_MSC_EP_BULK_IN, CyTrue);
    CyU3PBusyWait (125);

    /* Stall the IN endpoint */
    CyU3PUsbStall(CY_FX_MSC_EP_BULK_IN, CyTrue, CyFalse);

    /* Stall the OUT endpoint */
    CyU3PUsbStall(CY_FX_MSC_EP_BULK_OUT, CyTrue, CyFalse);

    CyU3PDmaChannelReset(&glChHandleMscIn);
    CyU3PDmaChannelReset(&glChHandleMscOut);

    CyU3PUsbFlushEp(CY_FX_MSC_EP_BULK_IN);
    CyU3PUsbFlushEp(CY_FX_MSC_EP_BULK_OUT);

    CyU3PUsbSetEpNak (CY_FX_MSC_EP_BULK_IN, CyFalse);
    }
    //}}}

  /* Check for Command failed */
  else if (cswReturnStatus == CY_FX_CBW_CMD_FAILED) {
    //{{{  cmd failed
    /* Only when data is expected or to be sent stall the EPs */
    if (glDataTxLength != 0) {
      /* Check direction */
      if (glCmdDirection == 0x00)
        /* Stall the OUT endpoint */
        CyU3PUsbStall(CY_FX_MSC_EP_BULK_OUT, CyTrue, CyFalse);

      else {
        CyU3PUsbSetEpNak (CY_FX_MSC_EP_BULK_IN, CyTrue);
        CyU3PBusyWait (125);

        /* Stall the IN endpoint */
        CyU3PUsbStall(CY_FX_MSC_EP_BULK_IN, CyTrue, CyFalse);
        CyU3PDmaChannelReset (&glChHandleMscIn);
        CyU3PUsbFlushEp(CY_FX_MSC_EP_BULK_IN);
        CyU3PUsbSetEpNak (CY_FX_MSC_EP_BULK_IN, CyFalse);
        }
      }
    }
    //}}}

  /* Send the status to the host */
  if (CyFxMscSendUSBData (glMscCswStatus, CY_FX_MSC_CSW_MAX_COUNT) != CY_U3P_SUCCESS)
    cswReturnStatus = CY_FX_CBW_CMD_MSC_RESET;

  return cswReturnStatus;
  }
//}}}
//{{{
/* This function checks the direction bit and verifies against given command */
static CyFxMscCswReturnStatus_t CyFxCheckCmdDirection (uint8_t scsiCmd, uint8_t cmdDirection) {

  CyFxMscCswReturnStatus_t retStatus = CY_FX_CBW_CMD_PASSED;

  /* Check for IN or OUT command and verify direction */
  /* Phase error checked only for supported commands */
  if (cmdDirection == 0x00) {
    if ((scsiCmd == CY_FX_MSC_SCSI_INQUIRY) ||
        (scsiCmd == CY_FX_MSC_SCSI_READ_CAPACITY) ||
        (scsiCmd == CY_FX_MSC_SCSI_REQUEST_SENSE) ||
        (scsiCmd == CY_FX_MSC_SCSI_FORMAT_UNIT) ||
        (scsiCmd == CY_FX_MSC_SCSI_START_STOP_UNIT) ||
        (scsiCmd == CY_FX_MSC_SCSI_MODE_SENSE_6) ||
        (scsiCmd == CY_FX_MSC_SCSI_PREVENT_ALLOW_MEDIUM) ||
        (scsiCmd == CY_FX_MSC_SCSI_READ_10) ||
        (scsiCmd == CY_FX_MSC_SCSI_READ_FORMAT_CAPACITY) ||
        (scsiCmd == CY_FX_MSC_SCSI_VERIFY_10) ||
        (scsiCmd == CY_FX_MSC_SCSI_TEST_UNIT_READY))
      retStatus = CY_FX_CBW_CMD_FAILED;
    }
  else if (scsiCmd == CY_FX_MSC_SCSI_WRITE_10)
    retStatus = CY_FX_CBW_CMD_FAILED;

  return retStatus;
  }
//}}}
//{{{
// This function parses the CBW for the SCSI commands and services the command
static CyFxMscCswReturnStatus_t CyFxMscParseScsiCmd (uint8_t* mscCbw) {

  CyFxMscCswReturnStatus_t retParseStatus = CY_FX_CBW_CMD_PASSED;
  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

  uint32_t allocLength;
  uint32_t mscLba;
  uint16_t mscSector;
  uint8_t idx, numBytes;

  uint32_t temp = 0;
  glCswDataResidue = 0;
  uint32_t dataTxLength = ((uint32_t)mscCbw[11] << 24) | ((uint32_t)mscCbw[10] << 16) |
                          ((uint32_t)mscCbw[9] << 8) | ((uint32_t)mscCbw[8]);
  glDataTxLength = dataTxLength;

  // Retrieve the SCSI command */
  uint8_t scsiCmd = mscCbw[15];

  // Verify if the direction bit is valid for the command, Ignore the direction when Tx length is 0
  if (dataTxLength != 0) {
    //{{{  determine direction
    glCmdDirection = ((mscCbw[12] & 0x80) >> 7);
    glCswDataResidue = glDataTxLength;

    // Check for phase error
    retParseStatus = CyFxCheckCmdDirection (scsiCmd, glCmdDirection);
    }
    //}}}

  // Execute commands when there is no phase error
  if (retParseStatus == CY_FX_CBW_CMD_PASSED) {
    line3 ("cmd", scsiCmd);

    switch (scsiCmd) {
      //{{{
      case CY_FX_MSC_SCSI_TEST_UNIT_READY: {

        /* Check transfer length */
        if (dataTxLength != 0) {
          retParseStatus = CY_FX_CBW_CMD_FAILED;
          glCswDataResidue = dataTxLength;
          glReqSenseIndex = CY_FX_MSC_SENSE_INVALID_FIELD_IN_CBW;
          }

        else {
          glReqSenseIndex = CY_FX_MSC_SENSE_OK;
          glCswDataResidue = 0;
          }

        break;
        }
      //}}}
      //{{{
      case CY_FX_MSC_SCSI_INQUIRY: {
        /* Get the allocation length */
        allocLength = ((uint16_t)(mscCbw[15 + 3] >> 8) |  (uint16_t)(mscCbw[15 + 4]));

        /* If data length requested is zero then no operation */
        if ((dataTxLength != 0) &&  (allocLength != 0)) {
          /* Update the residue length */
          if (dataTxLength < allocLength )
            glCswDataResidue = (uint32_t)(allocLength - dataTxLength);
          else
            glCswDataResidue = (uint32_t)(dataTxLength - allocLength);

          /* Construct the Inquiry response, Return the standard Inquiry data */
          apiRetStatus = CyFxMscSendUSBData (CyFxMscScsiInquiryData, allocLength);
          }

        /* Set sense index to OK */
        glReqSenseIndex = CY_FX_MSC_SENSE_OK;

        break;
        }
      //}}}
      case CY_FX_MSC_SCSI_READ_FORMAT_CAPACITY:
      //{{{
      case  CY_FX_MSC_SCSI_READ_CAPACITY: {
        /* Check transfer length */
        if (dataTxLength != 0) {
          idx = 0;
          numBytes = 8;
          CyU3PMemSet (glMscOutBuffer, 0, 12);

          /* Check for Read Format Capacity */
          if (scsiCmd == CY_FX_MSC_SCSI_READ_FORMAT_CAPACITY) {
            glMscOutBuffer[0+idx] = 0x00;
            glMscOutBuffer[1+idx] = 0x00;
            glMscOutBuffer[2+idx] = 0x00;
            glMscOutBuffer[3+idx] = 0x08;
            idx = 4;
            numBytes = 12;
            }

          /* Report to the host the capacity of the device, Report the LBA */
          glMscOutBuffer[0+idx] = (uint8_t)((glMscMaxSectors & 0xFF000000) >> 24);
          glMscOutBuffer[1+idx] = (uint8_t)((glMscMaxSectors & 0x00FF0000) >> 16);
          glMscOutBuffer[2+idx] = (uint8_t)((glMscMaxSectors & 0x0000FF00) >> 8);
          glMscOutBuffer[3+idx] = (uint8_t)(glMscMaxSectors & 0x000000FF);

          /* Check for Read Format Capacity,  Report bytes per sector */
          if (scsiCmd == CY_FX_MSC_SCSI_READ_FORMAT_CAPACITY)
            /* Indicate format flag */
            glMscOutBuffer[4+idx] = 0x02;
          else
            glMscOutBuffer[4+idx] = 0x00;
          glMscOutBuffer[5+idx] = 0x00;
          glMscOutBuffer[6+idx] = (uint8_t)((glMscSectorSize & 0xFF00) >> 8); /* Sector Size */
          glMscOutBuffer[7+idx] = (uint8_t)(glMscSectorSize & 0x00FF);

          /* Update the residue length */
          if (dataTxLength < numBytes )
            glCswDataResidue = (uint32_t)(numBytes - dataTxLength);
          else
            glCswDataResidue = (uint32_t)(dataTxLength - numBytes);

          /* Send data to USB */
          apiRetStatus = CyFxMscSendUSBData (glMscOutBuffer, numBytes);

          /* Set sense index to OK */
          glReqSenseIndex = CY_FX_MSC_SENSE_OK;
          }

        else {
          retParseStatus = CY_FX_CBW_CMD_FAILED;
          glCswDataResidue = dataTxLength;
          glReqSenseIndex = CY_FX_MSC_SENSE_INVALID_FIELD_IN_CBW;
          }
        break;
        }
      //}}}
      //{{{
      case CY_FX_MSC_SCSI_REQUEST_SENSE: {
        /* Check transfer length */
        if (dataTxLength != 0) {
          /* Clear the response array */
          CyU3PMemSet (glMscOutBuffer, 0, 18);

          /* Report Sense codes */
          glMscOutBuffer[0] = 0x70; /* Current errors */
          glMscOutBuffer[1] = 0x00;
          glMscOutBuffer[2] = glReqSenseCode[glReqSenseIndex][0]; /* SK */
          glMscOutBuffer[7] = 0x0A; /* Length of following data */
          glMscOutBuffer[12] = glReqSenseCode[glReqSenseIndex][1]; /* ASC */
          glMscOutBuffer[13] = glReqSenseCode[glReqSenseIndex][2]; /* ASCQ */

          /* Set sense index to OK */
          glReqSenseIndex = CY_FX_MSC_SENSE_OK;

          temp = dataTxLength >= 18 ? 18 : dataTxLength;
          /* Send data to USB */
          apiRetStatus = CyFxMscSendUSBData (glMscOutBuffer, temp);
          glCswDataResidue = (uint32_t)(dataTxLength - temp);
          if (glCswDataResidue > 18)
            apiRetStatus = CY_FX_CBW_CMD_FAILED;
          }

        else {
          retParseStatus = CY_FX_CBW_CMD_FAILED;
          glCswDataResidue = dataTxLength;
          glReqSenseIndex = CY_FX_MSC_SENSE_INVALID_FIELD_IN_CBW;
          }

        break;
        }
      //}}}
      case CY_FX_MSC_SCSI_FORMAT_UNIT:
      case CY_FX_MSC_SCSI_START_STOP_UNIT:
      //{{{
      case CY_FX_MSC_SCSI_MODE_SENSE_6: {

        // Check transfer length
        if (dataTxLength != 0) {
          // Report Sense codes
          glMscOutBuffer[0] = 0x03;
          glMscOutBuffer[1] = 0x00;
          glMscOutBuffer[2] = 0x00;
          glMscOutBuffer[3] = 0x00;

          // Set sense index to OK
          glReqSenseIndex = CY_FX_MSC_SENSE_OK;

          // Update the residue length
          if (dataTxLength < 4 )
            glCswDataResidue = (uint32_t)(4 - dataTxLength);
          else
            glCswDataResidue = (uint32_t)(dataTxLength - 4);

          // Send data to USB
          apiRetStatus = CyFxMscSendUSBData(glMscOutBuffer, 4);
          }

        else {
          retParseStatus = CY_FX_CBW_CMD_FAILED;
          glCswDataResidue = 0;
          glReqSenseIndex = CY_FX_MSC_SENSE_INVALID_FIELD_IN_CBW;
          }

        break;
        }
      //}}}
      //{{{
      case CY_FX_MSC_SCSI_PREVENT_ALLOW_MEDIUM: {

        /* Check transfer length */
        /* Allow Medium removal only */
        if (dataTxLength != 0) {
          retParseStatus = CY_FX_CBW_CMD_FAILED;
          glCswDataResidue = dataTxLength;
          glReqSenseIndex = CY_FX_MSC_SENSE_INVALID_FIELD_IN_CBW;
          }
        else {
          glReqSenseIndex = CY_FX_MSC_SENSE_OK;
          glCswDataResidue = 0;
          }

        break;
        }
      //}}}
      //{{{
      case CY_FX_MSC_SCSI_READ_10: {

        mscLba = ((uint32_t)mscCbw[15+2] << 24) | ((uint32_t)mscCbw[15+3] << 16) |
                    ((uint32_t)mscCbw[15+4] << 8) | ((uint32_t)mscCbw[15+5]);
        mscSector = ((uint32_t)mscCbw[15+7] << 8) | (uint32_t)mscCbw[15+8];

        // Check transfer length
        if ((dataTxLength == 0) && (mscSector == 0)) {
          glReqSenseIndex = CY_FX_MSC_SENSE_OK;
          glCswDataResidue = 0;
          break;
          }

        // Check LBA and Sectors requested
        if ((mscLba > glMscMaxSectors) ||
            ((mscLba + mscSector) > (glMscMaxSectors+1)) ||
            (((mscSector * glMscSectorSize)) != dataTxLength)) {
          retParseStatus = CY_FX_CBW_CMD_FAILED;
          glCswDataResidue = dataTxLength;
          glReqSenseIndex = CY_FX_MSC_SENSE_INVALID_FIELD_IN_CBW;
          break;
          }

        // Send the data blocks to USB one by one
        while ((mscSector > 0) && dataTxLength) {
          /* Send data to USB */
          apiRetStatus = CyFxMscSendUSBData (&glMscStorageDeviceMemory[((mscLba++)*glMscSectorSize)], glMscSectorSize);
          if (apiRetStatus != CY_U3P_SUCCESS)
            /* Stop Sending further data */
            break;

          mscSector--;
          dataTxLength -= glMscSectorSize;
          }

        glCswDataResidue = dataTxLength;
        glReqSenseIndex = CY_FX_MSC_SENSE_OK;

        break;
        }
      //}}}
      //{{{
      case CY_FX_MSC_SCSI_WRITE_10: {

        mscLba = ((uint32_t)mscCbw[15+2] << 24) | ((uint32_t)mscCbw[15+3] << 16) |
                 ((uint32_t)mscCbw[15+4] << 8) | ((uint32_t)mscCbw[15+5]);
        mscSector = ((uint32_t)mscCbw[15+7] << 8) | (uint32_t)mscCbw[15+8];

        // Check transfer length
        if ((dataTxLength == 0) && (mscSector == 0)) {
          glReqSenseIndex = CY_FX_MSC_SENSE_OK;
          glCswDataResidue = 0;
          break;
          }

        // Check LBA
        if ((mscLba > glMscMaxSectors) ||
            ((mscLba + mscSector) > (glMscMaxSectors+1)) ||
            (((mscSector * glMscSectorSize)) != dataTxLength)) {
          retParseStatus = CY_FX_CBW_CMD_FAILED;
          glCswDataResidue = dataTxLength;
          glReqSenseIndex = CY_FX_MSC_SENSE_INVALID_FIELD_IN_CBW;
          break;
          }

        // Send the data blocks to USB one by one
        while ((mscSector > 0) && dataTxLength) {
          // Receive data from USB
          apiRetStatus = CyFxMscReceiveUSBData(&glMscStorageDeviceMemory[((mscLba++)*glMscSectorSize)]);
          if (apiRetStatus != CY_U3P_SUCCESS)
            // Stop receving further data
            break;
          mscSector--;
          dataTxLength -= glMscSectorSize;
          }

        glCswDataResidue = dataTxLength;
        glReqSenseIndex = CY_FX_MSC_SENSE_OK;

        break;
        }
      //}}}
      //{{{
      case CY_FX_MSC_SCSI_VERIFY_10: {
        /* This is spoofed command */
        glCswDataResidue = 0;
        glReqSenseIndex = CY_FX_MSC_SENSE_OK;
        break;
        }
      //}}}
      //{{{
      default : {
        /* Command Failed */
        CyU3PDebugPrint (4, "Unknown command %x\r\n", scsiCmd);

        retParseStatus = CY_FX_CBW_CMD_FAILED;
        glCswDataResidue = dataTxLength;
        glReqSenseIndex = CY_FX_MSC_SENSE_INVALID_OP_CODE;

        break;
        }
      //}}}
      }
    }

  // Check for API failure, API failures lead to MSC Reset
  if (apiRetStatus != CY_U3P_SUCCESS)
    // Update the return status
    retParseStatus = CY_FX_CBW_CMD_FAILED;

  return retParseStatus;
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
static void debugInit() {

  CyU3PUartInit();

  CyU3PUartConfig_t uartConfig;
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
  CyU3PDebugPreamble (CyFalse);
  }
//}}}
//{{{
static void appInit() {

  CyU3PEventCreate (&MscAppEvent);

  CyU3PUsbStart();

  /* Setup the Callback to Handle the USB Setup Requests */
  /* Set Fast enumeration to False */
  CyU3PUsbRegisterSetupCallback (CyFxMscApplnUSBSetupCB, CyFalse);

  /* Setup the Callback to Handle the USB Events */
  CyU3PUsbRegisterEventCallback (CyFxMscApplnUSBEventCB);

  /* Register a callback to handle LPM requests from the USB 3.0 host. */
  CyU3PUsbRegisterLPMRequestCallback (CyFxMscApplnLPMRqtCB);

  /* Set the USB Enumeration descriptors */
  CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t*)CyFxUSB20DeviceDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t*)CyFxUSB30DeviceDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t*)CyFxUSBDeviceQualDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t*)CyFxUSBHSConfigDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t*)CyFxUSBFSConfigDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t*)CyFxUSBSSConfigDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t*)CyFxUSBBOSDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t*)CyFxUSBStringLangIDDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t*)CyFxUSBManufactureDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t*)CyFxUSBProductDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 3, (uint8_t*)CyFxUSBSerialNumberDscr);

  // Connect the USB Pins Enable Super Speed operation */
  CyU3PConnectState (1, CyTrue);
  }
//}}}
//{{{
static void mscThread (uint32_t input) {

  CyU3PReturnStatus_t apiRetStatus;
  CyFxMscCswReturnStatus_t cswReturnStatus = CY_FX_CBW_CMD_PASSED;

  CyU3PMemSet (glMscStorageDeviceMemory, 0, (CY_FX_MSC_CARD_CAPACITY));

  gpioInit();
  displayInit ("USB MSC");
  debugInit();
  appInit();

  for (;;) {
    /* Wait for a Set Configuration Event */
    uint32_t eventFlag;
    uint32_t txApiRetStatus = CyU3PEventGet (&MscAppEvent, CY_FX_MSC_SET_CONFIG_EVENT,CYU3P_EVENT_AND_CLEAR,
                                             &eventFlag, CYU3P_WAIT_FOREVER);
    CyU3PDmaChannelConfig_t dmaConfig;
    if (txApiRetStatus == CY_U3P_SUCCESS) {
      //{{{  setConf event
      CyU3PDebugPrint (4, "SetConf event received\r\n");

      /* Based on the Bus Speed configure the DMA buffer size */
      if (glUsbSpeed == CY_U3P_FULL_SPEED) {
        dmaConfig.size = 64;
        glMscSectorSize = 64;     /* Sector size */
        glMscMaxSectors = (CY_FX_MSC_CARD_CAPACITY/64);   /* Maximum no. of sectors on the storage device */
        }
      else if (glUsbSpeed == CY_U3P_HIGH_SPEED) {
        dmaConfig.size = 512;
        glMscSectorSize = 512;    /* Sector size */
        glMscMaxSectors = (CY_FX_MSC_CARD_CAPACITY/512);   /* Maximum no. of sectors on the storage device */
        }
      else if (glUsbSpeed == CY_U3P_SUPER_SPEED) {
        dmaConfig.size = 1024;
        glMscSectorSize = 1024;   /* Sector size */
        glMscMaxSectors = (CY_FX_MSC_CARD_CAPACITY/1024);   /* Maximum no. of sectors on the storage device */
        }
      else {
        CyU3PDebugPrint (4, "Error! USB Not connected\r\n");
        CyFxAppErrorHandler(CY_U3P_ERROR_INVALID_CONFIGURATION);
        }

      /* Create a DMA Manual IN channel between USB Producer socket and the CPU */
      /* DMA size is set above based on the USB Bus Speed */
      dmaConfig.count = CY_FX_MSC_DMA_BUF_COUNT;
      dmaConfig.prodSckId = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_PROD_0 | CY_FX_MSC_EP_BULK_OUT_SOCKET);
      dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
      dmaConfig.dmaMode = CY_U3P_DMA_MODE_BYTE;
      dmaConfig.notification = 0;
      dmaConfig.cb = NULL;
      dmaConfig.prodHeader = 0;
      dmaConfig.prodFooter = 0;
      dmaConfig.consHeader = 0;
      dmaConfig.prodAvailCount = 0;

      /* Create the channel */
      apiRetStatus = CyU3PDmaChannelCreate (&glChHandleMscIn, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);
      if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "DMA IN Channel Creation Failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
        }

      /* Create a DMA Manual OUT channel between CPU and USB consumer socket */
      dmaConfig.count = CY_FX_MSC_DMA_BUF_COUNT;
      dmaConfig.prodSckId = CY_U3P_CPU_SOCKET_PROD;
      dmaConfig.consSckId = (CyU3PDmaSocketId_t)(CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_MSC_EP_BULK_IN_SOCKET);
      dmaConfig.dmaMode = CY_U3P_DMA_MODE_BYTE;
      dmaConfig.cb = NULL;
      dmaConfig.prodHeader = 0;
      dmaConfig.prodFooter = 0;
      dmaConfig.consHeader = 0;
      dmaConfig.prodAvailCount = 0;

      /* Create the channel */
      apiRetStatus = CyU3PDmaChannelCreate (&glChHandleMscOut, CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaConfig);
      if (apiRetStatus != CY_U3P_SUCCESS) {
        CyU3PDebugPrint (4, "DMA OUT Channel Creation Failed, Error Code = %d\r\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
        }

      /* Flush the Endpoint memory */
      CyU3PUsbFlushEp (CY_FX_MSC_EP_BULK_OUT);
      CyU3PUsbFlushEp (CY_FX_MSC_EP_BULK_IN);

      glMscChannelCreated = CyTrue;
      CyU3PDebugPrint (4, "SetConfig handling complete\r\n");
      }
      //}}}

    //{{{  init DMA IN buffer members
    CyU3PDmaBuffer_t dmaMscInBuffer;
    dmaMscInBuffer.buffer = glMscInBuffer;
    dmaMscInBuffer.status = 0;
    dmaMscInBuffer.count = 0;
    dmaMscInBuffer.size = dmaConfig.size;
    //}}}

    /* Whenever we have restarted the USB connection, the read needs to be queued afresh. */
    CyBool_t readQueued = CyFalse;
    for (;;) {
      if (glInPhaseError == CyTrue) {
        //{{{  We will need to queue the read again after the stall is cleared. */
        readQueued = CyFalse;
        continue;
        }
        //}}}

      // Setup IN buffer
      if (!readQueued) {
        apiRetStatus = CyU3PDmaChannelSetupRecvBuffer (&glChHandleMscIn, &dmaMscInBuffer);
        if (apiRetStatus == CY_U3P_SUCCESS) {
          //{{{  allow the USB link to move to U1/U2 while waiting for a command
          readQueued = CyTrue;
          CyU3PUsbLPMEnable();
          }
          //}}}
        }
      if (readQueued) {
        //{{{  wait for CBW received on the IN buffer
        apiRetStatus = CyU3PDmaChannelWaitForRecvBuffer (&glChHandleMscIn, &dmaMscInBuffer, CYU3P_WAIT_FOREVER);
        if (apiRetStatus == CY_U3P_SUCCESS) {
          // Command received. Disable LPM and move link back to U0
          readQueued = CyFalse;
          CyU3PUsbLPMDisable ();
          if (glUsbSpeed == CY_U3P_SUPER_SPEED)
              CyU3PUsbSetLinkPowerState (CyU3PUsbLPM_U0);

          // Validate command
          cswReturnStatus = CyFxMscCheckScsiCmd(glMscInBuffer, dmaMscInBuffer.count);
          if (cswReturnStatus == CY_FX_CBW_CMD_PASSED) {
            // Save the tags in the CSW
            glMscCswStatus[4] = glMscInBuffer[4];
            glMscCswStatus[5] = glMscInBuffer[5];
            glMscCswStatus[6] = glMscInBuffer[6];
            glMscCswStatus[7] = glMscInBuffer[7];

            // Parse the SCSI command and execute the command
            cswReturnStatus = CyFxMscParseScsiCmd (glMscInBuffer);
            }

          CyFxMscSendCsw (cswReturnStatus);
          }
        else
          // Command not received as yet. Don't treat this as an error
          apiRetStatus = CY_U3P_SUCCESS;
        }
        //}}}

      // Check for Reset conditions
      if ((apiRetStatus != CY_U3P_SUCCESS) ||
          (cswReturnStatus == CY_FX_CBW_CMD_MSC_RESET)) {
        //{{{  reset if USB resetEvent
        CyU3PDebugPrint (4, "Stalling MSC endpoints: %d %d\r\n", apiRetStatus, cswReturnStatus);

        /* Stall both the IN and OUT Endpoints. */
        CyU3PUsbStall (CY_FX_MSC_EP_BULK_OUT, 1, 0);
        CyU3PUsbStall (CY_FX_MSC_EP_BULK_IN, 1, 0);

        if (glMscChannelCreated) {
          /* Reset the DMA channels */
          /* Reset the IN channel */
          CyU3PDmaChannelReset(&glChHandleMscIn);

          /* Reset the OUT Channel */
          CyU3PDmaChannelReset(&glChHandleMscOut);
          }

        /* Request Sense Index */
        glReqSenseIndex = CY_FX_MSC_SENSE_DEVICE_RESET;
        }
        //}}}
      if (glUsbSpeed == CY_U3P_NOT_CONNECTED) {
        //{{{  reset if USB disconnected
        if (glMscChannelCreated) {
          /* Destroy the IN channel */
          CyU3PDmaChannelDestroy (&glChHandleMscIn);

          /* Destroy the OUT Channel */
          CyU3PDmaChannelDestroy (&glChHandleMscOut);

          glMscChannelCreated = CyFalse;
          }

        /* Request Sense Index */
        glReqSenseIndex = CY_FX_MSC_SENSE_DEVICE_RESET;

        break;
        }
        //}}}
      }
    }
  }
//}}}

//{{{
void CyFxApplicationDefine() {

  glMscStorageDeviceMemory = (uint8_t*)CyU3PDmaBufferAlloc (CY_FX_MSC_CARD_CAPACITY);

  CyU3PThreadCreate (&MscAppThread,                /* MSC App Thread structure */
                     "25:MSC Application",         /* Thread ID and Thread name */
                     mscThread,                    /* MSC App Thread Entry function */
                     0,                            /* No input parameter to thread */
                     CyU3PMemAlloc (CY_FX_MSC_THREAD_STACK), /* Pointer to the allocated thread stack */
                     CY_FX_MSC_THREAD_STACK,       /* MSC App Thread stack size */
                     CY_FX_MSC_THREAD_PRIORITY,    /* MSC App Thread priority */
                     CY_FX_MSC_THREAD_PRIORITY,    /* MSC App Thread priority */
                     CYU3P_NO_TIME_SLICE,          /* No time slice for the application thread */
                     CYU3P_AUTO_START              /* Start the Thread immediately */
                     );
  }
//}}}

//{{{
int main() {

  CyU3PDeviceInit (0);

  // enable instruction cache for firmware performance
  CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);

  CyU3PIoMatrixConfig_t io_cfg;
  io_cfg.isDQ32Bit        = CyFalse;
  io_cfg.s0Mode           = CY_U3P_SPORT_INACTIVE;
  io_cfg.s1Mode           = CY_U3P_SPORT_INACTIVE;
  io_cfg.useUart          = CyTrue;
  io_cfg.useI2C           = CyFalse;
  io_cfg.useI2S           = CyFalse;
  io_cfg.useSpi           = CyTrue;   // spi display
  io_cfg.lppMode          = CY_U3P_IO_MATRIX_LPP_DEFAULT;
  io_cfg.gpioSimpleEn[0]  = 0;
  io_cfg.gpioSimpleEn[1]  = 0;
  io_cfg.gpioComplexEn[0] = 0;
  io_cfg.gpioComplexEn[1] = 0;
  CyU3PDeviceConfigureIOMatrix (&io_cfg);

  CyU3PKernelEntry ();

  return 0;
  }
//}}}
