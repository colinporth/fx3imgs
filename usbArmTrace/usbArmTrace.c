// usbUVC.c - usb UVC class driver
/*{{{  includes*/
#include <cyu3system.h>
#include <cyu3os.h>
#include <cyu3dma.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3uart.h>
#include <cyu3gpif.h>
#include <cyu3i2c.h>
#include <cyu3gpio.h>
#include <cyu3pib.h>
#include <cyu3utils.h>
#include <cyu3spi.h>

#include "../common/display.h"
#include "../common/sensor.h"
#include "../common/ptz.h"
#include "cyfxgpif2config.h"
/*}}}*/
//#define lines1200
/*{{{  defines*/
#define BUTTON_GPIO 45

// endpoints
#define CY_FX_EP_CONSUMER       0x81 // EP1 in
#define CY_FX_EP_CONTROL_STATUS 0x82 // EP2 IN
#define CY_FX_EP_BULK_VID       0x83 // EP3 IN

/*{{{  BOS for SS codes*/
#define CY_FX_BOS_DSCR_TYPE             15
#define CY_FX_DEVICE_CAPB_DSCR_TYPE     16
#define CY_FX_SS_EP_COMPN_DSCR_TYPE     48
/*}}}*/
/*{{{  Device Capability Type codes*/
#define CY_FX_WIRELESS_USB_CAPB_TYPE    1
#define CY_FX_USB2_EXTN_CAPB_TYPE       2
#define CY_FX_SS_USB_CAPB_TYPE          3
#define CY_FX_CONTAINER_ID_CAPBD_TYPE   4
/*}}}*/

// events
#define STREAM_EVENT        (1 << 0)
#define STREAM_ABORT_EVENT  (1 << 1)
#define VIDEO_CONTROL_EVENT (1 << 2)
#define VIDEO_STREAM_EVENT  (1 << 3)
#define BUTTON_DOWN_EVENT   (1 << 4)
#define BUTTON_UP_EVENT     (1 << 5)

/*{{{  USB and UVC defines*/
#define CY_FX_INTF_ASSN_DSCR_TYPE       (0x0B)          // Type code for Interface Association Descriptor (IAD)

#define CY_FX_USB_SETUP_REQ_TYPE_MASK   (uint32_t)(0x000000FF)  // Mask for bmReqType field from a control request.
#define CY_FX_USB_SETUP_REQ_MASK        (uint32_t)(0x0000FF00)  // Mask for bRequest field from a control request.
#define CY_FX_USB_SETUP_VALUE_MASK      (uint32_t)(0xFFFF0000)  // Mask for wValue field from a control request.
#define CY_FX_USB_SETUP_INDEX_MASK      (uint32_t)(0x0000FFFF)  // Mask for wIndex field from a control request.
#define CY_FX_USB_SETUP_LENGTH_MASK     (uint32_t)(0xFFFF0000)  // Mask for wLength field from a control request.

#define CY_FX_USB_SET_INTF_REQ_TYPE     (uint8_t)(0x01)         // USB SET_INTERFACE Request Type.
#define CY_FX_USB_SET_INTERFACE_REQ     (uint8_t)(0x0B)         // USB SET_INTERFACE Request code.

#define CY_FX_UVC_MAX_HEADER           (12)             // Maximum UVC header size, in bytes.
#define CY_FX_UVC_HEADER_DEFAULT_BFH   (0x8C)           // Default BFH (Bit Field Header) for the UVC Header
#define CY_FX_UVC_MAX_PROBE_SETTING    (26)             // Maximum number of bytes in Probe Control
#define CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED (32)        // Probe control data size aligned to 16 bytes.

#define CY_FX_UVC_HEADER_FRAME          (0)                     // UVC header normal frame indication
#define CY_FX_UVC_HEADER_EOF            (uint8_t)(1 << 1)       // UVC header end of frame indication
#define CY_FX_UVC_HEADER_FRAME_ID       (uint8_t)(1 << 0)       // Frame ID toggle bit in UVC header.

#define CY_FX_USB_UVC_SET_REQ_TYPE      (uint8_t)(0x21)         // UVC Interface SET Request Type
#define CY_FX_USB_UVC_GET_REQ_TYPE      (uint8_t)(0xA1)         // UVC Interface GET Request Type
#define CY_FX_USB_UVC_GET_CUR_REQ       (uint8_t)(0x81)         // UVC GET_CUR Request
#define CY_FX_USB_UVC_SET_CUR_REQ       (uint8_t)(0x01)         // UVC SET_CUR Request
#define CY_FX_USB_UVC_GET_MIN_REQ       (uint8_t)(0x82)         // UVC GET_MIN Request
#define CY_FX_USB_UVC_GET_MAX_REQ       (uint8_t)(0x83)         // UVC GET_MAX Request
#define CY_FX_USB_UVC_GET_RES_REQ       (uint8_t)(0x84)         // UVC GET_RES Request
#define CY_FX_USB_UVC_GET_LEN_REQ       (uint8_t)(0x85)         // UVC GET_LEN Request
#define CY_FX_USB_UVC_GET_INFO_REQ      (uint8_t)(0x86)         // UVC GET_INFO Request
#define CY_FX_USB_UVC_GET_DEF_REQ       (uint8_t)(0x87)         // UVC GET_DEF Request

#define CY_FX_UVC_STREAM_INTERFACE      (uint8_t)(1)            // Streaming Interface : Alternate Setting 1
#define CY_FX_UVC_CONTROL_INTERFACE     (uint8_t)(0)            // Control Interface
#define CY_FX_UVC_PROBE_CTRL            (uint16_t)(0x0100)      // wValue PROBE control.
#define CY_FX_UVC_COMMIT_CTRL           (uint16_t)(0x0200)      // wValue COMMIT control.

#define CY_FX_UVC_INTERFACE_CTRL        (uint8_t)(0)            // wIndex value UVC interface control.
#define CY_FX_UVC_CAMERA_TERMINAL_ID    (uint8_t)(1)            // wIndex value Camera terminal.
#define CY_FX_UVC_PROCESSING_UNIT_ID    (uint8_t)(2)            // wIndex value Processing Unit.
#define CY_FX_UVC_EXTENSION_UNIT_ID     (uint8_t)(3)            // wIndex value Extension Unit.
/*}}}*/
/*{{{  Processing Unit UVC control selector codes*/
#define CY_FX_UVC_PU_BACKLIGHT_COMPENSATION_CONTROL         (uint16_t)(0x0100)
#define CY_FX_UVC_PU_BRIGHTNESS_CONTROL                     (uint16_t)(0x0200)
#define CY_FX_UVC_PU_CONTRAST_CONTROL                       (uint16_t)(0x0300)
#define CY_FX_UVC_PU_GAIN_CONTROL                           (uint16_t)(0x0400)
#define CY_FX_UVC_PU_POWER_LINE_FREQUENCY_CONTROL           (uint16_t)(0x0500)
#define CY_FX_UVC_PU_HUE_CONTROL                            (uint16_t)(0x0600)
#define CY_FX_UVC_PU_SATURATION_CONTROL                     (uint16_t)(0x0700)
#define CY_FX_UVC_PU_SHARPNESS_CONTROL                      (uint16_t)(0x0800)
#define CY_FX_UVC_PU_GAMMA_CONTROL                          (uint16_t)(0x0900)
#define CY_FX_UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL      (uint16_t)(0x0A00)
#define CY_FX_UVC_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL (uint16_t)(0x0B00)
#define CY_FX_UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL        (uint16_t)(0x0C00)
#define CY_FX_UVC_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL   (uint16_t)(0x0D00)
#define CY_FX_UVC_PU_DIGITAL_MULTIPLIER_CONTROL             (uint16_t)(0x0E00)
#define CY_FX_UVC_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL       (uint16_t)(0x0F00)
#define CY_FX_UVC_PU_HUE_AUTO_CONTROL                       (uint16_t)(0x1000)
#define CY_FX_UVC_PU_ANALOG_VIDEO_STANDARD_CONTROL          (uint16_t)(0x1100)
#define CY_FX_UVC_PU_ANALOG_LOCK_STATUS_CONTROL             (uint16_t)(0x1200)
/*}}}*/
/*{{{  Camera Terminal UVC control selector codes*/
#define CY_FX_UVC_CT_SCANNING_MODE_CONTROL                  (uint16_t)(0x0100)
#define CY_FX_UVC_CT_AE_MODE_CONTROL                        (uint16_t)(0x0200)
#define CY_FX_UVC_CT_AE_PRIORITY_CONTROL                    (uint16_t)(0x0300)
#define CY_FX_UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL         (uint16_t)(0x0400)
#define CY_FX_UVC_CT_EXPOSURE_TIME_RELATIVE_CONTROL         (uint16_t)(0x0500)
#define CY_FX_UVC_CT_FOCUS_ABSOLUTE_CONTROL                 (uint16_t)(0x0600)
#define CY_FX_UVC_CT_FOCUS_RELATIVE_CONTROL                 (uint16_t)(0x0700)
#define CY_FX_UVC_CT_FOCUS_AUTO_CONTROL                     (uint16_t)(0x0800)
#define CY_FX_UVC_CT_IRIS_ABSOLUTE_CONTROL                  (uint16_t)(0x0900)
#define CY_FX_UVC_CT_IRIS_RELATIVE_CONTROL                  (uint16_t)(0x0A00)
#define CY_FX_UVC_CT_ZOOM_ABSOLUTE_CONTROL                  (uint16_t)(0x0B00)
#define CY_FX_UVC_CT_ZOOM_RELATIVE_CONTROL                  (uint16_t)(0x0C00)
#define CY_FX_UVC_CT_PANTILT_ABSOLUTE_CONTROL               (uint16_t)(0x0D00)
#define CY_FX_UVC_CT_PANTILT_RELATIVE_CONTROL               (uint16_t)(0x0E00)
#define CY_FX_UVC_CT_ROLL_ABSOLUTE_CONTROL                  (uint16_t)(0x0F00)
#define CY_FX_UVC_CT_ROLL_RELATIVE_CONTROL                  (uint16_t)(0x1000)
#define CY_FX_UVC_CT_PRIVACY_CONTROL                        (uint16_t)(0x1100)
/*}}}*/
/*}}}*/
/*{{{  USB descriptors*/
/*{{{*/
/* Standard Device Descriptor */
static const uint8_t CyFxUSBDeviceDscrHS[] = {
  0x12,                           /* Descriptor Size */
  CY_U3P_USB_DEVICE_DESCR,        /* Device Descriptor Type */
  0x00,0x02,                      /* USB 2.0 */
  0xEF,                           /* Device Class */
  0x02,                           /* Device Sub-class */
  0x01,                           /* Device protocol */
  0x40,                           /* Maxpacket size for EP0 : 64 bytes */
  0xB4,0x04,                      /* Vendor ID */
  0xF8,0x00,                      /* Product ID */
  0x00,0x00,                      /* Device release number */
  0x01,                           /* Manufacture string index */
  0x02,                           /* Product string index */
  0x00,                           /* Serial number string index */
  0x01                            /* Number of configurations */
  };
/*}}}*/
/*{{{*/
/* Device Descriptor for SS */
static const uint8_t CyFxUSBDeviceDscrSS[] = {
  0x12,                           /* Descriptor Size */
  CY_U3P_USB_DEVICE_DESCR,        /* Device Descriptor Type */
  0x00,0x03,                      /* USB 3.0 */
  0xEF,                           /* Device Class */
  0x02,                           /* Device Sub-class */
  0x01,                           /* Device protocol */
  0x09,                           /* Maxpacket size for EP0 : 2^9 Bytes */
  0xB4,0x04,                      /* Vendor ID */
  0xF9,0x00,                      /* Product ID */
  0x00,0x00,                      /* Device release number */
  0x01,                           /* Manufacture string index */
  0x02,                           /* Product string index */
  0x00,                           /* Serial number string index */
  0x01                            /* Number of configurations */
  };
/*}}}*/
/*{{{*/
/* Standard Device Qualifier Descriptor */
static const uint8_t CyFxUSBDeviceQualDscr[] = {
  0x0A,                           /* Descriptor Size */
  CY_U3P_USB_DEVQUAL_DESCR,       /* Device Qualifier Descriptor Type */
  0x00,0x02,                      /* USB 2.0 */
  0xEF,                           /* Device Class */
  0x02,                           /* Device Sub-class */
  0x01,                           /* Device protocol */
  0x40,                           /* Maxpacket size for EP0 : 64 bytes */
  0x01,                           /* Number of configurations */
  0x00                            /* Reserved */
  };
/*}}}*/
/*{{{*/
static const uint8_t CyFxUSBBOSDscr[] = {
  0x05,                           /* Descriptor Size */
  CY_FX_BOS_DSCR_TYPE,            /* Device Descriptor Type */
  0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
  0x02,                           /* Number of device capability descriptors */

  /* USB 2.0 Extension */
  0x07,                           /* Descriptor Size */
  CY_FX_DEVICE_CAPB_DSCR_TYPE,    /* Device Capability Type descriptor */
  CY_FX_USB2_EXTN_CAPB_TYPE,      /* USB 2.0 Extension Capability Type */
  0x00,0x00,0x00,0x00,            /* Supported device level features  */

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
/*}}}*/

// strings
/*{{{*/
/* Standard Language ID String Descriptor */
static const uint8_t CyFxUSBStringLangIDDscr[] = {
  0x04,                           /* Descriptor Size */
  CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
  0x09,0x04                       /* Language ID supported */
  };
/*}}}*/
/*{{{*/
/* Standard Manufacturer String Descriptor */
static const uint8_t CyFxUSBManufactureDscr[] = {
  0x10,                           /* Descriptor Size */
  CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
  'C',0x00, 'y',0x00, 'p',0x00, 'r',0x00, 'e',0x00, 's',0x00, 's',0x00
  };
/*}}}*/
/*{{{*/
/* Standard Product String Descriptor */
static const uint8_t CyFxUSBProductDscr[] = {
  0x08,                           /* Descriptor Size */
  CY_U3P_USB_STRING_DESCR,        /* Device Descriptor Type */
  'F',0x00, 'X',0x00, '3',0x00
  };
/*}}}*/

/*{{{*/
static const uint8_t FullSpeedConfigurationDescriptor[] = {
  // Configuration Descriptor Type
  0x09,                           /* Descriptor Size */
  CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
  0x09,0x00,                      /* Length of this descriptor and all sub descriptors */
  0x00,                           /* Number of interfaces */
  0x01,                           /* Configuration number */
  0x00,                           /* Configuration string index */
  0x80,                           /* Config characteristics - Bus powered */
  0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */
  };
/*}}}*/
/*{{{*/
static const uint8_t HighSpeedConfigurationDescriptor[] = {
  /*{{{  Configuration Descriptor Type*/
  0x09,                           /* Descriptor Size */
  CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
  0xDD,0x00,                      /* Length of this descriptor and all sub descriptors */
  0x03,                           /* Number of interfaces */
  0x01,                           /* Configuration number */
  0x00,                           /* Configuration string index */
  0x80,                           /* Config characteristics - Bus powered */
  0xFA,                           /* Max power consumption of device (in 2mA unit) : 500mA */
  /*}}}*/
  /*{{{  Interface Association Descriptor*/
  0x08,                           /* Descriptor Size */
  CY_FX_INTF_ASSN_DSCR_TYPE,      /* Interface Association Descr Type: 11 */
  0x00,                           /* I/f number of first VideoControl i/f */
  0x02,                           /* Number of Video i/f */
  0x0E,                           /* CC_VIDEO : Video i/f class code */
  0x03,                           /* SC_VIDEO_INTERFACE_COLLECTION : Subclass code */
  0x00,                           /* Protocol : Not used */
  0x00,                           /* String desc index for interface */
  /*}}}*/
  /*{{{  Standard Video Control Interface Descriptor*/
  0x09,                           /* Descriptor size */
  CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
  0x00,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x01,                           /* Number of end points */
  0x0E,                           /* CC_VIDEO : Interface class */
  0x01,                           /* CC_VIDEOCONTROL : Interface sub class */
  0x00,                           /* Interface protocol code */
  0x00,                           /* Interface descriptor string index */
  /*}}}*/
  /*{{{  Class specific VC Interface Header Descriptor*/
  0x0D,                           /* Descriptor size */
  0x24,                           /* Class Specific I/f Header Descriptor type */
  0x01,                           /* Descriptor Sub type : VC_HEADER */
  0x00,0x01,                      /* Revision of class spec : 1.0 */
  0x50,0x00,                      /* Total Size of class specific descriptors (till Output terminal) */
  0x00,0x6C,0xDC,0x02,            /* Clock frequency : 48MHz(Deprecated) */
  0x01,                           /* Number of streaming interfaces */
  0x01,                           /* Video streaming I/f 1 belongs to VC i/f */
  /*}}}*/
  /*{{{  Input (Camera) Terminal Descriptor*/
  0x12,                           /* Descriptor size */
  0x24,                           /* Class specific interface desc type */
  0x02,                           /* Input Terminal Descriptor type */
  0x01,                           /* ID of this terminal */
  0x01,0x02,                      /* Camera terminal type */
  0x00,                           /* No association terminal */
  0x00,                           /* String desc index : Not used */
  (uint8_t)(wObjectiveFocalLengthMin&0xFF),
  (uint8_t)((wObjectiveFocalLengthMin>>8)&0xFF),
  (uint8_t)(wObjectiveFocalLengthMax&0xFF),
  (uint8_t)((wObjectiveFocalLengthMax>>8)&0xFF),
  (uint8_t)(wOcularFocalLength&0xFF),
  (uint8_t)((wOcularFocalLength>>8)&0xFF),
  0x03,                           /* Size of controls field for this terminal : 3 bytes */
                                  /* A bit set to 1 indicates that the mentioned Control is
                                   * supported for the video stream in the bmControls field
                                   * D0: Scanning Mode
                                   * D1: Auto-Exposure Mode
                                   * D2: Auto-Exposure Priority
                                   * D3: Exposure Time (Absolute)
                                   * D4: Exposure Time (Relative)
                                   * D5: Focus (Absolute)
                                   * D6: Focus (Relative)
                                   * D7: Iris (Absolute)
                                   * D8: Iris (Relative)
                                   * D9: Zoom (Absolute)
                                   * D10: Zoom (Relative)
                                   * D11: PanTilt (Absolute)
                                   * D12: PanTilt (Relative)
                                   * D13: Roll (Absolute)
                                   * D14: Roll (Relative)
                                   * D15: Reserved
                                   * D16: Reserved
                                   * D17: Focus, Auto
                                   * D18: Privacy
                                   * D19: Focus, Simple
                                   * D20: Window
                                   * D21: Region of Interest
                                   * D22 – D23: Reserved, set to zero
                                   */
  0x00,0x0A,0x00,                 /* bmControls field of camera terminal: PTZ supported */
  /*}}}*/
  /*{{{  Processing Unit Descriptor*/
  0x0C,                           /* Descriptor size */
  0x24,                           /* Class specific interface desc type */
  0x05,                           /* Processing Unit Descriptor type */
  0x02,                           /* ID of this terminal */
  0x01,                           /* Source ID : 1 : Conencted to input terminal */
  0x00,0x40,                      /* Digital multiplier */
  0x03,                           /* Size of controls field for this terminal : 3 bytes */
                                  /* A bit set to 1 in the bmControls field indicates that
                                   * the mentioned Control is supported for the video stream.
                                   * D0: Brightness
                                   * D1: Contrast
                                   * D2: Hue
                                   * D3: Saturation
                                   * D4: Sharpness
                                   * D5: Gamma
                                   * D6: White Balance Temperature
                                   * D7: White Balance Component
                                   * D8: Backlight Compensation
                                   * D9: Gain
                                   * D10: Power Line Frequency
                                   * D11: Hue, Auto
                                   * D12: White Balance Temperature, Auto
                                   * D13: White Balance Component, Auto
                                   * D14: Digital Multiplier
                                   * D15: Digital Multiplier Limit
                                   * D16: Analog Video Standard
                                   * D17: Analog Video Lock Status
                                   * D18: Contrast, Auto
                                   * D19 – D23: Reserved. Set to zero.
                                   */
  0x01,0x00,0x00,                 /* bmControls field of processing unit: Brightness control supported */
  0x00,                           /* String desc index : Not used */
  /*}}}*/
  /*{{{  Extension Unit Descriptor*/
  0x1C,                           /* Descriptor size */
  0x24,                           /* Class specific interface desc type */
  0x06,                           /* Extension Unit Descriptor type */
  0x03,                           /* ID of this terminal */
  0xFF,0xFF,0xFF,0xFF,            /* 16 byte GUID */
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,
  0x00,                           /* Number of controls in this terminal */
  0x01,                           /* Number of input pins in this terminal */
  0x02,                           /* Source ID : 2 : Connected to Proc Unit */
  0x03,                           /* Size of controls field for this terminal : 3 bytes */
  0x00,0x00,0x00,                 /* No controls supported */
  0x00,                           /* String desc index : Not used */
  /*}}}*/
  /*{{{  Output Terminal Descriptor*/
  0x09,                           /* Descriptor size */
  0x24,                           /* Class specific interface desc type */
  0x03,                           /* Output Terminal Descriptor type */
  0x04,                           /* ID of this terminal */
  0x01,0x01,                      /* USB Streaming terminal type */
  0x00,                           /* No association terminal */
  0x03,                           /* Source ID : 3 : Connected to Extn Unit */
  0x00,                           /* String desc index : Not used */
  /*}}}*/
  /*{{{  Video Control Status Interrupt Endpoint Descriptor*/
  0x07,                           /* Descriptor size */
  CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
  CY_FX_EP_CONTROL_STATUS,        /* Endpoint address and description */
  CY_U3P_USB_EP_INTR,             /* Interrupt End point Type */
  0x40,0x00,                      /* Max packet size = 64 bytes */
  0x08,                           /* Servicing interval : 8ms */
  /*}}}*/
  /*{{{  Class Specific Interrupt Endpoint Descriptor*/
  0x05,                           /* Descriptor size */
  0x25,                           /* Class Specific Endpoint Descriptor Type */
  CY_U3P_USB_EP_INTR,             /* End point Sub Type */
  0x40,0x00,                      /* Max packet size = 64 bytes */
  /*}}}*/
  /*{{{  Standard Video Streaming Interface Descriptor (Alternate Setting 0)*/
  0x09,                           /* Descriptor size */
  CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
  0x01,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x01,                           /* Number of end points : Zero Bandwidth */
  0x0E,                           /* Interface class : CC_VIDEO */
  0x02,                           /* Interface sub class : CC_VIDEOSTREAMING */
  0x00,                           /* Interface protocol code : Undefined */
  0x00,                           /* Interface descriptor string index */
  /*}}}*/
  /*{{{  Class-specific Video Streaming Input Header Descriptor*/
  0x0E,                           /* Descriptor size */
  0x24,                           /* Class-specific VS I/f Type */
  0x01,                           /* Descriptotor Subtype : Input Header */
  0x01,                           /* 1 format desciptor follows */
  0x29,0x00,                      /* Total size of Class specific VS descr: 41 Bytes */
  CY_FX_EP_BULK_VID,              /* EP address for BULK video data */
  0x00,                           /* No dynamic format change supported */
  0x04,                           /* Output terminal ID : 4 */
  0x01,                           /* Still image capture method 1 supported */
  0x00,                           /* Hardware trigger NOT supported */
  0x00,                           /* Hardware to initiate still image capture NOT supported */
  0x01,                           /* Size of controls field : 1 byte */
  0x00,                           /* D2 : Compression quality supported */
  /*}}}*/
  /*{{{  Class specific Uncompressed VS format descriptor*/
  0x1B,                           /* Descriptor size */
  0x24,                           /* Class-specific VS I/f Type */
  0x04,                           /* Subtype : uncompressed format I/F */
  0x01,                           /* Format desciptor index (only one format is supported) */
  0x01,                           /* number of frame descriptor followed */
  0x59,0x55,0x59,0x32, 0x00,0x00,0x10,0x00, 0x80,0x00,0x00,0xAA, 0x00,0x38,0x9B,0x71, // YUY2 guid
  0x10,                           /* Number of bits per pixel used to specify color in the decoded video frame. 0 if not applicable: 10 bit per pixel */
  0x01,                           /* Optimum Frame Index for this stream: 1 */
  0x08,                           /* X dimension of the picture aspect ratio: Non-interlaced in progressive scan */
  0x06,                           /* Y dimension of the picture aspect ratio: Non-interlaced in progressive scan*/
  0x00,                           /* Interlace Flags: Progressive scanning, no interlace */
  0x00,                           /* duplication of the video stream restriction: 0 - no restriction */
  /*}}}*/
  /*{{{  Class specific Uncompressed VS Frame descriptor*/
  0x1E,                           /* Descriptor size */
  0x24,                           /* Descriptor type*/

  0x05,                           /* Subtype: uncompressed frame I/F */
  0x01,                           /* Frame Descriptor Index */
  0x03,                           /* Still image capture method 1 supported, fixed frame rate */

  0x20, 0x03,                     /* Width in pixel - 800 */
  0x58, 0x02,                     /* Height in pixel- 600 */

  0x00,0x50,0x97,0x31,            /* Min bit rate bits/s. Not specified, taken from MJPEG */
  0x00,0x50,0x97,0x31,            /* Max bit rate bits/s. Not specified, taken from MJPEG */

  0x00,0x60,0x09,0x00,            /* Maximum video or still frame size in bytes(Deprecated) */

  0x2A,0x2C,0x0A,0x00,            /* Default Frame Interval */
  0x01,                           /* Frame interval(Frame Rate) types: Only one frame interval supported */
  0x2A,0x2C,0x0A,0x00,            /* Shortest Frame Interval */
  /*}}}*/
  /*{{{  Endpoint Descriptor for BULK Streaming Video Data*/
  0x07,                           /* Descriptor size */
  CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
  CY_FX_EP_BULK_VID,              /* Endpoint address and description */
  0x02,                           /* BULK End point */
  (uint8_t)(512 & 0x00FF),        /* High speed max packet size is always 512 bytes. */
  (uint8_t)((512 & 0xFF00)>>8),
  0x01,                           /* Servicing interval for data transfers */
  /*}}}*/
  /*{{{  interface descriptor*/
  0x09,                           /* Descriptor size */
  CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
  0x02,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x01,                           /* Number of end points */
  0xFF,                           /* Interface class */
  0x00,                           /* Interface sub class */
  0x00,                           /* Interface protocol code */
  0x00,                           /* Interface descriptor string index */
  /*}}}*/
  /*{{{  Endpoint descriptor for consumer EP*/
  0x07,                           /* Descriptor size */
  CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
  CY_FX_EP_CONSUMER,              /* Endpoint address and description */
  CY_U3P_USB_EP_BULK,             /* Bulk endpoint type */
  0x00,0x02,                      /* Max packet size = 512 bytes */
  0x00                            /* Servicing interval for data transfers : 0 for bulk */
  /*}}}*/
  };
/*}}}*/
/*{{{*/
static const uint8_t SuperSpeedConfigurationDescriptor[] = {
  /*{{{  Configuration Descriptor Type*/
  0x09,                           /* Descriptor Size */
  CY_U3P_USB_CONFIG_DESCR,        /* Configuration Descriptor Type */
  0xEF,0x00,                      /* Total length of this and all sub-descriptors. */
  0x03,                           /* Number of interfaces */
  0x01,                           /* Configuration number */
  0x00,                           /* Configuration string index */
  0x80,                           /* Config characteristics - Bus powered */
  0x32,                           /* Max power consumption of device (in 8mA unit) : 400mA */
  /*}}}*/
  /*{{{  Interface Association Descriptor*/
  0x08,                           /* Descriptor Size */
  CY_FX_INTF_ASSN_DSCR_TYPE,      /* Interface Association Descr Type: 11 */
  0x00,                           /* I/f number of first VideoControl i/f */
  0x02,                           /* Number of Video i/f */
  0x0E,                           /* CC_VIDEO : Video i/f class code */
  0x03,                           /* SC_VIDEO_INTERFACE_COLLECTION : Subclass code */
  0x00,                           /* Protocol : Not used */
  0x00,                           /* String desc index for interface */
  /*}}}*/
  /*{{{  Standard Video Control Interface Descriptor*/
  0x09,                           /* Descriptor size */
  CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
  0x00,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x01,                           /* Number of end points */
  0x0E,                           /* CC_VIDEO : Interface class */
  0x01,                           /* CC_VIDEOCONTROL : Interface sub class */
  0x00,                           /* Interface protocol code */
  0x00,                           /* Interface descriptor string index */
  /*}}}*/
  /*{{{  Class specific VC Interface Header Descriptor*/
  0x0D,                           /* Descriptor size */
  0x24,                           /* Class Specific I/f Header Descriptor type */
  0x01,                           /* Descriptor Sub type : VC_HEADER */
  0x00,0x01,                      /* Revision of class spec : 1.0 */
  0x50,0x00,                      /* Total Size of class specific descriptors (till Output terminal) */
  0x00,0x6C,0xDC,0x02,            /* Clock frequency : 48MHz(Deprecated) */
  0x01,                           /* Number of streaming interfaces */
  0x01,                           /* Video streaming I/f 1 belongs to VC i/f */
  /*}}}*/
  /*{{{  Input (Camera) Terminal Descriptor*/
  0x12,                           /* Descriptor size */
  0x24,                           /* Class specific interface desc type */
  0x02,                           /* Input Terminal Descriptor type */
  0x01,                           /* ID of this terminal */
  0x01,0x02,                      /* Camera terminal type */
  0x00,                           /* No association terminal */
  0x00,                           /* String desc index : Not used */
  (uint8_t)(wObjectiveFocalLengthMin&0xFF),
  (uint8_t)((wObjectiveFocalLengthMin>>8)&0xFF),
  (uint8_t)(wObjectiveFocalLengthMax&0xFF),
  (uint8_t)((wObjectiveFocalLengthMax>>8)&0xFF),
  (uint8_t)(wOcularFocalLength&0xFF),
  (uint8_t)((wOcularFocalLength>>8)&0xFF),
  0x03,                           /* Size of controls field for this terminal : 3 bytes */
                                  /* A bit set to 1 in the bmControls field indicates that
                                   * the mentioned Control is supported for the video stream.
                                   * D0: Scanning Mode
                                   * D1: Auto-Exposure Mode
                                   * D2: Auto-Exposure Priority
                                   * D3: Exposure Time (Absolute)
                                   * D4: Exposure Time (Relative)
                                   * D5: Focus (Absolute)
                                   * D6: Focus (Relative)
                                   * D7: Iris (Absolute)
                                   * D8: Iris (Relative)
                                   * D9: Zoom (Absolute)
                                   * D10: Zoom (Relative)
                                   * D11: PanTilt (Absolute)
                                   * D12: PanTilt (Relative)
                                   * D13: Roll (Absolute)
                                   * D14: Roll (Relative)
                                   * D15: Reserved
                                   * D16: Reserved
                                   * D17: Focus, Auto
                                   * D18: Privacy
                                   * D19: Focus, Simple
                                   * D20: Window
                                   * D21: Region of Interest
                                   * D22 – D23: Reserved, set to zero
                                   */
  0x00,0x0A,0x00,                 /* bmControls field of camera terminal: PTZ supported */
  /*}}}*/
  /*{{{  Processing Unit Descriptor*/
  0x0C,                           /* Descriptor size */
  0x24,                           /* Class specific interface desc type */
  0x05,                           /* Processing Unit Descriptor type */
  0x02,                           /* ID of this terminal */
  0x01,                           /* Source ID : 1 : Conencted to input terminal */
  0x00,0x40,                      /* Digital multiplier */
  0x03,                           /* Size of controls field for this terminal : 3 bytes */
  0x01,0x00,0x00,                 /* bmControls field of processing unit: Brightness control supported */
  0x00,                           /* String desc index : Not used */
  /*}}}*/
  /*{{{  Extension Unit Descriptor*/
  0x1C,                           /* Descriptor size */
  0x24,                           /* Class specific interface desc type */
  0x06,                           /* Extension Unit Descriptor type */
  0x03,                           /* ID of this terminal */
  0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFF, 0xFF,0xFF,0xFF,0xFF,  /* 16 byte GUID */
  0x00,                           /* Number of controls in this terminal */
  0x01,                           /* Number of input pins in this terminal */
  0x02,                           /* Source ID : 2 : Connected to Proc Unit */
  0x03,                           /* Size of controls field for this terminal : 3 bytes */
                                  /* A bit set to 1 in the bmControls field indicates that
                                   * the mentioned Control is supported for the video stream.
                                   * D0: Brightness
                                   * D1: Contrast
                                   * D2: Hue
                                   * D3: Saturation
                                   * D4: Sharpness
                                   * D5: Gamma
                                   * D6: White Balance Temperature
                                   * D7: White Balance Component
                                   * D8: Backlight Compensation
                                   * D9: Gain
                                   * D10: Power Line Frequency
                                   * D11: Hue, Auto
                                   * D12: White Balance Temperature, Auto
                                   * D13: White Balance Component, Auto
                                   * D14: Digital Multiplier
                                   * D15: Digital Multiplier Limit
                                   * D16: Analog Video Standard
                                   * D17: Analog Video Lock Status
                                   * D18: Contrast, Auto
                                   * D19 – D23: Reserved. Set to zero.
                                   */
  0x00,0x00,0x00,                 /* No controls supported */
  0x00,                           /* String desc index : Not used */
  /*}}}*/
  /*{{{  Output Terminal Descriptor*/
  0x09,                           /* Descriptor size */
  0x24,                           /* Class specific interface desc type */
  0x03,                           /* Output Terminal Descriptor type */
  0x04,                           /* ID of this terminal */
  0x01,0x01,                      /* USB Streaming terminal type */
  0x00,                           /* No association terminal */
  0x03,                           /* Source ID : 3 : Connected to Extn Unit */
  0x00,                           /* String desc index : Not used */
  /*}}}*/
  /*{{{  Video Control Status Interrupt Endpoint Descriptor*/
  0x07,                           /* Descriptor size */
  CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
  CY_FX_EP_CONTROL_STATUS,        /* Endpoint address and description */
  CY_U3P_USB_EP_INTR,             /* Interrupt End point Type */
  0x00,0x04,                      /* Max packet size = 1024 bytes */
  0x01,                           /* Servicing interval */
  /*}}}*/
  /*{{{  Super Speed Endpoint Companion Descriptor*/
  0x06,                           /* Descriptor size */
  CY_U3P_SS_EP_COMPN_DESCR,       /* SS Endpoint Companion Descriptor Type */
  0x00,                           /* Max no. of packets in a Burst : 1 */
  0x00,                           /* Attribute: N.A. */
  0x00,                           /* Bytes per interval:1024 */
  0x04,
  /*}}}*/
  /*{{{  Class Specific Interrupt Endpoint Descriptor*/
  0x05,                           /* Descriptor size */
  0x25,                           /* Class Specific Endpoint Descriptor Type */
  CY_U3P_USB_EP_INTR,             /* End point Sub Type */
  0x40,0x00,                      /* Max packet size = 64 bytes */
  /*}}}*/
  /*{{{  Standard Video Streaming Interface Descriptor (Alternate Setting 0)*/
  0x09,                           /* Descriptor size */
  CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
  0x01,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x01,                           /* Number of end points */
  0x0E,                           /* Interface class : CC_VIDEO */
  0x02,                           /* Interface sub class : CC_VIDEOSTREAMING */
  0x00,                           /* Interface protocol code : Undefined */
  0x00,                           /* Interface descriptor string index */
  /*}}}*/
  /*{{{  Class-specific Video Streaming Input Header Descriptor*/
  0x0E,                           /* Descriptor size */
  0x24,                           /* Class-specific VS I/f Type */
  0x01,                           /* Descriptotor Subtype : Input Header */
  0x01,                           /* 1 format desciptor follows */
  0x47,0x00,                      /* Total size of Class specific VS descr */
  CY_FX_EP_BULK_VID,              /* EP address for BULK video data */
  0x00,                           /* No dynamic format change supported */
  0x04,                           /* Output terminal ID : 4 */
  0x01,                           /* Still image capture method 1 supported */
  0x00,                           /* Hardware trigger NOT supported */
  0x00,                           /* Hardware to initiate still image capture NOT supported */
  0x01,                           /* Size of controls field : 1 byte */
  0x00,                           /* D2 : Compression quality supported */
  /*}}}*/
  /*{{{  Class specific Uncompressed VS format descriptor*/
  0x1B,                           /* Descriptor size */
  0x24,                           /* Class-specific VS I/f Type */
  0x04,                           /* Subtype : uncompressed format I/F */

  0x01,                           /* Format desciptor index */
  0x01,                           /* Number of frame descriptor followed */

  0x59,0x55,0x59,0x32, 0x00,0x00,0x10,0x00, 0x80,0x00,0x00,0xAA, 0x00,0x38,0x9B,0x71, // YUY2 guid
  0x10,                           /* Number of bits per pixel */
  0x01,                           /* Optimum Frame Index for this stream: 1 */

  0x08,                           /* X dimension of the picture aspect ratio; Non-interlaced */
  0x06,                           /* Y dimension of the pictuer aspect ratio: Non-interlaced */

  0x00,                           /* Interlace Flags: Progressive scanning, no interlace */
  0x00,                           /* duplication of the video stream restriction: 0 - no restriction */
  /*}}}*/
  /*{{{  Class specific Uncompressed VS frame descriptor*/
  0x1E,                           /* Descriptor size */
  0x24,                           /* Descriptor type*/
  0x05,                           /* Subtype: uncompressed frame I/F */

  0x01,                           /* Frame Descriptor Index */
  0x03,                           /* Still image capture method 1 supported, fixed frame rate */

  #ifdef lines1200
  0x40, 0x06,                     /* Width in pixel - 1600 */
  0xB0, 0x04,                     /* Height in pixel- 1200 */
  #else
  0x20, 0x03,                     /* Width in pixel - 800 */
  0x58, 0x02,                     /* Height in pixel- 600 */
  #endif

  0x00,0x50,0x97,0x31,            /* Min bit rate bits/s. */
  0x00,0x50,0x97,0x31,            /* Max bit rate bits/s. */

  0x00,0xA4,0x1F,0x00,            /* Maximum video or still frame size in bytes(Deprecated)*/

  0x15, 0x16, 0x05, 0x00, 0x01, 0x15,0x16,0x05,0x00,  /* 30fps */
  /*}}}*/
  /*{{{  Endpoint Descriptor for BULK Streaming Video Data*/
  0x07,                           /* Descriptor size */
  CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
  CY_FX_EP_BULK_VID,              /* Endpoint address and description */
  CY_U3P_USB_EP_BULK,             /* BULK End point */
  0x00,0x04,                      /* Max packet size = 1024 bytes */
  0x01,                           /* Servicing interval for data transfers */
  /*}}}*/
  /*{{{  Super Speed Endpoint Companion Descriptor*/
  0x06,                           /* Descriptor size */
  CY_U3P_SS_EP_COMPN_DESCR,       /* SS Endpoint Companion Descriptor Type */
  0x0F,                           /* Max number of packets per burst: 16 */
  0x00,                           /* Attribute: Streams not defined */
  0x00,                           /* No meaning for bulk */
  0x00,
  /*}}}*/
  /*{{{  interface descriptor - bulk endpoints*/
  0x09,                           /* Descriptor size */
  CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
  0x02,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x01,                           /* Number of end points */
  0xFF,                           /* Interface class */
  0x00,                           /* Interface sub class */
  0x00,                           /* Interface protocol code */
  0x00,                           /* Interface descriptor string index */
  /*}}}*/
  /*{{{  Endpoint descriptor for consumer in EP1*/
  0x07,                           /* Descriptor size */
  CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint descriptor type */
  CY_FX_EP_CONSUMER,              /* Endpoint address and description */
  CY_U3P_USB_EP_BULK,             /* Bulk endpoint type */
  0x00,0x04,                      /* Max packet size = 1024 bytes */
  0x01,                           /* Servicing interval for data transfers : 0 for Bulk */
  /*}}}*/
  /*{{{  Super speed endpoint companion descriptor for consumer in EP3*/
  0x06,                           /* Descriptor size */
  CY_U3P_SS_EP_COMPN_DESCR,       /* SS endpoint companion descriptor type */
  15,                             /* Max no. of packets in a burst(0-15) - 0: burst 1 packet at a time */
  0x00,                           /* Max streams for bulk EP = 0 (No streams) */
  0x00,0x00,                      /* Service interval for the EP : 0 for bulk */
  /*}}}*/
  };
/*}}}*/
/*}}}*/
/*{{{  vars*/
static CyU3PThread vidThread;        // UVC video streaming thread
static CyU3PThread controlThread;    // UVC control request handling thread
static CyU3PDmaMultiChannel dmaMultiChannel;
static CyU3PEvent uvcEvent;          // Event group used to signal threads

// Current UVC control request fields. See USB specification for definition
static uint8_t bReqType;
static uint8_t bRequest;
static uint8_t bType;
static uint8_t bTarget;
static uint16_t wValue;
static uint16_t wIndex;
static uint16_t wLength;

static CyBool_t analyserMode = CyFalse;         // Whether USB host has started streaming data
static CyBool_t streamingStarted = CyFalse;         // Whether USB host has started streaming data
static CyBool_t clearFeatureRqtReceived = CyFalse;  // Whether a CLEAR_FEATURE (stop streaming) request
static CyU3PUSBSpeed_t usbSpeed = CY_U3P_NOT_CONNECTED; // Current USB connection speed
static uint8_t backFlowDetected = 0;                // Whether buffer overflow error is detected

// Scratch buffer used for handling UVC class requests with a data phase
static uint8_t glEp0Buffer[32];
/*{{{*/
/* UVC Header to be prefixed at the top of each 16 KB video data buffer. */
static uint8_t volatile uvcHeader[CY_FX_UVC_MAX_HEADER] = {
  0x0C,                               /* Header Length */
  0x8C,                               /* Bit field header field */
  0x00, 0x00, 0x00, 0x00,             /* Presentation time stamp field */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* Source clock reference field */
  };
/*}}}*/
/*{{{*/
/* UVC Header to be prefixed at the top of each 16 KB video data buffer. */
static uint8_t volatile uvcHeaderEOF[CY_FX_UVC_MAX_HEADER] = {
  0x0C,                               /* Header Length */
  0x8E,                               /* Bit field header field + EOF */
  0x00, 0x00, 0x00, 0x00,             /* Presentation time stamp field */
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* Source clock reference field */
  };
/*}}}*/

volatile static CyBool_t gpifInitialized = CyFalse;  // Whether the GPIF init function has been called
volatile static CyBool_t gotPartial = CyFalse;       // track last partial buffer ensure committed to USB
volatile static CyBool_t hitFV = CyFalse;            // Whether end of frame (FV) signal has been hit
volatile static uint16_t prodCount = 0;              // Count of buffers received and committed during the current video frame
volatile static uint16_t consCount = 0;              // Count of buffers received and committed during the current video frame

// Video Probe Commit Control, filled out when the host sends down the SET_CUR request
static uint8_t commitCtrl[CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED];
/*{{{*/
/* UVC Probe Control Setting for a USB 2.0 connection. */
static uint8_t probeCtrl20[CY_FX_UVC_MAX_PROBE_SETTING] = {
  0x00, 0x00,                 /* bmHint : no hit */
  0x01,                       /* Use 1st Video format index */
  0x01,                       /* Use 1st Video frame index */
  0x2A, 0x2C, 0x0A, 0x00,     /* Desired frame interval in the unit of 100ns: 15 fps */
  0x00, 0x00,                 /* Key frame rate in key frame/video frame units: only applicable
                                 to video streaming with adjustable compression parameters */
  0x00, 0x00,                 /* PFrame rate in PFrame / key frame units: only applicable to
                                video streaming with adjustable compression parameters */
  0x00, 0x00,                 /* Compression quality control: only applicable to video streaming
                                 with adjustable compression parameters */
  0x00, 0x00,                 /* Window size for average bit rate: only applicable to video
                                 streaming with adjustable compression parameters */
  0x00, 0x00,                 /* Internal video streaming i/f latency in ms */
  0x00, 0x60, 0x09, 0x00,     /* Max video frame size in bytes */
  0x00, 0x40, 0x00, 0x00      /* No. of bytes device can rx in single payload = 16 KB */
  };
/*}}}*/
/*{{{*/
/* UVC Probe Control Settings for a USB 3.0 connection. */
static uint8_t probeCtrl30[CY_FX_UVC_MAX_PROBE_SETTING] = {
  0x00, 0x00,                 /* bmHint : no hit */
  0x01,                       /* Use 1st Video format index */
  0x01,                       /* Use 1st Video frame index */
  0x15, 0x16, 0x05, 0x00,     /* Desired frame interval in the unit of 100ns: 30 fps */
  0x00, 0x00,                 /* Key frame rate in key frame/video frame units: only applicable
                                 to video streaming with adjustable compression parameters */
  0x00, 0x00,                 /* PFrame rate in PFrame / key frame units: only applicable to
                                 video streaming with adjustable compression parameters */
  0x00, 0x00,                 /* Compression quality control: only applicable to video streaming
                                 with adjustable compression parameters */
  0x00, 0x00,                 /* Window size for average bit rate: only applicable to video
                                 streaming with adjustable compression parameters */
  0x00, 0x00,                 /* Internal video streaming i/f latency in ms */
  0x00, 0x48, 0x3F, 0x00,     /* Max video frame size in bytes */
  0x00, 0x40, 0x00, 0x00      /* No. of bytes device can rx in single payload = 16 KB */
  };
/*}}}*/
/*}}}*/

// button interrupt
/*{{{*/
static void gpioInterruptCallback (uint8_t gpioId) {

  CyBool_t gpioValue = CyFalse;
  if (gpioId == BUTTON_GPIO)
    if (CyU3PGpioGetValue (gpioId, &gpioValue) == CY_U3P_SUCCESS)
      CyU3PEventSet (&uvcEvent, gpioValue ? BUTTON_UP_EVENT : BUTTON_DOWN_EVENT, CYU3P_EVENT_OR);
  }
/*}}}*/

// vid thread
/*{{{*/
static void vidDmaCallback (CyU3PDmaMultiChannel* multiChHandle, CyU3PDmaCbType_t type, CyU3PDmaCBInput_t* input) {
// vid manual DMA callback, each buffer produced by gpif vid

  if (type == CY_U3P_DMA_CB_CONS_EVENT) {
    consCount++;
    streamingStarted = CyTrue;
    }
  }
/*}}}*/
/*{{{*/
static void gpifCallback (CyU3PGpifEventType event, uint8_t currentState) {
// vid gpif endOfFrame callback

  if (event == CYU3P_GPIF_EVT_SM_INTERRUPT) {
    hitFV = CyTrue;

    switch (currentState) {
      //case FULL_BUF_IN_SCK0:
      //case FULL_BUF_IN_SCK1:
        // Buffer is already full and would have been committed. Do nothing
      //  break;

      //case PARTIAL_BUF_IN_SCK0:
      //  if (CyU3PDmaMultiChannelSetWrapUp (&dmaMultiChannel, 0) != CY_U3P_SUCCESS)
      //    CyU3PDebugPrint (4, "CyFxGpifCallback Channel Set WrapUp failed\r\n");
      //  gotPartial = CyTrue;
      //  break;

      //case PARTIAL_BUF_IN_SCK1:
      //  if (CyU3PDmaMultiChannelSetWrapUp (&dmaMultiChannel, 1) != CY_U3P_SUCCESS)
      //    CyU3PDebugPrint (4, "CyFxGpifCallback Channel Set WrapUp failed\r\n");
      //  gotPartial = CyTrue;
      //  break;

      default:
        CyU3PDebugPrint (4, "CyFxGpifCallback failed!\n");
        break;
      }
    }
  }
/*}}}*/

/*{{{*/
static void vidThreadFunc (uint32_t input) {

  //uint32_t frameCnt = 0;
  CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

  for (;;) {
    uint32_t flag;
    if (CyU3PEventGet (&uvcEvent, STREAM_EVENT, CYU3P_EVENT_AND, &flag, CYU3P_NO_WAIT) == CY_U3P_SUCCESS) {
      // gpif producer buffer ready
      CyU3PDmaBuffer_t produced_buffer;
      if (CyU3PDmaMultiChannelGetBuffer (&dmaMultiChannel, &produced_buffer, CYU3P_NO_WAIT) == CY_U3P_SUCCESS) {
        /*{{{  add header, commit to consumer endpoint*/
        if (produced_buffer.count == 16384 - 16) {
          // full buffer, add normal header to buffer
          }

        else {
          // partial buffer, add EOF header to buffer
          gotPartial = CyFalse;
          }

        // commit buffer to consumer endpoint
        prodCount++;

        status = CyU3PDmaMultiChannelCommitBuffer (&dmaMultiChannel, produced_buffer.count, 0);
        if (status != CY_U3P_SUCCESS) {
          //line3 ("err", status);
          prodCount--;
          }
        }
        /*}}}*/

      if (hitFV && (prodCount == consCount) && !gotPartial) {
        /*{{{  endOfFrame, restart next frame*/
        //line3 ("f", frameCnt++);
        prodCount = 0;
        consCount = 0;
        hitFV = CyFalse;
        backFlowDetected = 0;

        // Toggle UVC header FRAME ID bit
        uvcHeader[1] ^= CY_FX_UVC_HEADER_FRAME_ID;
        uvcHeaderEOF[1] =  uvcHeader[1];

        // restart dma, gpif
        CyU3PDmaMultiChannelReset (&dmaMultiChannel);
        CyU3PDmaMultiChannelSetXfer (&dmaMultiChannel, 0, 0);
        CyU3PGpifSMSwitch (257, 0, 257, 0, 2);
        }
        /*}}}*/
      }
    else if (CyU3PEventGet (&uvcEvent, STREAM_ABORT_EVENT, CYU3P_EVENT_AND_CLEAR, &flag, CYU3P_NO_WAIT) == CY_U3P_SUCCESS) {
      /*{{{  stream abort request pending*/
      hitFV = CyFalse;
      prodCount = 0;
      consCount = 0;

      if (!clearFeatureRqtReceived) {
        CyU3PDmaMultiChannelReset (&dmaMultiChannel);
        CyU3PUsbFlushEp (CY_FX_EP_BULK_VID);
        }

      clearFeatureRqtReceived = CyFalse;
      }
      /*}}}*/
    else {
      /*{{{  idle, wait for start streaming request*/
      CyU3PEventGet (&uvcEvent, STREAM_EVENT, CYU3P_EVENT_AND, &flag, CYU3P_WAIT_FOREVER);

      // Set DMA Channel transfer size, first producer socket
      CyU3PDmaMultiChannelSetXfer (&dmaMultiChannel, 0, 0);

      if (gpifInitialized == CyFalse) {
        // init gpif configuration
        CyU3PGpifLoad ((CyU3PGpifConfig_t*)&CyFxGpifConfig);
        CyU3PGpifSMStart (START, ALPHA_START);
        gpifInitialized = CyTrue;
        }

      else
        // Jump to startState of  GPIF state machine,  257 arbitrary invalid state (> 255) number
        CyU3PGpifSMSwitch (257, 0, 257, 0, 2);
      }
      /*}}}*/

    CyU3PThreadRelinquish();
    }
  }
/*}}}*/

// control thread
/*{{{*/
static void abortHandler() {

  uint32_t flag;
  if (CyU3PEventGet (&uvcEvent, STREAM_EVENT, CYU3P_EVENT_AND, &flag, CYU3P_NO_WAIT) == CY_U3P_SUCCESS) {
    // Clear the Video Stream Request Event
    CyU3PEventSet (&uvcEvent, ~(STREAM_EVENT), CYU3P_EVENT_AND);

    // Set Video Stream Abort Event
    CyU3PEventSet (&uvcEvent, STREAM_ABORT_EVENT, CYU3P_EVENT_OR);
    }
  }
/*}}}*/
/*{{{*/
static void stopStreaming() {

  analyserMode = CyFalse;
  streamingStarted = CyFalse;

  // Disable the GPIF state machine
  CyU3PGpifDisable (CyTrue);
  gpifInitialized = 0;

  // Place the EP in NAK mode before cleaning up the pipe
  CyU3PUsbSetEpNak (CY_FX_EP_BULK_VID, CyTrue);
  CyU3PBusyWait (100);

  // Reset and flush the endpoint pipe
  CyU3PDmaMultiChannelReset (&dmaMultiChannel);
  CyU3PUsbFlushEp (CY_FX_EP_BULK_VID);
  CyU3PUsbSetEpNak (CY_FX_EP_BULK_VID, CyFalse);
  CyU3PBusyWait (100);

  // Clear the stall condition and sequence numbers
  CyU3PUsbStall (CY_FX_EP_BULK_VID, CyFalse, CyTrue);
  }
/*}}}*/

/*{{{*/
static void USBEventCallback (CyU3PUsbEventType_t evtype, uint16_t  evdata ) {

  switch (evtype) {
    case CY_U3P_USB_EVENT_RESET:
      CyU3PDebugPrint (4, "RESET encountered...\r\n");
      CyU3PGpifDisable (CyTrue);
      gpifInitialized = 0;
      streamingStarted = CyFalse;
      abortHandler();
      break;

    case CY_U3P_USB_EVENT_SUSPEND:
      CyU3PDebugPrint (4, "SUSPEND encountered...\r\n");
      CyU3PGpifDisable (CyTrue);
      gpifInitialized = 0;
      streamingStarted = CyFalse;
      abortHandler();
      break;

    case CY_U3P_USB_EVENT_DISCONNECT:
      CyU3PDebugPrint (4, "USB disconnected...\r\n");
      CyU3PGpifDisable (CyTrue);
      gpifInitialized = 0;
      usbSpeed = CY_U3P_NOT_CONNECTED;
      streamingStarted = CyFalse;
      abortHandler();
      break;

    case CY_U3P_USB_EVENT_EP_UNDERRUN:
      CyU3PDebugPrint (4, "CY_U3P_USB_EVENT_EP_UNDERRUN encountered...\r\n");
      break;

    default:
      break;
    }
  }
/*}}}*/
/*{{{*/
static CyBool_t USBSetupCallback (uint32_t setupdat0, uint32_t setupdat1) {
// Callback to handle the USB Setup Requests and UVC Class events

  bReqType =  setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK;
  bType    =  bReqType  & CY_U3P_USB_TYPE_MASK;
  bTarget  =  bReqType  & CY_U3P_USB_TARGET_MASK;
  bRequest = (setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS;
  wValue   = (setupdat0 & CY_U3P_USB_VALUE_MASK) >> CY_U3P_USB_VALUE_POS;
  wIndex   = (setupdat1 & CY_U3P_USB_INDEX_MASK) >> CY_U3P_USB_INDEX_POS;
  wLength  = (setupdat1 & CY_U3P_USB_LENGTH_MASK) >> CY_U3P_USB_LENGTH_POS;

  CyBool_t isHandled = CyFalse;

  if (bType == CY_U3P_USB_VENDOR_RQT) {
    switch (bRequest) {
      case 0xA0: // streamer example test
        break;
      case 0xAC:
        /*{{{  sensorFocus*/
        CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);

        isHandled = CyTrue;
        break;
        /*}}}*/
      case 0xAD:
        /*{{{  readReg*/
        //line3 ("vRead", wValue);
        I2C_Read (wValue >> 8, wValue & 0xFF, glEp0Buffer);
        CyU3PUsbSendEP0Data (2, glEp0Buffer);
        isHandled = CyTrue;
        break;
        /*}}}*/
      case 0xAE:
        /*{{{  writeReg*/
        //line3 ("vWrite", wValue);
        CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
        I2C_Write (wValue >> 8, wValue & 0xFF, glEp0Buffer[0], glEp0Buffer[1]);
        isHandled = CyTrue;
        break;
        /*}}}*/
      case 0xAF:
        /*{{{  start analyser streaming*/
        CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);

        if (streamingStarted == CyTrue) {
          stopStreaming();
          abortHandler();
          }

        if (analyserMode == CyFalse) {
          analyserMode = CyTrue;
          CyU3PDmaMultiChannelDestroy (&dmaMultiChannel);

          // create manual dmaMultiChannel for video to USB host
          CyU3PDmaMultiChannelConfig_t dmaMultiChannelConfig;
          CyU3PMemSet ((uint8_t*)&dmaMultiChannelConfig, 0, sizeof(dmaMultiChannelConfig));

          dmaMultiChannelConfig.size           = 16384;
          dmaMultiChannelConfig.count          = 4;
          dmaMultiChannelConfig.validSckCount  = 2;
          dmaMultiChannelConfig.prodSckId [0]  = CY_U3P_PIB_SOCKET_0;
          dmaMultiChannelConfig.prodSckId [1]  = CY_U3P_PIB_SOCKET_1;

          dmaMultiChannelConfig.consSckId [0]  = CY_U3P_UIB_SOCKET_CONS_1; // ep1
          dmaMultiChannelConfig.prodHeader     = 0;
          dmaMultiChannelConfig.prodFooter     = 16;

          dmaMultiChannelConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
          dmaMultiChannelConfig.notification   = CY_U3P_DMA_CB_CONS_EVENT;
          dmaMultiChannelConfig.cb             = vidDmaCallback;

          CyU3PDmaMultiChannelCreate (&dmaMultiChannel, CY_U3P_DMA_TYPE_MANUAL_MANY_TO_ONE, &dmaMultiChannelConfig);
          }

        CyU3PEventSet (&uvcEvent, STREAM_EVENT, CYU3P_EVENT_OR);

        isHandled = CyTrue;
        break;
        /*}}}*/
      default: // other vendor request
        line3 ("vendor", bRequest);
        break;
      }
    }
  else {
    // UVC Class Requests
    switch (bReqType) {
      case CY_FX_USB_UVC_GET_REQ_TYPE:
      /*{{{*/
      case CY_FX_USB_UVC_SET_REQ_TYPE:   // start UVC streaming
        // UVC Specific requests are handled in the EP0 thread
        switch (wIndex & 0xFF) {
          case CY_FX_UVC_CONTROL_INTERFACE: {
            CyU3PEventSet (&uvcEvent, VIDEO_CONTROL_EVENT, CYU3P_EVENT_OR);
            isHandled = CyTrue;
            break;
            }

          case CY_FX_UVC_STREAM_INTERFACE: {
            analyserMode = CyFalse;
            CyU3PDmaMultiChannelDestroy (&dmaMultiChannel);

            // create manual dmaMultiChannel for video to USB host
            CyU3PDmaMultiChannelConfig_t dmaMultiChannelConfig;
            CyU3PMemSet ((uint8_t*)&dmaMultiChannelConfig, 0, sizeof(dmaMultiChannelConfig));
            dmaMultiChannelConfig.size           = 16384;
            dmaMultiChannelConfig.count          = 4;
            dmaMultiChannelConfig.validSckCount  = 2;
            dmaMultiChannelConfig.prodSckId [0]  = CY_U3P_PIB_SOCKET_0;
            dmaMultiChannelConfig.prodSckId [1]  = CY_U3P_PIB_SOCKET_1;
            dmaMultiChannelConfig.consSckId [0]  = CY_U3P_UIB_SOCKET_CONS_3; // ep3
            dmaMultiChannelConfig.prodHeader     = 12; // 12 byte UVC header to be added
            dmaMultiChannelConfig.prodFooter     = 4;  // byte footer to compensate for the 12 byte header
            dmaMultiChannelConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
            dmaMultiChannelConfig.notification   = CY_U3P_DMA_CB_CONS_EVENT;
            dmaMultiChannelConfig.cb             = vidDmaCallback;
            CyU3PDmaMultiChannelCreate (&dmaMultiChannel, CY_U3P_DMA_TYPE_MANUAL_MANY_TO_ONE, &dmaMultiChannelConfig);

            CyU3PEventSet (&uvcEvent, VIDEO_STREAM_EVENT, CYU3P_EVENT_OR);
            isHandled = CyTrue;
            break;
            }

          default:
            break;
          }

        break;
      /*}}}*/
      /*{{{*/
      case CY_FX_USB_SET_INTF_REQ_TYPE:
        if (bRequest == CY_FX_USB_SET_INTERFACE_REQ) {
          // MAC OS sends Set Interface Alternate Setting 0 command after
          //stopping to stream. This application needs to stop streaming
          }
        break;
      /*}}}*/
      /*{{{*/
      case CY_U3P_USB_TARGET_ENDPT:      // abort UVC streaming
        if (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE) {
          if (wIndex == CY_FX_EP_BULK_VID) {
            // Windows OS sends Clear Feature Request after it stops streaming,
            // however MAC OS sends clear feature request right after it sends a
            // Commit -> SET_CUR request. Hence, stop streaming only of streaming has started
            if (streamingStarted == CyTrue) {
              line2 ("clearFeature");

              stopStreaming();

              // Complete Control request handshake
              CyU3PUsbAckSetup();

              // Indicate stop streaming to main thread
              clearFeatureRqtReceived = CyTrue;
              abortHandler();

              isHandled = CyTrue;
              }
            else {
              CyU3PUsbAckSetup();
              isHandled = CyTrue;
              }
            }
          }
        break;
      /*}}}*/
      default:
        break;
      }
    }

  return isHandled;
  }
/*}}}*/
/*{{{*/
static void pibCallback (CyU3PPibIntrType cbType, uint16_t cbArg) {

  if ((cbType == CYU3P_PIB_INTR_ERROR) && ((cbArg == 0x1005) || (cbArg == 0x1006))) {
    if (!backFlowDetected) {
      //CyU3PDebugPrint (4, "Backflow detected\r\n");
      line2 ("pib err");
      backFlowDetected = 1;
      }
    }
  }
/*}}}*/

/*{{{*/
static void processingUnitRequests() {
  CyU3PUsbStall (0, CyTrue, CyFalse);
  }
/*}}}*/
/*{{{*/
static void cameraTerminalRequests() {
  CyU3PUsbStall (0, CyTrue, CyFalse);
  }
/*}}}*/
/*{{{*/
static void videoStreamingRequests() {

  CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
  uint16_t readCount;

  switch (wValue) {
    case CY_FX_UVC_PROBE_CTRL:
      switch (bRequest) {
        /*{{{*/
        case CY_FX_USB_UVC_GET_INFO_REQ:
          glEp0Buffer[0] = 3;                /* GET/SET requests are supported. */
          CyU3PUsbSendEP0Data (1, (uint8_t*)glEp0Buffer);
          break;
        /*}}}*/
        /*{{{*/
        case CY_FX_USB_UVC_GET_LEN_REQ:
          glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
          CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
          break;
        /*}}}*/
        case CY_FX_USB_UVC_GET_CUR_REQ:
        case CY_FX_USB_UVC_GET_MIN_REQ:
        case CY_FX_USB_UVC_GET_MAX_REQ:
        /*{{{*/
        case CY_FX_USB_UVC_GET_DEF_REQ: // There is only one setting per USB speed
          if (usbSpeed == CY_U3P_SUPER_SPEED)
            CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t*)probeCtrl30);
          else
            CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t*)probeCtrl20);
          break;
        /*}}}*/
        /*{{{*/
        case CY_FX_USB_UVC_SET_CUR_REQ:
          apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED, commitCtrl, &readCount);
          if (apiRetStatus == CY_U3P_SUCCESS) {
            if (usbSpeed == CY_U3P_SUPER_SPEED) {
              // Copy from the host provided data into the active data
              probeCtrl30[2] = commitCtrl[2];
              probeCtrl30[3] = commitCtrl[3];
              probeCtrl30[4] = commitCtrl[4];
              probeCtrl30[5] = commitCtrl[5];
              probeCtrl30[6] = commitCtrl[6];
              probeCtrl30[7] = commitCtrl[7];
              }
            }
          break;
        /*}}}*/
        /*{{{*/
        default:
          CyU3PUsbStall (0, CyTrue, CyFalse);
          break;
        /*}}}*/
        }
      break;

    case CY_FX_UVC_COMMIT_CTRL:
      switch (bRequest) {
        /*{{{*/
        case CY_FX_USB_UVC_GET_INFO_REQ:
          glEp0Buffer[0] = 3;                        /* GET/SET requests are supported. */
          CyU3PUsbSendEP0Data (1, (uint8_t*)glEp0Buffer);
          break;
        /*}}}*/
        /*{{{*/
        case CY_FX_USB_UVC_GET_LEN_REQ:
          glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
          CyU3PUsbSendEP0Data (1, (uint8_t*)glEp0Buffer);
          break;
        /*}}}*/
        /*{{{*/
        case CY_FX_USB_UVC_GET_CUR_REQ:
          if (usbSpeed == CY_U3P_SUPER_SPEED)
            CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t*)probeCtrl30);
          else
            CyU3PUsbSendEP0Data (CY_FX_UVC_MAX_PROBE_SETTING, (uint8_t*)probeCtrl20);
          break;
        /*}}}*/
        /*{{{*/
        case CY_FX_USB_UVC_SET_CUR_REQ: // start video stream
          // The host has selected the parameters for the video stream
          apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED, commitCtrl, &readCount);
          if (apiRetStatus == CY_U3P_SUCCESS)
            CyU3PEventSet (&uvcEvent, STREAM_EVENT, CYU3P_EVENT_OR);

          break;
        /*}}}*/
        /*{{{*/
        default:
          CyU3PUsbStall (0, CyTrue, CyFalse);
          break;
        /*}}}*/
        }
      break;

    default:
      CyU3PUsbStall (0, CyTrue, CyFalse);
      break;
    }
  }
/*}}}*/
/*{{{*/
static void controlThreadFunc (uint32_t input) {

  uint32_t eventMask = VIDEO_CONTROL_EVENT | VIDEO_STREAM_EVENT | BUTTON_DOWN_EVENT | BUTTON_UP_EVENT;
  for (;;) {
    uint32_t eventFlag;
    if (CyU3PEventGet (&uvcEvent, eventMask, CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS) {
      if (usbSpeed == CY_U3P_NOT_CONNECTED) {
        /*{{{  get and display usb speed*/
        usbSpeed = CyU3PUsbGetSpeed();
        if (usbSpeed == CY_U3P_SUPER_SPEED)
          line3 ("USB", 3);
        else if (usbSpeed == CY_U3P_HIGH_SPEED)
          line3 ("USB", 2);
        else if (usbSpeed == CY_U3P_FULL_SPEED)
          line3 ("USB", 1);
        else
          line3 ("USB", 0);
        }
        /*}}}*/
      if (eventFlag & VIDEO_CONTROL_EVENT) {
        /*{{{  videoControl requests*/
        switch ((wIndex >> 8)) {
          case CY_FX_UVC_PROCESSING_UNIT_ID:
            processingUnitRequests();
            break;

          case CY_FX_UVC_CAMERA_TERMINAL_ID:
            cameraTerminalRequests();
            break;

          case CY_FX_UVC_INTERFACE_CTRL:
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;

          case CY_FX_UVC_EXTENSION_UNIT_ID:
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;

          default:
            // Unsupported request. Fail by stalling the control endpoint
            CyU3PUsbStall (0, CyTrue, CyFalse);
            break;
          }
        }
        /*}}}*/
      if (eventFlag & VIDEO_STREAM_EVENT) {
        if (wIndex == CY_FX_UVC_STREAM_INTERFACE)
          videoStreamingRequests();
        else
          CyU3PUsbStall (0, CyTrue, CyFalse);
        }
      if (eventFlag & BUTTON_DOWN_EVENT) {}
      if (eventFlag & BUTTON_UP_EVENT) {}
      }

    CyU3PThreadRelinquish();
    }
  }
/*}}}*/

// init
/*{{{*/
static void debugInit() {

  CyU3PUartInit();

  CyU3PUartConfig_t uartConfig;
  uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
  uartConfig.stopBit  = CY_U3P_UART_ONE_STOP_BIT;
  uartConfig.parity   = CY_U3P_UART_NO_PARITY;
  uartConfig.txEnable = CyTrue;
  uartConfig.rxEnable = CyFalse;
  uartConfig.flowCtrl = CyFalse;
  uartConfig.isDma    = CyTrue;
  CyU3PUartSetConfig (&uartConfig, NULL);

  CyU3PUartTxSetBlockXfer (0xFFFFFFFF);

  CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 4);
  CyU3PDebugPreamble (CyFalse);
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
  }
/*}}}*/
/*{{{*/
static void appInit() {

  CyU3PEventCreate (&uvcEvent);

  // init P-port clock
  CyU3PPibClock_t pibClock;
  pibClock.clkDiv = 2;
  pibClock.clkSrc = CY_U3P_SYS_CLK;
  pibClock.isDllEnable = CyFalse;
  pibClock.isHalfDiv = CyFalse;
  CyU3PPibInit (CyTrue, &pibClock);

  // register gpif pib callbacks
  CyU3PGpifRegisterCallback (gpifCallback);
  CyU3PPibRegisterCallback (pibCallback, CYU3P_PIB_INTR_ERROR);

  CyU3PUsbStart();

  // register USB callbacks
  CyU3PUsbRegisterSetupCallback (USBSetupCallback, CyFalse);
  CyU3PUsbRegisterEventCallback (USBEventCallback);

  // USB device descriptors
  CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t*)CyFxUSBDeviceDscrHS);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t*)CyFxUSBDeviceDscrSS);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t*)CyFxUSBBOSDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t*)CyFxUSBDeviceQualDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t*)FullSpeedConfigurationDescriptor);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t*)HighSpeedConfigurationDescriptor);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t*)SuperSpeedConfigurationDescriptor);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t*)CyFxUSBStringLangIDDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t*)CyFxUSBManufactureDscr);
  CyU3PUsbSetDesc (CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t*)CyFxUSBProductDscr);

  /*{{{  config status interrupt endpoint*/
  // Note: This endpoint is not being used by the application as of now. This can be used in case
  // UVC device needs to notify the host about any error conditions. A MANUAL_OUT DMA channel
  // can be associated with this endpoint and used to send these data packets.
  CyU3PEpConfig_t epConfig;

  epConfig.enable   = 1;
  epConfig.epType   = CY_U3P_USB_EP_INTR;
  epConfig.pcktSize = 64;
  epConfig.burstLen = 1;

  CyU3PSetEpConfig (CY_FX_EP_CONTROL_STATUS, &epConfig);
  /*}}}*/
  /*{{{  config video streaming endpoints*/
  CyU3PMemSet ((uint8_t*)&epConfig, 0, sizeof(epConfig));

  epConfig.enable   = 1;
  epConfig.epType   = CY_U3P_USB_EP_BULK;
  epConfig.pcktSize = 1024;
  epConfig.burstLen = 16;

  CyU3PSetEpConfig (CY_FX_EP_CONSUMER, &epConfig);
  CyU3PSetEpConfig (CY_FX_EP_BULK_VID, &epConfig);
  /*}}}*/

  // enable USB connection from the FX3 device, preferably at USB 3.0 speed
  CyU3PConnectState (CyTrue, CyTrue);
  }
/*}}}*/
/*{{{*/
void CyFxApplicationDefine() {

  debugInit();
  gpioInit();
  displayInit ("anal 8 ext fv");
  appInit();

  // Create the UVC application thread
  CyU3PThreadCreate (&vidThread,
    "30:UVC vidThread",      // Thread Id and name
    vidThreadFunc,           // UVC Application video Thread
    0,                       // No input parameter to thread
    CyU3PMemAlloc (0x1000),  // Pointer to the allocated thread stack
    0x1000,                  // UVC Application Thread stack size
    8,                       // UVC Application Thread priority
    8,                       // Threshold value for thread pre-emption.
    CYU3P_NO_TIME_SLICE,     // No time slice for the application thread
    CYU3P_AUTO_START         // Start the Thread immediately
    );

  // Create the control request handling thread
  CyU3PThreadCreate (&controlThread,
    "31:UVC conThread",      // Thread Id and name
    controlThreadFunc,       // UVC Application control Thread
    0,                       // No input parameter to thread
    CyU3PMemAlloc (0x0800),  // Pointer to the allocated thread stack
    0x0800,                  // UVC Application Thread stack size
    8,                       // UVC Application Thread priority
    8,                       // Threshold value for thread pre-emption.
    CYU3P_NO_TIME_SLICE,     // No time slice for the application thread
    CYU3P_AUTO_START         // Start the Thread immediately
    );
  }
/*}}}*/

// main
/*{{{*/
int main() {

  CyU3PDeviceInit (0);

  // enable instruction cache for firmware performance
  CyU3PDeviceCacheControl (CyTrue, CyFalse, CyFalse);

  // Configure the IO matrix for the device
  CyU3PIoMatrixConfig_t io_cfg;
  io_cfg.isDQ32Bit        = CyFalse;
  io_cfg.s0Mode           = CY_U3P_SPORT_INACTIVE;
  io_cfg.s1Mode           = CY_U3P_SPORT_INACTIVE;
  io_cfg.useUart          = CyTrue;   // Uart debug - DQ30 because of spi
  io_cfg.useI2C           = CyTrue;   // I2C sensor
  io_cfg.useI2S           = CyFalse;
  io_cfg.useSpi           = CyTrue;   // spi display
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
