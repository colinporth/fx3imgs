// This file defines the variables and functions used to control and query the Pan, Tilt
// and Zoom controls for this UVC camera function.
#pragma once

#include <cyu3types.h>

#define wObjectiveFocalLengthMin        (uint16_t)(0)       /* Minimum Lobjective value for the sensor lens. */
#define wObjectiveFocalLengthMax        (uint16_t)(255)      /* Maximum Lobjective value for the sensor lens. */
#define wOcularFocalLength              (uint16_t)(1)       /* Locular value for the sensor lens. */
#define ZOOM_DEFAULT                    (uint16_t)(0)       /* Default zoom setting that we start with. */

#define CyFxUvcAppGetMinimumZoom()      (wObjectiveFocalLengthMin) /* Minimum supported zoom value. */
#define CyFxUvcAppGetMaximumZoom()      (wObjectiveFocalLengthMax) /* Maximum supported zoom value. */
#define CyFxUvcAppGetZoomResolution()   ((uint16_t)1)              /* Zoom resolution is one unit. */
#define CyFxUvcAppGetDefaultZoom()      ((uint16_t)ZOOM_DEFAULT)   /* Default zoom setting. */

#define PANTILT_MIN                     (int32_t)(-648000)  /* Minimum value for Pan and Tilt controls. */
#define PANTILT_MAX                     (int32_t)(648000)   /* Maximum value for Pan and Tilt controls. */

#define CyFxUvcAppGetMinimumPan()       (PANTILT_MIN)       /* Minimum pan value. */
#define CyFxUvcAppGetMaximumPan()       (PANTILT_MAX)       /* Maximum pan value. */
#define CyFxUvcAppGetPanResolution()    ((int32_t)1)        /* Resolution for pan setting. */
#define CyFxUvcAppGetDefaultPan()       ((int32_t)0)        /* Default pan setting. */

#define CyFxUvcAppGetMinimumTilt()      (PANTILT_MIN)       /* Minimum tilt value. */
#define CyFxUvcAppGetMaximumTilt()      (PANTILT_MAX)       /* Maximum tilt value. */
#define CyFxUvcAppGetTiltResolution()   ((int32_t)1)        /* Resolution for tilt setting. */
#define CyFxUvcAppGetDefaultTilt()      ((int32_t)0)        /* Default tilt setting. */

extern void PTZInit();

extern uint16_t CyFxUvcAppGetCurrentZoom();
extern int32_t CyFxUvcAppGetCurrentPan();
extern int32_t CyFxUvcAppGetCurrentTilt();

extern void CyFxUvcAppModifyPan (int32_t panValue);
extern void CyFxUvcAppModifyTilt (int32_t tiltValue);
extern void CyFxUvcAppModifyZoom  (uint16_t zoomValue);
