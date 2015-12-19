// Pan, Tilt Zoom controls for the UVC camera application
//{{{  includes
#include <cyu3system.h>

#include "ptz.h"
#include "sensor.h"
#include "display.h"
//}}}

static int32_t pan_cur;     /* Current pan value. */
static int32_t tilt_cur;    /* Current tilt value. */
static uint16_t zoom_cur;   /* Current zoom value. */

//{{{
uint16_t CyFxUvcAppGetCurrentZoom() {

  return zoom_cur;
  }
//}}}
//{{{
int32_t CyFxUvcAppGetCurrentPan() {

  return pan_cur;
  }
//}}}
//{{{
int32_t CyFxUvcAppGetCurrentTilt() {

  return tilt_cur;
  }
//}}}

//{{{
void CyFxUvcAppModifyPan (int32_t panValue) {

  pan_cur = panValue;
  line3 ("pan", panValue);
  CyU3PDebugPrint (4, "Pan %d\r\n", panValue);
  }
//}}}
//{{{
void CyFxUvcAppModifyTilt (int32_t tiltValue) {

  tilt_cur = tiltValue;
  line3 ("tilt", tiltValue);
  CyU3PDebugPrint (4, "Tilt %d\r\n", tiltValue);
  }
//}}}
//{{{
void CyFxUvcAppModifyZoom (uint16_t zoomValue) {

  zoom_cur = zoomValue;
  line3 ("zoom", zoomValue);
  sensorFocus (zoomValue);
  CyU3PDebugPrint (4, "Zoom %d\r\n", zoomValue);
  }
//}}}

//{{{
void PTZInit() {

  zoom_cur = ZOOM_DEFAULT;
  pan_cur = 0;
  tilt_cur = 0;
  }
//}}}
