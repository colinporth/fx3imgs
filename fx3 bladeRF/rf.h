#ifndef _RF_H_
#define _RF_H_

#include "bladeRF.h"

extern const struct NuandApplication NuandRFLink;

/* Enable FW sample loopback */
void NuandRFLinkLoopBack(int);

/* Check if FW sample loopback is enabled */
int NuandRFLinkGetLoopBack();

#endif /* _RF_H_ */
