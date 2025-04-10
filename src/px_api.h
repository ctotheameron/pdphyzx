#ifndef PDPHYZX_API_H
#define PDPHYZX_API_H

#include "pd_api.h"

#include "body_api.h"
#include "world_api.h"

typedef struct {
  const PxBodyAPI *body;
  const PxWorldAPI *world;
} PdPhyzxAPI;

PdPhyzxAPI *registerPdPhyzx(PlaydateAPI *playdate);

#endif // PDPHYZX_API_H
