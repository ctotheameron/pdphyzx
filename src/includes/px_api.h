#ifndef PDPHYZX_PX_API_H
#define PDPHYZX_PX_API_H

#include "pd_api.h"

#include "px_body_api.h"
#include "px_world_api.h"

typedef struct {
  const PxBodyAPI *body;
  const PxWorldAPI *world;
} PdPhyzxAPI;

PdPhyzxAPI *registerPdPhyzx(PlaydateAPI *playdate);

#endif // PDPHYZX_PX_API_H
