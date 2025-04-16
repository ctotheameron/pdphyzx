#include "px_platform.h"

#include "px_api.h"

PlaydateAPI *pd = NULL;

PdPhyzxAPI *registerPdPhyzx(PlaydateAPI *playdate) {
  pd = playdate;

  PdPhyzxAPI *api = pxalloc(sizeof(*api));
  api->body = newPxBodyAPI();
  api->world = newPxWorldAPI();

  return api;
}
