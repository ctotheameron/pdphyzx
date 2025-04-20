#include "px_platform.h"

#include "px_api.h"

PlaydateAPI *pd = NULL;

PdPhyzxAPI *registerPdPhyzx(PlaydateAPI *playdate) {
  pd = playdate;

  pxInitSinTable();

  PdPhyzxAPI *api = pxalloc(sizeof(*api));
  api->world = newPxWorldAPI();
  api->body = newPxBodyAPI();

  return api;
}
