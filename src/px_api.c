#include "px_api.h"
#include "body_api.h"
#include "platform.h"
#include "world_api.h"

PlaydateAPI *pd = NULL;

PdPhyzxAPI *registerPdPhyzx(PlaydateAPI *playdate) {
  pd = playdate;

  PdPhyzxAPI *api = pxalloc(sizeof(*api));
  api->body = newPxBodyAPI();
  api->world = newPxWorldAPI();

  return api;
}
