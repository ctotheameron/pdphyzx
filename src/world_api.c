#include <assert.h>

#include "platform.h"
#include "world.h"

#include "world_api.h"

const PxWorldAPI *px_world;

/**
 * Constructor function to initialize the Body API.
 * Automatically called when the library is loaded.
 */
PxWorldAPI *newPxWorldAPI(void) {
  PxWorldAPI *api = pxalloc(sizeof(*px_world));

  if (api == NULL) {
    assert(0 && "Failed to allocate memory for PxWorldAPI");
  }

  // Initialize all function pointers
  api->new = &pxWorldNew;
  api->free = &pxWorldFree;
  api->newStaticBody = &pxWorldNewStaticBody;
  api->newDynamicBody = &pxWorldNewDynamicBody;
  api->freeBody = &pxWorldFreeBody;
  api->step = &pxWorldStep;
  api->drawDebug = &pxWorldDrawDebug;

  px_world = api;

  return api;
}

/**
 * Destructor function to clean up the Body API.
 * Automatically called when the library is unloaded.
 */
static void freePxWorldAPI(void) __attribute__((destructor));
static void freePxWorldAPI(void) { pxfree((void *)px_world); }
