#include <assert.h>

#include "px_body.h"
#include "px_platform.h"

#include "px_body_api.h"

const PxBodyAPI *px_body;

/**
 * Constructor function to initialize the Body API.
 * Automatically called when the library is loaded.
 */
PxBodyAPI *newPxBodyAPI(void) {
  PxBodyAPI *api = pxalloc(sizeof(*px_body));

  if (api == NULL) {
    assert(0 && "Failed to allocate memory for PxBodyAPI");
  }

  // Initialize all function pointers
  api->isValid = &pxBodyIsValid;
  api->setPosition = &pxBodySetPosition;
  api->setOrientation = &pxBodySetOrientation;
  api->moveBy = &pxBodyMoveBy;
  api->rotate = &pxBodyRotate;
  api->applyForce = &pxBodyApplyForce;
  api->applyImpulse = &pxBodyApplyImpulse;

  px_body = api;

  return api;
}

/**
 * Destructor function to clean up the Body API.
 * Automatically called when the library is unloaded.
 */
static void freePxBodyAPI(void) __attribute__((destructor));
static void freePxBodyAPI(void) { pxfree((void *)px_body); }
