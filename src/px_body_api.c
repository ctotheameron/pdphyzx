#include <assert.h>

#include "px_body.h"
#include "px_platform.h"
#include "px_unit.h"
#include "px_vec2.h"

#include "px_body_api.h"

const PxBodyAPI *px_body;

/**
 * @brief Applies a (scaled unit) force to a body at a specific contact point
 *        and wakes it
 *
 * @note This is exposed on the public API as `px->body->applyForce(..)`.
 *
 * @see pxBodyApplyForce
 */
static void pxBodyApiApplyForce(PxBody *body, float forceX, float forceY,
                                float contactX, float contactY) {

  PxVec2 scaledForce = pxVec2Multf(pxVec2(forceX, forceY), PDPHYZX_UNIT);
  pxBodyApplyForce(body, scaledForce, pxVec2(contactX, contactY));
  pxBodyWakeUp(body);
}

/**
 * @brief Applies a (scaled unit) impulse to a body at a specific contact point
 *        and wakes it
 *
 *
 * @note This is exposed on the public API as `px->body->applyImpulse(..)`.
 *
 * @see pxBodyApplyImpulse
 */
static void pxBodyApiApplyImpulse(PxBody *body, float impulseX, float impulseY,
                                  float contactX, float contactY) {
  PxVec2 scaledImpulse = pxVec2Multf(pxVec2(impulseX, impulseY), PDPHYZX_UNIT);
  pxBodyApplyImpulse(body, scaledImpulse, pxVec2(contactX, contactY));
  pxBodyWakeUp(body);
}

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
  api->applyForce = &pxBodyApiApplyForce;
  api->applyImpulse = &pxBodyApiApplyImpulse;

  px_body = api;

  return api;
}

/**
 * Destructor function to clean up the Body API.
 * Automatically called when the library is unloaded.
 */
static void freePxBodyAPI(void) __attribute__((destructor));
static void freePxBodyAPI(void) { pxfree((void *)px_body); }
