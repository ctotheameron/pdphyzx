/**
 * @file px_body_api.h
 * @brief Public API for manipulating physics bodies
 *
 * This header provides the interface for modifying properties of physics
 * bodies in the PdPhyzx engine. All interactions with physics bodies
 * should use these functions rather than modifying body data directly.
 *
 * Example usage:
 *   px_body->setOrientation(myBody, 1.57f); // Set to 90 degrees
 */

#ifndef PDPHYZX_PX_BODY_API_H
#define PDPHYZX_PX_BODY_API_H

#include "px_body.h"

typedef struct {
  bool (*isValid)(PxBody *body);
  void (*setPosition)(PxBody *body, float x, float y);
  void (*setOrientation)(PxBody *body, float radians);
  void (*moveBy)(PxBody *body, PxVec2 distance);
  void (*rotate)(PxBody *body, float radians);

  void (*applyForce)(PxBody *body, float forceX, float forceY, float contactX,
                     float contactY);

  void (*applyImpulse)(PxBody *body, float impulseX, float impulseY,
                       float contactX, float contactY);
} PxBodyAPI;

PxBodyAPI *newPxBodyAPI(void);

#endif // PDPHYZX_PX_BODY_API_H
