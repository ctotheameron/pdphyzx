/**
 * @file api.h
 * @brief Public API for manipulating physics bodies
 *
 * This header provides the interface for modifying properties of physics
 * bodies in the PdPhyzx engine. All interactions with physics bodies
 * should use these functions rather than modifying body data directly.
 *
 * Example usage:
 *   px_body->setOrientation(myBody, 1.57f); // Set to 90 degrees
 */

#ifndef PX_BODY_API_H
#define PX_BODY_API_H

#include "body.h"

typedef struct {
  bool (*isValid)(PxBody *body);
  void (*setPosition)(PxBody *body, PxVec2 position);
  void (*setOrientation)(PxBody *body, float radians);
  void (*moveBy)(PxBody *body, PxVec2 distance);
  void (*rotate)(PxBody *body, float radians);
  void (*applyForce)(PxBody *body, PxVec2 force, PxVec2 contact);
  void (*applyImpulse)(PxBody *body, PxVec2 impulse, PxVec2 force);
} PxBodyAPI;

PxBodyAPI *newPxBodyAPI(void);

#endif // PX_BODY_API_H
