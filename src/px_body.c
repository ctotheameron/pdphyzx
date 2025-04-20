#include <stdint.h>
#include <stdio.h>

#include "px_circle.h"
#include "px_math.h"
#include "px_polygon.h"
#include "px_unit.h"
#include "px_vec2.h"

#include "px_body.h"

PxBody pxBodyNew(PxShape shape, PxBodyFlags type, float density,
                 PxVec2 position) {
  PxCollider collider;

  if (shape.box.width != 0 && shape.box.height != 0) {
    collider = pxBoxColliderNew(shape.box.width, shape.box.height);
  } else if (shape.circle.radius != 0) {
    collider = pxCircleColliderNew(shape.circle.radius);
  } else if (shape.polygon.vertexCount != 0) {
    collider =
        pxPolygonColliderNew(shape.polygon.vertices, shape.polygon.vertexCount);
  } else {
    return (PxBody){0};
  }

  PxAABB aabb = {0};
  collider.updateAABB(&collider, &aabb, position);

  PxMassData massData = (PxMassData){0};
  switch (type) {
  case PX_BODY_DYNAMIC:
    if (density <= 0) {
      return (PxBody){0}; // Invalid density for dynamic body
    }

    // Only dynamic bodies get real mass data
    massData =
        collider.computeMass(&collider, pxFastDiv(density, PDPHYZX_UNIT_SQ));
    break;

  default:
    break;
  }

  // Set up flags
  uint8_t flags = PX_BODY_FLAG_VALID | (type & PX_BODY_TYPE_MASK);

  return (PxBody){
      .aabb = aabb,
      .collider = collider,
      .density = density,
      .position = position,

      // Defaults
      .angularVelocity = 0,
      .dynamicFriction = 0.3,
      .staticFriction = 0.5,
      .restitution = 0.2,
      .torque = 0,
      .sleepTime = 0.0f,

      // Initialize orientation to identity matrix
      .orientationAngle = 0,
      .orientation = pxMat2Identity(),

      // Initialize default vectors
      .velocity = pxVec2(0, 0),
      .force = pxVec2(0, 0),

      // Computed mass data
      .mass = massData.mass,
      .iMass = massData.iMass,
      .momentOfInertia = massData.momentOfInertia,
      .iMomentOfInertia = massData.iMomentOfInertia,

      // Initialize an "invalid" world index. This will be updated when the body
      // is added to a world.
      .worldIndex = PX_INVALID_WORLD_INDEX,

      .flags = flags,
  };
}

void pxBodySetPosition(PxBody *body, float x, float y) {
  if (!body) {
    return;
  }

  body->position.x = x;
  body->position.y = y;
  body->collider.updateAABB(&body->collider, &body->aabb, body->position);
}

void pxBodyMoveBy(PxBody *body, PxVec2 distance) {
  if (!body) {
    return;
  }

  pxVec2AddAssign(&body->position, distance);
  body->collider.updateAABB(&body->collider, &body->aabb, body->position);
}

void pxBodySetOrientation(PxBody *body, float radians) {
  if (!body) {
    return;
  }

  // Normalize angle to [0, 2π) range
  if ((radians = fmodf(radians, PX_2_PI)) < 0) {
    radians += PX_2_PI;
  }

  body->orientationAngle = radians;
  body->orientation = pxMat2Orientation(radians);
}

void pxBodyRotate(PxBody *body, float radians) {
  if (!body) {
    return;
  }

  pxBodySetOrientation(body, body->orientationAngle + radians);
}

bool pxBodyAABBsOverlap(PxBody *bodyA, PxBody *bodyB) {
  if (!bodyA || !bodyB || !pxBodyIsValid(bodyA) || !pxBodyIsValid(bodyB)) {
    return false;
  }

  PxAABB a = bodyA->aabb;
  PxAABB b = bodyB->aabb;

  // Check for separation on either axis
  if (a.max.x < b.min.x || a.min.x > b.max.x) {
    return false;
  }

  if (a.max.y < b.min.y || a.min.y > b.max.y) {
    return false;
  }

  // If not separated on either axis, they must overlap
  return true;
}

void pxBodyApplyImpulse(PxBody *body, PxVec2 impulse, PxVec2 contact) {
  if (!body || !pxBodyIsDynamic(body)) {
    return;
  }

  pxVec2AddAssign(&body->velocity, pxVec2Multf(impulse, body->iMass));

  body->angularVelocity +=
      pxVec2Cross(contact, impulse) * body->iMomentOfInertia;
}

void pxBodyApplyForce(PxBody *body, PxVec2 force, PxVec2 contact) {
  if (!body || !pxBodyIsDynamic(body)) {
    return;
  }

  pxVec2AddAssign(&body->force, force);
  body->torque += pxVec2Cross(contact, force);
}

void pxBodyIntegrateForces(PxBody *body, PxVec2 g, float dt) {
  if (!body || pxBodyIsStatic(body) || pxBodyIsSleeping(body)) {
    return;
  }

  float halfDt = pxFastDiv(dt, 2);

  pxVec2AddAssign(
      &body->velocity,
      pxVec2Multf(pxVec2Add(pxVec2Multf(body->force, body->iMass), g), halfDt));

  body->angularVelocity += body->torque * body->iMomentOfInertia * halfDt;
}

void pxBodyIntegrateVelocity(PxBody *body, float dt) {
  if (!body || pxBodyIsStatic(body) || pxBodyIsSleeping(body)) {
    return;
  }

  pxBodyMoveBy(body, pxVec2Multf(body->velocity, dt));
  pxBodyRotate(body, body->angularVelocity * dt);
}

void pxBodyClearForces(PxBody *body) {
  pxVec2Set(&body->force, 0, 0);
  body->torque = 0;
}

void pxBodyWakeUp(PxBody *body) {
  if (!body) {
    return;
  }

  body->sleepTime = 0.0f;
  pxBodySetFlag(body, PX_BODY_FLAG_SLEEPING, false);
}
