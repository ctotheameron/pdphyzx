#include <stdint.h>
#include <stdio.h>

#include "circle.h"
#include "collider.h"
#include "mat2.h"
#include "polygon.h"
#include "vec2.h"

#include "body.h"

PxBody pxBodyNew(PxShape shape, float density, PxVec2 position) {
  PxCollider collider;

  if (shape.box.width != 0 && shape.box.height != 0) {
    collider = pxBoxColliderNew(shape.box.width, shape.box.height);
  } else if (shape.circle.radius != 0) {
    collider = pxCircleColliderNew(shape.circle.radius);
  } else if (shape.polygon.vertexCount != 0) {
    collider =
        pxPolygonColliderNew(shape.polygon.vertices, shape.polygon.vertexCount);
  } else {
    return (PxBody){.isValid = false};
  }

  PxAABB aabb = {0};
  collider.updateAABB(&collider, &aabb, position);

  PxMassData massData =
      density > 0 ? collider.computeMass(&collider, density)
                  : (PxMassData){0}; // infer static body when density is 0

  if (density != 0 && (massData.mass <= 0 || massData.momentOfInertia <= 0)) {
    return (PxBody){.isValid = false};
  }

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

      .isValid = true,
  };
}

void pxBodySetPosition(PxBody *body, PxVec2 position) {
  if (!body) {
    return;
  }

  body->position = position;
  body->collider.updateAABB(&body->collider, &body->aabb, position);
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
  if (!bodyA || !bodyB || !bodyA->isValid || !bodyB->isValid) {
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
  pxVec2AddAssign(&body->velocity, pxVec2Multf(impulse, body->iMass));

  body->angularVelocity +=
      pxVec2Cross(contact, impulse) * body->iMomentOfInertia;

  // Only wake the body if impulse is significant
  float impulseSquaredMagnitude = pxVec2LenSqr(impulse);

  if (impulseSquaredMagnitude > PX_BODY_WAKE_THRESHOLD) {
    pxBodyWakeUp(body);
  }
}

void pxBodyApplyForce(PxBody *body, PxVec2 force, PxVec2 contact) {
  pxVec2AddAssign(&body->force, force);
  body->torque += pxVec2Cross(contact, force);

  pxBodyWakeUp(body);
}

void pxBodyIntegrateForces(PxBody *body, PxVec2 g, float dt) {
  if (body->iMomentOfInertia == 0) {
    return;
  }

  float halfDt = pxFastDiv(dt, 2);

  pxVec2AddAssign(
      &body->velocity,
      pxVec2Multf(pxVec2Add(pxVec2Multf(body->force, body->iMass), g), halfDt));

  body->angularVelocity += body->torque * body->iMomentOfInertia * halfDt;
}

void pxBodyIntegrateVelocity(PxBody *body, float dt) {
  if (body->iMomentOfInertia == 0) {
    return;
  }

  pxBodyMoveBy(body, pxVec2Multf(body->velocity, dt));
  pxBodyRotate(body, body->angularVelocity * dt);
}

void pxBodyClearForces(PxBody *body) {
  pxVec2Set(&body->force, 0, 0);
  body->torque = 0;
}

void pxBodyWakeUp(PxBody *body) { body->sleepTime = 0.0f; }

PxBody *pxBodyArrayAdd(PxBodyArray *array, PxBody *body) {
  if (!body->isValid) {
    return NULL;
  }

  /**
   * Memory management strategy for body array:
   *
   * - Bodies are stored in a contiguous array (array->items)
   *
   * - We maintain two tracking mechanisms:
   *
   *   1. activeIndices: Contains indices of all active bodies for efficient
   *                     iteration
   *
   *   2. freedIndices:  A stack of indices that have been freed and can be
   *                     reused
   *
   * - When adding a body, we prefer to reuse a freed index before extending the
   *   array
   *
   * - This approach allows O(1) addition/removal while maintaining memory
   *   locality
   */
  uint8_t index;

  if (array->freedCount > 0) {
    // Reuse a "freed" index
    index = array->freedIndices[--array->freedCount];
  } else if (array->length < PX_MAX_BODIES) {
    // Add to the end of the array (using current length as next index)
    index = array->length;
  } else {
    return NULL; // Array is full
  }

  // Update activeIndices and increment length.
  //
  // activeIndices maintains a list of indices that are currently in use.
  // This allows for efficient iteration through only active bodies.
  uint8_t currentLength = array->length;
  array->activeIndices[currentLength] = index;
  array->length++;

  // Store the world index on the body
  body->worldIndex = index;

  // Update the array
  array->items[index] = *body;

  // Return a pointer to the stored body
  return &array->items[index];
}

void pxBodyArrayRemove(PxBodyArray *array, uint8_t index) {
  if (index >= array->length || array->freedCount >= PX_MAX_BODIES) {
    return;
  }

  // Add the index to the freedIndices array
  array->freedIndices[array->freedCount++] = index;

  // Find the index in activeIndices and remove it by swapping with the last
  // element
  for (uint8_t i = 0; i < array->length; i++) {
    if (array->activeIndices[i] == index) {
      array->activeIndices[i] = array->activeIndices[--array->length];
      break;
    }
  }
}

/**
 * Sort a body array's activeIndices by the min X coordinate of each body's
 * AABB.
 *
 * This preserves body pointers since only the iteration order changes.
 *
 * @param bodies
 *
 * Note: Using insertion sort as it's efficient for small arrays (PX_MAX_BODIES
 *       is small) and performs well when data is already partially sorted.
 */
void pxBodyArraySortByAxis(PxBodyArray *bodies) {
  for (uint8_t i = 1; i < bodies->length; i++) {
    uint8_t currentActiveIndex = bodies->activeIndices[i];
    PxBody *currentBody = &bodies->items[currentActiveIndex];
    float currentMinX = currentBody->aabb.min.x;

    int j = i - 1;
    while (j >= 0) {
      uint8_t compareActiveIndex = bodies->activeIndices[j];
      PxBody *compareBody = &bodies->items[compareActiveIndex];

      if (compareBody->aabb.min.x <= currentMinX) {
        break;
      }

      bodies->activeIndices[j + 1] = bodies->activeIndices[j];
      j--;
    }

    bodies->activeIndices[j + 1] = currentActiveIndex;
  }
}

uint8_t pxBodyArrayFindFirstIndexAfterX(PxBodyArray *bodies, float minX) {
  if (bodies->length == 0) {
    return 0;
  }

  uint8_t low = 0;
  uint8_t high = bodies->length;

  while (low < high) {
    uint8_t mid = low + (high - low) / 2; // Prevents potential overflow
    uint8_t bodyIndex = bodies->activeIndices[mid];
    PxBody *body = &bodies->items[bodyIndex];

    if (!body || body->worldIndex == PX_INVALID_WORLD_INDEX) {
      // Handle invalid body (shouldn't happen if array is properly maintained)
      low = mid + 1;
      continue;
    }

    if (body->aabb.max.x < minX) {
      low = mid + 1;
    } else {
      high = mid;
    }
  }

  return low;
}
