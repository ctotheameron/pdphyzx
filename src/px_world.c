#include <assert.h>

#include "px_body.h"
#include "px_body_array.h"
#include "px_clock.h"
#include "px_manifold.h"
#include "px_manifold_pool.h"
#include "px_platform.h"
#include "px_unit.h"
#include "px_vec2.h"

#include "px_world.h"

#define PX_DEFAULT_ITERATIONS 8
#define PX_DEFAULT_SLEEP_THRESH_SQ 0.001f * PDPHYZX_UNIT_SQ // 1cm/s
#define PX_DEFAULT_ANGULAR_SLEEP_THRESH 0.01f               // ~0.6 degrees/s
#define PX_DEFAULT_TIME_TO_SLEEP 0.5f                       // half a second

PxWorld *pxWorldAlloc(void) {
  PxWorld *world = (PxWorld *)pxalloc(sizeof(PxWorld));

  if (!world) {
    assert(0 && "Failed to allocate memory for PxWorld");
  }

  return world;
}

PxWorld *pxWorldInit(PxWorld *world, uint8_t iterations, uint8_t targetFps) {
  world->iterations = iterations;
  world->staticBodies = (PxBodyArray){0};
  world->dynamicBodies = (PxBodyArray){0};
  world->contacts = (PxManifoldPool){0};

  // Default sleep settings
  world->linearSleepThresholdSq = PX_DEFAULT_SLEEP_THRESH_SQ;
  world->angularSleepThreshold = PX_DEFAULT_ANGULAR_SLEEP_THRESH;
  world->timeToSleep = PX_DEFAULT_TIME_TO_SLEEP;

  pxClockInit(&world->clock, targetFps);

  return world;
}

PxWorld *pxWorldNew(uint8_t targetFps) {
  return pxWorldInit(pxWorldAlloc(), PX_DEFAULT_ITERATIONS, targetFps);
}

void pxWorldFree(PxWorld *world) {
  if (!world) {
    return;
  }

  pxfree(world);
}

void pxWorldSetGravity(PxWorld *world, float x, float y) {
  if (!world) {
    return;
  }

  world->gravity = pxVec2Multf(pxVec2(x, y), PDPHYZX_UNIT);
}

void pxWorldSetIterations(PxWorld *world, uint8_t iterations) {
  if (!world) {
    return;
  }

  world->iterations = iterations;
}

/**
 * @brief Adds a new body to the specified body array
 *
 * @param world
 * @param shape    - shape of the new body
 * @param density  - density of the new body (0 density = static body)
 * @param position - initial position of the new body
 *
 * @return PxBody* Pointer to the newly added body
 */
static PxBody *pxWorldAddBody(PxWorld *world, PxShape shape, PxBodyFlags type,
                              float density, PxVec2 position) {

  PxBody body = pxBodyNew(shape, type, density, position);

  PxBodyArray *bodies =
      type == PX_BODY_DYNAMIC ? &world->dynamicBodies : &world->staticBodies;

  return pxBodyArrayAdd(bodies, &body);
}

PxBody *pxWorldNewStaticBody(PxWorld *world, PxShape shape, PxVec2 position) {
  PxBody *body = pxWorldAddBody(world, shape, PX_BODY_STATIC, 0, position);

  // Sort static bodies array when adding new body
  pxBodyArraySortByAxis(&world->staticBodies);

  return body;
}

PxBody *pxWorldNewDynamicBody(PxWorld *world, PxShape shape, float density,
                              PxVec2 position) {

  assert(density > 0 && "Dynamic bodies must have positive density");

  return pxWorldAddBody(world, shape, PX_BODY_DYNAMIC, density, position);
}

void pxWorldFreeBody(PxWorld *world, PxBody *body) {
  if (!world || !body) {
    return;
  }

  PxBodyArray *bodies =
      body->density > 0 ? &world->dynamicBodies : &world->staticBodies;

  pxBodyArrayRemove(bodies, body->worldIndex);
}

//===============================================================================
// Physics calculations
//===============================================================================

static void pxDetectRestingContacts(PxWorld *world, float dt) {
  pxManifoldPoolEach(&world->contacts, manifold) {
    pxManifoldDetectRestingContact(manifold, world->gravity, dt);
  }
}

/**
 * @brief Updates sleep states for all dynamic bodies
 *
 * Bodies moving below the sleep threshold accumulate sleep time.
 * Bodies moving above the threshold are awakened.
 *
 * @param world
 * @param dt - time step in seconds
 */
static void pxUpdateSleepStates(PxWorld *world, float dt) {
  // Bit-arrays to track body states (using uint64_t to handle up to 64 bodies)
  uint64_t hasRestingStaticContact = 0;
  uint64_t shouldWakeUp = 0;

  // Single pass through manifolds to collect all needed information
  pxManifoldPoolEach(&world->contacts, manifold) {
    PxBody *bodyA = manifold->bodyA;
    PxBody *bodyB = manifold->bodyB;

    bool isResting = (manifold->mixedRestitution == 0.0f);

    bool isDynBodyA = pxBodyIsDynamic(bodyA);
    bool isDynBodyB = pxBodyIsDynamic(bodyB);

    // Case 1: Dynamic body resting on static body
    if (isDynBodyA && !isDynBodyB && isResting) {
      assert(bodyA->worldIndex < 64 && "Dynamic body index exceeds 64");

      hasRestingStaticContact |= (1ULL << bodyA->worldIndex);
    } else if (bodyB->density > 0 && bodyA->density == 0 && isResting) {
      assert(bodyB->worldIndex < 64 && "Dynamic body index exceeds 64");

      hasRestingStaticContact |= (1ULL << bodyB->worldIndex);
    }

    // Case 2: Dynamic-dynamic collision where one is already awake
    else if (isDynBodyA && isDynBodyB && !isResting) {
      if (pxBodyIsSleeping(bodyA)) {
        shouldWakeUp |= (1ULL << bodyB->worldIndex);
      }

      if (pxBodyIsSleeping(bodyB)) {
        shouldWakeUp |= (1ULL << bodyA->worldIndex);
      }
    }
  }

  // Single pass through bodies to update sleep states
  pxBodyArrayEachWithIdx(&world->dynamicBodies, body, idx) {
    // Check if body should be woken up due to collision
    if (shouldWakeUp & (1ULL << idx)) {
      pxBodyWakeUp(body);
      continue;
    }

    // Check velocity thresholds
    bool isBelowThreshold =
        (pxVec2LenSqr(body->velocity) <= world->linearSleepThresholdSq) &&
        (fabsf(body->angularVelocity) <= world->angularSleepThreshold);

    if (!isBelowThreshold) {
      pxBodyWakeUp(body);
      continue;
    }

    if (pxBodyIsSleeping(body)) {
      // No need to continue adding if the body is already sleeping
      continue;
    }

    // Normal sleep time increase
    body->sleepTime += dt;

    // Extra sleep time if resting on static body
    if (hasRestingStaticContact & (1ULL << idx)) {
      body->sleepTime += dt; // Double sleep time accumulation
    }

    if (body->sleepTime >= world->timeToSleep) {
      body->velocity = pxVec2(0, 0);
      body->angularVelocity = 0.0f;
      pxBodyClearForces(body);
      pxBodySetFlag(body, PX_BODY_FLAG_SLEEPING, true);
    }

    continue;
  }
}

/**
 * @brief Applies forces to bodies and updates their velocities
 *
 * Integrates external forces (including gravity) for dynamic bodies that are
 * not sleeping.
 *
 * @param world
 * @param g  - gravity vector
 * @param dt - time step in seconds
 */
static void pxIntegrateForces(PxWorld *world, float dt) {
  pxBodyArrayEach(&world->dynamicBodies, body) {
    if (pxBodyIsSleeping(body)) {
      continue;
    }

    pxBodyIntegrateForces(body, world->gravity, dt);
  }
}

/**
 * @brief Resolves collisions by applying impulses
 *
 * Iterates through all contacts multiple times according to world->iterations
 * to resolve velocity constraints.
 *
 * @param world
 */
static void pxApplyImpulses(PxWorld *world) {
  for (uint8_t j = 0; j < world->iterations; j++) {
    pxManifoldPoolEach(&world->contacts, manifold) {
      if (pxBodyIsSleeping(manifold->bodyA) &&
          pxBodyIsSleeping(manifold->bodyB)) {
        continue;
      }

      pxManifoldApplyImpulse(manifold);
    }
  }
}

/**
 * @brief Updates positions of bodies based on their velocities
 *
 * Integrates velocities to determine new positions of dynamic bodies that are
 * not sleeping.
 *
 * @param world
 * @param dt - time step in seconds
 */
static void pxIntegrateVelocities(PxWorld *world, float dt) {
  pxBodyArrayEach(&world->dynamicBodies, body) {
    if (pxBodyIsSleeping(body)) {
      continue;
    }

    pxBodyIntegrateVelocity(body, dt);
  }
}

/**
 * @brief Corrects positions to prevent penetration
 *
 * Resolves position constraints by adjusting overlapping bodies.
 *
 * @param world
 */
static void pxCorrectPositions(PxWorld *world) {
  pxManifoldPoolEach(&world->contacts, manifold) {
    pxManifoldCorrectPosition(manifold);
  }
}

/**
 * @brief Resets forces on all dynamic bodies
 *
 * Clears accumulated forces and torques on dynamic bodies that are not
 * sleeping.
 *
 * @param world physics world
 */
static void pxClearForces(PxWorld *world) {
  pxBodyArrayEach(&world->dynamicBodies, body) {
    // Skip sleeping bodies (optional, depending on your design)
    if (body->sleepTime >= world->timeToSleep) {
      continue;
    }

    pxBodyClearForces(body);
  }
}

//==============================================================================
// Step
//==============================================================================

/**
 * @brief Performs collision detection between two bodies
 *
 * First checks AABB overlap, then performs detailed collision detection.
 * Creates a manifold if collision occurs and wakes up sleeping bodies.
 *
 * @param world
 * @param bodyA
 * @param bodyB
 */
static void pxDetectCollision(PxWorld *world, PxBody *bodyA, PxBody *bodyB) {
  // Only perform detailed collision if AABBs overlap
  if (!pxBodyAABBsOverlap(bodyA, bodyB)) {
    return;
  }

  PxManifold *manifold = pxManifoldPoolAcquire(&world->contacts);
  if (!manifold) {
    assert(0 && "Manifold pool exhausted");
  }

  pxManifoldInit(manifold, bodyA, bodyB);
  pxManifoldSolve(manifold);

  // Nothing collided, release the manifold
  if (manifold->contacts.length == 0) {
    pxManifoldPoolReleaseLast(&world->contacts);
  }
}

/**
 * @brief Generates collision information for all potentially colliding pairs
 *
 * Implements a sweep and prune algorithm on the x-axis to efficiently find
 * potentially colliding body pairs.
 *
 * @param world
 */
static void pxGenerateCollisionInfo(PxWorld *world) {
  // Sort dynamic bodies by min x-coordinate (uses insertion sort)
  pxBodyArraySortByAxis(&world->dynamicBodies);

  // For each dynamic body
  pxBodyArrayEachWithIdx(&world->dynamicBodies, bodyA, idx) {
    float minX = bodyA->aabb.min.x;
    float maxX = bodyA->aabb.max.x;

    // Check against other dynamic bodies with potential overlap
    pxBodyArrayEachFrom(&world->dynamicBodies, idx + 1, bodyB) {
      // If both bodies are sleeping, skip this pair
      if (pxBodyIsSleeping(bodyA) && pxBodyIsSleeping(bodyB)) {
        continue;
      }

      // Early exit when we've moved past possible collisions
      if (bodyB->aabb.min.x > maxX) {
        break;
      }

      pxDetectCollision(world, bodyA, bodyB);
    }

    // Binary search to find first static body that might collide
    uint8_t start = pxBodyArrayFindFirstIndexAfterX(&world->staticBodies, minX);

    // Check static bodies until we find one past our max X
    pxBodyArrayEachFrom(&world->staticBodies, start, bodyB) {
      // Early exit when we've moved past possible collisions
      if (bodyB->aabb.min.x > maxX) {
        break;
      }

      pxDetectCollision(world, bodyA, bodyB);
    }
  }
}

/**
 * @brief Advances the physics simulation by one time step
 *
 * The sequence of operations is:
 * 1. Clear existing contacts
 * 2. Detect new collisions
 * 3. Integrate forces (apply gravity and other forces)
 * 4. Solve velocity constraints (apply impulses)
 * 5. Integrate velocities (update positions)
 * 6. Correct positions (prevent penetration)
 * 7. Clear forces for next step
 *
 * @param world - physics world to update
 * @param g     - gravity vector
 * @param dt    - time step delta in seconds
 */
static void pxWorldStepWithDt(PxWorld *world, float dt) {
  pxManifoldPoolClear(&world->contacts);
  pxGenerateCollisionInfo(world);
  pxDetectRestingContacts(world, dt);
  pxIntegrateForces(world, dt);
  pxApplyImpulses(world);
  pxIntegrateVelocities(world, dt);
  pxCorrectPositions(world);
  pxUpdateSleepStates(world, dt);
  pxClearForces(world);
}

bool pxWorldStep(PxWorld *world) {
  PxClock *clock = &world->clock;
  bool stepped = false;
  uint8_t stepCount = 0;

  const int SUBSTEPS = 5;

  pxClockBeginFrame(clock);

  while (pxClockShouldStep(clock) && stepCount < clock->maxStepsPerFrame) {
    float substepDt = pxFastDiv(pxClockGetFixedDeltaTime(clock), SUBSTEPS);

    // Perform multiple substeps for each physics step
    for (int i = 0; i < SUBSTEPS; i++) {
      pxWorldStepWithDt(world, substepDt);
    }

    pxClockAdvance(clock);
    stepped = true;
    stepCount++;
  }

  return stepped;
}

//==============================================================================
// Debug
//==============================================================================

void pxWorldDrawDebug(PxWorld *world) {
  // Track sleep statistics
  uint8_t sleepingCount = 0;

  // Draw AABBs for all bodies in the static array
  pxBodyArrayEach(&world->staticBodies, body) {
    PxAABB aabb = body->aabb;

    pd->graphics->drawRect(aabb.min.x, aabb.min.y, aabb.max.x - aabb.min.x,
                           aabb.max.y - aabb.min.y, kColorXOR);
  }

  // Draw AABBs for all bodies in the dynamic array
  pxBodyArrayEach(&world->dynamicBodies, body) {
    PxAABB aabb = body->aabb;

    // Draw AABB for all bodies
    pd->graphics->drawRect(aabb.min.x, aabb.min.y, aabb.max.x - aabb.min.x,
                           aabb.max.y - aabb.min.y, kColorXOR);

    if (pxBodyIsSleeping(body)) {
      sleepingCount++;

      // Mark sleeping bodies with a square
      float centerX = (aabb.min.x + aabb.max.x) / 2.0f;
      float centerY = (aabb.min.y + aabb.max.y) / 2.0f;
      float size = 4;

      pd->graphics->drawRect(centerX - size, centerY - size, size * 2, size * 2,
                             kColorXOR);
    }

    // Draw velocity vector (scaled for visibility)
    float velLen = pxVec2Len(body->velocity);
    if (velLen > 0.01f) {
      float centerX = (aabb.min.x + aabb.max.x) / 2;
      float centerY = (aabb.min.y + aabb.max.y) / 2;
      pd->graphics->drawLine(centerX, centerY, centerX + body->velocity.x * 10,
                             centerY + body->velocity.y * 10, 1, kColorBlack);
    }
  }

  // Display statistics
  char *countText = NULL;
  int len = pd->system->formatString(&countText, "Contacts: %d  Sleeping: %d ",
                                     world->contacts.length, sleepingCount);

  pd->graphics->drawText(countText, len, kASCIIEncoding, 5, 20);
}
