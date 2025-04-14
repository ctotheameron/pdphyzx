#ifndef PDPHYZX_BODY_H
#define PDPHYZX_BODY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "collider.h"
#include "mat2.h"
#include "vec2.h"

/**
 * @brief Maximum number of bodies that can exist in the simulation
 */
#define PX_MAX_BODIES 64

/**
 * @brief Special value indicating a body is not assigned to any world
 */
#define PX_INVALID_WORLD_INDEX 255

/**
 * @brief Represents a circular shape in the physics system
 */
typedef struct {
  float radius; /**< Radius of the circle */
} PxCircle;

/**
 * @brief Represents a polygon shape in the physics system
 */
typedef struct __attribute__((aligned(4))) {
  PxVec2 vertices[PX_MAX_POLY_VERTEX_COUNT]; /**< Array of vertices defining the
                                           polygon */

  size_t vertexCount; /**< Number of vertices in use
                           (must be <=PX_MAX_POLY_VERTEX_COUNT) */
} PxPolygon;

/**
 * @brief Represents a box shape in the physics system
 */
typedef struct {
  float width;  /**< Width of the box */
  float height; /**< Height of the box */
} PxBox;

/**
 * @brief Container for shape data to simplify body creation API
 *
 * @note Initialize only ONE of these fields when creating a shape.
 * - Example for circle: (PxShape){ .circle = { .radius = 1.0f } }
 * - Example for box: (PxShape){ .box = { .width = 2.0f, .height = 1.0f } }
 */
typedef struct __attribute__((aligned(4))) {
  PxCircle
      circle; /**< Circle shape data (initialize this for circular bodies) */
  PxPolygon
      polygon; /**< Polygon shape data (initialize this for polygon bodies) */
  PxBox box;   /**< Box shape data (initialize this for box bodies) */
} PxShape;

/**
 * Body flags (8 bits)
 */
typedef enum {
  // Collision types (2 bits, positions 0-1)
  PX_BODY_STATIC = 0,    /**< does not move */
  PX_BODY_DYNAMIC = 1,   /**< can move */
  PX_BODY_KINEMATIC = 2, /**< moves but not affected by forces */

  // States (single bit flags, starting from position 2)
  PX_BODY_FLAG_VALID = (1 << 2),    /**< body is valid and in use */
  PX_BODY_FLAG_SLEEPING = (1 << 3), /**< body is sleeping and not moving */

  // Placeholder for future flags:
  // XX_BODY_FLAG_1 = (1 << 4),
  // XX_BODY_FLAG_2 = (1 << 5),
  // XX_BODY_FLAG_3 = (1 << 6),
  // XX_BODY_FLAG_4 = (1 << 7),

  // Type mask
  PX_BODY_TYPE_MASK = 0x03 // First two bits for type
} PxBodyFlags;

/**
 * @brief Represents a physical body in the simulation
 */
typedef struct __attribute__((aligned(4))) {
  // Group 1: Highest frequency access
  uint8_t flags;      /**< Flags for body type + states (includes validity) */
  uint8_t worldIndex; /**< Index in body array of world containing this body
                           or INVALID_WORLD_INDEX */

  /** 2 bytes of compiler-inserted padding here */

  float sleepTime; /**< Time the body has been below motion threshold */
  float mass;      /**< Mass of the body */
  float iMass;     /**< Inverse mass (1/mass) for efficient calculations */

  // Group 2: Position/movement data (frequent access in main loop) (24 bytes)
  PxVec2 position;        /**< Current position */
  PxVec2 velocity;        /**< Linear velocity */
  float angularVelocity;  /**< Rotational velocity in radians/sec */
  float orientationAngle; /**< Orientation angle in radians */

  // Group 3: Current frame force accumulation (modified each frame) (12 bytes)
  PxVec2 force; /**< Accumulated force for current step */
  float torque; /**< Accumulated torque for current step */

  // Group 4: Inertial properties (read-only after init) (8 bytes)
  float momentOfInertia;  /**< Moment of inertia for rotation */
  float iMomentOfInertia; /**< Inverse moment of inertia for calculations */

  // Group 5: Material properties (read during collision) (16 bytes)
  float restitution;     /**< Bounciness coefficient (0-1) */
  float staticFriction;  /**< Friction coefficient when stationary */
  float dynamicFriction; /**< Friction coefficient when moving */
  float density;         /**< Material density for mass calculations */

  // Group 6: Derived data (occasional updates) (16 bytes)
  PxMat2 orientation; /**< Rotation matrix for the body */

  // Group 7: Collision data (AABB accessed frequently, collider less so)
  PxAABB aabb; /** < Axis-aligned bounding box for broad-phase collision */
  PxCollider collider; /**< Collision shape of the body */
} PxBody;

/**
 * @brief Checks if a body has a specific flag set
 *
 * @param body
 * @param flag
 * @return true if the flag is set, false otherwise
 */
static inline bool pxBodyHasFlag(PxBody *body, PxBodyFlags flag) {
  return (body->flags & flag) != 0;
}

/**
 * @brief Sets or clears a specific flag on a body
 *
 * @param body
 * @param flag
 * @param value - true to set the flag, false to clear it
 */
static inline void pxBodySetFlag(PxBody *body, PxBodyFlags flag, bool value) {
  body->flags = value ? (body->flags | flag) : (body->flags & ~flag);
}

/**
 * @brief Sets the body type (static, dynamic, or kinematic)
 *
 * @param body
 * @param type - (PX_BODY_STATIC, PX_BODY_DYNAMIC, or PX_BODY_KINEMATIC)
 */
static inline void pxBodySetType(PxBody *body, PxBodyFlags type) {
  body->flags = (body->flags & ~PX_BODY_TYPE_MASK) | (type & PX_BODY_TYPE_MASK);
}

/**
 * @brief Gets the body type (static, dynamic, or kinematic)
 *
 * @param body
 *
 * @return PxBodyFlags The body type as flags
 */
static inline PxBodyFlags pxBodyGetType(PxBody *body) {
  return body->flags & PX_BODY_TYPE_MASK;
}

/**
 * @brief Checks if a body is currently sleeping
 *
 * @param body
 *
 * @return true if the body is sleeping, false if awake
 */
static inline bool pxBodyIsSleeping(PxBody *body) {
  return pxBodyHasFlag(body, PX_BODY_FLAG_SLEEPING);
}

/**
 * @brief Checks if a body is valid (non-NULL and has the valid flag set)
 *
 * @param body
 * @return true if the body is valid, false otherwise
 */
static inline bool pxBodyIsValid(PxBody *body) {
  return body != NULL && pxBodyHasFlag(body, PX_BODY_FLAG_VALID);
}

/**
 * @brief Checks if a body is static (has zero mass and cannot move)
 *
 * @param body
 * @return true if the body is static, false otherwise
 */
static inline bool pxBodyIsStatic(PxBody *body) {
  return pxBodyGetType(body) == PX_BODY_STATIC;
}

/**
 * @brief Checks if a body is dynamic (has mass and responds to forces)
 *
 * @param body
 * @return true if the body is dynamic, false otherwise
 */
static inline bool pxBodyIsDynamic(PxBody *body) {
  return pxBodyGetType(body) == PX_BODY_DYNAMIC;
}

/**
 * @brief Checks if a body is kinematic (has infinite mass but can be moved)
 *
 * @param body
 * @return true if the body is kinematic, false otherwise
 */
static inline bool pxBodyIsKinematic(PxBody *body) {
  return pxBodyGetType(body) == PX_BODY_KINEMATIC;
}

/**
 * @brief Wakes up a sleeping body
 *
 * Resets sleep time to zero and clears the sleeping flag.
 *
 * @param body
 */
void pxBodyWakeUp(PxBody *body);

/**
 * @brief Sets the position of a body
 *
 * Updates the body's position and recalculates its AABB.
 *
 * @param body
 * @param position
 */
void pxBodySetPosition(PxBody *body, PxVec2 position);

/**
 * @brief Moves a body by the specified distance
 *
 * Updates the body's position by adding the distance vector and recalculates
 * its AABB.
 *
 * @param body
 * @param distance
 */
void pxBodyMoveBy(PxBody *body, PxVec2 distance);

/**
 * @brief Creates a new physics body
 *
 * @param shape    - collision shape of the body (initialize only one field)
 * @param type     - body type flag (`PX_BODY_<STATIC, DYNAMIC, KINEMATIC>`)
 * @param density  - material density used for mass calculations
 * @param position - initial position of the body
 *
 * @return A new PxBody instance
 *
 * @example
 * ```c
 * // Create a circular dynamic body
 * PxBody circle = pxBodyNew(
 *   (PxShape){ .circle = { .radius = 1.0f } },
 *   PX_BODY_DYNAMIC, 1.0f, vec2(0, 0)
 * );
 *
 * // Create a box static body
 * PxBody box = pxBodyNew(
 *   (PxShape){ .box = { .width = 2.0f, .height = 1.0f } },
 *   PX_BODY_STATIC, 0.0f, vec2(0, 0)
 * );
 * ```
 *
 * @note Returns by value for simplicity and to avoid memory management
 *       concerns; check the isValid field to confirm successful creation
 */
PxBody pxBodyNew(PxShape shape, PxBodyFlags type, float density,
                 PxVec2 position);

/**
 * @brief Updates a body's rotation to a desired angle
 *
 * @param body    - pointer to the body to update (if NULL, function is a no-op)
 * @param radians - orientation angle in radians
 */
void pxBodySetOrientation(PxBody *body, float radians);

/**
 * @brief Rotates a body by a specified angle
 *
 * @param body    - pointer to the body to rotate (if NULL, function is a no-op)
 * @param radians - angle to rotate the body by in radians
 */
void pxBodyRotate(PxBody *body, float radians);

/**
 * @brief Applies a force to a body at a specific contact point
 *
 * This function adds a force to a body's accumulated forces for the current
 * physics step. The force is applied at the specified contact point, which
 * may generate torque depending on the point's offset from the center of mass.
 *
 * @param body    - body to apply force to (must be dynamic)
 * @param force   - force vector to apply (direction and magnitude)
 * @param contact - point where the force is applied, in world coordinates
 *
 * @note Forces are accumulated and only applied during the next integration
 *       step. Force application has no effect on static or kinematic bodies.
 */
void pxBodyApplyForce(PxBody *body, PxVec2 force, PxVec2 contact);

/**
 * @brief Applies an impulse to a body at a specific contact point
 *
 * This function directly modifies a body's velocity based on the impulse.
 * The impulse is applied at the specified contact point, which may generate
 * angular impulse depending on the point's offset from the center of mass.
 *
 * @param body    - body to apply impulse to (must be dynamic)
 * @param impulse - impulse vector to apply (direction and magnitude)
 * @param contact - point where the impulse is applied, in world coordinates
 *
 * @note Unlike forces, impulses are applied immediately and directly affect
 *       velocity. Impulse application has no effect on static or kinematic
 *       bodies.
 */
void pxBodyApplyImpulse(PxBody *body, PxVec2 impulse, PxVec2 contact);

/**
 * @brief Updates the Axis-Aligned Bounding Box (AABB) for a physics body
 *
 * This function updates the body's AABB by calling the collider's updateAABB
 * method. For both circles and polygons, a radius-based approach is used
 * where the AABB is calculated as position ± radius. This provides a fast,
 * conservative bound for broad-phase collision detection.
 *
 * @param body - physics body whose AABB needs to be updated
 *
 * @note For efficiency, uses a circular approximation for all collider shapes
 */
void pxBodyUpdateAABB(PxBody *body);

/**
 * @brief Determines whether two AABBs overlap
 *
 * Two AABBs overlap when they intersect on both the x and y axes.
 * This is a fast broad-phase collision check.
 *
 * @param bodyA
 * @param bodyB
 * @return true if the AABBs overlap, false otherwise
 */
bool pxBodyAABBsOverlap(PxBody *bodyA, PxBody *bodyB);

/**
 * @brief Integrates forces acting on a body over a time step
 *
 * This function updates the body's velocity based on the accumulated forces
 * and the provided gravitational acceleration.
 *
 * @param body - pointer to the body to integrate (if NULL, function is a no-op)
 * @param g    - gravitational acceleration vector
 * @param dt   - time step duration
 */
void pxBodyIntegrateForces(PxBody *body, PxVec2 g, float dt);

/**
 * @brief Integrates the body's velocity over a time step
 *
 * This function updates the body's position based on its current velocity
 * and the provided time step duration.
 *
 * @param body - pointer to the body to integrate (if NULL, function is a no-op)
 * @param dt   - time step duration
 */
void pxBodyIntegrateVelocity(PxBody *body, float dt);

/**
 * @brief Clears all accumulated forces acting on the body
 *
 * This function resets the forces acting on the specified body to zero,
 * effectively removing any applied forces.
 *
 * @param - pointer to the body whose forces are to be cleared
 */
void pxBodyClearForces(PxBody *body);

/**
 * @brief Array container for efficiently managing body instances
 *
 * Uses separate active/freed index arrays to allow fast iteration over only
 * active bodies while maintaining O(1) addition and removal.
 */
typedef struct __attribute__((aligned(4))) {
  PxBody items[PX_MAX_BODIES]; /**< Storage for body data */

  // Small metadata arrays for efficient body management
  uint8_t length;     /**< Number of active bodies */
  uint8_t freedCount; /**< Number of freed slots available for reuse */

  /** 2 bytes of compiler-inserted padding here */

  // These small arrays have good cache locality
  uint8_t activeIndices[PX_MAX_BODIES]; /**< Indices of active bodies for fast
                                        iteration */
  uint8_t freedIndices[PX_MAX_BODIES];  /**< Indices of freed slots available
                                        for  reuse */
} PxBodyArray;

/**
 * @brief Macro for iterating through active bodies
 *
 * Usage:
 *
 * ```c
 * pxBodyArrayEach(bodyArray, body) { // Start from beginning
 *    // body is a pointer to the current active body
 *    pxBodyApplyForce(body, ...);
 *  }
 * ```
 */
#define pxBodyArrayEach(array, body)                                           \
  for (uint8_t _i = 0; _i < (array)->length; _i++)                             \
    for (PxBody *body = &(array)->items[(array)->activeIndices[_i]]; body;     \
         body = NULL)

/**
 * @brief Macro for iterating through active bodies starting from an index
 *
 * Usage:
 *
 * ```c
 * pxBodyArrayEachFrom(bodyArray, 5, body) { // Start from 5th
 *    // body is a pointer to the current active body
 *    pxBodyApplyForce(body, ...);
 *  }
 * ```
 */
#define pxBodyArrayEachFrom(array, offset, body)                               \
  for (uint8_t _i = (offset); _i < (array)->length; ++_i)                      \
    for (PxBody *body = &(array)->items[(array)->activeIndices[_i]]; body;     \
         body = NULL)

/**
 * @brief Macro for iterating through active bodies while tracking current index
 *
 * Usage:
 *
 * ```c
 * pxBodyArrayEachFrom(bodyArray, body, idx) {
 *    printf("Current index: %d\n", idx);
 *
 *    // body is a pointer to the current active body
 *    pxBodyApplyForce(body, ...);
 *  }
 * ```
 */
#define pxBodyArrayEachWithIdx(array, body, idx)                               \
  for (uint8_t idx = 0; idx < (array)->length; idx++)                          \
    for (PxBody *body = &(array)->items[(array)->activeIndices[idx]]; body;    \
         body = NULL)

/**
 * @brief Adds a body to the array
 *
 * @param array - pointer to the body array
 * @param body  - pointer to the body to add
 *
 * @return pointer to stored body or NULL if array is full or body is invalid
 */
PxBody *pxBodyArrayAdd(PxBodyArray *array, PxBody *body);

/**
 * @brief Removes a body from the array
 *
 * @param array - pointer to the body array
 * @param index - index of the body to remove
 *
 * @note Attempting to remove invalid index or already removed body is a no-op
 */
void pxBodyArrayRemove(PxBodyArray *array, uint8_t index);

/**
 * @brief Sorts the bodies in the array by their minimum X axis coordinate.
 *
 * This function reorders only the internal activeIndices array to optimize
 * broad-phase collision detection using a sweep-and-prune algorithm. The
 * bodies themselves remain in their original memory locations, so external
 * pointers to bodies remain valid after sorting.
 *
 * @param bodies - pointer to the body array to be sorted
 *
 * @note This is typically used before collision detection to reduce the number
 * of detailed collision checks required
 */
void pxBodyArraySortByAxis(PxBodyArray *bodies);

/**
 * @brief Finds the index of the first body in a sorted array that could
 * potentially overlap with a body having the given minimum x-coordinate
 *
 * Uses binary search on a PxBodyArray that has been sorted by min.x
 *
 * @param bodies - bodies sorted by min.x coordinate
 * @param minX   - minimum x-coordinate to test against
 *
 * @return index of the first potential overlapping body
 */
uint8_t pxBodyArrayFindFirstIndexAfterX(PxBodyArray *bodies, float minX);

#endif // PDPHYZX_BODY_H
