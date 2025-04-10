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

#define PX_BODY_WAKE_THRESHOLD 0.0001f

/**
 * @brief Represents a circular shape in the physics system
 */
typedef struct {
  float radius; /**< Radius of the circle */
} PxCircle;

/**
 * @brief Represents a polygon shape in the physics system
 */
typedef struct {
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
typedef struct {
  PxCircle
      circle; /**< Circle shape data (initialize this for circular bodies) */
  PxPolygon
      polygon; /**< Polygon shape data (initialize this for polygon bodies) */
  PxBox box;   /**< Box shape data (initialize this for box bodies) */
} PxShape;

/**
 * @brief Represents a physical body in the simulation
 */
typedef struct {
  // Group by simulation step access patterns (hot data first)
  PxVec2 position;        /**< Current position */
  PxVec2 velocity;        /**< Linear velocity */
  PxMat2 orientation;     /**< Rotation matrix for the body */
  PxVec2 force;           /**< Accumulated force for current step */
  float angularVelocity;  /**< Rotational velocity in radians/sec */
  float torque;           /**< Accumulated torque for current step */
  float orientationAngle; /**< Orientation angle in radians */

  // Mass properties (accessed together in force calculations)
  float mass;  /**< Mass of the body */
  float iMass; /**< Inverse mass (1/mass) for efficient calculations */
  float momentOfInertia;  /**< Moment of inertia for rotation */
  float iMomentOfInertia; /**< Inverse moment of inertia for calculations */

  // Material properties (accessed together in collision response)
  float restitution;     /**< Bounciness coefficient (0-1) */
  float staticFriction;  /**< Friction coefficient when stationary */
  float dynamicFriction; /**< Friction coefficient when moving */
  float density;         /**< Material density for mass calculations */

  // Collision related data
  PxAABB aabb; /** < Axis-aligned bounding box for broad-phase collision */
  PxCollider collider; /**< Collision shape of the body */

  // Small fields at the end to maintain alignment
  uint8_t worldIndex; /**< Index in body array of world containing this body or
                           INVALID_WORLD_INDEX */

  bool isValid;    /**< Flag indicating if the body is valid */
  uint8_t padding; /**< Padding to maintain 4-byte alignment */
  float sleepTime; /**< Time the body has been below motion threshold */
} PxBody;

void pxBodySetPosition(PxBody *body, PxVec2 position);

void pxBodyMoveBy(PxBody *body, PxVec2 distance);

/**-
 * @brief Creates a new physics body
 *
 * @param shape    - collision shape of the body (initialize only one field)
 * @param density  - material density used for mass calculations
 * @param position - initial position of the body
 *
 * @return A new PxBody instance
 *
 * @example
 * // Create a circular body
 * PxBody circle = pxBodyNew(
 *   (PxShape){ .circle = { .radius = 1.0f } }, 1.0f, vec2(0, 0)
 * );
 *
 * // Create a box body
 * PxBody box = pxBodyNew(
 *   (PxShape){ .box = { .width = 2.0f, .height = 1.0f } }, 1.0f, vec2(0, 0)
 * );
 *
 * @note Returns by value for simplicity and to avoid memory management
 *       concerns; check the isValid field to confirm successful creation
 */
PxBody pxBodyNew(PxShape shape, float density, PxVec2 position);

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

// TODO: Flesh these out + add comments
void pxBodyApplyForce(PxBody *body, PxVec2 force, PxVec2 contact);
void pxBodyApplyImpulse(PxBody *body, PxVec2 impulse, PxVec2 contact);

/**
 * @brief Updates the Axis-Aligned Bounding Box (AABB) for a physics body
 *
 * This function updates the body's AABB by calling the collider's updateAABB
 * method. For both circles and polygons, a radius-based approach is used where
 * the AABB is calculated as position ± radius. This provides a fast,
 * conservative bound for broad-phase collision detection.
 *
 * @param body - physics body whose AABB needs to be updated
 *
 * @note For efficiency, this uses a circular approximation for all collider
 *       shapes
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
 * @param body - pointer to the body to integrate (if NULL, function is a
 * no-op)
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
 * @param body - pointer to the body to integrate (if NULL, function is a
 * no-op)
 * @param dt   - time step duration
 */
void pxBodyIntegrateVelocity(PxBody *body, float dt);

/**
 * @brief Clears all accumulated forces acting on the body
 *
 * This function resets the forces acting on the specified body to zero,
 * effectively removing any applied forces.
 *
 * @param body Pointer to the body whose forces are to be cleared
 */
void pxBodyClearForces(PxBody *body);

void pxBodyWakeUp(PxBody *body);

/**
 * @brief Array container for efficiently managing body instances
 *
 * Uses separate active/freed index arrays to allow fast iteration over only
 * active bodies while maintaining O(1) addition and removal.
 */
typedef struct {
  PxBody items[PX_MAX_BODIES]; /**< Storage for body data */

  // Small metadata arrays for efficient body management
  uint8_t length;     /**< Number of active bodies */
  uint8_t freedCount; /**< Number of freed slots available for reuse */

  uint16_t padding; // Align to 4 bytes

  // These small arrays have good cache locality
  uint8_t activeIndices[PX_MAX_BODIES]; /**< Indices of active bodies for fast
                                        iteration */
  uint8_t freedIndices[PX_MAX_BODIES]; /**< Indices of freed slots available for
                                       reuse */
} PxBodyArray;

/**
 * @brief Macro for iterating through active bodies
 *
 * Usage:
 *
 * ```
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
 * ```
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
 * ```
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
 * @return Pointer to the stored body or NULL if array is full or body is
 *         invalid
 */
PxBody *pxBodyArrayAdd(PxBodyArray *array, PxBody *body);

/**
 * @brief Removes a body from the array
 *
 * @param array - pointer to the body array
 * @param index - index of the body to remove
 *
 * @note Attempting to remove an invalid index or already removed body is a
 *       no-op
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
 * @note This is typically used before collision detection to reduce the
 * number of detailed collision checks required
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
