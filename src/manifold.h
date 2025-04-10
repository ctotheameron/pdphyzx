#ifndef PDPHYZX_MANIFOLD_H
#define PDPHYZX_MANIFOLD_H

#include <stdint.h>

#include "body.h"
#include "vec2.h"

/**
 * Maximum number of contact points stored in a manifold.
 * For 2D physics, two points is typically sufficient.
 */
#define PX_MAX_CONTACTS 2

// Maximum penetration allowed before position correction
#define PX_ALLOWED_PENETRATION 0.01f

// Baumgarte stabilization factor (0.2-0.8) controlling correction strength
#define PX_POSITIONAL_CORRECTION_FACTOR 0.2f

/**
 * @brief Contact manifold between two colliding bodies.
 *
 * A manifold stores information about a collision between two bodies, including
 * the collision normal, penetration depth, contact points, and material
 * properties needed for collision resolution.
 */
typedef struct PxManifold {
  // Start with 4-byte pointer fields
  PxBody *bodyA; /**< First body involved in collision */
  PxBody *bodyB; /**< Second body involved in collision */

  // Vec2 (8 bytes, naturally aligned to 4 bytes on ARM)
  PxVec2 normal; /**< Contact normal vector (points from bodyA to bodyB) */

  // Group all floats together (4 bytes each)
  float penetration;      /**< Penetration depth between bodies */
  float staticFriction;   /**< Combined static friction */
  float dynamicFriction;  /**< Combined dynamic friction */
  float mixedRestitution; /**< Combined coefficient of restitution (bounce) */

  // Place Vec2Array last (contains two Vec2 and a uint8_t length)
  PxVec2Array2 contacts; /**< Array of contact points between bodies */
} PxManifold;

/**
 * @brief Initialize an existing manifold.
 *
 * @param manifold - manifold to initialize
 * @param bodyA    - first body in the collision pair
 * @param bodyB    - second body in the collision pair
 *
 * @return Initialized manifold (same as input)
 */
PxManifold *pxManifoldInit(PxManifold *manifold, PxBody *bodyA, PxBody *bodyB);

/**
 * @brief Detect collision between the bodies in the manifold.
 *
 * Populates the manifold with contact information if a collision is detected.
 * After calling, check if manifold->contacts.length > 0 to confirm collision.
 *
 * @param manifold - manifold to solve (modified in-place)
 */
void pxManifoldSolve(PxManifold *manifold);

/**
 * @brief Determine if the contacts represent a resting collision.
 *
 * Resting contacts occur when objects are stacked or sitting on each other.
 * This function adjusts restitution to prevent objects from bouncing when
 * their relative velocity is below what would be caused by gravity in one step.
 *
 * @param manifold - manifold to analyze (modified in-place)
 * @param gy       - current gravity vector
 * @param dt       - time step in seconds
 */
void pxManifoldDetectRestingContact(PxManifold *manifold, PxVec2 g, float dt);

/**
 * @brief Apply collision impulses to resolve the contact.
 *
 * Calculates and applies impulses to both bodies to resolve velocity
 * constraints based on friction and restitution.
 *
 * @param manifold - manifold containing collision data (bodies modified
 *                   in-place)
 */
void pxManifoldApplyImpulse(PxManifold *manifold);

/**
 * @brief Correct position overlap between colliding bodies.
 *
 * Adjusts body positions to eliminate excessive interpenetration.
 * This helps prevent jittering and improves simulation stability.
 *
 * @param manifold - manifold containing collision data (bodies modified
 *                   in-place)
 */
void pxManifoldCorrectPosition(PxManifold *manifold);

#endif // PDPHYZX_MANIFOLD_H
