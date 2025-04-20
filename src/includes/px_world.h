/**
 * @file px_world.h
 * @brief Physics world management for the PDPhyzx 2D physics engine
 *
 * This header defines the public interface for the physics world component,
 * which is the central container for physical simulation in PDPhyzx.
 * It provides functionality for:
 *  - Creating and destroying physics worlds
 *  - Adding static and dynamic bodies to the simulation
 *  - Advancing the physics simulation with collision detection and resolution
 *  - Managing time steps for stable physics simulation
 *
 * The physics world handles collections of bodies, detects collisions between
 * them, and resolves those collisions according to physical principles.
 */

#ifndef PDPHYZX_PX_WORLD_H
#define PDPHYZX_PX_WORLD_H

#include "px_body_array.h"
#include "px_clock.h"
#include "px_manifold_pool.h"

/**
 * @brief Physics world container that manages physical bodies and simulations
 *
 * The PxWorld struct maintains collections of static and dynamic bodies,
 * handles collision detection, and steps the physics simulation forward in
 * time.
 */
typedef struct {
  PxBodyArray staticBodies;
  PxBodyArray dynamicBodies;
  PxManifoldPool contacts;

  uint8_t iterations;
  PxVec2 gravity; // m/s^2

  float linearSleepThresholdSq; // m/s (squared)
  float angularSleepThreshold;  // rad/s
  float timeToSleep;            // seconds

  PxClock clock;
} PxWorld;

/**
 * @brief Creates a new physics world
 *
 * @param targetFps  - target frames per second for simulation stability
 *
 * @return pointer to newly created PxWorld or NULL if allocation fails
 */
PxWorld *pxWorldNew(uint8_t targetFps);

/**
 * @brief Destroys a physics world and frees associated memory
 *
 * @param world
 */
void pxWorldFree(PxWorld *world);

/**
 * @brief Sets the gravity vector for the physics world
 *
 * This function sets the global gravity acceleration that affects all dynamic
 * bodies within the physics world. The values are automatically scaled to
 * internal units.
 *
 * @param world - the physics world to update
 * @param x     - x component of gravity in m/s²
 * @param y     - y component of gravity in m/s²
 */
void pxWorldSetGravity(PxWorld *world, float x, float y);

/**
 * @brief Sets the number of constraint solving iterations per physics step
 *
 * Higher iteration counts result in more accurate collision resolution but
 * require more computation time. Typically values between 4-20 are used.
 *
 * @param world      - the physics world to update
 * @param iterations - number of iterations for constraint solving
 */
void pxWorldSetIterations(PxWorld *world, uint8_t iterations);

/**
 * @brief Creates a new static body in the physics world
 *
 * Static bodies do not move in response to collisions but affect dynamic
 * bodies.
 *
 * @param world
 * @param shape    - shape for collision detection
 * @param position - initial position of the body
 *
 * @return pointer to the newly created static body
 */
PxBody *pxWorldNewStaticBody(PxWorld *world, PxShape shape, PxVec2 position);

/**
 * @brief Creates a new dynamic body in the physics world
 *
 * Dynamic bodies move in response to forces and collisions.
 *
 * @param world
 * @param shape    - shape for collision detection
 * @param density  - density used to calculate mass properties
 * @param position - initial position of the body
 *
 * @return pointer to the newly created dynamic body
 */
PxBody *pxWorldNewDynamicBody(PxWorld *world, PxShape shape, float density,
                              PxVec2 position);

/**
 * @brief Removes a body from the physics world
 *
 * This function removes the specified body from either the static or dynamic
 * body array in the physics world. The body must have been previously added
 * to the world using pxWorldNewStaticBody or pxWorldNewDynamicBody.
 *
 * If the body is not found in either array, this function is a no-op.
 *
 * @param world
 * @param body
 */
void pxWorldFreeBody(PxWorld *world, PxBody *body);

/**
 * @brief Advances the physics simulation by one time step
 *
 * Performs collision detection and resolution, integrates forces, and updates
 * positions of all dynamic bodies.
 *
 * @param world
 *
 * @return true if step was performed, false if no step was needed based on time
 *         since last call
 */
bool pxWorldStep(PxWorld *world);

/**
 * @brief Renders debug visualization of the physics world
 *
 * This function draws debug information including:
 * - AABBs (bounding boxes) for all bodies
 * - Indicators for sleeping bodies
 * - Velocity vectors for moving bodies
 * - Statistics about contacts and sleeping bodies
 *
 * Useful for debugging physics behavior and verifying collision detection.
 *
 * @param world
 */
void pxWorldDrawDebug(PxWorld *world);

#endif // PDPHYZX_PX_WORLD_H
