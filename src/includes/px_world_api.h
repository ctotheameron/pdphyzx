/**
 * @file px_world_api.h
 *
 * @brief Function pointer API for the PDPhyzx world physics system
 *
 * This header defines the function pointer interface that provides access to
 * the world management functionality of the PDPhyzx 2D physics engine.
 *
 * The API allows for creating and managing physics worlds, adding and removing
 * bodies, and advancing the physics simulation.
 */

#ifndef PDPHYZX_PX_WORLD_API_H
#define PDPHYZX_PX_WORLD_API_H

#include "px_world.h"

/**
 * @brief API structure containing function pointers for world operations
 *
 * This structure provides a consistent interface to the physics world
 * functionality, allowing applications to create, manipulate, and simulate
 * physics worlds.
 */
typedef struct {
  PxWorld *(*new)(uint8_t targetFps);
  void (*free)(PxWorld *world);
  void (*setIterations)(PxWorld *world, uint8_t iterations);
  void (*setGravity)(PxWorld *world, float x, float y);
  PxBody *(*newStaticBody)(PxWorld *world, PxShape shape, PxVec2 position);
  PxBody *(*newDynamicBody)(PxWorld *world, PxShape shape, float density,
                            PxVec2 position);
  void (*freeBody)(PxWorld *world, PxBody *body);
  bool (*step)(PxWorld *world);
  void (*drawDebug)(PxWorld *world);
} PxWorldAPI;

PxWorldAPI *newPxWorldAPI(void);

#endif // PDPHYZX_PX_WORLD_API_H
