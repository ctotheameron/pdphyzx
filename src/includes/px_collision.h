/**
 * @file px_collision.h
 * @brief This file contains functions for handling collision detection and
 * response.
 *
 * The functions provided in this file are used to detect and handle collisions
 * between different shapes (circles and polygons) and to clip line segments
 * against lines defined by normal vectors and offsets. These functions are
 * essential for collision detection in physics simulations.
 */

#ifndef PDPHYZX_PX_COLLISION_H
#define PDPHYZX_PX_COLLISION_H

#include "px_body.h"
#include "px_manifold.h"

/**
 * Detects and handles the collision between two bodies.
 *
 * @param bodyA    - first body involved in the collision.
 * @param bodyB    - second body involved in the collision.
 * @param manifold - manifold to store collision information.
 */
void pxCollide(PxBody *bodyA, PxBody *bodyB, PxManifold *manifold);

#endif // PDPHYZX_PX_COLLISION_H
