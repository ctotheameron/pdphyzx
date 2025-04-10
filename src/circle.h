/**
 * @file circle.h
 *
 * @brief Circle collider implementation for the pdphyzx engine
 *
 * This header provides functions for creating and working with circular
 * colliders in 2D physics simulations.
 */

#ifndef PDPHYZX_CIRCLE_H
#define PDPHYZX_CIRCLE_H

#include "collider.h" /* For PxCollider type */

/**
 * Creates a new circular collider with the specified radius.
 *
 * @param radius - radius of the circle (must be positive)
 *
 * @return A new PxCollider instance configured as a circle
 *
 * @note Returns an invalid collider if radius <= 0
 */
extern PxCollider pxCircleColliderNew(float radius);

extern PxMassData pxCircleComputeMass(PxCollider *collider, float density);

#endif // PDPHYZX_CIRCLE_H
