/**
 * @file px_polygon.h
 *
 * @brief Defines functions and structures for polygon-based colliders in the
 *        pdphyzx engine.
 *
 * This header provides the interface for creating and manipulating polygon
 * colliders, including functions for creating new polygon and box colliders,
 * processing polygon data, and computing mass properties for polygon colliders.
 *
 * The functions in this file are designed to work with the pdphyzx engine's
 * collider system, allowing for the creation of complex polygon shapes and the
 * calculation of their physical properties.
 */

#ifndef PDPHYZX_PX_POLYGON_H
#define PDPHYZX_PX_POLYGON_H

#include <stddef.h>
#include <stdint.h>

#include "px_collider.h"
#include "px_vec2.h"

/**
 * Creates a new polygon collider from the given vertices.
 *
 * @param vertices - array of vertex positions defining the polygon outline
 * @param count    - number of vertices in the array
 *
 * @return A new PxCollider initialized as a polygon shape
 */
PxCollider pxPolygonColliderNew(PxVec2 vertices[], uint8_t count);

/**
 * Creates a new box collider centered at the origin.
 *
 * @param width  - width of the box along x-axis
 * @param height - height of the box along y-axis
 *
 * @return A new PxCollider initialized as a rectangular shape
 */
PxCollider pxBoxColliderNew(float width, float height);

/**
 * @brief Finds the support point of a polygon in a given direction
 *
 * The support point is the vertex with the maximum projection along the
 * specified direction. This function is commonly used in collision detection
 * algorithms such as SAT (Separating Axis Theorem) and GJK.
 *
 * @param polygon   - polygon data containing vertices to search
 * @param direction - direction vector to project vertices onto
 *
 * @return The vertex with the maximum projection in the given direction.
 *         If direction is (near) zero, returns a zero vector.
 *
 * @note Time complexity: O(n) where n is the number of vertices in the polygon
 */
PxVec2 pxPolygonGetSupport(const PxPolygonData polygon, const PxVec2 direction);

#endif // PDPHYZX_PX_POLYGON_H
