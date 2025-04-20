/**
 * @file types/px_collider.h
 *
 * @brief Defines the interface for collider objects in the pdphyzx engine.
 *
 * This header file provides the necessary functions and structures to create
 * and manage different types of colliders, including their shapes and
 * properties.
 *
 * It includes definitions for creating new colliders, handling shape data, and
 * specifying collider types.
 *
 * The functions and structures in this file are essential for the collision
 * detection and response system within the pdphyzx engine.
 */

#ifndef PDPHYZX_PX_COLLIDER_H
#define PDPHYZX_PX_COLLIDER_H

#include <stdint.h>

#include "px_vec2.h"

/**
 * @brief Defines the type of physical collider
 */
typedef enum {
  PX_CIRCLE, /**< Circular collider */
  PX_POLYGON /**< Polygon-shaped collider */
} PxColliderType;

/**
 * @brief Data structure for circular colliders
 */
typedef struct PxCircleData {
  float radius; /**< Radius of the circle */
} PxCircleData;

/**
 * @brief Maximum number of vertices allowed in a polygon collider
 * @note Derived from VEC2_ARRAY_MAX_SIZE which defines maximum vector array
 *       size
 */
#define PX_MAX_POLY_VERTEX_COUNT PX_VEC2_ARRAY_MAX_SIZE

typedef struct PxPolygonData {
  PxVec2Array vertices;
  PxVec2Array normals;
  float maxRadius; /**< Maximum distance from center to any vertex */
} PxPolygonData;

/**
 * @brief Union containing data for different collider types
 */
typedef union {
  PxCircleData circle;   /**< Data for circular colliders */
  PxPolygonData polygon; /**< Data for polygon colliders */
} PxShapeData;

// Forward declaration
typedef struct PxCollider PxCollider;

typedef void (*PxColliderDestroyImpl)(PxCollider *c);

/**
 * @brief Physical mass properties for a collider
 */
typedef struct {
  float mass;  /**< Mass of the collider */
  float iMass; /**< Inverse mass (1/mass) for efficient calculations */
  float momentOfInertia;  /**< Moment of inertia around the center of mass */
  float iMomentOfInertia; /**< Inverse moment of inertia for calculations */
} PxMassData;

typedef PxMassData (*PxColliderComputeMassImpl)(PxCollider *collider,
                                                float density);
typedef struct {
  PxVec2 min;
  PxVec2 max;
} PxAABB;

typedef void (*PxColliderUpdateAABB)(PxCollider *collider, PxAABB *outAABB,
                                     PxVec2 position);

typedef struct PxCollider {
  PxShapeData shape;
  PxColliderType type;
  PxColliderComputeMassImpl computeMass;
  PxColliderUpdateAABB updateAABB;
  PxColliderDestroyImpl destroy;
} PxCollider;

/**
 * Creates a new collider with the specified type and shape data
 *
 * @param type  - type of collider (circle, box, etc.) from PxColliderType enum
 * @param shape - union containing shape-specific data based on the collider
 *                type
 *
 * @return A new PxCollider instance with the specified properties
 *
 * @note This is a low-level function. Consider using shape-specific creation
 *       functions like pxCreateCircleCollider instead of calling this
 *       directly.
 *
 * @see pxCreateCircleCollider, pxCreateBoxCollider
 */
PxCollider pxColliderNew(PxColliderType type, PxShapeData shape);

#endif // PDPHYZX_PX_COLLIDER_H
