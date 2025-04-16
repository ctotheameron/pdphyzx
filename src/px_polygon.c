#include <assert.h>
#include <float.h>

#include "px_polygon.h"

/**
 * Creates polygon data from a set of vertices.
 *
 * @param vertices - array of vertex positions defining the polygon outline
 * @param count    - number of vertices in the array
 *
 * @return A PxPolygonData structure containing the processed polygon
 *         information
 */
PxPolygonData pxPolygonData(uint8_t count, PxVec2 *vertices) {
  // No hulls with less than 3 vertices (ensure actual polygon)
  assert(count > 2 && count <= PX_MAX_POLY_VERTEX_COUNT);

  PxPolygonData polygon = {0};

  // Find the right-most point on the hull (point with maximum x coordinate)
  // If tied, choose the one with the lowest y coordinate
  uint8_t rightMost = 0;
  float rightmostX = vertices[0].x;

  for (uint8_t i = 1; i < count; i++) {
    float x = vertices[i].x;

    if (x > rightmostX) {
      rightmostX = x;
      rightMost = i;
      continue;
    }

    // If x coordinates are equal, choose the lowest y-coordinate point
    // as this ensures a consistent rightmost point for the algorithm
    if (x == rightmostX && vertices[i].y < vertices[rightMost].y) {
      rightMost = i;
    }
  }

  // Implement convex hull algorithm (Graham scan variant)
  // We start from the rightmost point and find the convex hull
  // by selecting the most counter-clockwise points in sequence
  uint8_t hull[PX_MAX_POLY_VERTEX_COUNT];
  uint8_t outCount = 0;
  uint8_t indexHull = rightMost;

  while (true) {
    hull[outCount] = indexHull;

    // Search for the next index that wraps around the hull by computing cross
    // products to find the most counter-clockwise vertex in the set, given the
    // previous hull index
    uint8_t nextHullIndex = 0;

    for (uint8_t i = 1; i < count; i++) {
      // Skip if same coordinate as we need three unique points in the set to
      // perform a cross product
      if (nextHullIndex == indexHull) {
        nextHullIndex = i;
        continue;
      }

      // Cross every set of three unique vertices, recording each
      // counter-clockwise third vertex and add to the output hull

      PxVec2 e1 = pxVec2Sub(vertices[nextHullIndex], vertices[hull[outCount]]);
      PxVec2 e2 = pxVec2Sub(vertices[i], vertices[hull[outCount]]);

      float c = pxVec2Cross(e1, e2);

      if (c < 0) {
        nextHullIndex = i;
        continue;
      }

      // In case of collinear points (c == 0), pick point that's further away
      if (c == 0 && pxVec2LenSqr(e2) > pxVec2LenSqr(e1)) {
        nextHullIndex = i;
        continue;
      }
    }

    outCount++;
    indexHull = nextHullIndex;

    // Finish if we've gone full circle
    if (nextHullIndex == rightMost) {
      polygon.vertices.length = outCount;
      polygon.normals.length = outCount;
      break;
    }
  }

  // Copy vertices into shape's vertices and calculate maxRadius
  polygon.maxRadius = 0;
  for (uint8_t i = 0; i < polygon.vertices.length; i++) {
    PxVec2 vertex = vertices[hull[i]];
    pxVec2ArraySet(&polygon.vertices, i, vertex);

    float distSqr = pxVec2LenSqr(vertex);
    if (distSqr > polygon.maxRadius * polygon.maxRadius) {
      polygon.maxRadius = pxFastSqrt(distSqr);
    }
  }

  // Compute face normals
  for (uint8_t curIdx = 0; curIdx < polygon.normals.length; curIdx++) {
    uint8_t nextIdx = curIdx + 1 < polygon.normals.length ? curIdx + 1 : 0;

    PxVec2 face = pxVec2Sub(pxVec2ArrayGet(polygon.vertices, nextIdx),
                            pxVec2ArrayGet(polygon.vertices, curIdx));

    // Ensure no zero-length edges
    assert(pxVec2LenSqr(face) > PX_EPSILON * PX_EPSILON);

    // Calculate normal with 2D cross product between vector and scalar
    // Normal points outward from the polygon face
    PxVec2 normal = pxVec2(face.y, -face.x);
    pxVec2Normalize(&normal);
    pxVec2ArraySet(&polygon.normals, curIdx, normal);
  }

  return polygon;
}

/**
 * Computes mass properties for a polygon collider.
 *
 * @param collider - collider to compute mass properties for (must be a polygon
 *                   type)
 *
 * @param density  - material density in kg/m^2
 *
 * @return MassData structure containing mass, center of mass, and moment of
 *         inertia
 */
PxMassData pxPolygonComputeMass(PxCollider *collider, float density) {
  if (collider == NULL) {
    return (PxMassData){0};
  }

  // Calculate centroid and moment of interia
  PxVec2 centroid = pxVec2(0, 0);
  float area = 0;
  float momentOfInertia = 0;

  PxVec2Array vertices = collider->shape.polygon.vertices;

  for (uint8_t curIdx = 0; curIdx < vertices.length; curIdx++) {
    uint8_t nextIdx = curIdx + 1 < vertices.length ? curIdx + 1 : 0;

    // Triangle vertices, third vertex implied as (0, 0)
    PxVec2 curVertex = pxVec2ArrayGet(vertices, curIdx);
    PxVec2 nextVertex = pxVec2ArrayGet(vertices, nextIdx);

    float d = pxVec2Cross(curVertex, nextVertex);

    // Area of triangle formed with origin
    float triangleArea = 0.5f * d;
    area += triangleArea;

    // Use area to weight the centroid average, not just vertex position
    // Formula: (v1+v2)/3 * triangleArea (since the third vertex is at origin)
    pxVec2AddAssign(&centroid, pxVec2Multf(pxVec2Add(curVertex, nextVertex),
                                           triangleArea * PX_ONE_THIRD));

    // Calculate moment of inertia for the triangle using the standard physics
    // formula: ∫∫(x²+y²) dA = (1/12) * (x1²+x1*x2+x2²+y1²+y1*y2+y2²) * area
    float intx2 = curVertex.x * curVertex.x + nextVertex.x * curVertex.x +
                  nextVertex.x * nextVertex.x;

    float inty2 = curVertex.y * curVertex.y + nextVertex.y * curVertex.y +
                  nextVertex.y * nextVertex.y;

    momentOfInertia += (0.25f * PX_ONE_THIRD * d) * (intx2 + inty2);
  }

  // Normalize the centroid by the total area
  pxVec2MultfAssign(&centroid, pxFastRcp(area));

  // Translate vertices to centroid (make the centroid (0, 0)
  // This improves numerical stability and helps collision detection
  // by having the body rotate around its center of mass
  for (uint8_t i = 0; i < vertices.length; ++i) {
    pxVec2SubAssign(&vertices.items[i], centroid);
  }

  float mass = density * area;
  momentOfInertia = mass * momentOfInertia;

  return (PxMassData){
      .mass = mass,
      .iMass = pxFastSafeRcp(mass),
      .momentOfInertia = momentOfInertia,
      .iMomentOfInertia = pxFastSafeRcp(momentOfInertia),
  };
}

/**
 * Updates radius-based AABB for a polygon collider
 *
 * @param collider - polygon collider
 * @param position - current position of the body
 * @param outAABB  - AABB to update
 */
void pxPolygonUpdateAABB(PxCollider *collider, PxAABB *outAABB,
                         PxVec2 position) {
  float radius = collider->shape.polygon.maxRadius;
  outAABB->min = pxVec2Subf(position, radius);
  outAABB->max = pxVec2Addf(position, radius);
}

PxCollider pxPolygonColliderNew(PxVec2 vertices[], uint8_t count) {
  PxCollider collider = pxColliderNew(
      PX_POLYGON, (PxShapeData){.polygon = pxPolygonData(count, vertices)});

  collider.computeMass = &pxPolygonComputeMass;
  collider.updateAABB = &pxPolygonUpdateAABB;

  return collider;
}

PxCollider pxBoxColliderNew(float width, float height) {
  if (width <= 0.0f || height <= 0.0f) {
    return (PxCollider){0}; // Return invalid collider
  }

  const float halfWidth = width * 0.5f;
  const float halfHeight = height * 0.5f;

  PxVec2Array vertices = {
      .length = 4,
      .items =
          {
              pxVec2(-halfWidth, -halfHeight),
              pxVec2(halfWidth, -halfHeight),
              pxVec2(halfWidth, halfHeight),
              pxVec2(-halfWidth, halfHeight),
          },
  };

  PxVec2Array normals = {
      .length = 4,
      .items =
          {
              pxVec2(0, -1),
              pxVec2(1, 0),
              pxVec2(0, 1),
              pxVec2(-1, 0),
          },
  };

  // The maximum radius is the distance from center to a corner
  float maxRadius = pxFastSqrt(halfWidth * halfWidth + halfHeight * halfHeight);

  PxPolygonData polygon = {
      .vertices = vertices,
      .normals = normals,
      .maxRadius = maxRadius,
  };

  PxCollider collider =
      pxColliderNew(PX_POLYGON, (PxShapeData){.polygon = polygon});

  collider.computeMass = &pxPolygonComputeMass;
  collider.updateAABB = &pxPolygonUpdateAABB;

  return collider;
}

PxVec2 pxPolygonGetSupport(const PxPolygonData polygon,
                           const PxVec2 direction) {
  // Ensure direction is not zero
  if (pxVec2LenSqr(direction) < PX_EPSILON * PX_EPSILON) {
    return pxVec2(0, 0);
  }

  float bestProjection = -FLT_MAX;
  PxVec2 bestVertex = pxVec2ArrayGet(polygon.vertices, 0); // Safe default

  for (uint8_t i = 0; i < polygon.vertices.length; i++) {
    PxVec2 vertex = pxVec2ArrayGet(polygon.vertices, i);
    float projection = pxVec2Dot(vertex, direction);

    if (projection > bestProjection) {
      bestVertex = vertex;
      bestProjection = projection;
    }
  }

  return bestVertex;
}
