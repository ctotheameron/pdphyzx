#include <float.h>
#include <stdint.h>

#include "px_manifold.h"

#include "px_polygon.h"

/**
 * Detects and handles the collision between two circles.
 *
 * @param circleA  - first circle involved in the collision.
 * @param circleB  - second circle involved in the collision.
 * @param manifold - manifold to store collision information.
 */
static void pxCollideCircles(PxBody *circleA, PxBody *circleB,
                             PxManifold *manifold) {
  if (circleA == NULL || circleB == NULL || manifold == NULL) {
    return;
  }

  PxVec2 positionA = circleA->position;
  PxVec2 positionB = circleB->position;
  PxVec2 normal = pxVec2Sub(positionB, positionA);

  float radiusA = circleA->collider.shape.circle.radius;
  float radiusB = circleB->collider.shape.circle.radius;
  float combinedRadius = radiusA + radiusB;

  float distanceSquared = pxVec2LenSqr(normal);

  if (distanceSquared >= combinedRadius * combinedRadius) {
    manifold->contacts.length = 0;
    return;
  }

  float distance = pxFastSqrt(distanceSquared);

  if (distance < PX_EPSILON) {
    manifold->penetration = radiusA;
    manifold->normal = pxVec2(1, 0);
    manifold->contacts.length = 1;
    manifold->contacts.items[0] = positionA;
    return;
  }

  manifold->penetration = combinedRadius - distance;
  manifold->normal = pxVec2Divf(normal, distance);

  PxVec2 contact = pxVec2Add(positionA, pxVec2Multf(manifold->normal, radiusA));
  manifold->contacts.length = 1;
  manifold->contacts.items[0] = contact;
}

/**
 * Detects and handles the collision between a circle and a polygon.
 *
 * @param circle   - circle involved in the collision.
 * @param polygon  - polygon involved in the collision.
 * @param manifold - manifold to store collision information.
 */
static void pxCollideCirclePolygon(PxBody *cBody, PxBody *pBody,
                                   PxManifold *manifold) {

  if (cBody == NULL || pBody == NULL || manifold == NULL) {
    return;
  }

  manifold->contacts.length = 0;

  PxPolygonData polygon = pBody->collider.shape.polygon;
  PxCircleData circle = cBody->collider.shape.circle;

  // Transform circle center to polygon model space
  PxMat2 orientation = pBody->orientation;

  PxVec2 center = pxMat2MultVec2(pxMat2Transpose(orientation),
                                 pxVec2Sub(cBody->position, pBody->position));

  // Find edge with minimum penetration
  // Exact concept as using support points in polygon vs polygon
  float separation = -FLT_MAX;
  uint8_t faceNormalIdx = 0;

  for (uint8_t idx = 0; idx < polygon.vertices.length; idx++) {
    PxVec2 normal = pxVec2ArrayGet(polygon.normals, idx);
    PxVec2 vertex = pxVec2ArrayGet(polygon.vertices, idx);

    // Calculate separation between circle center and current vertex
    float s = pxVec2Dot(normal, pxVec2Sub(center, vertex));

    if (s > circle.radius) {
      return; // No collision
    }

    if (s > separation) {
      separation = s;
      faceNormalIdx = idx;
    }
  }

  // Check to see if center is within polygon
  if (separation < PX_EPSILON) {
    PxVec2 normal = pxVec2Neg(pxMat2MultVec2(
        orientation, pxVec2ArrayGet(polygon.normals, faceNormalIdx)));

    manifold->normal = normal;
    manifold->penetration = circle.radius;

    manifold->contacts.length = 1;
    manifold->contacts.items[0] =
        pxVec2Add(cBody->position, pxVec2Multf(normal, circle.radius));

    return;
  }

  // Grab face's vertices
  PxVec2 v1 = pxVec2ArrayGet(polygon.vertices, faceNormalIdx);
  PxVec2 v2 = pxVec2ArrayGet(
      polygon.vertices,
      faceNormalIdx + 1 < polygon.vertices.length ? faceNormalIdx + 1 : 0);

  float radiusSquared = circle.radius * circle.radius;
  manifold->penetration = circle.radius - separation;

  // Determine which voronoi region of the edge center of circle lies within
  float dot1 = pxVec2Dot(pxVec2Sub(center, v1), pxVec2Sub(v2, v1));

  // Closest to v1
  if (dot1 <= 0) {
    if (pxVec2DistSqr(center, v1) > radiusSquared) {
      return;
    }

    PxVec2 normal = pxMat2MultVec2(orientation, pxVec2Sub(v1, center));
    pxVec2Normalize(&normal);
    manifold->normal = normal;

    manifold->contacts.length = 1;
    manifold->contacts.items[0] =
        pxVec2Add(pxMat2MultVec2(orientation, v1), pBody->position);

    return;
  }

  float dot2 = pxVec2Dot(pxVec2Sub(center, v2), pxVec2Sub(v1, v2));

  // Closest to v2
  if (dot2 <= 0) {

    if (pxVec2DistSqr(center, v2) > radiusSquared) {
      return;
    }

    PxVec2 normal = pxMat2MultVec2(orientation, pxVec2Sub(v2, center));
    pxVec2Normalize(&normal);
    manifold->normal = normal;

    manifold->contacts.length = 1;
    manifold->contacts.items[0] =
        pxVec2Add(pxMat2MultVec2(orientation, v2), pBody->position);

    return;
  }

  // Closest to face
  PxVec2 normal = pxVec2ArrayGet(polygon.normals, faceNormalIdx);

  if (pxVec2Dot(pxVec2Sub(center, v1), normal) > circle.radius) {
    return;
  }

  normal = pxVec2Neg(pxMat2MultVec2(orientation, normal));
  manifold->normal = normal;

  manifold->contacts.length = 1;
  manifold->contacts.items[0] =
      pxVec2Add(cBody->position, pxVec2Multf(normal, circle.radius));
}

static float pxFindAxisLeastPenetration(uint8_t *faceIndex, PxBody *polyA,
                                        PxBody *polyB) {
  PxPolygonData polyDataA = polyA->collider.shape.polygon;
  PxPolygonData polyDataB = polyB->collider.shape.polygon;

  float bestDistance = -FLT_MAX;
  uint8_t bestIndex = 0;

  for (uint8_t i = 0; i < polyDataA.vertices.length; i++) {
    // Get face normal from A in world space
    PxVec2 n = pxVec2ArrayGet(polyDataA.normals, i);
    PxVec2 nw = pxMat2MultVec2(polyA->orientation, n);

    // Transform normal into B's model space
    PxMat2 buT = pxMat2Transpose(polyB->orientation);
    n = pxMat2MultVec2(buT, nw);

    // Get support point from B along -n
    PxVec2 s = pxPolygonGetSupport(polyDataB, pxVec2Neg(n));

    // Get vertex on face from A in B's model space
    PxVec2 v = pxVec2ArrayGet(polyDataA.vertices, i);
    v = pxVec2Add(pxMat2MultVec2(polyA->orientation, v), polyA->position);
    v = pxVec2Sub(v, polyB->position);
    v = pxMat2MultVec2(buT, v);

    // Compute penetration distance in B's model space
    float d = pxVec2Dot(n, pxVec2Sub(s, v));

    // Store greatest distance (least penetration)
    if (d > bestDistance) {
      bestDistance = d;
      bestIndex = i;
    }
  }

  *faceIndex = bestIndex;
  return bestDistance;
}

static void pxFindIncidentFace(PxVec2 v[2], PxBody *refPoly, PxBody *incPoly,
                               uint8_t referenceIndex) {
  PxPolygonData refPolyData = refPoly->collider.shape.polygon;
  PxPolygonData incPolyData = incPoly->collider.shape.polygon;

  // Get reference face normal in world space
  PxVec2 referenceNormal = pxVec2ArrayGet(refPolyData.normals, referenceIndex);
  referenceNormal = pxMat2MultVec2(refPoly->orientation, referenceNormal);

  // Transform to incident's model space
  PxVec2 incRefNormal =
      pxMat2MultVec2(pxMat2Transpose(incPoly->orientation), referenceNormal);

  // Find most anti-normal face on incident polygon
  int incidentFace = 0;
  float minDot = FLT_MAX;

  for (uint8_t i = 0; i < incPolyData.vertices.length; i++) {
    PxVec2 normal = pxVec2ArrayGet(incPolyData.normals, i);
    float dot = pxVec2Dot(incRefNormal, normal);

    if (dot < minDot) {
      minDot = dot;
      incidentFace = i;
    }
  }

  // Get incident face vertices in world space
  v[0] = pxVec2Add(
      incPoly->position,
      pxMat2MultVec2(incPoly->orientation,
                     pxVec2ArrayGet(incPolyData.vertices, incidentFace)));

  uint8_t nextVertex = (incidentFace + 1) % incPolyData.vertices.length;

  v[1] = pxVec2Add(
      incPoly->position,
      pxMat2MultVec2(incPoly->orientation,
                     pxVec2ArrayGet(incPolyData.vertices, nextVertex)));
}

static int pxClipSegmentToLine(PxVec2 out[2], const PxVec2 in[2], PxVec2 normal,
                               float offset) {
  // Output point count
  int sp = 0;

  // Calculate distances from each endpoint to the clip line
  float d1 = pxVec2Dot(normal, in[0]) - offset;
  float d2 = pxVec2Dot(normal, in[1]) - offset;

  // Points on opposite sides - calculate intersection point
  if (d1 * d2 < 0.0f) {
    // Calculate interpolation coefficient
    float alpha = d1 / (d1 - d2);

    // Calculate intersection point
    out[sp] = pxVec2Add(in[0], pxVec2Multf(pxVec2Sub(in[1], in[0]), alpha));
    sp++;
  }

  // Keep points behind the clip line (d <= 0)
  if (d2 <= 0.0f) {
    out[sp++] = in[1];
  }

  if (d1 <= 0.0f) {
    out[sp++] = in[0];
  }

  return sp;
}

/**
 * Detects and handles the collision between two polygons.
 *
 * @param polygonA - first polygon involved in the collision.
 * @param polygonB - second polygon involved in the collision.
 * @param manifold - manifold to store collision information.
 */
void pxCollidePolygons(PxBody *bodyA, PxBody *bodyB, PxManifold *manifold) {

  manifold->contacts.length = 0;

  // Check for a separating axis with A's face planes
  uint8_t faceA;
  float penetrationA = pxFindAxisLeastPenetration(&faceA, bodyA, bodyB);

  if (penetrationA >= 0.0f) {
    return; // Separation - no collision
  }

  // Check for a separating axis with B's face planes
  uint8_t faceB;
  float penetrationB = pxFindAxisLeastPenetration(&faceB, bodyB, bodyA);

  if (penetrationB >= 0.0f) {
    return; // Separation - no collision
  }

  // We have a collision. Determine which shape features to use.
  uint8_t referenceIndex;
  bool flip; // Always point from a to b

  PxBody *referenceBody; // Reference
  PxBody *incidentBody;  // Incident

  // Determine which shape contains reference face (deeper penetration)
  if (pxBiasGt(penetrationA, penetrationB)) {
    referenceBody = bodyA;
    incidentBody = bodyB;
    referenceIndex = faceA;
    flip = false;
  } else {
    referenceBody = bodyB;
    incidentBody = bodyA;
    referenceIndex = faceB;
    flip = true;
  }

  // World space incident face
  PxVec2 incidentFace[2];
  pxFindIncidentFace(incidentFace, referenceBody, incidentBody, referenceIndex);

  // Setup reference face vertices
  PxPolygonData refData = referenceBody->collider.shape.polygon;
  PxVec2 refFace[2];

  refFace[0] = pxVec2Add(
      referenceBody->position,
      pxMat2MultVec2(referenceBody->orientation,
                     pxVec2ArrayGet(refData.vertices, referenceIndex)));

  referenceIndex = (referenceIndex + 1) % refData.vertices.length;

  refFace[1] = pxVec2Add(
      referenceBody->position,
      pxMat2MultVec2(referenceBody->orientation,
                     pxVec2ArrayGet(refData.vertices, referenceIndex)));

  // Calculate reference face side normal in world space
  PxVec2 sidePlaneNormal = pxVec2Sub(refFace[1], refFace[0]);
  pxVec2Normalize(&sidePlaneNormal);

  // Orthogonalize
  PxVec2 refFaceNormal = pxVec2(-sidePlaneNormal.y, sidePlaneNormal.x);

  // Make sure reference face normal points outward
  if (flip) {
    pxVec2NegAssign(&refFaceNormal);
  }

  // ax + by = c (plane equation)
  // c is distance from origin
  float refC = pxVec2Dot(refFaceNormal, refFace[0]);

  // Compute negative side and positive side plane offsets
  float negSide = -pxVec2Dot(sidePlaneNormal, refFace[0]);
  float posSide = pxVec2Dot(sidePlaneNormal, refFace[1]);

  // Clip incident face to reference face side planes
  PxVec2 clipPoints1[2];
  PxVec2 clipPoints2[2];
  int cp; // Clipped points array

  // Clip against the first side plane
  cp = pxClipSegmentToLine(clipPoints1, incidentFace,
                           pxVec2Neg(sidePlaneNormal), negSide);

  if (cp < 2) {
    return; // Due to numerical issues, we don't have enough points
  }

  // Clip against the second side plane
  cp = pxClipSegmentToLine(clipPoints2, clipPoints1, sidePlaneNormal, posSide);

  if (cp < 2) {
    return; // Due to numerical issues, we don't have enough points
  }

  // Now clipPoints2 contains the clipped incident face points in world space

  // Set the collision normal
  manifold->normal = flip ? pxVec2Neg(refFaceNormal) : refFaceNormal;

  // Keep points behind reference face
  int numContacts = 0;

  // Check if points are behind reference face
  for (int i = 0; i < cp; i++) {
    float separation = pxVec2Dot(refFaceNormal, clipPoints2[i]) - refC;

    if (separation <= 0.0f) {
      manifold->contacts.items[numContacts] = clipPoints2[i];

      if (numContacts == 0) {
        manifold->penetration = -separation;
      } else {
        manifold->penetration += -separation;
      }

      numContacts++;

      // Safety check, shouldn't happen with only 2 inputs
      if (numContacts >= PX_MAX_CONTACTS) {
        break;
      }
    }
  }

  // Calculate average penetration if multiple points
  if (numContacts > 1) {
    manifold->penetration /= numContacts;
  }

  manifold->contacts.length = numContacts;
}

void pxCollide(PxBody *bodyA, PxBody *bodyB, PxManifold *manifold) {
  if (bodyA == NULL || bodyB == NULL || manifold == NULL) {
    return;
  }

  PxColliderType typeA = bodyA->collider.type;
  PxColliderType typeB = bodyB->collider.type;

  if (typeA == PX_CIRCLE && typeB == PX_POLYGON) {
    pxCollideCirclePolygon(bodyA, bodyB, manifold);
    return;
  }

  if (typeA == PX_CIRCLE && typeB == PX_CIRCLE) {
    pxCollideCircles(bodyA, bodyB, manifold);
    return;
  }

  if (typeA == PX_POLYGON && typeB == PX_CIRCLE) {
    pxCollideCirclePolygon(bodyB, bodyA, manifold);
    pxVec2NegAssign(&manifold->normal);
    return;
  }

  if (typeA == PX_POLYGON && typeB == PX_POLYGON) {
    pxCollidePolygons(bodyA, bodyB, manifold);
    return;
  }
}
