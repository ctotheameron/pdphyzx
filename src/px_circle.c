#include "px_circle.h"

/**
 * Computes mass properties for a circular collider.
 * This is primarily for internal use by the physics engine.
 *
 * @param collider - pointer to the circular collider
 * @param density  - mass density of the material (mass per unit area)
 *
 * @return Mass data including mass, inverse mass, moment of inertia and inverse
 *         moment
 */
PxMassData pxCircleComputeMass(PxCollider *collider, float density) {
  if (collider == NULL) {
    return (PxMassData){0};
  }

  float radius = collider->shape.circle.radius;
  float radiusSquared = radius * radius;

  // Calculate mass = π * r² * density
  float mass = PX_PI * radiusSquared * density;

  // Calculate moment of inertia = mass * r² = π * r⁴ * density
  float momentOfInertia = mass * radiusSquared;

  return (PxMassData){
      .mass = mass,
      .iMass = pxFastSafeRcp(mass), // Inverse mass (1/mass)
      .momentOfInertia = momentOfInertia,
      .iMomentOfInertia =
          pxFastSafeRcp(momentOfInertia), // Inverse moment of inertia
  };
}

/**
 * Updates AABB for a circle collider
 *
 * @param collider - circle collider
 * @param position - current position of the body
 * @param outAABB  - AABB to update
 */
void pxCircleUpdateAABB(PxCollider *collider, PxAABB *outAABB,
                        PxVec2 position) {
  float radius = collider->shape.circle.radius;
  outAABB->min = pxVec2Subf(position, radius);
  outAABB->max = pxVec2Addf(position, radius);
}

PxCollider pxCircleColliderNew(float radius) {
  PxCollider collider =
      pxColliderNew(PX_CIRCLE, (PxShapeData){.circle = {.radius = radius}});

  collider.computeMass = &pxCircleComputeMass;
  collider.updateAABB = &pxCircleUpdateAABB;

  return collider;
}
