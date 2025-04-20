#include <stdint.h>

#include "px_body.h"
#include "px_collision.h"

#include "px_manifold.h"

//==============================================================================
// Allocation
//==============================================================================

PxManifold *pxManifoldInit(PxManifold *manifold, PxBody *bodyA, PxBody *bodyB) {
  // Allocate space for up to 2 contact points (typical for 2D collision)
  manifold->contacts = (PxVec2Array2){0};
  manifold->normal = pxVec2(0, 0);
  manifold->penetration = 0;
  manifold->mixedRestitution = 0;

  manifold->bodyA = bodyA;
  manifold->bodyB = bodyB;

  // Use geometric mean to combine friction coefficients
  // This produces a balanced value between the two materials
  manifold->staticFriction =
      pxFastSqrt(bodyA->staticFriction * bodyB->staticFriction);

  manifold->dynamicFriction =
      pxFastSqrt(bodyA->dynamicFriction * bodyB->dynamicFriction);

  return manifold;
}

//==============================================================================
// Solve
//==============================================================================

void pxManifoldSolve(PxManifold *manifold) {
  // Detect collision between two bodies and populate the manifold with contacts
  pxCollide(manifold->bodyA, manifold->bodyB, manifold);
}

//==============================================================================
// Detect resting contacts
//==============================================================================

void pxManifoldDetectRestingContact(PxManifold *manifold, PxVec2 g, float dt) {
  if (manifold->contacts.length == 0) {
    return;
  }

  // Use minimum restitution for better stability in stacked objects
  float minRestitution =
      pxMin(manifold->bodyA->restitution, manifold->bodyB->restitution);

  // Check for resting contacts
  //
  // If relative velocity is less than what gravity would cause in one timestep,
  // we consider it a resting contact
  PxVec2 gravityStep = pxVec2Multf(g, dt);
  float gravityStepSqr = pxVec2LenSqr(gravityStep);

  PxBody *bodyA = manifold->bodyA;
  PxBody *bodyB = manifold->bodyB;

  for (uint8_t i = 0; i < manifold->contacts.length; i++) {
    // Calculate radii from COM to contact
    PxVec2 contact = manifold->contacts.items[i];
    PxVec2 ra = pxVec2Sub(contact, bodyA->position);
    PxVec2 rb = pxVec2Sub(contact, bodyB->position);

    // Calculate relative velocity at contact
    PxVec2 raPerp = pxVec2(-ra.y, ra.x);
    PxVec2 rbPerp = pxVec2(-rb.y, rb.x);

    PxVec2 va =
        pxVec2Add(bodyA->velocity, pxVec2Multf(raPerp, bodyA->angularVelocity));

    PxVec2 vb =
        pxVec2Add(bodyB->velocity, pxVec2Multf(rbPerp, bodyB->angularVelocity));

    PxVec2 rv = pxVec2Sub(vb, va);

    // Determine if this is a resting contact
    if (pxVec2LenSqr(rv) < gravityStepSqr + PX_EPSILON) {
      minRestitution = 0.0f;
      break;
    }
  }

  // Store the restitution for later use in impulse application
  manifold->mixedRestitution = minRestitution;
}

//==============================================================================
// Apply Impulse
//==============================================================================

void pxManifoldApplyImpulse(PxManifold *manifold) {
  if (!manifold->contacts.length) {
    return;
  }

  float mixedRestitution = manifold->mixedRestitution;

  PxBody *bodyA = manifold->bodyA;
  PxBody *bodyB = manifold->bodyB;

  bool bodyADynamic = pxBodyIsDynamic(bodyA);
  bool bodyBDynamic = pxBodyIsDynamic(bodyB);

  // Skip if both bodies are non-dynamic
  if (!bodyADynamic && !bodyBDynamic) {
    return;
  }

  // Calculate radii from center of mass to contact
  for (uint8_t i = 0; i < manifold->contacts.length; i++) {
    PxVec2 contact = manifold->contacts.items[i];
    PxVec2 ra = pxVec2Sub(contact, bodyA->position);
    PxVec2 rb = pxVec2Sub(contact, bodyB->position);

    // Relative velocity at contact point
    PxVec2 raPerp = pxVec2(-ra.y, ra.x);
    PxVec2 rbPerp = pxVec2(-rb.y, rb.x);

    PxVec2 va =
        pxVec2Add(bodyA->velocity, pxVec2Multf(raPerp, bodyA->angularVelocity));

    PxVec2 vb =
        pxVec2Add(bodyB->velocity, pxVec2Multf(rbPerp, bodyB->angularVelocity));

    PxVec2 relativeVelocity = pxVec2Sub(vb, va);

    // Get velocity along the normal
    float velAlongNormal = pxVec2Dot(relativeVelocity, manifold->normal);

    // Skip if velocities are separating
    if (velAlongNormal > 0) {
      continue;
    }

    // Calculate cross products of radius vectors with normal
    // These determine how much the collision affects angular motion
    // Higher value = more torque applied from this contact point
    float raCrossN = pxVec2Cross(ra, manifold->normal);
    float rbCrossN = pxVec2Cross(rb, manifold->normal);

    float invMassSum = bodyA->iMass + bodyB->iMass +
                       raCrossN * raCrossN * bodyA->iMomentOfInertia +
                       rbCrossN * rbCrossN * bodyB->iMomentOfInertia;

    // Calculate impulse magnitude
    // Normal impulse formula: j = -(1+e)*vn / (invMassSum * contactCount)
    float normalImpulseMagnitude = -(1.0f + mixedRestitution) * velAlongNormal;
    float impulseDenom = invMassSum * manifold->contacts.length;
    float j = pxFastDiv(normalImpulseMagnitude, impulseDenom);

    // Apply normal impulse
    PxVec2 impulse = pxVec2Multf(manifold->normal, j);

    if (bodyADynamic) {
      pxBodyApplyImpulse(bodyA, pxVec2Neg(impulse), ra);
    }

    if (bodyBDynamic) {
      pxBodyApplyImpulse(bodyB, impulse, rb);
    }

    // Friction impulse
    // Recalculate relative velocity after normal impulse is applied
    va =
        pxVec2Add(bodyA->velocity, pxVec2Multf(raPerp, bodyA->angularVelocity));

    vb =
        pxVec2Add(bodyB->velocity, pxVec2Multf(rbPerp, bodyB->angularVelocity));

    relativeVelocity = pxVec2Sub(vb, va);

    // Calculate tangent vector (perpendicular to normal)
    PxVec2 tangent = pxVec2Tangent(relativeVelocity, manifold->normal);

    // Check if tangent vector is non-zero
    float tangentLen = pxVec2Len(tangent);
    if (tangentLen > PX_EPSILON) {
      tangent = pxVec2Multf(tangent, pxFastRcp(tangentLen)); // Normalize
    } else {
      continue; // Skip friction for this contact
    }

    // Calculate friction impulse scalar
    // (jt = -vt / (invMassSum * contactCount))
    float tangentialVelocityMagnitude = -pxVec2Dot(relativeVelocity, tangent);
    float tangentialVelocityDenom = invMassSum * manifold->contacts.length;
    float jt = pxFastDiv(tangentialVelocityMagnitude, tangentialVelocityDenom);

    // Coulomb's law: friction force ≤ μ * normal force
    PxVec2 frictionImpulse;

    if (fabsf(jt) < j * manifold->staticFriction) {
      frictionImpulse = pxVec2Multf(tangent, jt); // Static friction
    } else {
      frictionImpulse = pxVec2Multf(
          tangent, -j * manifold->dynamicFriction); // Dynamic friction
    }

    // Apply friction impulse
    if (bodyADynamic) {
      pxBodyApplyImpulse(bodyA, pxVec2Neg(frictionImpulse), ra);
    }

    if (bodyBDynamic) {
      pxBodyApplyImpulse(bodyB, frictionImpulse, rb);
    }
  }
}

void pxManifoldCorrectPosition(PxManifold *manifold) {
  // Skip correction if penetration is small or no contacts
  if (manifold->penetration <= PX_ALLOWED_PENETRATION ||
      manifold->contacts.length == 0) {
    return;
  }

  PxBody *bodyA = manifold->bodyA;
  PxBody *bodyB = manifold->bodyB;

  bool bodyADynamic = pxBodyIsDynamic(bodyA);
  bool bodyBDynamic = pxBodyIsDynamic(bodyB);

  if (!bodyADynamic && !bodyBDynamic) {
    return; // Both bodies are non-dynamic
  }

  // Calculate positional correction:
  // 1. Find excess penetration beyond allowed limit
  float excessPenetration = manifold->penetration - PX_ALLOWED_PENETRATION;

  // 2. Scale by PX_POSITIONAL_CORRECTION_FACTOR (Baumgarte factor)
  // 3. Divide by total inverse mass to get impulse-like correction
  float invMassSum = bodyA->iMass + bodyB->iMass;
  float correctionalMagnitude = pxFastDiv(
      excessPenetration * PX_POSITIONAL_CORRECTION_FACTOR, invMassSum);

  PxVec2 correctionVector =
      pxVec2Multf(manifold->normal, correctionalMagnitude);

  // Apply position correction proportional to inverse mass
  if (bodyADynamic) {
    pxBodyMoveBy(bodyA, pxVec2Neg(pxVec2Multf(correctionVector, bodyA->iMass)));
  }

  if (bodyBDynamic) {
    pxBodyMoveBy(bodyB, pxVec2Multf(correctionVector, bodyB->iMass));
  }
}
