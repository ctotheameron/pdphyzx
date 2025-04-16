#include "px_collider.h"

PxCollider pxColliderNew(PxColliderType type, PxShapeData shape) {
  return (PxCollider){.type = type, .shape = shape};
}
