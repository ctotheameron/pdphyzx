#ifndef PDPHYZX_PX_MANIFOLD_POOL_H
#define PDPHYZX_PX_MANIFOLD_POOL_H

#include <stdint.h>

#include "px_manifold.h"

#define PX_MAX_MANIFOLDS 64

typedef struct {
  PxManifold items[PX_MAX_MANIFOLDS];
  uint8_t length;
} PxManifoldPool;

/**
 * @brief Macro for iterating through all manifolds in a pool
 *
 * Usage:
 *   pxManifoldPoolEach(pool, manifold) {
 *     // manifold is a pointer to the current manifold
 *   }
 */
#define pxManifoldPoolEach(pool, manifold)                                     \
  for (uint8_t _i = 0; _i < (pool)->length; ++_i)                              \
    for (PxManifold *manifold = &(pool)->items[_i]; manifold; manifold = NULL)

PxManifold *pxManifoldPoolAcquire(PxManifoldPool *pool);
void pxManifoldPoolReleaseLast(PxManifoldPool *pool);
void pxManifoldPoolClear(PxManifoldPool *pool);

#endif // PDPHYZX_PX_MANIFOLD_POOL_H
