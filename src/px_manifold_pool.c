#include "px_manifold_pool.h"

/**
 * @brief Clears the manifold pool.
 *
 * @param pool - pointer to the manifold pool.
 */
void pxManifoldPoolClear(PxManifoldPool *pool) { pool->length = 0; }

/**
 * @brief Acquires a new manifold from the pool.
 *
 * @param pool - pointer to the manifold pool.
 *
 * @return pointer to the acquired manifold, or NULL if the pool is exhausted.
 */
PxManifold *pxManifoldPoolAcquire(PxManifoldPool *pool) {
  if (pool->length >= PX_MAX_MANIFOLDS) {
    return NULL; // Pool is exhausted
  }

  return &pool->items[pool->length++];
}

/**
 * @brief Releases the most recently acquired manifold back to the pool.
 *
 * @param pool - pointer to the manifold pool.
 */
void pxManifoldPoolReleaseLast(PxManifoldPool *pool) {
  if (pool->length > 0) {
    pool->length--;
  }
}
