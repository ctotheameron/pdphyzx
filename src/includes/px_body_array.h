#ifndef PDPHYZX_PX_BODY_ARRAY_H
#define PDPHYZX_PX_BODY_ARRAY_H

#include <stdint.h>

#include "px_body.h"

/**
 * @brief Maximum number of bodies that can exist in the simulation
 */
#define PX_MAX_BODIES 64

/**
 * @brief Array container for efficiently managing body instances
 *
 * Uses separate active/freed index arrays to allow fast iteration over only
 * active bodies while maintaining O(1) addition and removal.
 */
typedef struct __attribute__((aligned(4))) {
  PxBody items[PX_MAX_BODIES]; /**< Storage for body data */

  // Small metadata arrays for efficient body management
  uint8_t length;     /**< Number of active bodies */
  uint8_t freedCount; /**< Number of freed slots available for reuse */

  /** 2 bytes of compiler-inserted padding here */

  // These small arrays have good cache locality
  uint8_t activeIndices[PX_MAX_BODIES]; /**< Indices of active bodies for fast
                                        iteration */
  uint8_t freedIndices[PX_MAX_BODIES];  /**< Indices of freed slots available
                                        for  reuse */
} PxBodyArray;

/**
 * @brief Macro for iterating through active bodies
 *
 * Usage:
 *
 * ```c
 * pxBodyArrayEach(bodyArray, body) { // Start from beginning
 *    // body is a pointer to the current active body
 *    pxBodyApplyForce(body, ...);
 *  }
 * ```
 */
#define pxBodyArrayEach(array, body)                                           \
  for (uint8_t _i = 0; _i < (array)->length; _i++)                             \
    for (PxBody *body = &(array)->items[(array)->activeIndices[_i]]; body;     \
         body = NULL)

/**
 * @brief Macro for iterating through active bodies starting from an index
 *
 * Usage:
 *
 * ```c
 * pxBodyArrayEachFrom(bodyArray, 5, body) { // Start from 5th
 *    // body is a pointer to the current active body
 *    pxBodyApplyForce(body, ...);
 *  }
 * ```
 */
#define pxBodyArrayEachFrom(array, offset, body)                               \
  for (uint8_t _i = (offset); _i < (array)->length; ++_i)                      \
    for (PxBody *body = &(array)->items[(array)->activeIndices[_i]]; body;     \
         body = NULL)

/**
 * @brief Macro for iterating through active bodies while tracking current index
 *
 * Usage:
 *
 * ```c
 * pxBodyArrayEachFrom(bodyArray, body, idx) {
 *    printf("Current index: %d\n", idx);
 *
 *    // body is a pointer to the current active body
 *    pxBodyApplyForce(body, ...);
 *  }
 * ```
 */
#define pxBodyArrayEachWithIdx(array, body, idx)                               \
  for (uint8_t idx = 0; idx < (array)->length; idx++)                          \
    for (PxBody *body = &(array)->items[(array)->activeIndices[idx]]; body;    \
         body = NULL)

/**
 * @brief Adds a body to the array
 *
 * @param array - pointer to the body array
 * @param body  - pointer to the body to add
 *
 * @return pointer to stored body or NULL if array is full or body is invalid
 */
PxBody *pxBodyArrayAdd(PxBodyArray *array, PxBody *body);

/**
 * @brief Removes a body from the array
 *
 * @param array - pointer to the body array
 * @param index - index of the body to remove
 *
 * @note Attempting to remove invalid index or already removed body is a no-op
 */
void pxBodyArrayRemove(PxBodyArray *array, uint8_t index);

/**
 * @brief Sorts the bodies in the array by their minimum X axis coordinate.
 *
 * This function reorders only the internal activeIndices array to optimize
 * broad-phase collision detection using a sweep-and-prune algorithm. The
 * bodies themselves remain in their original memory locations, so external
 * pointers to bodies remain valid after sorting.
 *
 * @param bodies - pointer to the body array to be sorted
 *
 * @note This is typically used before collision detection to reduce the number
 * of detailed collision checks required
 */
void pxBodyArraySortByAxis(PxBodyArray *bodies);

/**
 * @brief Finds the index of the first body in a sorted array that could
 * potentially overlap with a body having the given minimum x-coordinate
 *
 * Uses binary search on a PxBodyArray that has been sorted by min.x
 *
 * @param bodies - bodies sorted by min.x coordinate
 * @param minX   - minimum x-coordinate to test against
 *
 * @return index of the first potential overlapping body
 */
uint8_t pxBodyArrayFindFirstIndexAfterX(PxBodyArray *bodies, float minX);

#endif // PDPHYZX_PX_BODY_ARRAY_H
