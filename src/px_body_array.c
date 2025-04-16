#include "px_body.h"

#include "px_body_array.h"

PxBody *pxBodyArrayAdd(PxBodyArray *array, PxBody *body) {
  if (!pxBodyIsValid(body)) {
    return NULL;
  }

  /**
   * Memory management strategy for body array:
   *
   * - Bodies are stored in a contiguous array (array->items)
   *
   * - We maintain two tracking mechanisms:
   *
   *   1. activeIndices: Contains indices of all active bodies for efficient
   *                     iteration
   *
   *   2. freedIndices:  A stack of indices that have been freed and can be
   *                     reused
   *
   * - When adding a body, we prefer to reuse a freed index before extending the
   *   array
   *
   * - This approach allows O(1) addition/removal while maintaining memory
   *   locality
   */
  uint8_t index;

  if (array->freedCount > 0) {
    // Reuse a "freed" index
    index = array->freedIndices[--array->freedCount];
  } else if (array->length < PX_MAX_BODIES) {
    // Add to the end of the array (using current length as next index)
    index = array->length;
  } else {
    return NULL; // Array is full
  }

  // Update activeIndices and increment length.
  //
  // activeIndices maintains a list of indices that are currently in use.
  // This allows for efficient iteration through only active bodies.
  uint8_t currentLength = array->length;
  array->activeIndices[currentLength] = index;
  array->length++;

  // Store the world index on the body
  body->worldIndex = index;

  // Update the array
  array->items[index] = *body;

  // Return a pointer to the stored body
  return &array->items[index];
}

void pxBodyArrayRemove(PxBodyArray *array, uint8_t index) {
  if (index >= array->length || array->freedCount >= PX_MAX_BODIES) {
    return;
  }

  // Add the index to the freedIndices array
  array->freedIndices[array->freedCount++] = index;

  // Find the index in activeIndices and remove it by swapping with the last
  // element
  for (uint8_t i = 0; i < array->length; i++) {
    if (array->activeIndices[i] == index) {
      array->activeIndices[i] = array->activeIndices[--array->length];
      break;
    }
  }
}

void pxBodyArraySortByAxis(PxBodyArray *bodies) {
  for (uint8_t i = 1; i < bodies->length; i++) {
    uint8_t currentActiveIndex = bodies->activeIndices[i];
    PxBody *currentBody = &bodies->items[currentActiveIndex];
    float currentMinX = currentBody->aabb.min.x;

    int j = i - 1;
    while (j >= 0) {
      uint8_t compareActiveIndex = bodies->activeIndices[j];
      PxBody *compareBody = &bodies->items[compareActiveIndex];

      if (compareBody->aabb.min.x <= currentMinX) {
        break;
      }

      bodies->activeIndices[j + 1] = bodies->activeIndices[j];
      j--;
    }

    bodies->activeIndices[j + 1] = currentActiveIndex;
  }
}

uint8_t pxBodyArrayFindFirstIndexAfterX(PxBodyArray *bodies, float minX) {
  if (bodies->length == 0) {
    return 0;
  }

  uint8_t low = 0;
  uint8_t high = bodies->length;

  while (low < high) {
    uint8_t mid = low + (high - low) / 2; // Prevents potential overflow
    uint8_t bodyIndex = bodies->activeIndices[mid];
    PxBody *body = &bodies->items[bodyIndex];

    if (!body || body->worldIndex == PX_INVALID_WORLD_INDEX) {
      // Handle invalid body (shouldn't happen if array is properly maintained)
      low = mid + 1;
      continue;
    }

    if (body->aabb.max.x < minX) {
      low = mid + 1;
    } else {
      high = mid;
    }
  }

  return low;
}
