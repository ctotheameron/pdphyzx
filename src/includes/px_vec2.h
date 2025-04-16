/**
 * @file px_vec2.h
 *
 * @brief This file contains functions for handling 2D vector operations.
 *
 * The functions provided in this file are used to create, manipulate, and use
 * 2D vectors. These operations include addition, subtraction, multiplication,
 * division, dot product, cross product, length calculation, normalization, and
 * more.
 */

#ifndef PDPHYZX_PX_VEC2_H
#define PDPHYZX_PX_VEC2_H

#include <math.h>
#include <stdint.h>

#include "px_math.h"

#define PX_VEC2_ARRAY_MAX_SIZE 32

/**
 * @brief Definition of a 2D vector structure with multiple access methods.
 */
typedef struct {
  union {
    float v[2];
    struct {
      float x, y;
    };
  };
} PxVec2;

/**
 * @brief Creates a 2D vector from x and y components.
 *
 * @param x
 * @param y
 *
 * @return 2D vector
 */
static inline PxVec2 pxVec2(float x, float y) {
  return (PxVec2){.x = x, .y = y};
}

/**
 * @brief Sets the x and y components of an existing 2D vector.
 *
 * @param vector
 * @param x
 * @param y
 *
 * @return The updated vector
 */
static inline PxVec2 *pxVec2Set(PxVec2 *vector, float x, float y) {
  vector->x = x;
  vector->y = y;

  return vector;
}

//-------------------------------------------------------------------------------
// Negation
//-------------------------------------------------------------------------------

/**
 * @brief Negates a 2D vector.
 *
 * @param vector
 *
 * @return negated vector
 */
static inline PxVec2 pxVec2Neg(const PxVec2 vector) {
  return (PxVec2){.x = -vector.x, .y = -vector.y};
}

/**
 * @brief Negates a 2D vector in place.
 *
 * @param vector
 *
 * @return updated vector
 */
static inline PxVec2 *pxVec2NegAssign(PxVec2 *vector) {
  vector->x = -vector->x;
  vector->y = -vector->y;

  return vector;
}

//-------------------------------------------------------------------------------
// Addition
//-------------------------------------------------------------------------------

/**
 * @brief Adds a scalar to a 2D vector.
 *
 * @param vector
 * @param scalar
 *
 * @return resulting vector
 */
static inline PxVec2 pxVec2Addf(const PxVec2 vector, float scalar) {
  return (PxVec2){.x = vector.x + scalar, .y = vector.y + scalar};
}

/**
 * @brief Adds a scalar to a 2D vector in place.
 *
 * @param vector
 * @param scalar
 *
 * @return The updated vector
 */
static inline PxVec2 *pxVec2AddfAssign(PxVec2 *vector, float scalar) {
  vector->x += scalar;
  vector->y += scalar;

  return vector;
}

/**
 * @brief Adds two 2D vectors.
 *
 * @param vector1
 * @param vector2
 *
 * @return The resulting vector
 */
static inline PxVec2 pxVec2Add(const PxVec2 vector1, const PxVec2 vector2) {
  return (PxVec2){.x = vector1.x + vector2.x, .y = vector1.y + vector2.y};
}

/**
 * @brief Adds a 2D vector to another 2D vector in place.
 *
 * @param vector  - vector to be updated
 * @param vector2 - vector to add
 *
 * @return updated vector
 */
static inline PxVec2 *pxVec2AddAssign(PxVec2 *vector, const PxVec2 vector2) {
  vector->x += vector2.x;
  vector->y += vector2.y;

  return vector;
}

//-------------------------------------------------------------------------------
// Subtraction
//-------------------------------------------------------------------------------

/**
 * @brief Subtracts a scalar from a 2D vector.
 *
 * @param vector
 * @param scalar
 *
 * @return resulting vector
 */
static inline PxVec2 pxVec2Subf(const PxVec2 vector, float scalar) {
  return (PxVec2){.x = vector.x - scalar, .y = vector.y - scalar};
}

/**
 * @brief Subtracts a scalar from a 2D vector in place.
 *
 * @param vector
 * @param scalar
 *
 * @return updated vector
 */
static inline PxVec2 *pxVec2SubfAssign(PxVec2 *vector, float scalar) {
  vector->x -= scalar;
  vector->y -= scalar;

  return vector;
}

/**
 * @brief Subtracts one 2D vector from another.
 *
 * @param vector1
 * @param vector2
 *
 * @return resulting vector
 */
static inline PxVec2 pxVec2Sub(const PxVec2 vector1, const PxVec2 vector2) {
  return (PxVec2){.x = vector1.x - vector2.x, .y = vector1.y - vector2.y};
}

/**
 * @brief Subtracts a 2D vector from another 2D vector in place.
 *
 * @param vector  - vector to be updated
 * @param vector2 - vector to subtract
 *
 * @return updated vector
 */
static inline PxVec2 *pxVec2SubAssign(PxVec2 *vector, const PxVec2 vector2) {
  vector->x -= vector2.x;
  vector->y -= vector2.y;

  return vector;
}

//-------------------------------------------------------------------------------
// Multiplication
//-------------------------------------------------------------------------------

/**
 * @brief Multiplies a 2D vector by a scalar.
 *
 * @param vector
 * @param scalar
 *
 * @return resulting vector
 */
static inline PxVec2 pxVec2Multf(const PxVec2 vector, float scalar) {
  return (PxVec2){.x = vector.x * scalar, .y = vector.y * scalar};
}

/**
 * @brief Multiplies a 2D vector by a scalar in place.
 *
 * @param vector - vector to be updated
 * @param scalar
 *
 * @return updated vector
 */
static inline PxVec2 *pxVec2MultfAssign(PxVec2 *vector, float scalar) {
  vector->x *= scalar;
  vector->y *= scalar;

  return vector;
}

/**
 * @brief Multiplies two 2D vectors component-wise.
 *
 * @param vector1
 * @param vector2
 *
 * @return resulting vector
 */
static inline PxVec2 pxVec2Mult(const PxVec2 vector1, const PxVec2 vector2) {
  return (PxVec2){.x = vector1.x * vector2.x, .y = vector1.y * vector2.y};
}

/**
 * @brief Multiplies a 2D vector by another 2D vector component-wise in place.
 *
 * @param vector  - vector to be updated.
 * @param vector2 - vector to multiply with.
 *
 * @return updated vector
 */
static inline PxVec2 *pxVec2MultAssign(PxVec2 *vector, const PxVec2 vector2) {
  vector->x *= vector2.x;
  vector->y *= vector2.y;

  return vector;
}

//-------------------------------------------------------------------------------
// Division
//-------------------------------------------------------------------------------

/**
 * @brief Divides a 2D vector by a scalar.
 *
 * @param vector
 * @param scalar
 *
 * @return resulting vector
 */
static inline PxVec2 pxVec2Divf(const PxVec2 vector, float scalar) {
  return (PxVec2){.x = pxFastDiv(vector.x, scalar),
                  .y = pxFastDiv(vector.y, scalar)};
}

/**
 * @brief Divides a 2D vector by a scalar in place.
 *
 * @param vector - vector to be updated
 * @param scalar
 *
 * @return updated vector
 */
static inline PxVec2 *pxVec2DivfAssign(PxVec2 *vector, float scalar) {
  vector->x = pxFastDiv(vector->x, scalar);
  vector->y = pxFastDiv(vector->y, scalar);

  return vector;
}

/**
 * @brief Divides one 2D vector by another component-wise.
 *
 * @param vector1
 * @param vector2
 *
 * @return resulting vector
 */
static inline PxVec2 pxVec2Div(const PxVec2 vector1, const PxVec2 vector2) {
  return (PxVec2){.x = pxFastDiv(vector1.x, vector2.x),
                  .y = pxFastDiv(vector1.y, vector2.y)};
}

/**
 * @brief Divides a 2D vector by another 2D vector component-wise in place.
 *
 * @param vector  - vector to be updated
 * @param vector2 - vector to divide by
 *
 * @return updated vector
 */
static inline PxVec2 *pxVec2DivAssign(PxVec2 *vector, const PxVec2 vector2) {
  vector->x = pxFastDiv(vector->x, vector2.x);
  vector->y = pxFastDiv(vector->y, vector2.y);

  return vector;
}

//-------------------------------------------------------------------------------
// Cross Product
//-------------------------------------------------------------------------------

/**
 * @brief Computes the cross product of a 2D vector and a scalar.
 *
 * @param vector
 * @param scalar
 *
 * @return resulting vector
 */
static inline PxVec2 pxVec2Crossf(const PxVec2 vector, float scalar) {
  return (PxVec2){.x = vector.y * scalar, .y = -vector.x * scalar};
}

/**
 * @brief Computes the cross product of two 2D vectors.
 *
 * @param vector1
 * @param vector2
 *
 * @return resulting scalar
 */
static inline float pxVec2Cross(const PxVec2 vector1, const PxVec2 vector2) {
  return vector1.x * vector2.y - vector1.y * vector2.x;
}

//-------------------------------------------------------------------------------
// Dot Product
//-------------------------------------------------------------------------------

/**
 * @brief Computes the dot product of two 2D vectors.
 *
 * @param vector1
 * @param vector2
 *
 * @return resulting scalar
 */
static inline float pxVec2Dot(const PxVec2 vector1, const PxVec2 vector2) {
  return vector1.x * vector2.x + vector1.y * vector2.y;
}

//-------------------------------------------------------------------------------
// Length
//-------------------------------------------------------------------------------

/**
 * @brief Computes the squared length of a 2D vector.
 *
 * @param vector
 *
 * @return squared length
 */
static inline float pxVec2LenSqr(const PxVec2 vector) {
  return vector.x * vector.x + vector.y * vector.y;
}

/**
 * @brief Computes the length of a 2D vector.
 *
 * @param vector
 *
 * @return length
 */
static inline float pxVec2Len(const PxVec2 vector) {
  return pxFastSqrt(pxVec2LenSqr(vector));
}

//-------------------------------------------------------------------------------
// Tangent
//-------------------------------------------------------------------------------

/**
 * @brief Computes the tangent of a 2D vector with respect to a normal vector.
 *
 * @param vector
 * @param normal - normal vector
 *
 * @return resulting tangent vector
 */
static inline PxVec2 pxVec2Tangent(const PxVec2 vector, const PxVec2 normal) {
  float projection = pxVec2Dot(vector, normal);
  return pxVec2Sub(vector, pxVec2Multf(normal, projection));
}

//-------------------------------------------------------------------------------
// Distance
//-------------------------------------------------------------------------------

/**
 * @brief Computes the squared distance between two 2D vectors.
 *
 * @param vector1
 * @param vector2
 *
 * @return squared distance
 */
static inline float pxVec2DistSqr(const PxVec2 vector1, const PxVec2 vector2) {
  PxVec2 d = pxVec2Sub(vector1, vector2);
  return pxVec2Dot(d, d);
}

//-------------------------------------------------------------------------------
// Transform
//-------------------------------------------------------------------------------

/**
 * @brief Rotates a 2D vector by a given angle in radians.
 *
 * @param vector
 * @param radians - angle in radians
 *
 * @return rotated vector
 */
static inline PxVec2 *pxVec2Rotate(PxVec2 *vector, float radians) {
  float c = cosf(radians);
  float s = sinf(radians);

  float originalX = vector->x;
  vector->x = vector->x * c - vector->y * s;
  vector->y = originalX * s + vector->y * c;

  return vector;
}

/**
 * @brief Normalizes a 2D vector.
 *
 * @param vector
 *
 * @return normalized vector
 */
static inline PxVec2 *pxVec2Normalize(PxVec2 *vector) {
  float len = pxVec2Len(*vector);

  if (len <= PX_EPSILON) {
    return vector;
  }

  float invLen = pxFastRcp(len);

  vector->x *= invLen;
  vector->y *= invLen;

  return vector;
}

//===============================================================================
// Array
//===============================================================================

/**
 * @brief Definition of a fixed-size array of 2D vectors with a length.
 */
typedef struct {
  PxVec2 items[2];
  uint8_t length;
} PxVec2Array2;

/**
 * @brief Definition of a variable-size array of 2D vectors with a length.
 */
typedef struct {
  PxVec2 items[PX_VEC2_ARRAY_MAX_SIZE];
  uint8_t length;
} PxVec2Array;

/**
 * @brief Gets a 2D vector from an array at a specified index.
 *
 * @param array
 * @param index
 *
 * @return vector at the specified index, or a zero vector if the index is out
 *         of bounds.
 */
static inline PxVec2 pxVec2ArrayGet(PxVec2Array array, uint8_t index) {
  if (index >= array.length) {
    return pxVec2(0.0f, 0.0f);
  }

  return array.items[index];
}

/**
 * @brief Sets a 2D vector in an array at a specified index.
 *
 * @param array
 * @param index
 * @param value
 */
static inline void pxVec2ArraySet(PxVec2Array *array, uint8_t index,
                                  PxVec2 value) {
  if (index >= array->length) {
    return;
  }

  array->items[index] = value;
}

#endif // PDPHYZX_PX_VEC2_H
