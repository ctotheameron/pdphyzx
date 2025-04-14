/**
 * @file px_math.h
 *
 * @brief This file contains mathematical functions and constants for the
 *        PDPHYZX library.
 *
 * The functions provided in this file include fast inverse square root,
 * reciprocal, division, square root, clamping, rounding, comparison, and random
 * number generation.
 */

#ifndef PDPHYZX_MATH_H
#define PDPHYZX_MATH_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define PX_ONE_THIRD 0.333333333f
#define PX_EPSILON 0.0001f
#define PX_PI 3.141592741f
#define PX_2_PI PX_PI * 2.0f
#define PX_H_PI PX_PI * 0.5f
#define PX_3_H_PI PX_PI * 1.5f
#define PX_BIAS_RELATIVE 0.95f
#define PX_BIAS_ABSOLUTE 0.01f

// Fast trigonometric approximation constants
#define PX_SIN_TABLE_SIZE 256
#define PX_SIN_TABLE_MASK (PX_SIN_TABLE_SIZE - 1)
#define PX_2PI_OVER_TABLE_SIZE (PX_2_PI / PX_SIN_TABLE_SIZE)

//-------------------------------------------------------------------------------
// Fast Inverse Square Root
//-------------------------------------------------------------------------------

/**
 * @brief Computes the fast inverse square root of a number.
 *
 * This function uses a fast approximation algorithm to compute the inverse
 * square root.
 *
 * @param x - input value
 *
 * @return inverse square root of the input value
 */
static inline float pxFastRsqrt(float x) {
  union {
    float f;
    uint32_t u;
  } f32u32;

  f32u32.f = x;

  f32u32.u =
      0x5f3759df - (f32u32.u >> 1); // Magic number for fast inverse square root

  f32u32.f *= 1.5f - (x * 0.5f * f32u32.f * f32u32.f);

  return f32u32.f;
}

/**
 * @brief Computes the fast reciprocal of a number.
 *
 * This function uses the fast inverse square root to compute the reciprocal.
 *
 * @param x - input value
 *
 * @return reciprocal of the input value
 */
static inline float pxFastRcp(float x) {
  x = pxFastRsqrt(x);
  return x * x;
}

/**
 * @brief Computes the fast safe reciprocal of a number.
 *
 * This function returns 0 if the input is 0 to avoid division by zero.
 *
 * @param x - input value
 *
 * @return The safe reciprocal of the input value
 */
static inline float pxFastSafeRcp(float x) { return x ? pxFastRcp(x) : 0; }

/**
 * @brief Computes the fast division of two numbers.
 *
 * This function uses the fast reciprocal to compute the division.
 *
 * @param numerator
 * @param denominator
 *
 * @return result of the division.
 */
static inline float pxFastDiv(float numerator, float denominator) {
  return numerator * pxFastRcp(denominator);
}

/**
 * @brief Computes the fast square root of a number.
 *
 * This function uses the fast inverse square root to compute the square root.
 *
 * @param x - input value
 *
 * @return square root of the input value
 */
static inline float pxFastSqrt(float x) { return x * pxFastRsqrt(x); }

//-------------------------------------------------------------------------------
// Rounding
//-------------------------------------------------------------------------------

/**
 * @brief Clamps a value between a minimum and maximum.
 *
 * @param x   - input value
 * @param min - minimum value
 * @param max - maximum value
 *
 * @return clamped value
 */
static inline float pxClamp(float x, float min, float max) {
  if (x < min) {
    return min;
  }

  if (x > max) {
    return max;
  }

  return x;
}

/**
 * @brief Converts a float to an integer with rounding.
 *
 * @param x - input value
 *
 * @return rounded integer value
 */
static inline int32_t pxToInt(float x) { return (int32_t)(x + 0.5f); }

//-------------------------------------------------------------------------------
// Comparison
//-------------------------------------------------------------------------------

/**
 * @brief Compares two values with a bias.
 *
 * This function checks if `a` is greater than or equal to `b` with a relative
 * and absolute bias.
 *
 * @param a - first value
 * @param b - second value
 *
 * @return true if `a` is greater than or equal to `b` with bias, false
 *         otherwise
 */
static inline bool pxBiasGt(float a, float b) {
  return a >= b * PX_BIAS_RELATIVE + a * PX_BIAS_ABSOLUTE;
}

/**
 * @brief Returns the minimum of two values.
 *
 * @param a - first value
 * @param b - second value
 *
 * @return minimum value
 */
static inline float pxMin(float a, float b) { return a < b ? a : b; }

//-------------------------------------------------------------------------------
// Random
//-------------------------------------------------------------------------------

/**
 * @brief Generates a random float between a specified range.
 *
 * @param low  - lower bound of the range
 * @param high - upper bound of the range
 *
 * @return random float between `low` and `high`
 */
static inline float pxRandf(float low, float high) {
  return low + (high - low) * pxFastDiv(rand(), (float)RAND_MAX);
}

//-------------------------------------------------------------------------------
// Trigonometric
//-------------------------------------------------------------------------------

/**
 * @brief Lookup table for sine values
 *
 * This table stores pre-calculated sine values for angles 0 to 2π,
 * divided into PX_SIN_TABLE_SIZE increments for fast access.
 */
static float pxSinTable[PX_SIN_TABLE_SIZE];

/**
 * @brief Initialize the sine lookup table.
 *
 * Must be called once during engine initialization.
 */
static inline void pxInitSinTable(void) {
  for (int i = 0; i < PX_SIN_TABLE_SIZE; i++) {
    float angle = (i * PX_2_PI) / PX_SIN_TABLE_SIZE;
    pxSinTable[i] = sinf(angle);
  }
}

/**
 * @brief Fast approximation of sine using lookup table.
 *
 * @param radians - angle in radians.
 * @return approximate sine value.
 */
static inline float pxFastSin(float radians) {
  // Normalize angle to [0, 2π) range
  if ((radians = fmodf(radians, PX_2_PI)) < 0) {
    radians += PX_2_PI;
  }

  // Convert angle to table position
  float position = pxFastDiv(radians, PX_2PI_OVER_TABLE_SIZE);
  int index1 = (int)position & PX_SIN_TABLE_MASK;
  int index2 = (index1 + 1) & PX_SIN_TABLE_MASK;

  // Linear interpolation between the two closest values
  float fraction = position - (int)position;

  return pxSinTable[index1] * (1.0f - fraction) + pxSinTable[index2] * fraction;
}

/**
 * @brief Fast approximation of cosine using lookup table.
 *
 * @param radians - angle in radians.
 * @return approximate cosine value.
 */
static inline float pxFastCos(float radians) {
  // Cosine is just sine with a quarter circle phase shift
  return pxFastSin(radians + PX_H_PI);
}

#endif // PDPHYZX_MATH_H
