/**
 * @file mat2.h
 * @brief This file contains functions for handling 2x2 matrix operations.
 *
 * The functions provided in this file are used to create, manipulate, and use
 * 2x2 matrices. These operations include creating rotation matrices, setting
 * matrix orientation, transposing matrices, and multiplying matrices by 2D
 * vectors.
 */

#ifndef PDPHYZX_MAT2_H
#define PDPHYZX_MAT2_H

#include "vec2.h"

/**
 * @brief Definition of a 2x2 matrix structure with multiple access methods.
 */
typedef struct {
  union {
    struct {
      float m00, m01;
      float m10, m11;
    };
    float m[2][2];
    float v[4];
  };
} PxMat2;

/**
 * @brief Creates a 2x2 identity matrix.
 *
 * `[ 1 0 ]`
 * `[ 0 1 ]`
 *
 * @return 2x2 identity matrix.
 */
static inline PxMat2 pxMat2Identity(void) {
  return (PxMat2){.m00 = 1, .m01 = 0, .m10 = 0, .m11 = 1};
}

/**
 * @brief Creates a 2x2 rotation matrix from an angle in radians.
 *
 * `[ cos(r) -sin(r) ]`
 * `[ sin(r)  cos(r) ]`
 *
 * Uses lookup table approximations of sine and cosine for better performance.
 *
 * @param radians - angle in radians.
 *
 * @return 2x2 rotation matrix.
 */
static inline PxMat2 pxMat2Orientation(float radians) {
  float c = pxFastCos(radians);
  float s = pxFastSin(radians);

  return (PxMat2){.m00 = c, .m01 = -s, .m10 = s, .m11 = c};
}

/**
 * @brief Transposes a 2x2 matrix.
 *
 * @param mat2 - 2x2 matrix to transpose.
 *
 * @return transposed 2x2 matrix.
 */
static inline PxMat2 pxMat2Transpose(PxMat2 mat2) {
  return (PxMat2){
      .m00 = mat2.m00,
      .m01 = mat2.m10,
      .m10 = mat2.m01,
      .m11 = mat2.m11,
  };
}

/**
 * @brief Multiplies a 2x2 matrix by a 2D vector.
 *
 * @param mat2 - 2x2 matrix.
 * @param v    - 2D vector.
 *
 * @return resulting 2D vector.
 */
static inline PxVec2 pxMat2MultVec2(PxMat2 mat2, PxVec2 v) {
  return pxVec2(mat2.m00 * v.x + mat2.m01 * v.y,
                mat2.m10 * v.x + mat2.m11 * v.y);
}

#endif // PDPHYZX_MAT2_H
