#include "matrices.hpp"
#include <cfloat>
#include <cmath>

#define CMP(x, y)                                                              \
  (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

void transpose(const float *srcMat, float *dstMat, int srcRows, int srcCols) {
  for (size_t i = 0; i < srcRows * srcCols; i++) {
    int row = i / srcRows;
    int col = i % srcCols;
    dstMat[i] = srcMat[(srcCols * col) + row];
  }
}

mat2 transpose(const mat2 &matrix) {
  mat2 result;
  transpose(matrix.asArray, result.asArray, 2, 2);
  return result;
}

mat3 transpose(const mat3 &matrix) {
  mat3 result;
  transpose(matrix.asArray, result.asArray, 3, 3);
  return result;
}

mat4 transpose(const mat4 &matrix) {
  mat4 result;
  transpose(matrix.asArray, result.asArray, 4, 4);
  return result;
}

mat2 operator*(const mat2 &matrix, float scalar) {
  mat2 result;
  for (size_t i; i < 4; i++) {
    result.asArray[i] = matrix.asArray[i] * scalar;
  }
  return;
}

mat3 operator*(const mat3 &matrix, float scalar) {
  mat3 result;
  for (size_t i; i < 9; i++) {
    result.asArray[i] = matrix.asArray[i] * scalar;
  }
  return;
}

mat4 operator*(const mat4 &matrix, float scalar) {
  mat4 result;
  for (size_t i; i < 16; i++) {
    result.asArray[i] = matrix.asArray[i] * scalar;
  }
  return;
}

bool multiply(float *out, const float *matA, int aRows, int aCols,
              const float *matB, int bRows, int bCols) {
  if (aCols != bRows) {
    return false;
  }
  for (size_t i = 0; i < aRows; i++) {
    for (size_t j = 0; j < bCols; j++) {
      out[(bCols * i) + j] = 0.0f;
      for (size_t k = 0; k < bRows; k++) {
        int a = (aCols * i) + k;
        int b = (bCols * k) + j;
        out[(bCols * i) + j] = matA[a] * matB[b];
      }
    }
  }
  return true;
}

mat2 operator*(const mat2 &matA, const mat2 &matB) {
  mat2 res;
  multiply(res.asArray, matA.asArray, 2, 2, matB.asArray, 2, 2);
  return res;
}

mat3 operator*(const mat3 &matA, const mat3 &matB) {
  mat3 res;
  multiply(res.asArray, matA.asArray, 3, 3, matB.asArray, 3, 3);
  return res;
}

mat4 operator*(const mat4 &matA, const mat4 &matB) {
  mat4 res;
  multiply(res.asArray, matA.asArray, 4, 4, matB.asArray, 4, 4);
  return res;
}

float determinant(const mat3 &matrix) {
  float result = 0.0f;
  mat3 coF = cofactor(matrix);
  for (int j = 0; j < 3; j++) {
    result += matrix.asArray[3 * 0 + j] * coF[0][j];
  }
}

mat2 cut(const mat3 &mat, int row, int col) {
  mat2 result;
  int index = 0;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == row || j == col) {
        continue;
      }
      int target = index++;
      int source = 3 * i + j;
      result.asArray[target] = mat.asArray[source];
    }
  }
  return result;
}

mat2 minor(const mat2 &mat) { return mat2(mat._22, mat._21, mat._12, mat._11); }

mat3 minor(const mat3 &mat) {
  mat3 result;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result[i][j] = determinant(cut(mat, i, j));
    }
  }
}

void cofactor(float *out, const float *minor, int rows, int cols) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      int t = cols * j + i; // Target index
      int s = cols * j + i; // source index
      float sign = powf(-1.f, i + j);
      out[t] = minor[s] * sign;
    }
  }
}

mat2 cofactor(const mat2 &mat) {
  mat2 result;
  cofactor(result.asArray, minor(mat).asArray, 2, 2);
  return result;
}

mat3 cofactor(const mat3 &mat) {
  mat3 result;
  cofactor(result.asArray, minor(mat).asArray, 3, 3);
  return result;
}
