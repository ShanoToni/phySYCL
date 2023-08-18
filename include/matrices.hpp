#ifndef _H_MATH_MATRICES_
#define _H_MATH_MATRICES_

#include "vectors.hpp"

struct mat2 {
  union {
    struct {
      float _11, _12, _21, _22;
    };
    float asArray[4];
  };

  inline float *operator[](int i) { return &(asArray[i * 2]); }
  inline mat2() {
    _11 = _22 = 1.0f;
    _12 = _21 = 0.0f;
  };
  inline mat2(float f11, float f12, float f21, float f22) {
    _11 = f11;
    _12 = f12;
    _21 = f21;
    _22 = f22;
  };
};

struct mat3 {
  union {
    struct {
      float _11, _12, _13, _21, _22, _23, _31, _32, _33;
    };
    float asArray[9];
  };

  inline float *operator[](int i) { return &(asArray[i * 3]); }
  inline mat3() {
    _11 = _22 = _33 = 1.0f;
    _12 = _13 = _21 = 0.0f;
    _23 = _31 = _32 = 0.0f;
  };

  inline mat3(float f11, float f12, float f13, float f21, float f22, float f23,
              float f31, float f32, float f33) {
    _11 = f11;
    _12 = f12;
    _13 = f13;
    _21 = f21;
    _22 = f22;
    _23 = f23;
    _31 = f31;
    _32 = f32;
    _33 = f33;
  };
};

struct mat4 {
  union {
    struct {
      float _11, _12, _13, _14, _21, _22, _23, _24, _31, _32, _33, _34, _41,
          _42, _43, _44;
    };
    float asArray[16];
  };

  inline float *operator[](int i) { return &(asArray[i * 4]); }
  inline mat4() {
    _11 = _22 = _33 = _44 = 1.0f;
    _12 = _13 = _14 = _21 = 0.0f;
    _23 = _24 = _31 = _32 = 0.0f;
    _34 = _41 = _42 = _43 = 0.0f;
  };
  inline mat4(float f11, float f12, float f13, float f14, float f21, float f22,
              float f23, float f24, float f31, float f32, float f33, float f34,
              float f41, float f42, float f43, float f44) {
    _11 = f11;
    _12 = f12;
    _13 = f13;
    _14 = f14;
    _21 = f21;
    _22 = f22;
    _23 = f23;
    _24 = f24;
    _31 = f31;
    _32 = f32;
    _33 = f33;
    _34 = f34;
    _41 = f41;
    _42 = f42;
    _43 = f43;
    _44 = f44;
  };
};

void transpose(const float *srcMat, float *dstMat, int srcRows, int srcCols);
mat2 transpose(const mat2 &matrix);
mat3 transpose(const mat3 &matrix);
mat4 transpose(const mat4 &matrix);

mat2 operator*(const mat2 &matrix, float scalar);
mat3 operator*(const mat3 &matrix, float scalar);
mat4 operator*(const mat4 &matrix, float scalar);

bool multiply(float *out, const float *matA, int aRows, int aCols,
              const float *matB, int bRows, int bCols);
mat2 operator*(const mat2 &matA, const mat2 &matB);
mat3 operator*(const mat3 &matA, const mat3 &matB);
mat4 operator*(const mat4 &matA, const mat4 &matB);

void cofactor(float *out, const float *minor, int rows, int cols);
mat2 cofactor(const mat2 &mat);
mat3 cofactor(const mat3 &mat);
mat4 cofactor(const mat4 &mat);

// if matrix determinant is non-zero it has an inverse
float determinant(const mat2 &matrix) {
  return matrix._11 * matrix._22 - matrix._12 * matrix._21;
}

float determinant(const mat3 &matrix);
float determinant(const mat4 &matrix);

mat2 cut(const mat3 &mat, int row, int col);
mat3 cut(const mat4 &mat, int row, int col);

mat2 minor(const mat2 &mat);
mat3 minor(const mat3 &mat);
mat4 minor(const mat4 &mat);

mat2 adjugate(const mat2 &mat) { return transpose(cofactor(mat)); }
mat3 adjugate(const mat3 &mat) { return transpose(cofactor(mat)); }
mat4 adjugate(const mat4 &mat) { return transpose(cofactor(mat)); }

mat2 inverse(const mat2 &mat);
mat3 inverse(const mat3 &mat);
mat4 inverse(const mat4 &mat);

// transforms

mat4 translation(float x, float y, float z);
mat4 translation(const vec3 &pos);
vec3 getTranslation(const mat4 &mat);

mat4 scale(float x, float y, float z);
mat4 scale(const vec3 &pos);
vec3 getScale(const mat4 &mat);

// rotations
mat4 z_rotation(float angle);
mat3 z_rotation3x3(float angle);
mat4 y_rotation(float angle);
mat3 y_rotation3x3(float angle);
mat4 x_rotation(float angle);
mat3 x_rotation3x3(float angle);

mat4 rotation(float pitch, float yaw, float roll);
mat3 rotation3x3(float pitch, float yaw, float roll);

mat4 axis_angle(const vec3 &axis, float angle);
mat3 axis_angle3x3(const vec3 &axis, float angle);

// Vector Matrix multiplication
vec3 multiply_point(const vec3 &vec, const mat4 &mat);
vec3 multiply_vec(const vec3 &vec, const mat4 &mat);
vec3 multiply_vec(const vec3 &vec, const mat3 &mat);

mat4 transform(const vec3 &scale, const vec3 &eulerRotation,
               const vec3 &translate);
mat4 transform(const vec3 &scale, const vec3 &rotationAxis, float rotationAngle,
               const vec3 &translate);

mat4 look_at(const vec3 &position, const vec3 &target, const vec3 &up);

mat4 projection(float fov, float aspect, float zNear, float zFar);
mat4 ortho(float left, float right, float bottom, float top, float zNear,
           float zFar);

#endif // _H_MATH_MATRICES_