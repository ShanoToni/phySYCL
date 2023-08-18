#include "matrices.hpp"
#include <cfloat>
#include <cmath>

#define DEG2RAD(x) ((x)*0.0174533f)

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

float determinant(const mat4 &matrix) {
  float result = 0.0f;
  mat4 coF = cofactor(matrix);
  for (int i = 0; i < 4; i++) {
    result += matrix.asArray[4 * 0 + i] * coF[0][i];
  }
  return result;
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

mat3 cut(const mat4 &mat, int row, int col) {
  mat3 result;
  int index = 0;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (i == row || j == col) {
        continue;
      }
      int target = index++;
      int source = 4 * i + j;

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

mat4 minor(const mat4 &mat) {
  mat4 result;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      result[i][j] = determinant(cut(mat, i, j));
    }
  }
  return result;
}

mat2 inverse(const mat2 &mat) {
  float det = determinant(mat);
  if (CMP(det, 0.0f)) {
    return mat2();
  }
  return adjugate(mat) * (1.0f * det);
}

mat3 inverse(const mat3 &mat) {
  float det = determinant(mat);
  if (CMP(det, 0.0f)) {
    return mat3();
  }
  return adjugate(mat) * (1.0f * det);
}

mat4 inverse(const mat4 &mat) {
  float det = determinant(mat);
  if (CMP(det, 0.0f)) {
    return mat4();
  }
  return adjugate(mat) * (1.0f * det);
}

mat4 translation(float x, float y, float z) {
  return mat4(1.0f, 0.0f, 0.0f, 0.0f, //
              0.0f, 1.0f, 0.0f, 0.0f, //
              0.0f, 0.0f, 1.0f, 0.0f, //
              x, y, z, 1.0f);
}

mat4 translation(const vec3 &pos) {
  return mat4(1.0f, 0.0f, 0.0f, 0.0f, //
              0.0f, 1.0f, 0.0f, 0.0f, //
              0.0f, 0.0f, 1.0f, 0.0f, //
              pos.x, pos.y, pos.z, 1.0f);
}

vec3 getTranslation(const mat4 &mat) { return vec3{mat._41, mat._42, mat._43}; }

mat4 scale(float x, float y, float z) {
  return mat4(x, 0.0f, 0.0f, 0.0f, //
              0.0f, y, 0.0f, 0.0f, //
              0.0f, 0.0f, z, 0.0f, //
              0.0f, 0.0f, 0.0f, 1.0f);
}

mat4 scale(const vec3 &pos) {
  return mat4(pos.x, 0.0f, 0.0f, 0.0f, //
              0.0f, pos.y, 0.0f, 0.0f, //
              0.0f, 0.0f, pos.z, 0.0f, //
              0.0f, 0.0f, 0.0f, 1.0f);
}

vec3 getScale(const mat4 &mat) { return vec3{mat._11, mat._22, mat._33}; }

mat4 z_rotation(float angle) {
  angle = DEG2RAD(angle);
  return mat4(cosf(angle), sinf(angle), 0.0f, 0.0f,  //
              -sinf(angle), cosf(angle), 0.0f, 0.0f, //
              0.0f, 0.0f, 1.0f, 0.0f,                //
              0.0f, 0.0f, 0.0f, 1.0f);
}

mat3 z_rotation3x3(float angle) {
  angle = DEG2RAD(angle);
  return mat3(cosf(angle), sinf(angle), 0.0f,  //
              -sinf(angle), cosf(angle), 0.0f, //
              0.0f, 0.0f, 1.0f);
}

mat4 y_rotation(float angle) {
  angle = DEG2RAD(angle);
  return mat4(cosf(angle), 0.0f, -sinf(angle), 0.0f, //
              0.0f, 1.0f, 0.0f, 0.0f,                //
              sinf(angle), 0.0f, cosf(angle), 0.0f,  //
              0.0f, 0.0f, 0.0f, 1.0f);
}

mat3 y_rotation3x3(float angle) {
  angle = DEG2RAD(angle);
  return mat3(cosf(angle), 0.0f, -sinf(angle), //
              0.0f, 1.0f, 0.0f,                //
              sinf(angle), 0.0f, cosf(angle));
}

mat4 x_rotation(float angle) {
  angle = DEG2RAD(angle);
  return mat4(1.0f, 0.0f, 0.0f, 0.0f,               //
              0.0f, cosf(angle), sinf(angle), 0.0f, //
              0.0f, -sinf(angle), cos(angle), 0.0f, //
              0.0f, 0.0f, 0.0f, 1.0f);
}

mat3 x_rotation3x3(float angle) {
  angle = DEG2RAD(angle);
  return mat3(1.0f, 0.0f, 0.0f,               //
              0.0f, cosf(angle), sinf(angle), //
              0.0f, -sinf(angle), cos(angle));
}

mat4 rotation(float pitch, float yaw, float roll) {
  return z_rotation(pitch) * y_rotation(yaw) * x_rotation(roll);
}

mat3 rotation3x3(float pitch, float yaw, float roll) {
  return z_rotation3x3(pitch) * y_rotation3x3(yaw) * x_rotation3x3(roll);
}

mat4 axis_angle(const vec3 &axis, float angle) {
  angle = DEG2RAD(angle);
  float c = cosf(angle);
  float s = sinf(angle);
  float t = 1.0f - cosf(angle);
  float x = axis.x;
  float y = axis.y;
  float z = axis.z;
  if (!CMP(magnitude_sq(axis), 1.0f)) {
    float inv_len = 1.0f / magnitude(axis);
    x *= inv_len; // Normalize x
    y *= inv_len; // Normalize y
    z *= inv_len; // Normalize z
  }               // x, y, and z are a normalized vector
  return mat4(t * (x * x) + c, t * x * y + s * z, t * x * z - s * y, 0.0f,
              t * x * y - s * z, t * (y * y) + c, t * y * z + s * x, 0.0f,
              t * x * z + s * y, t * y * z - s * x, t * (z * z) + c, 0.0f, 0.0f,
              0.0f, 0.0f, 1.0f);
}

mat3 axis_angle3x3(const vec3 &axis, float angle) {
  angle = DEG2RAD(angle);
  float c = cosf(angle);
  float s = sinf(angle);
  float t = 1.0f - cosf(angle);

  float x = axis.x;
  float y = axis.y;
  float z = axis.z;
  if (!CMP(magnitude_sq(axis), 1.0f)) {
    float inv_len = 1.0f / magnitude(axis);
    x *= inv_len;
    y *= inv_len;
    z *= inv_len;
  }
  return mat3(t * (x * x) + c, t * x * y + s * z, t * x * z - s * y,
              t * x * y - s * z, t * (y * y) + c, t * y * z + s * x,
              t * x * z + s * y, t * y * z - s * x, t * (z * z) + c);
}

vec3 multiply_point(const vec3 &vec, const mat4 &mat) {
  vec3 result;
  result.x =
      vec.x * mat._11 + vec.y * mat._21 + vec.z * mat._31 + 1.0f * mat._41;
  result.y =
      vec.x * mat._12 + vec.y * mat._22 + vec.z * mat._32 + 1.0f * mat._42;
  result.z =
      vec.x * mat._13 + vec.y * mat._23 + vec.z * mat._33 + 1.0f * mat._43;
  return result;
}

vec3 multiply_vec(const vec3 &vec, const mat4 &mat) {
  vec3 result;
  result.x =
      vec.x * mat._11 + vec.y * mat._21 + vec.z * mat._31 + 0.0f * mat._41;
  result.y =
      vec.x * mat._12 + vec.y * mat._22 + vec.z * mat._32 + 0.0f * mat._42;
  result.z =
      vec.x * mat._13 + vec.y * mat._23 + vec.z * mat._33 + 0.0f * mat._43;
  return result;
}

vec3 multiply_vec(const vec3 &vec, const mat3 &mat) {
  vec3 result;
  result.x = dot(vec, vec3{mat._11, mat._21, mat._31});
  result.y = dot(vec, vec3{mat._12, mat._22, mat._32});
  result.z = dot(vec, vec3{mat._13, mat._23, mat._33});
  return result;
}

mat4 transform(const vec3 &sc, const vec3 &eulerRot, const vec3 &tr) {
  return scale(sc) * rotation(eulerRot.x, eulerRot.y, eulerRot.z) *
         translation(tr);
}

mat4 transform(const vec3 &sc, const vec3 &rotationAxis, float rotationAngle,
               const vec3 &translate) {
  return scale(sc) * axis_angle(rotationAxis, rotationAngle) *
         translation(translate);
}

mat4 look_at(const vec3 &position, const vec3 &target, const vec3 &up) {
  vec3 forward = normalized(target - position);
  vec3 right = normalized(cross(up, forward));
  vec3 newUp = normalized(cross(forward, right));

  return mat4(right.x, newUp.x, forward.x, 0.0f, //
              right.y, newUp.y, forward.y, 0.0f, //
              right.z, newUp.z, forward.z, 0.0f, //
              -dot(right, position), -dot(newUp, position),
              -dot(forward, position), 1.0f);
}

mat4 projection(float fov, float aspect, float zNear, float zFar) {
  float tanHalfFov = tanf(DEG2RAD(fov * 0.5f));
  float fovY = 1.0f / tanHalfFov;
  float fovX = fovY / aspect;
  mat4 result;
  result._11 = fovX;
  result._22 = fovY;
  result._33 = zFar / (zFar - zNear);
  result._34 = 1.0f;
  result._43 = -zNear * result._33;
  result._44 = 0.0f;
  return result;
}

mat4 ortho(float left, float right, float bottom, float top, float zNear,
           float zFar) {
  float _11 = 2.0f / (right - left);
  float _22 = 2.0f / (top - bottom);
  float _33 = 1.0f / (zFar - zNear);
  float _41 = (left + right) / (left - right);
  float _42 = (top + bottom) / (bottom - top);
  float _43 = (zNear) / (zNear - zFar);
  return mat4(_11, 0.0f, 0.0f, 0.0f, 0.0f, _22, 0.0f, 0.0f, 0.0f, 0.0f, _33,
              0.0f, _41, _42, _43, 1.0f);
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

mat4 cofactor(const mat4 &mat) {
  mat4 result;
  cofactor(result.asArray, minor(mat).asArray, 4, 4);
  return result;
}
