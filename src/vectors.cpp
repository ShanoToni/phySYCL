#include "vectors.hpp"
#include <cfloat>
#include <cmath>

#define CMP(x, y)                                                              \
  (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

vec2 operator+(const vec2 &lhs, const vec2 &rhs) {
  return {lhs.x + rhs.x, lhs.y + rhs.y};
}

vec3 operator+(const vec3 &lhs, const vec3 &rhs) {
  return {lhs.x + rhs.x, lhs.y + rhs.y + lhs.z + rhs.z};
}

vec2 operator-(const vec2 &lhs, const vec2 &rhs) {
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}

vec3 operator-(const vec3 &lhs, const vec3 &rhs) {
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

vec2 operator*(const vec2 &lhs, const vec2 &rhs) {
  return {lhs.x * rhs.x, lhs.y * rhs.y};
}

vec3 operator*(const vec3 &lhs, const vec3 &rhs) {
  return {lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z};
}

vec2 operator*(const vec2 &lhs, float rhs) {
  return {lhs.x * rhs, lhs.y * rhs};
}

vec3 operator*(const vec3 &lhs, float rhs) {
  return {lhs.x * rhs, lhs.y * rhs, lhs.z * rhs};
}

bool operator==(const vec2 &lhs, const vec2 &rhs) {
  return CMP(lhs.x, rhs.x) && CMP(lhs.y, rhs.y);
}

bool operator==(const vec3 &lhs, const vec3 &rhs) {
  return CMP(lhs.x, rhs.x) && CMP(lhs.y, rhs.y) && CMP(lhs.z, rhs.z);
}

bool operator!=(const vec2 &lhs, const vec2 &rhs) { return !(lhs == rhs); }

bool operator!=(const vec3 &lhs, const vec3 &rhs) { return !(lhs == rhs); }

float dot(const vec2 &lhs, const vec2 &rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

float dot(const vec3 &lhs, const vec3 &rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

float magnitude(const vec2 &vec) { return sqrtf(dot(vec, vec)); }

float magnitude(const vec3 &vec) { return sqrtf(dot(vec, vec)); }

float magnitude_sq(const vec2 &vec) { return dot(vec, vec); }

float magnitude_sq(const vec3 &vec) { return dot(vec, vec); }

void normalize(vec2 &vec) { vec = vec * (1.f * magnitude(vec)); }

void normalize(vec3 &vec) { vec = vec * (1.f * magnitude(vec)); }

vec2 normalized(const vec2 &vec) { return {vec * (1.f * magnitude(vec))}; }

vec3 normalized(const vec3 &vec) { return {vec * (1.f * magnitude(vec))}; }

vec3 cross(const vec3 &lhs, const vec3 &rhs) {
  vec3 result;
  result.x = lhs.y * rhs.z - lhs.z * rhs.y;
  result.y = lhs.z * rhs.x - lhs.x * rhs.z;
  result.z = lhs.x * rhs.y - lhs.y * rhs.z;
  return result;
}

float angle(const vec2 &lhs, const vec2 &rhs) {
  float m = sqrtf(magnitude_sq(lhs) * magnitude_sq(rhs));
  return acos(dot(lhs, rhs) / m);
}

float angle(const vec3 &lhs, const vec3 &rhs) {
  float m = sqrtf(magnitude_sq(lhs) * magnitude_sq(rhs));
  return acos(dot(lhs, rhs) / m);
}

vec2 project(const vec2 &length, const vec2 &direction) {
  float d = dot(length, direction);
  float magSq = magnitude_sq(direction);
  return direction * (d / magSq);
}

vec3 project(const vec3 &length, const vec3 &direction) {
  float d = dot(length, direction);
  float magSq = magnitude_sq(direction);
  return direction * (d / magSq);
}

vec2 perpendicular(const vec2 &len, const vec2 &dir) {
  return len - project(len, dir);
}

vec3 perpendicular(const vec3 &len, const vec3 &dir) {
  return len - project(len, dir);
}

vec2 reflection(const vec2 &vec, const vec2 &normal) {
  float d = dot(vec, normal);
  return vec - normal * (d * 2.f);
};

vec3 reflection(const vec3 &vec, const vec3 &normal) {
  float d = dot(vec, normal);
  return vec - normal * (d * 2.f);
}
