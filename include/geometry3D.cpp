#include "geometry3D.hpp"
#include <cmath>
namespace geom3D {

float geom3D::length(const Line &line) {
  return magnitude(line.start - line.end);
}

float geom3D::length_sq(const Line &line) {
  return magnitude_sq(line.start - line.end);
}

Ray geom3D::from_points(const Point &s, const Point &to) {
  return (Ray(s, normalized(to - s)));
}
vec3 get_min(const AABB &aabb) {
  vec3 p1 = aabb.origin + aabb.size;
  vec3 p2 = aabb.origin - aabb.size;
  return vec3{fminf(p1.x, p2.x), fminf(p1.y, p2.y), fminf(p1.z, p2.z)};
}
vec3 get_max(const AABB &aabb) {
  vec3 p1 = aabb.origin + aabb.size;
  vec3 p2 = aabb.origin - aabb.size;
  return vec3{fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y), fmaxf(p1.z, p2.z)};
}
AABB from_min_max(const vec3 &min, const vec3 &max) {
  return AABB((min + max) * 0.5f, (max - min) * 0.5f);
}
float plane_equation(const Point &pt, const Plane &plane) {
  return (dot(pt, plane.normal) - plane.distance);
}
bool point_in_sphere(const Point &point, const Sphere &sphere) {
  float magSq = magnitude_sq(point - sphere.position);
  float radSq = sphere.radius * sphere.radius;

  return magSq < radSq;
}
Point closest_point(const Sphere &sphere, const Point &point) {
  vec3 sphereToPoint = point - sphere.position;
  normalize(sphereToPoint);
  sphereToPoint = sphereToPoint * sphere.radius;
  return sphereToPoint + sphere.position;
}
bool point_in_aabb(const Point &point, const AABB &aabb) {
  Point min = get_min(aabb);
  Point max = get_max(aabb);

  if (point.x < min.x || point.y < min.y || point.z < min.z) {
    return false;
  }
  if (point.x > max.x || point.y > max.y || point.z > max.z) {
    return false;
  }
  return true;
}
Point closest_point(const AABB &aabb, const Point &point) {
  Point result = point;
  Point min = get_min(aabb);
  Point max = get_max(aabb);

  result.x = (result.x < min.x) ? min.x : result.x;
  result.y = (result.y < min.y) ? min.y : result.y;
  result.z = (result.z < min.z) ? min.z : result.z;

  result.x = (result.x > max.x) ? max.x : result.x;
  result.y = (result.y > max.y) ? max.y : result.y;
  result.z = (result.z > max.z) ? max.z : result.z;

  return result;
}
bool point_in_obb(const Point &point, const OBB &obb) {
  vec3 dir = point - obb.position;
  for (int i = 0; i < 3; ++i) {
    const float *orientation = &obb.orientation.asArray[i * 3];
    vec3 axis{orientation[0], orientation[1], orientation[2]};
    float distance = dot(dir, axis);

    if (distance > obb.size.asArray[i]) {
      return false;
    }
    if (distance < -obb.size.asArray[i]) {
      return false;
    }
  }
  return true;
}
Point closest_point(const OBB &obb, const Point &point) {
  Point result = obb.position;
  vec3 dir = point - obb.position;

  for (int i = 0; i < 3; ++i) {
    const float *orientation = &obb.orientation.asArray[i * 3];
    vec3 axis{orientation[0], orientation[1], orientation[2]};
    float distance = dot(dir, axis);

    if (distance > obb.size.asArray[i]) {
      distance = obb.size.asArray[i];
    }
    if (distance < -obb.size.asArray[i]) {
      distance = -obb.size.asArray[i];
    }
    result = result + (axis * distance);
  }
  return result;
}
} // namespace geom3D
