#include "geometry3D.hpp"
#include <cmath>

#define CMP(x, y)                                                              \
  (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

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
bool point_on_plane(const Point &point, const Plane &plane) {
  float d = dot(point, plane.normal);
  return CMP(d - plane.distance, 0.0f);
}
Point closest_point(const Plane &plane, const Point &point) {
  float d = dot(plane.normal, point);
  float distance = d - plane.distance;
  return point - plane.normal * distance;
}
Point geom3D::closest_point(const Line &line, const Point &point) {
  vec3 lVec = line.end - line.start;
  float t = dot(point - line.start, lVec) / dot(lVec, lVec);
  t = fmaxf(t, 0.0f);
  t = fminf(t, 1.0f);
  return line.start + lVec * t;
}
bool geom3D::point_on_line(const Point &point, const Line &line) {
  Point closest = closest_point(line, point);
  float distanceSq = magnitude_sq(closest - point);
  return CMP(distanceSq, 0.0f);
}
bool geom3D::point_on_ray(const Point &point, const Ray &ray) {
  if (point == ray.origin) {
    return true;
  }

  vec3 norm = point - ray.origin;
  normalize(norm);
  float diff = dot(norm, ray.direction);
  return CMP(diff, 0.0f);
}

Point geom3D::closest_point(const Ray &ray, const Point &point) {
  float t = dot(point - ray.origin, ray.direction);
  // assuming ray.direction is normalized
  t = fmaxf(t, 0.0f);
  return Point(ray.origin + ray.direction * t);
}
} // namespace geom3D
