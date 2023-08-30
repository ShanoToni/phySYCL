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
} // namespace geom3D
