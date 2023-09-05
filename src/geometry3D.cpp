#include "geometry3D.hpp"
#include <cmath>

#define CMP(x, y)                                                              \
  (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

namespace geom3D {
Interval get_interval(const AABB &rect, const vec3 &axis) {
  vec3 i = GetMin(aabb);
  vec3 a = GetMax(aabb);
  vec3 vertex[8] = {vec3(i.x, a.y, a.z), vec3(i.x, a.y, i.z),
                    vec3(i.x, i.y, a.z), vec3(i.x, i.y, i.z),
                    vec3(a.x, a.y, a.z), vec3(a.x, a.y, i.z),
                    vec3(a.x, i.y, a.z), vec3(a.x, i.y, i.z)};
  Interval result;
  result.min = result.max = dot(axis, vertex[0]);

  for (int i = 0; i < 8; ++i) {
    float projection = dot(axis, vertex[i]);
    result.min = (projection < result.min) ? projection : result.min;
    result.max = (projection > result.max) ? projection : result.max;
  }
  return result;
}

Interval get_interval(const OBB &obb, const vec3 &axis) {
  vec3 vertex[8];
  vec3 C = obb.position;
  vec3 E = obb.size;
  const float *o = obb.orientation.asArray;
  vec3 A[] = {vec3{o[0], o[1], o[2]}, //
              vec3{o[3], o[4], o[5]}, //
              vec3{o[6], o[7], o[8]}};

  vertex[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
  vertex[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
  vertex[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
  vertex[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
  vertex[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
  vertex[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
  vertex[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
  vertex[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

  Interval result;
  result.min = result.max = dot(axis, vertex[0]);

  for (int i = 0; i < 8; ++i) {
    float projection = dot(axis, vertex[i]);
    result.min = (projection < result.min) ? projection : result.min;
    result.max = (projection > result.max) ? projection : result.max;
  }
  return result;
}

bool overlap_on_axis(const AABB &rect, const OBB &obb, const vec3 &axis) {
  Interval a = get_interval(rect, axis);
  Interval b = get_interval(obb, axis);
  return ((b.min <= a.max) && (a.min <= b.max));
}

bool overlap_on_axis(const OBB &obb1, const OBB &obb2, const vec3 &axis) {
  Interval a = get_interval(obb1, axis);
  Interval b = get_interval(obb2, axis);

  return ((b.min <= a.max) && (a.min <= b.max));
}

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
  if (ray.direction.x + ray.direction.y + ray.direction.z > 1.0) {
    t /= dot(ray.direction, ray.direction);
  }
  t = fmaxf(t, 0.0f);
  return Point(ray.origin + ray.direction * t);
}
bool sphere_sphere(const Sphere &sphere1, const Sphere &sphere2) {
  float sumRadii = sphere1.radius + sphere2.radius;
  float sqDistance = magnitude_sq(sphere1.position - sphere2.position);

  return sqDistance < sumRadii * sumRadii;
}
bool sphere_AABB(const Sphere &sphere, const AABB &aabb) {
  Point closestPoint = closest_point(aabb, sphere.position);
  float distSq = magnitude_sq(aabb.origin, sphere.position);
  float radiusSq = sphere.radius * sphere.radius;
  return distSq < radiusSq;
}
bool sphere_OBB(const Sphere &sphere, const OBB &obb) {
  Point closestPoint = closest_point(obb.position, sphere.position);
  float distSq = magnitude_sq(sphere.position - closestPoint);
  float radiusSq = sphere.radius * sphere.radius;
  return distSq < radiusSq;
}
bool sphere_plane(const Sphere &sphere, const Plane &plane) {
  Point closestPoint = closest_point(plane, sphere.position);
  float distSq = magnitude_sq(sphere.position - closestPoint);
  float radiusSq = sphere.radius * sphere.radius;
  return distSq < radiusSq;
}
bool aabb_aabb(const AABB &aabb1, const AABB &aabb2) {
  Point aMin = get_min(aabb1);
  Point aMax = get_max(aabb1);

  Point bMin = get_min(aabb2);
  Point bMax = get_max(aabb2);

  return (aMin.x <= bMax.x && aMax.x >= bMin.x) &&
         (aMin.y <= bMax.y && aMax.y >= bMin.y) &&
         (aMin.z <= bMax.z && aMax.z >= bMin.z);
}
bool aabb_obb(const AABB &aabb, const OBB &obb) {
  const float *o = obb.orientation.asArray;

  vec3 test[15] = {vec3(1, 0, 0),          //
                   vec3(0, 1, 0),          //
                   vec3(0, 0, 1),          //
                   vec3(o[0], o[1], o[2]), //
                   vec3(o[3], o[4], o[5]), //
                   vec3(o[6], o[7], o[8])};

  for (int i = 0; i < 3; ++i) {
    test[6 + i * 3 + 0] = cross(test[i], test[0]);
    test[6 + i * 3 + 1] = cross(test[i], test[1]);
    test[6 + i * 3 + 2] = cross(test[i], test[2]);
  }
  for (int i = 0; i < 15; ++i) {
    if (!overlap_on_axis(aabb, obb, test[i])) {
      return false;
    }
  }
  return true;
}
bool aabb_plane(const AABB &aabb, const Plane &plane) {
  float pLen = aabb.size.x * fabsf(plane.normal.x) +
               aabb.size.y * fabsf(plane.normal.y) +
               aabb.size.z * fabsf(plane.normal.z);
  float d = dot(plane.normal, aabb.origin);
  float dist = dot - plane.distance;

  return fabsf(dist) <= pLen;
}
bool obb_obb(const OBB &obb1, const OBB &obb2) {
  const float *o1 = obb1.orientation.asArray;
  const float *o2 = obb2.orientation.asArray;

  vec3 test[15] = {vec3{o1[0], o1[1], o1[2]}, vec3{o1[3], o1[4], o1[5]},
                   vec3{o1[6], o1[7], o1[8]}, vec3{o2[0], o2[1], o2[2]},
                   vec3{o2[3], o2[4], o2[5]}, vec3{o2[6], o2[7], o2[8]}};

  for (int i = 0; i < 3; ++i) {
    test[6 + i * 3 + 0] = cross(test[i], test[0]);
    test[6 + i * 3 + 1] = cross(test[i], test[1]);
    test[6 + i * 3 + 2] = cross(test[i], test[2]);
  }
  for (int i = 0; i < 15; ++i) {
    if (!overlap_on_axis(obb1, obb2, test[i])) {
      return false;
    }
  }
  return true;
}
bool obb_plane(const OBB &obb, const Plane &plane) {
  const float *o = obb.orientation.asArray;
  vec3 rot[] = {vec3{o[0], o[1], o[2]}, //
                vec3{o[3], o[4], o[5]}, //
                vec3{o[6], o[7], o[8]}};
  vec3 normal = plane.normal;

  float pLen = obb.size.x * fabsf(dot(normal, rot[0])) +
               obb.size.y * fabsf(dot(normal, rot[1])) +
               obb.size.z * fabsf(dot(normal, rot[2]));
  float dist = dot(plane.normal, obb.position) - plane.distance;

  return fabsf(dist) <= pLen;
}
bool plane_plane(const Plane &plane1, const Plane &plane2) {
  vec3 d = cross(plane1.normal, plane2.normal);
  return !CMP(dot(d, d), 0);
}
float Raycast(const Sphere &sphere, const Ray &ray) {

  vec3 e = sphere.position - ray.origin;
  float rSq = sphere.radius * sphere.radius;
  float eSq = magnitude_sq(e);

  // double check dir is normalised

  float a = dot(e, normalize(ray.direction));

  float bSq = eSq - (a * a);
  float f = sqrt(rSq - bSq);
  if (rSq - (esq - (a * a)) < -0.0f) {
    return -1;
  } else if (eSq < rSq) {
    return a + f;
  }
  return a - f;
}
} // namespace geom3D
