#ifndef _H_GEOMETRY_3D_
#define _H_GEOMETRY_3D_

#include "matrices.hpp"
#include "vectors.hpp"

namespace geom3D {

using Point = vec3;

struct Line {
  Point start;
  Point end;

  inline Line() {}
  inline Line(const Point &s, const Point &e) : start(s), end(e) {}
};

struct Ray {
  Point origin;
  vec3 direction;

  inline Ray() : direction{0.0f, 0.0f, 1.0f} {}
  inline Ray(const Point &o, const vec3 &dir) : origin(o), direction(dir) {
    normalize_direction();
  }
  inline void normalize_direction() { normalize(direction); }
};

struct Sphere {
  Point position;
  float radius;
  inline Sphere() : radius(1.0f) {}
  inline Sphere(const Point &p, float r) : position(p), radius(r) {}
};

struct AABB {
  Point origin;
  vec3 size;

  inline AABB() : size{1, 1, 1} {}
  inline AABB(const Point &o, const vec3 &s) : origin(o), size(s) {}
};

struct OBB {
  Point position;
  vec3 size;
  mat3 orientation;

  inline OBB() : size{1, 1, 1} {}
  inline OBB(const Point &p, vec3 &s) : position(p), size(s){};
  inline OBB(const Point &p, vec3 &s, const mat3 &o)
      : position(p), size(s), orientation(o){};
};

struct Plane {
  vec3 normal;
  float distance;

  inline Plane() : normal{1, 0, 0} {}
  inline Plane(const vec3 &n, float d) : normal{n}, distance(d) {}
};

struct Triangle {
  union {
    struct {
      Point a;
      Point b;
      Point c;
    }; // namespace geom3D
    Point points[3];
    float values[3];
  };

  inline Triangle() {}
  inline Triangle(const Point &p1, const Point &p2, const Point &p3)
      : a(p1), b(p2), c(p3) {}
};

struct Interval {
  float min;
  float max;
};

Interval get_interval(const AABB &rect, const vec3 &axis);
Interval get_interval(const OBB &obb, const vec3 &axis);
bool overlap_on_axis(const AABB &rect, const OBB &obb, const vec3 &axis);
bool overlap_on_axis(const OBB &obb1, const OBB &obb2, const vec3 &axis);

float length(const Line &line);
float length_sq(const Line &line);

Ray from_points(const Point &s, const Point &to);

vec3 get_min(const AABB &aabb);
vec3 get_max(const AABB &aabb);
AABB from_min_max(const vec3 &min, const vec3 &max);

float plane_equation(const Point &pt, const Plane &plane);

// point tests
bool point_in_sphere(const Point &point, const Sphere &sphere);
Point closest_point(const Sphere &sphere, const Point &point);
bool point_in_aabb(const Point &point, const AABB &aabb);
Point closest_point(const AABB &aabb, const Point &point);
bool point_in_obb(const Point &point, const OBB &obb);
Point closest_point(const OBB &obb, const Point &point);
bool point_on_plane(const Point &point, const Plane &plane);
Point closest_point(const Plane &plane, const Point &point);
Point closest_point(const Line &line, const Point &point);
bool point_on_line(const Point &point, const Line &line);
bool point_on_ray(const Point &point, const Ray &ray);
Point closest_point(const Ray &ray, const Point &point);

// 3D intersections
// sphere intersections
bool sphere_sphere(const Sphere &sphere1, const Sphere &sphere2);
bool sphere_AABB(const Sphere &sphere, const AABB &aabb);
bool sphere_OBB(const Sphere &sphere, const OBB &obb);
bool sphere_plane(const Sphere &sphere, const Plane &plane);
// AABB intersections
bool aabb_aabb(const AABB &aabb1, const AABB &aabb2);
bool aabb_obb(const AABB &aabb, const OBB &obb);
bool aabb_plane(const AABB &aabb, const Plane &plane);
// OBB intersections
bool obb_obb(const OBB &obb1, const OBB &obb2);
bool obb_plane(const OBB &obb, const Plane &plane);
bool plane_plane(const Plane &plane1, const Plane &plane2);

} // namespace geom3D

#endif // _H_GEOMETRY_3D_