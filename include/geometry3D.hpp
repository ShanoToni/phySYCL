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
} // namespace geom3D

#endif // _H_GEOMETRY_3D_