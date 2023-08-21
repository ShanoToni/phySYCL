#ifndef _H_GEOMETRY_2D_
#define _H_GEOMETRY_2D_

#include "vectors.hpp"

typedef vec2 Point2D;

// Line 2D
typedef struct Line2D {
  Point2D start;
  Point2D end;

  inline Line2D() {}
  inline Line2D(const Point2D &s, const Point2D &e) : start(s), end(e) {}
} Line2D;

float length(const Line2D &line);

float length_sq(const Line2D &line);

// Circle 2D
typedef struct Circle {
  Point2D position;
  float radius;
  inline Circle() : radius(1.0f) {}
  inline Circle(const Point2D &p, float r) : position(p), radius(r) {}
};

// Rectangle
typedef struct Rectangle2D {
  Point2D origin;
  vec2 size;

  Rectangle2D() : size({1, 1}) {}
  Rectangle2D(const Point2D &o, const vec2 &s) : origin(o), size(s) {}
} Rectangle2D;

vec2 get_min(const Rectangle2D &rec);
vec2 get_max(const Rectangle2D &rec);

Rectangle2D from_min_max(const vec2 &min, const vec2 &max);

// Oriented rectangle
typedef struct OrientedRectangle {
  Point2D position;
  vec2 halfExtents;
  float rotation;

  OrientedRectangle() : halfExtents({1, 1}), rotation(0.0f) {}

  OrientedRectangle(const Point2D &p, const vec2 &h_e)
      : position(p), halfExtents(h_e) {}

  OrientedRectangle(const Point2D &p, const vec2 &h_e, float rot)
      : position(p), halfExtents(h_e), rotation(rot) {}
} OrientedRectangle;

// Intersections
bool point_on_line(const Point2D &p, const Line2D &line);
bool point_in_circle(const Point2D &p, const Circle &c);
bool point_in_rectangle(const Point2D &p, const Rectangle2D &rec);
bool point_in_oriented_rectangle(const Point2D &p,
                                 const OrientedRectangle &or_rec);

#endif