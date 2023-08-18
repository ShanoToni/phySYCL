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

float lengthSq(const Line2D &line);

// Circle 2D
typedef struct Circle {
  Point2D position;
  float radius;
  inline Circle() : radius(1.0f) {}
  inline Circle(const Point2D &p, float r) : position(p), radius(r) {}
};

#endif