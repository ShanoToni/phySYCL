#ifndef _H_GEOMETRY_2D_
#define _H_GEOMETRY_2D_

#include "vectors.hpp"

namespace geom2D {

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

typedef struct Interval2D {
  float min;
  float max;
} Interval2D;

Interval2D get_interval(const Rectangle2D &rec, const vec2 &axis);
Interval2D get_interval(const OrientedRectangle &rec, const vec2 &axis);
bool overlap_on_axis(const Rectangle2D &rec1, const Rectangle2D &rec2,
                     const vec2 &axis);
bool overlap_on_axis(const Rectangle2D &rec, const OrientedRectangle &or_rec,
                     const vec2 &axis);

// Intersections
bool point_on_line(const Point2D &p, const Line2D &line);
bool point_in_circle(const Point2D &p, const Circle &c);
bool point_in_rectangle(const Point2D &p, const Rectangle2D &rec);
bool point_in_oriented_rectangle(const Point2D &p,
                                 const OrientedRectangle &or_rec);

bool line_circle(const Line2D &l, const Circle &c);
bool line_rectangle(const Line2D &l, const Rectangle2D &rec);
bool line_oriented_rectangle(const Line2D &l, const OrientedRectangle &or_rec);

// Collision
bool circle_circle(const Circle &c1, const Circle &c2);
bool circle_rectangle(const Circle &c, const Rectangle2D &rec);
bool circle_or_rectangle(const Circle &c, const OrientedRectangle &or_rec);

bool rectangle_rectangle(const Rectangle2D &rec1, const Rectangle2D &rec2);
bool rec_rec_sat(const Rectangle2D &rec1, const Rectangle2D &rec2);
bool rectangle_or_rectangle(const Rectangle2D &rec,
                            const OrientedRectangle &or_rec);

} // namespace geom2D
#endif