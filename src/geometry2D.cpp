#include "geometry2D.hpp"
#include "matrices.hpp"
#include <cfloat>
#include <cmath>

#define CMP(x, y)                                                              \
  (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

float length(const Line2D &line) { return magnitude(line.end - line.start); }

float length_sq(const Line2D &line) {
  return magnitude_sq(line.end - line.start);
}

vec2 get_min(const Rectangle2D &rec) {
  vec2 p1 = rec.origin;
  vec2 p2 = rec.origin + rec.size;

  return vec2{fminf(p1.x, p2.x), fminf(p1.y, p2.y)};
}

vec2 get_max(const Rectangle2D &rec) {
  vec2 p1 = rec.origin;
  vec2 p2 = rec.origin + rec.size;

  return vec2{fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y)};
}

Rectangle2D from_min_max(const vec2 &min, const vec2 &max) {
  return Rectangle2D{min, max - min};
}

bool point_on_line(const Point2D &p, const Line2D &line) {
  float dy = (line.end.y - line.start.y);
  float dx = (line.end.x - line.start.x);
  float M = dy / dx;
  float B = line.start.y - M * line.start.x;
  return CMP(p.y, M * p.x + B);
}

bool point_in_circle(const Point2D &p, const Circle &c) {
  Line2D line(p, c.position);
  if (length_sq(line) < c.radius * c.radius) {
    return true;
  }
  return false;
}

bool point_in_rectangle(const Point2D &p, const Rectangle2D &rec) {
  vec2 min = get_min(rec);
  vec2 max = get_max(rec);

  return min.x <= p.x && min.y <= p.y && p.x <= max.x && p.y <= max.y;
}

bool point_in_oriented_rectangle(const Point2D &p,
                                 const OrientedRectangle &or_rec) {
  vec2 rotVec = p - or_rec.position;
  float theta = -DEG2RAD(or_rec.rotation);
  float zRotation2x2[] = {cosf(theta), sinf(theta), //
                          -sinf(theta), cosf(theta)};

  Rectangle2D localRectangle(Point2D(), or_rec.halfExtents * 2.0f);
  vec2 localPoint = rotVec + or_rec.halfExtents;
  return point_in_rectangle(localPoint, localRectangle);
}
