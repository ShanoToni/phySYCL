#include "geometry2D.hpp"
#include "matrices.hpp"
#include <cfloat>
#include <cmath>

#define CMP(x, y)                                                              \
  (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

#define CLAMP(number, minimum, maximum)                                        \
  number =                                                                     \
      (number < minimum) ? minimum : ((number > maximum) ? maximum : number)

#define OVERLAP(aMin, aMax, bMin, bMax) ((bMin <= aMax) && (aMin <= bMax))

namespace geom2D {

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

Interval2D get_interval(const Rectangle2D &rec, const vec2 &axis) {
  Interval2D result;
  vec2 min = get_min(rec);
  vec2 max = get_max(rec);

  vec2 verts[] = {vec2{min.x, min.y}, vec2{min.x, max.y}, //
                  vec2{max.x, max.y}, vec2{max.x, min.y}};

  result.min = result.max = dot(axis, verts[0]);

  for (int i = 1; i < 4; ++i) {
    float proj = dot(axis, verts[i]);
    if (proj < result.min) {
      result.min = proj;
    }
    if (proj > result.max) {
      result.max = proj;
    }
  }
  return result;
}

bool overlap_on_axis(const Rectangle2D &rec1, const Rectangle2D &rec2,
                     const vec2 &axis) {
  Interval2D a = get_interval(rec1, axis);
  Interval2D b = get_interval(rec2, axis);
  return ((b.min <= a.max) && (a.min <= b.max));
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

bool line_circle(const Line2D &l, const Circle &c) {
  vec2 ab = l.end - l.start;
  float t = dot(c.position - l.start, ab) / dot(ab, ab);
  if (t < 0.0f || t > 1.0f) {
    return false;
  }

  Point2D closest = l.start + ab * t;

  Line2D circleToClosest(c.position, closest);
  return length_sq(circleToClosest) < c.radius * c.radius;
}

bool line_rectangle(const Line2D &l, const Rectangle2D &rec) {
  if (point_in_rectangle(l.start, rec) || point_in_rectangle(l.end, rec)) {
    return true;
  }
  vec2 norm = normalized(l.end - l.start);
  norm.x = (norm.x != 0) ? 1.f / norm.x : 0;

  norm.y = (norm.y != 0) ? 1.0f / norm.y : 0;
  vec2 min = (get_min(rec) - l.start) * norm;
  vec2 max = (get_max(rec) - l.start) * norm;

  float tmin = fmaxf(fminf(min.x, max.x), fminf(min.y, max.y));
  float tmax = fminf(fmaxf(min.x, max.x), fmaxf(min.y, max.y));

  if (tmax < 0 || tmin > tmax) {
    return false;
  }
  float t = (tmin < 0.0f) ? tmax : tmin;
  return t > 0.0f && t * t < length_sq(l);
}

bool line_oriented_rectangle(const Line2D &l, const OrientedRectangle &or_rec) {
  float theta = -DEG2RAD(or_rec.rotation);
  float zRotation2x2[] = {cosf(theta), sinf(theta), //
                          -sinf(theta), cosf(theta)};

  Line2D localLine;

  vec2 rotVec = l.start - or_rec.position;
  multiply(rotVec.asArray, vec2{rotVec.x, rotVec.y}.asArray, 1, 2, zRotation2x2,
           2, 2);

  localLine.end = rotVec + or_rec.halfExtents;

  Rectangle2D localRec(Point2D(), or_rec.halfExtents * 2.0f);
  return line_rectangle(localLine, localRec);
}

bool circle_circle(const Circle &c1, const Circle &c2) {
  Line2D l(c1.position, c2.position);

  float radSum = c1.radius + c2.radius;

  return length_sq(l) <= radSum * radSum;
}

bool circle_rectangle(const Circle &c, const Rectangle2D &rec) {
  vec2 min = get_min(rec);
  vec2 max = get_max(rec);

  Point2D closestPoint = c.position;
  CLAMP(closestPoint.x, min.x, max.x);
  CLAMP(closestPoint.y, min.y, max.y);

  Line2D line(c.position, closestPoint);
  return length_sq(line) <= c.radius * c.radius;
}

bool circle_or_rectangle(const Circle &c, const OrientedRectangle &or_rec) {
  vec2 r = c.position - or_rec.position;

  float theta = -DEG2RAD(or_rec.rotation);
  float zRot2x2[] = {cosf(theta), sinf(theta), //
                     -sinf(theta), cosf(theta)};

  multiply(r.asArray, vec2{r.x, r.y}.asArray, 1, 2, zRot2x2, 2, 2);
  Circle locCircle(r + or_rec.halfExtents, c.radius);
  Rectangle2D locRec(Point2D(), or_rec.halfExtents * 2);

  return circle_rectangle(locCircle, locRec);
}

bool rectangle_rectangle(const Rectangle2D &rec1, const Rectangle2D &rec2) {
  vec2 aMin = get_min(rec1);
  vec2 aMax = get_max(rec1);

  vec2 bMin = get_min(rec2);
  vec2 bMax = get_max(rec2);

  bool overX = OVERLAP(aMin.x, aMax.x, bMin.x, bMax.x);
  bool overY = OVERLAP(aMin.y, aMax.y, bMin.y, bMax.y);

  return overX && overY;
}

bool rec_rec_sat(const Rectangle2D &rec1, const Rectangle2D &rec2) {
  vec2 axisToTest[] = {vec2{1, 0}, vec2{0, 1}};

  for (int i = 0; i < 2; ++i) {
    if (!overlap_on_axis(rec1, rec2, axisToTest[i])) {
      return false;
    }
  }
  return true;
}

} // namespace geom2D
