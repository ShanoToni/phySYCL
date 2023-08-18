#include "geometry2D.hpp"
#include "matrices.hpp"
#include <cfloat>
#include <cmath>

#define CMP(x, y)                                                              \
  (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

float length(const Line2D &line) { return magnitude(line.end - line.start); }

float lengthSq(const Line2D &line) {
  return magnitude_sq(line.end - line.start);
}
