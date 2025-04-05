#include "geometry3D.hpp"
#include <cmath>
#include <float.h>
#include <list>

#define CMP(x, y) \
  (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

namespace geom3D
{
  void accelerate_mesh(Mesh &mesh)
  {
    if (mesh.accelerator != 0)
    {
      return;
    }

    vec3 min = mesh.vertices[0];
    vec3 max = mesh.vertices[0];

    for (int i = 1; i < mesh.numTriangles * 3; ++i)
    {
      min.x = fminf(mesh.vertices[i].x, min.x);
      max.x = fminf(mesh.vertices[i].x, max.x);
      min.y = fminf(mesh.vertices[i].y, min.y);
      max.y = fminf(mesh.vertices[i].y, max.y);
      min.z = fminf(mesh.vertices[i].z, min.z);
      max.z = fminf(mesh.vertices[i].z, max.z);
    }

    mesh.accelerator = new BVHNode();
    mesh.accelerator->bounds = from_min_max(min, max);
    mesh.accelerator->numTriangles = mesh.numTriangles;
    mesh.accelerator->triangles = new int(mesh.numTriangles);

    for (int i = 0; i < mesh.numTriangles; ++i)
    {
      mesh.accelerator->triangles[i] = i;
    }

    split_BVH_node(mesh.accelerator, mesh, 3);
  }

  void split_BVH_node(BVHNode *node, const Mesh &model, int depth)
  {
    if (depth-- == 0)
    {
      return;
    }
    if (node->children == 0)
    {
      if (node->numTriangles > 0)
      {
        node->children = new BVHNode[8];

        vec3 c = node->bounds.origin;
        vec3 e = node->bounds.size * 0.5f;

        node->children[0].bounds = AABB(c + vec3{-e.x, +e.y, -e.z}, e);
        node->children[1].bounds = AABB(c + vec3{+e.x, +e.y, -e.z}, e);
        node->children[2].bounds = AABB(c + vec3{-e.x, +e.y, +e.z}, e);
        node->children[3].bounds = AABB(c + vec3{+e.x, +e.y, +e.z}, e);
        node->children[4].bounds = AABB(c + vec3{-e.x, -e.y, -e.z}, e);
        node->children[5].bounds = AABB(c + vec3{+e.x, -e.y, -e.z}, e);
        node->children[6].bounds = AABB(c + vec3{+e.x, -e.y, -e.z}, e);
        node->children[7].bounds = AABB(c + vec3{+e.x, -e.y, +e.z}, e);

        if (node->children != 0 && node->numTriangles > 0)
        {
          for (int i = 0; i < 8; ++i)
          {
            node->children[i].numTriangles = 0;
            for (int j = 0; j < node->numTriangles; ++j)
            {
              Triangle t = model.tri[node->triangles[j]];

              if (aabb_triangle(node->children[i].bounds, t))
              {
                node->children[i].numTriangles += 1;
              }
            }

            if (node->children[i].numTriangles == 0)
            {
              continue;
            }

            node->children[i].triangles = new int[node->children[i].numTriangles];
            int index = 0;

            for (int j = 0; j < node->numTriangles; ++j)
            {
              Triangle t = model.tri[node->triangles[j]];
              if (aabb_triangle(node->children[i].bounds, t))
              {
                node->children[i].triangles[index++] = node->triangles[j];
              }
            }
          }

          node->numTriangles = 0;
          delete[] node->triangles;
          node->triangles = 0;

          for (int i = 0; i < 8; ++i)
          {
            split_BVH_node(&node->children[i], model, depth);
          }
        }
      }
    }
  }

  void free_BVH_node(BVHNode *node)
  {
    if (node->children != 0)
    {
      for (int i = 0; i < 8; ++i)
      {
        free_BVH_node((&node->children[i]));
      }
      delete[] node->children;
      node->children = 0;
    }

    if (node->numTriangles != 0 || node->triangles != 0)
    {
      delete[] node->triangles;
      node->triangles = 0;
      node->numTriangles = 0;
    }
  }

  Interval get_interval(const AABB &rect, const vec3 &axis)
  {
    vec3 i = get_min(rect);
    vec3 a = get_max(rect);
    vec3 vertex[8] = {vec3{i.x, a.y, a.z}, vec3{i.x, a.y, i.z},
                      vec3{i.x, i.y, a.z}, vec3{i.x, i.y, i.z},
                      vec3{a.x, a.y, a.z}, vec3{a.x, a.y, i.z},
                      vec3{a.x, i.y, a.z}, vec3{a.x, i.y, i.z}};
    Interval result;
    result.min = result.max = dot(axis, vertex[0]);

    for (int i = 0; i < 8; ++i)
    {
      float projection = dot(axis, vertex[i]);
      result.min = (projection < result.min) ? projection : result.min;
      result.max = (projection > result.max) ? projection : result.max;
    }
    return result;
  }

  Interval get_interval(const OBB &obb, const vec3 &axis)
  {
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

    for (int i = 0; i < 8; ++i)
    {
      float projection = dot(axis, vertex[i]);
      result.min = (projection < result.min) ? projection : result.min;
      result.max = (projection > result.max) ? projection : result.max;
    }
    return result;
  }

  Interval get_interval(const Triangle &tri, const vec3 &axis)
  {
    Interval result;

    result.min = dot(axis, tri.points[0]);
    result.max = result.min;

    for (int i = 0; i < 3; ++i)
    {
      float value = dot(axis, tri.points[i]);
      result.min = fminf(result.min, value);
      result.max = fmaxf(result.max, value);
    }
    return result;
  }

  bool overlap_on_axis(const AABB &rect, const OBB &obb, const vec3 &axis)
  {
    Interval a = get_interval(rect, axis);
    Interval b = get_interval(obb, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
  }

  bool overlap_on_axis(const OBB &obb1, const OBB &obb2, const vec3 &axis)
  {
    Interval a = get_interval(obb1, axis);
    Interval b = get_interval(obb2, axis);

    return ((b.min <= a.max) && (a.min <= b.max));
  }

  bool overlap_on_axis(const AABB &aabb, const Triangle &tri, const vec3 &axis)
  {
    Interval a = get_interval(aabb, axis);
    Interval b = get_interval(tri, axis);

    return (b.min <= a.max) && (a.min <= b.max);
  }

  bool overlap_on_axis(const OBB &obb, const Triangle &tri, const vec3 &axis)
  {
    Interval a = get_interval(obb, axis);
    Interval b = get_interval(tri, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
  }

  bool overlap_on_axis(const Triangle &tri1, const Triangle &tri2, const vec3 &axis)
  {
    Interval a = get_interval(tri1, axis);
    Interval b = get_interval(tri2, axis);

    return ((b.min <= a.max) && (a.min <= b.max));
  }

  vec3 sat_cross_edge(const vec3 &a, const vec3 &b, const vec3 &c, const vec3 &d)
  {
    vec3 ab = a - b;
    vec3 cd = c - d;
    vec3 result = cross(ab, cd);
    if (!CMP(magnitude_sq(result), 0.0f))
    {
      return result;
    }
    else
    {
      vec3 axis = cross(ab, c - a);
      result = cross(ab, axis);
      if (!CMP(magnitude_sq(result), 0.0f))
      {
        return result;
      }
    }
    return vec3();
  }

  float geom3D::length(const Line &line)
  {
    return magnitude(line.start - line.end);
  }

  float geom3D::length_sq(const Line &line)
  {
    return magnitude_sq(line.start - line.end);
  }

  Ray geom3D::from_points(const Point &s, const Point &to)
  {
    return (Ray(s, normalized(to - s)));
  }
  vec3 get_min(const AABB &aabb)
  {
    vec3 p1 = aabb.origin + aabb.size;
    vec3 p2 = aabb.origin - aabb.size;
    return vec3{fminf(p1.x, p2.x), fminf(p1.y, p2.y), fminf(p1.z, p2.z)};
  }
  vec3 get_max(const AABB &aabb)
  {
    vec3 p1 = aabb.origin + aabb.size;
    vec3 p2 = aabb.origin - aabb.size;
    return vec3{fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y), fmaxf(p1.z, p2.z)};
  }
  AABB from_min_max(const vec3 &min, const vec3 &max)
  {
    return AABB((min + max) * 0.5f, (max - min) * 0.5f);
  }
  float plane_equation(const Point &pt, const Plane &plane)
  {
    return (dot(pt, plane.normal) - plane.distance);
  }
  bool point_in_sphere(const Point &point, const Sphere &sphere)
  {
    float magSq = magnitude_sq(point - sphere.position);
    float radSq = sphere.radius * sphere.radius;

    return magSq < radSq;
  }
  Point closest_point(const Sphere &sphere, const Point &point)
  {
    vec3 sphereToPoint = point - sphere.position;
    normalize(sphereToPoint);
    sphereToPoint = sphereToPoint * sphere.radius;
    return sphereToPoint + sphere.position;
  }
  bool point_in_aabb(const Point &point, const AABB &aabb)
  {
    Point min = get_min(aabb);
    Point max = get_max(aabb);

    if (point.x < min.x || point.y < min.y || point.z < min.z)
    {
      return false;
    }
    if (point.x > max.x || point.y > max.y || point.z > max.z)
    {
      return false;
    }
    return true;
  }
  Point closest_point(const AABB &aabb, const Point &point)
  {
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
  bool point_in_obb(const Point &point, const OBB &obb)
  {
    vec3 dir = point - obb.position;
    for (int i = 0; i < 3; ++i)
    {
      const float *orientation = &obb.orientation.asArray[i * 3];
      vec3 axis{orientation[0], orientation[1], orientation[2]};
      float distance = dot(dir, axis);

      if (distance > obb.size.asArray[i])
      {
        return false;
      }
      if (distance < -obb.size.asArray[i])
      {
        return false;
      }
    }
    return true;
  }
  Point closest_point(const OBB &obb, const Point &point)
  {
    Point result = obb.position;
    vec3 dir = point - obb.position;

    for (int i = 0; i < 3; ++i)
    {
      const float *orientation = &obb.orientation.asArray[i * 3];
      vec3 axis{orientation[0], orientation[1], orientation[2]};
      float distance = dot(dir, axis);

      if (distance > obb.size.asArray[i])
      {
        distance = obb.size.asArray[i];
      }
      if (distance < -obb.size.asArray[i])
      {
        distance = -obb.size.asArray[i];
      }
      result = result + (axis * distance);
    }
    return result;
  }
  bool point_on_plane(const Point &point, const Plane &plane)
  {
    float d = dot(point, plane.normal);
    return CMP(d - plane.distance, 0.0f);
  }
  Point closest_point(const Plane &plane, const Point &point)
  {
    float d = dot(plane.normal, point);
    float distance = d - plane.distance;
    return point - plane.normal * distance;
  }
  Point geom3D::closest_point(const Line &line, const Point &point)
  {
    vec3 lVec = line.end - line.start;
    float t = dot(point - line.start, lVec) / dot(lVec, lVec);
    t = fmaxf(t, 0.0f);
    t = fminf(t, 1.0f);
    return line.start + lVec * t;
  }
  bool geom3D::point_on_line(const Point &point, const Line &line)
  {
    Point closest = closest_point(line, point);
    float distanceSq = magnitude_sq(closest - point);
    return CMP(distanceSq, 0.0f);
  }
  bool geom3D::point_on_ray(const Point &point, const Ray &ray)
  {
    if (point == ray.origin)
    {
      return true;
    }

    vec3 norm = point - ray.origin;
    normalize(norm);
    float diff = dot(norm, ray.direction);
    return CMP(diff, 0.0f);
  }

  Point geom3D::closest_point(const Ray &ray, const Point &point)
  {
    float t = dot(point - ray.origin, ray.direction);
    if (ray.direction.x + ray.direction.y + ray.direction.z > 1.0)
    {
      t /= dot(ray.direction, ray.direction);
    }
    t = fmaxf(t, 0.0f);
    return Point(ray.origin + ray.direction * t);
  }
  bool point_in_triagle(const Triangle &tri, const Point &point)
  {
    vec3 a = tri.a - point;
    vec3 b = tri.b - point;
    vec3 c = tri.c - point;

    vec3 normPBC = cross(b, c);
    vec3 normPCA = cross(c, a);
    vec3 normPAB = cross(a, b);

    if (dot(normPBC, normPCA) < 0.0f)
    {
      return false;
    }
    else if (dot(normPBC, normPAB) < 0.0f)
    {
      return false;
    }

    return true;
  }

  Plane from_triangle(const Triangle &tri)
  {
    Plane result;

    result.normal = normalized(cross(tri.b - tri.a, tri.c - tri.a));
    result.distance = dot(result.normal, tri.a);

    return result;
  }

  Point closest_point(const Triangle &tri, const Point &p)
  {
    Plane plane = from_triangle(tri);
    Point closest = closest_point(plane, p);

    if (point_in_triagle(tri, closest))
    {
      return closest;
    }

    Point c1 = closest_point(Line(tri.a, tri.b), p);
    Point c2 = closest_point(Line(tri.b, tri.c), p);
    Point c3 = closest_point(Line(tri.c, tri.a), p);

    float magSq1 = magnitude_sq(p - c1);
    float magSq2 = magnitude_sq(p - c2);
    float magSq3 = magnitude_sq(p - c3);

    if (magSq1 < magSq2 && magSq1 < magSq3)
    {
      return c1;
    }
    else if (magSq2 < magSq1 && magSq2 < magSq3)
    {
      return c2;
    }
    return c3;
  }

  bool sphere_sphere(const Sphere &sphere1, const Sphere &sphere2)
  {
    float sumRadii = sphere1.radius + sphere2.radius;
    float sqDistance = magnitude_sq(sphere1.position - sphere2.position);

    return sqDistance < sumRadii * sumRadii;
  }
  bool sphere_AABB(const Sphere &sphere, const AABB &aabb)
  {
    Point closestPoint = closest_point(aabb, sphere.position);
    float distSq = magnitude_sq(sphere.position - aabb.origin);
    float radiusSq = sphere.radius * sphere.radius;
    return distSq < radiusSq;
  }
  bool sphere_OBB(const Sphere &sphere, const OBB &obb)
  {
    Point closestPoint = closest_point(obb, sphere.position);
    float distSq = magnitude_sq(sphere.position - closestPoint);
    float radiusSq = sphere.radius * sphere.radius;
    return distSq < radiusSq;
  }
  bool sphere_plane(const Sphere &sphere, const Plane &plane)
  {
    Point closestPoint = closest_point(plane, sphere.position);
    float distSq = magnitude_sq(sphere.position - closestPoint);
    float radiusSq = sphere.radius * sphere.radius;
    return distSq < radiusSq;
  }
  bool sphere_triangle(const Sphere &sphere, const Triangle &tri)
  {
    Point closest = closest_point(tri, sphere.position);
    float magSq = magnitude_sq(closest - sphere.position);

    return magSq <= sphere.radius * sphere.radius;
  }
  bool aabb_aabb(const AABB &aabb1, const AABB &aabb2)
  {
    Point aMin = get_min(aabb1);
    Point aMax = get_max(aabb1);

    Point bMin = get_min(aabb2);
    Point bMax = get_max(aabb2);

    return (aMin.x <= bMax.x && aMax.x >= bMin.x) &&
           (aMin.y <= bMax.y && aMax.y >= bMin.y) &&
           (aMin.z <= bMax.z && aMax.z >= bMin.z);
  }
  bool aabb_obb(const AABB &aabb, const OBB &obb)
  {
    const float *o = obb.orientation.asArray;

    vec3 test[15] = {vec3{1, 0, 0},          //
                     vec3{0, 1, 0},          //
                     vec3{0, 0, 1},          //
                     vec3{o[0], o[1], o[2]}, //
                     vec3{o[3], o[4], o[5]}, //
                     vec3{o[6], o[7], o[8]}};

    for (int i = 0; i < 3; ++i)
    {
      test[6 + i * 3 + 0] = cross(test[i], test[0]);
      test[6 + i * 3 + 1] = cross(test[i], test[1]);
      test[6 + i * 3 + 2] = cross(test[i], test[2]);
    }
    for (int i = 0; i < 15; ++i)
    {
      if (!overlap_on_axis(aabb, obb, test[i]))
      {
        return false;
      }
    }
    return true;
  }
  bool aabb_plane(const AABB &aabb, const Plane &plane)
  {
    float pLen = aabb.size.x * fabsf(plane.normal.x) +
                 aabb.size.y * fabsf(plane.normal.y) +
                 aabb.size.z * fabsf(plane.normal.z);
    float d = dot(plane.normal, aabb.origin);
    float dist = d - plane.distance;

    return fabsf(dist) <= pLen;
  }
  bool aabb_triangle(const AABB &aabb, const Triangle &tri)
  {
    vec3 f0 = tri.b - tri.a;
    vec3 f1 = tri.c - tri.b;
    vec3 f2 = tri.a - tri.c;

    vec3 u0{1.0f, 0.0f, 0.0f};
    vec3 u1{0.0f, 1.0f, 0.0f};
    vec3 u2{0.0f, 0.0f, 1.0f};

    vec3 test[13] = {
        u0,
        u1,
        u2,
        cross(f0, f1),
        cross(u0, f0), cross(u0, f1), cross(u0, f2),
        cross(u1, f0), cross(u1, f1), cross(u1, f2),
        cross(u2, f0), cross(u2, f1), cross(u2, f2)};

    for (int i = 0; i < 13; ++i)
    {
      if (!overlap_on_axis(aabb, tri, test[i]))
        return false;
    }

    return true;
  }
  bool obb_obb(const OBB &obb1, const OBB &obb2)
  {
    const float *o1 = obb1.orientation.asArray;
    const float *o2 = obb2.orientation.asArray;

    vec3 test[15] = {vec3{o1[0], o1[1], o1[2]}, vec3{o1[3], o1[4], o1[5]},
                     vec3{o1[6], o1[7], o1[8]}, vec3{o2[0], o2[1], o2[2]},
                     vec3{o2[3], o2[4], o2[5]}, vec3{o2[6], o2[7], o2[8]}};

    for (int i = 0; i < 3; ++i)
    {
      test[6 + i * 3 + 0] = cross(test[i], test[0]);
      test[6 + i * 3 + 1] = cross(test[i], test[1]);
      test[6 + i * 3 + 2] = cross(test[i], test[2]);
    }
    for (int i = 0; i < 15; ++i)
    {
      if (!overlap_on_axis(obb1, obb2, test[i]))
      {
        return false;
      }
    }
    return true;
  }
  bool obb_plane(const OBB &obb, const Plane &plane)
  {
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

  bool obb_triangle(const OBB &obb, const Triangle &tri)
  {
    vec3 f0 = tri.b - tri.a;
    vec3 f1 = tri.c - tri.b;
    vec3 f2 = tri.a - tri.c;

    const float *orientation = obb.orientation.asArray;
    vec3 u0{orientation[0], orientation[1], orientation[2]};
    vec3 u1{orientation[2], orientation[3], orientation[4]};
    vec3 u2{orientation[6], orientation[7], orientation[8]};

    vec3 test[13] = {
        u0,
        u1,
        u2,
        cross(f0, f1),
        cross(u0, f0), cross(u0, f1), cross(u0, f2),
        cross(u1, f0), cross(u1, f1), cross(u1, f2),
        cross(u2, f0), cross(u2, f1), cross(u2, f2)};

    for (int i = 0; i < 13; ++i)
    {
      if (!overlap_on_axis(obb, tri, test[i]))
      {
        return false;
      }
    }
    return true;
  }

  bool plane_plane(const Plane &plane1, const Plane &plane2)
  {
    vec3 d = cross(plane1.normal, plane2.normal);
    return !CMP(dot(d, d), 0);
  }

  bool plane_triangle(const Plane &plane, const Triangle &tri)
  {
    float side1 = plane_equation(tri.a, plane);
    float side2 = plane_equation(tri.b, plane);
    float side3 = plane_equation(tri.c, plane);

    if (CMP(side1, 0.0f) && CMP(side2, 0.0f) && CMP(side3, 0.0f))
    {
      return true;
    }

    if (side1 > 0 && side2 > 0 && side3 > 0)
    {
      return false;
    }

    if (side1 < 0 && side2 < 0 && side3 < 0)
    {
      return false;
    }

    return true;
  }

  bool triangle_triangle(const Triangle &tri1, const Triangle tri2)
  {
    vec3 t1_f0 = tri1.b - tri1.a;
    vec3 t1_f1 = tri1.c - tri1.b;
    vec3 t1_f2 = tri1.a - tri1.c;

    vec3 t2_f0 = tri2.b - tri2.a;
    vec3 t2_f1 = tri2.c - tri2.b;
    vec3 t2_f2 = tri2.a - tri2.c;

    vec3 axisToTest[] = {
        cross(t1_f0, t1_f1),
        cross(t2_f0, t2_f1),
        cross(t2_f0, t1_f0), cross(t2_f0, t1_f1),
        cross(t2_f0, t1_f2), cross(t2_f1, t1_f0),
        cross(t2_f1, t1_f1), cross(t2_f1, t1_f2),
        cross(t2_f2, t1_f0), cross(t2_f2, t1_f1),
        cross(t2_f2, t1_f2)};

    for (int i = 0; i < 11; ++i)
    {
      if (!overlap_on_axis(tri1, tri2, axisToTest[i]))
      {
        return false;
      }
    }

    return true;
  }

  bool triangle_triangle_robust(const Triangle &tri1, const Triangle tri2)
  {

    vec3 axisToTest[] = {
        sat_cross_edge(tri1.a, tri1.b, tri1.b, tri1.c),
        sat_cross_edge(tri2.a, tri2.b, tri2.b, tri2.c),
        sat_cross_edge(tri2.a, tri2.b, tri1.a, tri1.b),
        sat_cross_edge(tri2.a, tri2.b, tri1.b, tri1.c),
        sat_cross_edge(tri2.a, tri2.b, tri1.c, tri1.a),
        sat_cross_edge(tri2.b, tri2.c, tri1.a, tri1.b),
        sat_cross_edge(tri2.b, tri2.c, tri1.b, tri1.c),
        sat_cross_edge(tri2.b, tri2.c, tri1.c, tri1.a),
        sat_cross_edge(tri2.c, tri2.a, tri1.a, tri1.b),
        sat_cross_edge(tri2.c, tri2.a, tri1.b, tri1.c),
        sat_cross_edge(tri2.c, tri2.a, tri1.c, tri1.a)};

    for (int i = 0; i < 11; ++i)
    {
      if (!overlap_on_axis(tri1, tri2, axisToTest[i]))
      {
        return false;
      }
    }
    return true;
  }

  bool mesh_aabb(const Mesh &mesh, const AABB &aabb)
  {
    if (mesh.accelerator == 0)
    {
      for (int i = 0; i < mesh.numTriangles; ++i)
      {
        if (aabb_triangle(aabb, mesh.tri[i]))
        {
          return true;
        }
      }
    }
    else
    {
      std::list<BVHNode *> toProcess;
      toProcess.push_front(mesh.accelerator);
      while (!toProcess.empty())
      {
        BVHNode *iterator = *(toProcess.begin());
        toProcess.erase(toProcess.begin());
        if (iterator->numTriangles >= 0)
        {
          for (int i = 0; i < iterator->numTriangles; ++i)
          {
            if (aabb_triangle(aabb, mesh.tri[iterator->triangles[i]]))
            {
              return true;
            }
          }
        }
        if (iterator->children != 0)
        {
          for (int i = 8 - 1; i >= 0; --i)
          {
            if (aabb_aabb(iterator->children->bounds, aabb))
            {
              toProcess.push_front(&iterator->children[i]);
            }
          }
        }
      }
    }
    return false;
  }

  float raycast(const Sphere &sphere, const Ray &ray)
  {

    vec3 e = sphere.position - ray.origin;
    float rSq = sphere.radius * sphere.radius;
    float eSq = magnitude_sq(e);

    // double check dir is normalised

    float a = dot(e, normalized(ray.direction));

    float bSq = eSq - (a * a);
    float f = sqrt(rSq - bSq);
    if (rSq - (eSq - (a * a)) < -0.0f)
    {
      return -1;
    }
    else if (eSq < rSq)
    {
      return a + f;
    }
    return a - f;
  }
  float raycast(const AABB &aabb, const Ray &ray)
  {
    vec3 min = get_min(aabb);
    vec3 max = get_max(aabb);
    float t1 = (min.x - ray.origin.x) / ray.direction.x;
    float t2 = (max.x - ray.origin.x) / ray.direction.x;
    float t3 = (min.y - ray.origin.y) / ray.direction.y;
    float t4 = (max.y - ray.origin.y) / ray.direction.y;
    float t5 = (min.z - ray.origin.z) / ray.direction.z;
    float t6 = (max.z - ray.origin.z) / ray.direction.z;

    float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
    float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));

    if (tmax < 0)
    {
      return -1;
    }
    if (tmin > tmax)
    {
      return -1;
    }
    if (tmin < 0.0f)
    {
      return tmax;
    }
    return tmin;
  }
  float raycast(const OBB &obb, const Ray &ray)
  {
    const float *o = obb.orientation.asArray;
    const float *size = obb.size.asArray;
    vec3 x{o[0], o[1], o[2]};
    vec3 y{o[3], o[4], o[5]};
    vec3 z{o[6], o[7], o[8]};

    vec3 p = obb.position - ray.origin;

    vec3 f{dot(x, ray.direction), dot(y, ray.direction), dot(z, ray.direction)};
    vec3 e{dot(x, p), dot(y, p), dot(z, p)};
    float t[6];
    for (int i = 0; i < 3; ++i)
    {
      if (CMP(f[i], 0))
      {
        if (-e[i] - size[i] > 0 || -e[i] + size[i] < 0)
        {
          return -1;
        }
        f[i] = 0.00001f; // so PC does not go boom!
        t[i * 2] = (e[i] + size[i] / f[i]);
        t[i * 2 + 1] = (e[i] - size[i] / f[i]);
      }
    }
    float tmin = fmaxf(fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), fminf(t[4], t[5]));
    float tmax = fminf(fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])), fmaxf(t[4], t[5]));

    if (tmax < 0)
    {
      return -1;
    }
    if (tmin > tmax)
    {
      return -1;
    }
    if (tmin < 0.0f)
    {
      return tmax;
    }
    return tmin;
  }

  float raycast(const Plane &plane, const Ray &ray)
  {
    float nd = dot(ray.direction, plane.normal);
    float pn = dot(ray.origin, plane.normal);

    if (nd >= 0.0f)
    {
      return -1;
    }
    float t = (plane.distance - pn) / nd;

    if (t >= 0.0f)
    {
      return t;
    }
    return -1;
  }

  float raycast(const Mesh &mesh, const Ray &ray)
  {
    if (mesh.accelerator == 0)
    {
      for (int i = 0; i < mesh.numTriangles; ++i)
      {
        float result = raycast(mesh.tri[i], ray);
        if (result >= 0)
        {
          return result;
        }
      }
    }
    else
    {
      std::list<BVHNode *> toProcess;
      toProcess.push_front(mesh.accelerator);
      while (!toProcess.empty())
      {
        BVHNode *iterator = *(toProcess.begin());
        toProcess.erase(toProcess.begin());
        if (iterator->numTriangles >= 0)
        {
          for (int i = 0; i < iterator->numTriangles; ++i)
          {
            float r = raycast(mesh.tri[iterator->triangles[i]], ray);
            if (r >= 0)
            {
              return r;
            }
          }
        }
        if (iterator->children != 0)
        {
          for (int i = 8 - 1; i >= 0; --i)
          {
            if (raycast(iterator->children[i].bounds, ray) >= 0)
            {
              toProcess.push_front(&iterator->children[i]);
            }
          }
        }
      }
    }
    return -1;
  }

  vec3 barycentric(const Point &point, const Triangle &tri)
  {
    vec3 ap = point - tri.a;
    vec3 bp = point - tri.b;
    vec3 cp = point - tri.c;

    vec3 ab = tri.b - tri.a;
    vec3 ac = tri.c - tri.a;
    vec3 bc = tri.c - tri.b;
    vec3 cb = tri.b - tri.c;
    vec3 ca = tri.a - tri.c;

    vec3 v = ab - project(ab, cb);
    float a = 1.0f - (dot(v, ap) / dot(v, ab));

    v = bc - project(bc, ac);
    float b = 1.0f - (dot(v, ap) / dot(v, ab));

    v = bc - project(ca, ab);
    float c = 1.0f - (dot(v, cp) / dot(v, ca));

    return vec3{a, b, c};
  }

  float raycast(const Triangle &tri, const Ray &ray)
  {
    Plane plane = from_triangle(tri);
    float t = raycast(plane, ray);
    if (t < 0.0f)
    {
      return t;
    }
    Point result = ray.origin + ray.direction * t;

    vec3 bary = barycentric(result, tri);
    if (bary.x >= 0.0f && bary.x <= 1.0f &&
        bary.y >= 0.0f && bary.y <= 1.0f &&
        bary.z >= 0.0f && bary.z <= 1.0f)
    {
      return t;
    }
    return -1.0f;
  }

  bool linetest(const Sphere &sphere, const Line &line)
  {

    Point closest = closest_point(line, sphere.position);
    float distSq = magnitude_sq(sphere.position - closest);
    return distSq <= (sphere.radius * sphere.radius);
  }
  bool linetest(const AABB &aabb, const Line &line)
  {
    Ray lineRay;
    lineRay.origin = line.start;
    lineRay.direction = normalized(line.end - line.start);
    float t = raycast(aabb, lineRay);

    return t >= 0 && t * t <= length_sq(line);
  }
  bool linetest(const OBB &obb, const Line &line)
  {
    Ray lineRay;
    lineRay.origin = line.start;
    lineRay.direction = normalized(line.end - line.start);
    float t = raycast(obb, lineRay);

    return t >= 0 && t * t <= length_sq(line);
  }
  bool linetest(const Plane &plane, const Line &line)
  {
    vec3 ab = line.end - line.start;
    float nA = dot(plane.normal, line.start);
    float nAB = dot(plane.normal, ab);

    float t = (plane.distance - nA) / nAB;
    return t >= 0.0f && t <= 1.0f;
  }
  bool linetest(const Triangle &tri, const Line &line)
  {
    Ray ray;
    ray.origin = line.start;
    ray.direction = normalized(line.end - line.start);

    float t = raycast(tri, ray);

    return t >= 0 && t * t <= length_sq(line);
  }
} // namespace geom3D
