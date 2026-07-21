// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "surface_route.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace mujoco::plugin::cable {
namespace {

constexpr mjtNum kGeometryTolerance = 1e-8;
constexpr mjtNum kCylinderEndTolerance = 1e-6;
constexpr mjtNum kMinimumWrapAngle = 1e-5;
constexpr mjtNum kMaximumWrapAngle = 2 * mjPI - kMinimumWrapAngle;
constexpr int kRuntimeSweeps = 6;

using Vec3 = std::array<mjtNum, 3>;

Vec3 Add(const Vec3 &a, const Vec3 &b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 Sub(const Vec3 &a, const Vec3 &b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 Scale(const Vec3 &a, mjtNum scale) {
  return {scale * a[0], scale * a[1], scale * a[2]};
}

mjtNum Dot(const Vec3 &a, const Vec3 &b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vec3 Cross(const Vec3 &a, const Vec3 &b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

mjtNum Norm(const Vec3 &a) { return std::sqrt(Dot(a, a)); }

Vec3 Normalize(const Vec3 &a) {
  mjtNum norm = Norm(a);
  return norm > mjMINVAL ? Scale(a, 1 / norm) : Vec3{0, 0, 0};
}

mjtNum Distance(const Vec3 &a, const Vec3 &b) { return Norm(Sub(a, b)); }

Vec3 PointerVec(const mjtNum *value) { return {value[0], value[1], value[2]}; }

Vec3 WorldToLocal(const mjData *d, int geom_id, const Vec3 &world) {
  mjtNum shifted[3];
  mjtNum local[3];
  mju_sub3(shifted, world.data(), d->geom_xpos + 3 * geom_id);
  mju_mulMatTVec3(local, d->geom_xmat + 9 * geom_id, shifted);
  return PointerVec(local);
}

Vec3 WorldVectorToLocal(const mjData *d, int geom_id, const Vec3 &world) {
  mjtNum local[3];
  mju_mulMatTVec3(local, d->geom_xmat + 9 * geom_id, world.data());
  return PointerVec(local);
}

Vec3 LocalToWorld(const mjData *d, int geom_id, const Vec3 &local) {
  mjtNum world[3];
  mju_mulMatVec3(world, d->geom_xmat + 9 * geom_id, local.data());
  mju_addTo3(world, d->geom_xpos + 3 * geom_id);
  return PointerVec(world);
}

mjtNum PositiveModulo(mjtNum value, mjtNum period) {
  value = std::fmod(value, period);
  return value < 0 ? value + period : value;
}

mjtNum Clamp(mjtNum value, mjtNum lower, mjtNum upper) {
  return std::max(lower, std::min(value, upper));
}

bool FiniteVector(const std::vector<mjtNum> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](mjtNum value) { return std::isfinite(value); });
}

using Objective =
    std::function<mjtNum(const std::vector<mjtNum> &, std::vector<mjtNum> *)>;
using Projection = std::function<void(std::vector<mjtNum> *)>;

bool OptimizeBfgs(std::vector<mjtNum> *x, int max_iterations,
                  const Objective &objective, const Projection &projection,
                  int *iterations) {
  const int n = static_cast<int>(x->size());
  if (n == 0) {
    *iterations = 0;
    return true;
  }

  projection(x);
  std::vector<mjtNum> gradient(n);
  mjtNum value = objective(*x, &gradient);
  if (!std::isfinite(value) || !FiniteVector(gradient)) {
    return false;
  }

  std::vector<mjtNum> inverse_hessian(n * n, 0);
  for (int i = 0; i < n; ++i) {
    inverse_hessian[i * n + i] = 1;
  }

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    *iterations = iteration + 1;
    mjtNum gradient_norm = 0;
    for (mjtNum component : gradient) {
      gradient_norm = std::max(gradient_norm, std::abs(component));
    }
    if (gradient_norm < 1e-9) {
      return true;
    }

    std::vector<mjtNum> direction(n, 0);
    for (int row = 0; row < n; ++row) {
      for (int column = 0; column < n; ++column) {
        direction[row] -= inverse_hessian[row * n + column] * gradient[column];
      }
    }
    mjtNum directional_derivative = std::inner_product(
        gradient.begin(), gradient.end(), direction.begin(), mjtNum(0));
    if (directional_derivative >= -1e-14) {
      for (int i = 0; i < n; ++i) {
        direction[i] = -gradient[i];
      }
    }

    std::vector<mjtNum> candidate(n);
    std::vector<mjtNum> candidate_gradient(n);
    mjtNum candidate_value = value;
    mjtNum alpha = 1;
    bool accepted = false;
    for (int line_search = 0; line_search < 24; ++line_search) {
      for (int i = 0; i < n; ++i) {
        candidate[i] = (*x)[i] + alpha * direction[i];
      }
      projection(&candidate);
      std::vector<mjtNum> step(n);
      for (int i = 0; i < n; ++i) {
        step[i] = candidate[i] - (*x)[i];
      }
      mjtNum projected_derivative = std::inner_product(
          gradient.begin(), gradient.end(), step.begin(), mjtNum(0));
      candidate_value = objective(candidate, &candidate_gradient);
      if (std::isfinite(candidate_value) && FiniteVector(candidate_gradient) &&
          candidate_value <= value + 1e-4 * projected_derivative + 1e-12) {
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) {
      return gradient_norm < 1e-6;
    }

    std::vector<mjtNum> step(n);
    std::vector<mjtNum> gradient_delta(n);
    for (int i = 0; i < n; ++i) {
      step[i] = candidate[i] - (*x)[i];
      gradient_delta[i] = candidate_gradient[i] - gradient[i];
    }
    mjtNum ys = std::inner_product(gradient_delta.begin(), gradient_delta.end(),
                                   step.begin(), mjtNum(0));
    if (ys > 1e-12) {
      mjtNum rho = 1 / ys;
      std::vector<mjtNum> first(n * n, 0);
      std::vector<mjtNum> second(n * n, 0);
      for (int row = 0; row < n; ++row) {
        for (int column = 0; column < n; ++column) {
          first[row * n + column] = (row == column ? 1 : 0) -
                                    rho * step[row] * gradient_delta[column];
          second[row * n + column] = (row == column ? 1 : 0) -
                                     rho * gradient_delta[row] * step[column];
        }
      }
      std::vector<mjtNum> temp(n * n, 0);
      std::vector<mjtNum> updated(n * n, 0);
      for (int row = 0; row < n; ++row) {
        for (int column = 0; column < n; ++column) {
          for (int k = 0; k < n; ++k) {
            temp[row * n + column] +=
                first[row * n + k] * inverse_hessian[k * n + column];
          }
        }
      }
      for (int row = 0; row < n; ++row) {
        for (int column = 0; column < n; ++column) {
          for (int k = 0; k < n; ++k) {
            updated[row * n + column] +=
                temp[row * n + k] * second[k * n + column];
          }
          updated[row * n + column] += rho * step[row] * step[column];
        }
      }
      inverse_hessian.swap(updated);
    } else {
      std::fill(inverse_hessian.begin(), inverse_hessian.end(), 0);
      for (int i = 0; i < n; ++i) {
        inverse_hessian[i * n + i] = 1;
      }
    }

    *x = candidate;
    gradient = candidate_gradient;
    value = candidate_value;
  }
  return true;
}

Vec3 CylinderPoint(mjtNum radius, mjtNum arc_coordinate, mjtNum z) {
  mjtNum theta = arc_coordinate / radius;
  return {radius * std::cos(theta), radius * std::sin(theta), z};
}

Vec3 CylinderArcTangent(mjtNum radius, mjtNum arc_delta, mjtNum z_delta,
                        mjtNum theta) {
  mjtNum arc_length = std::hypot(arc_delta, z_delta);
  if (arc_length < mjMINVAL) {
    return {0, 0, 0};
  }
  return {-std::sin(theta) * arc_delta / arc_length,
          std::cos(theta) * arc_delta / arc_length, z_delta / arc_length};
}

std::array<mjtNum, 2> TangentAngles(const Vec3 &point, mjtNum radius) {
  mjtNum radial = std::hypot(point[0], point[1]);
  mjtNum angle = std::atan2(point[1], point[0]);
  mjtNum offset = std::acos(Clamp(radius / radial, -1, 1));
  return {angle - offset, angle + offset};
}

mjtNum AngularDistanceToArc(mjtNum hint, mjtNum start, mjtNum end) {
  if (end >= start) {
    mjtNum unwrapped = start + PositiveModulo(hint - start, 2 * mjPI);
    if (unwrapped <= end) {
      return 0;
    }
    return std::min(unwrapped - end, start + 2 * mjPI - unwrapped);
  }
  mjtNum unwrapped = start - PositiveModulo(start - hint, 2 * mjPI);
  if (unwrapped >= end) {
    return 0;
  }
  return std::min(end - unwrapped, unwrapped + 2 * mjPI - start);
}

Vec3 ClosestPointTriangle(const Vec3 &point, const Vec3 &a, const Vec3 &b,
                          const Vec3 &c, std::array<mjtNum, 3> *barycentric) {
  Vec3 ab = Sub(b, a);
  Vec3 ac = Sub(c, a);
  Vec3 ap = Sub(point, a);
  mjtNum d1 = Dot(ab, ap);
  mjtNum d2 = Dot(ac, ap);
  if (d1 <= 0 && d2 <= 0) {
    *barycentric = {1, 0, 0};
    return a;
  }

  Vec3 bp = Sub(point, b);
  mjtNum d3 = Dot(ab, bp);
  mjtNum d4 = Dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) {
    *barycentric = {0, 1, 0};
    return b;
  }

  mjtNum vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    mjtNum v = d1 / (d1 - d3);
    *barycentric = {1 - v, v, 0};
    return Add(a, Scale(ab, v));
  }

  Vec3 cp = Sub(point, c);
  mjtNum d5 = Dot(ab, cp);
  mjtNum d6 = Dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) {
    *barycentric = {0, 0, 1};
    return c;
  }

  mjtNum vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    mjtNum w = d2 / (d2 - d6);
    *barycentric = {1 - w, 0, w};
    return Add(a, Scale(ac, w));
  }

  mjtNum va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    mjtNum w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    *barycentric = {0, 1 - w, w};
    return Add(b, Scale(Sub(c, b), w));
  }

  mjtNum denominator = 1 / (va + vb + vc);
  mjtNum v = vb * denominator;
  mjtNum w = vc * denominator;
  *barycentric = {1 - v - w, v, w};
  return Add(a, Add(Scale(ab, v), Scale(ac, w)));
}

Vec3 EdgePoint(const std::vector<Vec3> &vertices,
               const std::array<int, 2> &edge, mjtNum parameter) {
  return Add(vertices[edge[0]],
             Scale(Sub(vertices[edge[1]], vertices[edge[0]]), parameter));
}

mjtNum RayMesh(const mjModel *m, const mjData *d, int geom_id,
               const Vec3 &origin, const Vec3 &direction) {
#if mjVERSION_HEADER >= 3010000
  return mj_rayMesh(m, d, geom_id, origin.data(), direction.data(), nullptr);
#else
  return mj_rayMesh(m, d, geom_id, origin.data(), direction.data());
#endif
}

bool SegmentClearOfMesh(const mjModel *m, const mjData *d, int geom_id,
                        const Vec3 &start, const Vec3 &end) {
  Vec3 delta = Sub(end, start);
  mjtNum length = Norm(delta);
  if (length <= 1e-10) {
    return true;
  }
  Vec3 direction = Scale(delta, 1 / length);
  // Contact points lie on a triangulated surface and may differ from the ray
  // hit by mesh-facet and solver noise. Ignore only a 10 um neighborhood of
  // each segment endpoint; interior intersections remain invalid.
  mjtNum endpoint_tolerance = std::max(mjtNum(1e-5), length * 2e-6);
  Vec3 origin = Add(start, Scale(direction, endpoint_tolerance));
  mjtNum usable_length = length - 2 * endpoint_tolerance;
  if (usable_length <= 0) {
    return true;
  }
  mjtNum hit = RayMesh(m, d, geom_id, origin, direction);
  if (hit >= 0 && hit < usable_length - endpoint_tolerance) {
    return false;
  }
  return true;
}

struct TriangleBounds {
  Vec3 lower = {0, 0, 0};
  Vec3 upper = {0, 0, 0};
  Vec3 center = {0, 0, 0};
};

struct TriangleBvhNode {
  Vec3 lower = {0, 0, 0};
  Vec3 upper = {0, 0, 0};
  int begin = 0;
  int end = 0;
  int left = -1;
  int right = -1;
};

bool BoundsOverlap(const TriangleBvhNode &first, const TriangleBvhNode &second,
                   mjtNum tolerance) {
  for (int axis = 0; axis < 3; ++axis) {
    if (first.upper[axis] < second.lower[axis] - tolerance ||
        second.upper[axis] < first.lower[axis] - tolerance) {
      return false;
    }
  }
  return true;
}

bool PointInTriangle(const Vec3 &point, const Vec3 &a, const Vec3 &b,
                     const Vec3 &c, mjtNum tolerance) {
  std::array<mjtNum, 3> barycentric;
  return Distance(point, ClosestPointTriangle(point, a, b, c, &barycentric)) <=
         tolerance;
}

bool SegmentTriangleIntersection(const Vec3 &start, const Vec3 &end,
                                 const Vec3 &a, const Vec3 &b, const Vec3 &c,
                                 mjtNum tolerance, Vec3 *intersection) {
  Vec3 normal = Cross(Sub(b, a), Sub(c, a));
  mjtNum normal_norm = Norm(normal);
  if (normal_norm <= mjMINVAL) {
    return false;
  }
  mjtNum plane_tolerance = tolerance * normal_norm;
  mjtNum start_distance = Dot(normal, Sub(start, a));
  mjtNum end_distance = Dot(normal, Sub(end, a));
  if ((start_distance > plane_tolerance && end_distance > plane_tolerance) ||
      (start_distance < -plane_tolerance && end_distance < -plane_tolerance)) {
    return false;
  }
  mjtNum denominator = start_distance - end_distance;
  if (std::abs(denominator) <= plane_tolerance) {
    return false;
  }
  mjtNum parameter = start_distance / denominator;
  if (parameter < -tolerance || parameter > 1 + tolerance) {
    return false;
  }
  Vec3 point = Add(start, Scale(Sub(end, start), Clamp(parameter, 0, 1)));
  if (!PointInTriangle(point, a, b, c, tolerance)) {
    return false;
  }
  *intersection = point;
  return true;
}

using Vec2 = std::array<mjtNum, 2>;

mjtNum Cross2(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

bool PointInTriangle2(const Vec2 &point, const std::array<Vec2, 3> &triangle,
                      mjtNum tolerance) {
  mjtNum first = Cross2(triangle[0], triangle[1], point);
  mjtNum second = Cross2(triangle[1], triangle[2], point);
  mjtNum third = Cross2(triangle[2], triangle[0], point);
  bool negative =
      first < -tolerance || second < -tolerance || third < -tolerance;
  bool positive = first > tolerance || second > tolerance || third > tolerance;
  return !(negative && positive);
}

bool ProperSegmentsIntersect2(const Vec2 &a, const Vec2 &b, const Vec2 &c,
                              const Vec2 &d, mjtNum tolerance) {
  mjtNum first = Cross2(a, b, c);
  mjtNum second = Cross2(a, b, d);
  mjtNum third = Cross2(c, d, a);
  mjtNum fourth = Cross2(c, d, b);
  return ((first > tolerance && second < -tolerance) ||
          (first < -tolerance && second > tolerance)) &&
         ((third > tolerance && fourth < -tolerance) ||
          (third < -tolerance && fourth > tolerance));
}

bool CoplanarTrianglesIntersect(const std::array<Vec3, 3> &first,
                                const std::array<Vec3, 3> &second,
                                const Vec3 &normal, mjtNum tolerance,
                                int first_shared_vertex,
                                int second_shared_vertex) {
  int omitted_axis = 0;
  if (std::abs(normal[1]) > std::abs(normal[omitted_axis])) {
    omitted_axis = 1;
  }
  if (std::abs(normal[2]) > std::abs(normal[omitted_axis])) {
    omitted_axis = 2;
  }
  auto project = [omitted_axis](const Vec3 &point) {
    if (omitted_axis == 0) {
      return Vec2{point[1], point[2]};
    }
    if (omitted_axis == 1) {
      return Vec2{point[0], point[2]};
    }
    return Vec2{point[0], point[1]};
  };
  std::array<Vec2, 3> first_2;
  std::array<Vec2, 3> second_2;
  for (int vertex = 0; vertex < 3; ++vertex) {
    first_2[vertex] = project(first[vertex]);
    second_2[vertex] = project(second[vertex]);
  }
  mjtNum area_tolerance =
      tolerance *
      std::max({Distance(first[0], first[1]), Distance(first[1], first[2]),
                Distance(first[2], first[0]), Distance(second[0], second[1]),
                Distance(second[1], second[2]),
                Distance(second[2], second[0])});
  for (int vertex = 0; vertex < 3; ++vertex) {
    if (vertex != first_shared_vertex &&
        PointInTriangle2(first_2[vertex], second_2, area_tolerance)) {
      return true;
    }
    if (vertex != second_shared_vertex &&
        PointInTriangle2(second_2[vertex], first_2, area_tolerance)) {
      return true;
    }
  }
  for (int first_edge = 0; first_edge < 3; ++first_edge) {
    for (int second_edge = 0; second_edge < 3; ++second_edge) {
      if (ProperSegmentsIntersect2(
              first_2[first_edge], first_2[(first_edge + 1) % 3],
              second_2[second_edge], second_2[(second_edge + 1) % 3],
              area_tolerance)) {
        return true;
      }
    }
  }
  return false;
}

bool TrianglesIntersect(const std::vector<Vec3> &vertices,
                        const std::array<int, 3> &first_indices,
                        const std::array<int, 3> &second_indices,
                        mjtNum tolerance) {
  int shared_count = 0;
  int first_shared_vertex = -1;
  int second_shared_vertex = -1;
  for (int first_vertex = 0; first_vertex < 3; ++first_vertex) {
    for (int second_vertex = 0; second_vertex < 3; ++second_vertex) {
      if (first_indices[first_vertex] == second_indices[second_vertex]) {
        ++shared_count;
        first_shared_vertex = first_vertex;
        second_shared_vertex = second_vertex;
      }
    }
  }
  std::array<Vec3, 3> first;
  std::array<Vec3, 3> second;
  for (int vertex = 0; vertex < 3; ++vertex) {
    first[vertex] = vertices[first_indices[vertex]];
    second[vertex] = vertices[second_indices[vertex]];
  }
  Vec3 first_normal = Cross(Sub(first[1], first[0]), Sub(first[2], first[0]));
  Vec3 second_normal =
      Cross(Sub(second[1], second[0]), Sub(second[2], second[0]));
  mjtNum first_normal_norm = Norm(first_normal);
  mjtNum second_normal_norm = Norm(second_normal);
  if (first_normal_norm <= mjMINVAL || second_normal_norm <= mjMINVAL) {
    return false;
  }
  mjtNum parallel_measure = Norm(Cross(first_normal, second_normal));
  mjtNum parallel_tolerance =
      tolerance * first_normal_norm * second_normal_norm;
  bool coplanar = parallel_measure <= parallel_tolerance;
  if (coplanar) {
    mjtNum plane_tolerance = tolerance * first_normal_norm;
    for (const Vec3 &point : second) {
      if (std::abs(Dot(first_normal, Sub(point, first[0]))) > plane_tolerance) {
        coplanar = false;
        break;
      }
    }
  }
  if (shared_count == 3) {
    return true;
  }
  if (shared_count == 2) {
    if (!coplanar) {
      return false;
    }
    std::array<Vec3, 2> shared_points;
    int shared_position = 0;
    int first_unshared = -1;
    int second_unshared = -1;
    for (int first_vertex = 0; first_vertex < 3; ++first_vertex) {
      bool shared = false;
      for (int second_vertex = 0; second_vertex < 3; ++second_vertex) {
        if (first_indices[first_vertex] == second_indices[second_vertex]) {
          shared_points[shared_position++] = first[first_vertex];
          shared = true;
          break;
        }
      }
      if (!shared) {
        first_unshared = first_vertex;
      }
    }
    for (int second_vertex = 0; second_vertex < 3; ++second_vertex) {
      bool shared = false;
      for (int first_vertex = 0; first_vertex < 3; ++first_vertex) {
        if (second_indices[second_vertex] == first_indices[first_vertex]) {
          shared = true;
          break;
        }
      }
      if (!shared) {
        second_unshared = second_vertex;
      }
    }
    int omitted_axis = 0;
    if (std::abs(first_normal[1]) > std::abs(first_normal[omitted_axis])) {
      omitted_axis = 1;
    }
    if (std::abs(first_normal[2]) > std::abs(first_normal[omitted_axis])) {
      omitted_axis = 2;
    }
    auto project = [omitted_axis](const Vec3 &point) {
      if (omitted_axis == 0) {
        return Vec2{point[1], point[2]};
      }
      if (omitted_axis == 1) {
        return Vec2{point[0], point[2]};
      }
      return Vec2{point[0], point[1]};
    };
    Vec2 edge_first = project(shared_points[0]);
    Vec2 edge_second = project(shared_points[1]);
    mjtNum first_side =
        Cross2(edge_first, edge_second, project(first[first_unshared]));
    mjtNum second_side =
        Cross2(edge_first, edge_second, project(second[second_unshared]));
    mjtNum side_tolerance =
        tolerance * Distance(shared_points[0], shared_points[1]);
    return (first_side > side_tolerance && second_side > side_tolerance) ||
           (first_side < -side_tolerance && second_side < -side_tolerance);
  }
  if (coplanar) {
    return CoplanarTrianglesIntersect(
        first, second, first_normal, tolerance,
        shared_count == 1 ? first_shared_vertex : -1,
        shared_count == 1 ? second_shared_vertex : -1);
  }

  auto away_from_shared_vertex = [&](const Vec3 &point) {
    return shared_count == 0 ||
           Distance(point, first[first_shared_vertex]) > 4 * tolerance;
  };
  Vec3 intersection;
  for (int edge = 0; edge < 3; ++edge) {
    if (SegmentTriangleIntersection(first[edge], first[(edge + 1) % 3],
                                    second[0], second[1], second[2], tolerance,
                                    &intersection) &&
        away_from_shared_vertex(intersection)) {
      return true;
    }
    if (SegmentTriangleIntersection(second[edge], second[(edge + 1) % 3],
                                    first[0], first[1], first[2], tolerance,
                                    &intersection) &&
        away_from_shared_vertex(intersection)) {
      return true;
    }
  }
  return false;
}

bool FindMeshSelfIntersection(const std::vector<Vec3> &vertices,
                              const std::vector<std::array<int, 3>> &triangles,
                              mjtNum tolerance,
                              std::array<int, 2> *intersection) {
  constexpr int kLeafSize = 8;
  std::vector<TriangleBounds> bounds(triangles.size());
  std::vector<int> order(triangles.size());
  std::iota(order.begin(), order.end(), 0);
  for (int face = 0; face < static_cast<int>(triangles.size()); ++face) {
    const Vec3 &a = vertices[triangles[face][0]];
    const Vec3 &b = vertices[triangles[face][1]];
    const Vec3 &c = vertices[triangles[face][2]];
    for (int axis = 0; axis < 3; ++axis) {
      bounds[face].lower[axis] = std::min({a[axis], b[axis], c[axis]});
      bounds[face].upper[axis] = std::max({a[axis], b[axis], c[axis]});
      bounds[face].center[axis] =
          (bounds[face].lower[axis] + bounds[face].upper[axis]) / 2;
    }
  }

  std::vector<TriangleBvhNode> nodes;
  std::function<int(int, int)> build = [&](int begin, int end) {
    int node_index = static_cast<int>(nodes.size());
    nodes.push_back({});
    TriangleBvhNode &node = nodes.back();
    node.begin = begin;
    node.end = end;
    for (int axis = 0; axis < 3; ++axis) {
      node.lower[axis] = std::numeric_limits<mjtNum>::infinity();
      node.upper[axis] = -std::numeric_limits<mjtNum>::infinity();
      for (int position = begin; position < end; ++position) {
        int face = order[position];
        node.lower[axis] = std::min(node.lower[axis], bounds[face].lower[axis]);
        node.upper[axis] = std::max(node.upper[axis], bounds[face].upper[axis]);
      }
    }
    if (end - begin <= kLeafSize) {
      return node_index;
    }
    int split_axis = 0;
    for (int axis = 1; axis < 3; ++axis) {
      if (node.upper[axis] - node.lower[axis] >
          node.upper[split_axis] - node.lower[split_axis]) {
        split_axis = axis;
      }
    }
    int middle = begin + (end - begin) / 2;
    std::nth_element(order.begin() + begin, order.begin() + middle,
                     order.begin() + end, [&](int first, int second) {
                       return bounds[first].center[split_axis] <
                              bounds[second].center[split_axis];
                     });
    int left = build(begin, middle);
    int right = build(middle, end);
    nodes[node_index].left = left;
    nodes[node_index].right = right;
    return node_index;
  };
  int root = build(0, static_cast<int>(triangles.size()));

  std::function<bool(int, int)> visit = [&](int first_node_index,
                                            int second_node_index) {
    const TriangleBvhNode &first_node = nodes[first_node_index];
    const TriangleBvhNode &second_node = nodes[second_node_index];
    if (!BoundsOverlap(first_node, second_node, tolerance)) {
      return false;
    }
    bool same_node = first_node_index == second_node_index;
    bool first_leaf = first_node.left < 0;
    bool second_leaf = second_node.left < 0;
    if (first_leaf && second_leaf) {
      for (int first_position = first_node.begin;
           first_position < first_node.end; ++first_position) {
        int first_face = order[first_position];
        int second_begin = same_node ? first_position + 1 : second_node.begin;
        for (int second_position = second_begin;
             second_position < second_node.end; ++second_position) {
          int second_face = order[second_position];
          if (TrianglesIntersect(vertices, triangles[first_face],
                                 triangles[second_face], tolerance)) {
            *intersection = {first_face, second_face};
            return true;
          }
        }
      }
      return false;
    }
    if (same_node) {
      return visit(first_node.left, first_node.left) ||
             visit(first_node.left, first_node.right) ||
             visit(first_node.right, first_node.right);
    }
    if (!first_leaf &&
        (second_leaf || first_node.end - first_node.begin >=
                            second_node.end - second_node.begin)) {
      return visit(first_node.left, second_node_index) ||
             visit(first_node.right, second_node_index);
    }
    return visit(first_node_index, second_node.left) ||
           visit(first_node_index, second_node.right);
  };
  return visit(root, root);
}

std::uint64_t
MeshGeometryHash(const std::vector<Vec3> &vertices,
                 const std::vector<std::array<int, 3>> &triangles) {
  constexpr std::uint64_t kOffset = 1469598103934665603ULL;
  constexpr std::uint64_t kPrime = 1099511628211ULL;
  std::uint64_t hash = kOffset;
  auto mix = [&](std::uint64_t value) {
    hash ^= value;
    hash *= kPrime;
  };
  mix(vertices.size());
  mix(triangles.size());
  for (const Vec3 &vertex : vertices) {
    for (mjtNum coordinate : vertex) {
      std::uint64_t bits = 0;
      static_assert(sizeof(bits) == sizeof(coordinate));
      std::memcpy(&bits, &coordinate, sizeof(bits));
      mix(bits);
    }
  }
  for (const std::array<int, 3> &triangle : triangles) {
    for (int vertex : triangle) {
      mix(static_cast<std::uint64_t>(vertex));
    }
  }
  return hash;
}

bool FindMeshSelfIntersectionCached(
    const std::vector<Vec3> &vertices,
    const std::vector<std::array<int, 3>> &triangles, mjtNum tolerance,
    std::array<int, 2> *intersection) {
  struct CacheEntry {
    bool intersects = false;
    std::array<int, 2> faces = {-1, -1};
  };
  static std::mutex cache_mutex;
  static std::map<std::uint64_t, CacheEntry> cache;
  std::uint64_t hash = MeshGeometryHash(vertices, triangles);
  {
    std::lock_guard<std::mutex> lock(cache_mutex);
    auto found = cache.find(hash);
    if (found != cache.end()) {
      *intersection = found->second.faces;
      return found->second.intersects;
    }
  }
  CacheEntry entry;
  entry.intersects =
      FindMeshSelfIntersection(vertices, triangles, tolerance, &entry.faces);
  {
    std::lock_guard<std::mutex> lock(cache_mutex);
    if (cache.size() >= 64) {
      cache.clear();
    }
    cache.emplace(hash, entry);
  }
  *intersection = entry.faces;
  return entry.intersects;
}

Vec3 PointVelocity(const mjModel *m, const mjData *d,
                   const SurfaceEnvelopeRoute::Point &point) {
  if (point.body_id <= 0) {
    return {0, 0, 0};
  }
  mjtNum velocity[6] = {0, 0, 0, 0, 0, 0};
  if (point.site_id >= 0) {
    mj_objectVelocity(m, d, mjOBJ_SITE, point.site_id, velocity, 0);
    return {velocity[3], velocity[4], velocity[5]};
  }
  mj_objectVelocity(m, d, mjOBJ_BODY, point.body_id, velocity, 0);
  Vec3 omega = {velocity[0], velocity[1], velocity[2]};
  Vec3 linear = {velocity[3], velocity[4], velocity[5]};
  Vec3 offset = Sub(point.pos, PointerVec(d->xpos + 3 * point.body_id));
  return Add(linear, Cross(omega, offset));
}

} // namespace

std::optional<SurfaceEnvelopeRoute> SurfaceEnvelopeRoute::Create(
    const mjModel *m, int tendon_id, const std::vector<int> &wrap_geom_ids,
    int site_role_user_index, MeshRouteMode mesh_route_mode,
    mjtNum route_hysteresis, mjtNum composite_merge_distance,
    const std::optional<std::array<mjtNum, 3>> &mesh_guide_axis,
    mjtNum mesh_guide_weight,
    std::string *error) {
  SurfaceEnvelopeRoute route;
  route.tendon_id_ = tendon_id;
  route.site_role_user_index_ = site_role_user_index;
  route.mesh_route_mode_ = mesh_route_mode;
  route.route_hysteresis_ = route_hysteresis;
  route.composite_merge_distance_ = composite_merge_distance;
  route.mesh_guide_axis_ = mesh_guide_axis;
  route.mesh_guide_weight_ = std::max(mjtNum(0), mesh_guide_weight);
  if (tendon_id < 0 || tendon_id >= m->ntendon) {
    *error = "route_tendon does not name a valid tendon";
    return std::nullopt;
  }
  if (site_role_user_index < 0 || site_role_user_index >= m->nuser_site) {
    *error = "site_role_user_index is outside the configured site user array";
    return std::nullopt;
  }

  int address = m->tendon_adr[tendon_id];
  int count = m->tendon_num[tendon_id];
  if (count < 2) {
    *error = "surface route seed tendon requires at least two sites";
    return std::nullopt;
  }

  int hint_count = 0;
  for (int i = 0; i < count; ++i) {
    if (m->wrap_type[address + i] != mjWRAP_SITE) {
      *error = "surface route seed tendon may contain only sites";
      return std::nullopt;
    }
    int site_id = m->wrap_objid[address + i];
    int role = static_cast<int>(std::llround(
        m->site_user[site_id * m->nuser_site + site_role_user_index]));
    if (role < 1 || role > 3) {
      *error = "every surface route seed site must have role 1, 2, or 3";
      return std::nullopt;
    }
    if ((i == 0 || i == count - 1) && role != 1) {
      *error = "surface route first and last seed sites must have role 1";
      return std::nullopt;
    }
    if (i > 0 && i < count - 1 && role == 1) {
      *error = "role 1 is reserved for the two route endpoints; use role 3 for "
               "a guide";
      return std::nullopt;
    }
    route.seed_site_ids_.push_back(site_id);
    route.site_roles_.push_back(role);
    if (role == 2) {
      ++hint_count;
    }
  }
  if (hint_count != static_cast<int>(wrap_geom_ids.size())) {
    *error = "wrap_geoms count must equal the number of role-2 route hints";
    return std::nullopt;
  }

  std::map<std::pair<int, int>, std::vector<int>> mesh_edges;
  int wrap_cursor = 0;
  for (int i = 0; i < count; ++i) {
    int site_id = route.seed_site_ids_[i];
    if (route.site_roles_[i] != 2) {
      int hard_index = static_cast<int>(route.hard_site_ids_.size());
      route.hard_site_ids_.push_back(site_id);
      route.elements_.push_back({Element::Type::kSite, hard_index});
      continue;
    }

    int geom_id = wrap_geom_ids[wrap_cursor++];
    if (geom_id < 0 || geom_id >= m->ngeom) {
      *error = "wrap_geoms contains an invalid geometry";
      return std::nullopt;
    }
    int geom_type = m->geom_type[geom_id];
    if (geom_type != mjGEOM_CYLINDER && geom_type != mjGEOM_MESH) {
      *error = "surface route currently supports cylinder and mesh wrap geoms";
      return std::nullopt;
    }

    Wrap wrap;
    wrap.geom_id = geom_id;
    wrap.body_id = m->geom_bodyid[geom_id];
    wrap.hint_site_id = site_id;
    wrap.geom_type = geom_type;
    if (geom_type == mjGEOM_MESH) {
      int mesh_id = m->geom_dataid[geom_id];
      if (mesh_id < 0 || mesh_id >= m->nmesh) {
        *error = "mesh wrap geom has no compiled mesh data";
        return std::nullopt;
      }
      wrap.mesh.mesh_id = mesh_id;
      int vertex_address = m->mesh_vertadr[mesh_id];
      int vertex_count = m->mesh_vertnum[mesh_id];
      int face_address = m->mesh_faceadr[mesh_id];
      int face_count = m->mesh_facenum[mesh_id];
      if (vertex_count < 4 || face_count < 4) {
        *error = "mesh wrap proxy must be a closed three-dimensional mesh";
        return std::nullopt;
      }
      Vec3 centroid = {0, 0, 0};
      for (int vertex = 0; vertex < vertex_count; ++vertex) {
        const float *value = m->mesh_vert + 3 * (vertex_address + vertex);
        Vec3 point = {value[0], value[1], value[2]};
        wrap.mesh.vertices.push_back(point);
        centroid = Add(centroid, point);
      }
      centroid = Scale(centroid, 1.0 / vertex_count);
      wrap.mesh.faces.resize(face_count);
      wrap.mesh.neighbors.resize(face_count);
      wrap.mesh.convex = true;
      mjtNum scale = 0;
      for (const Vec3 &vertex : wrap.mesh.vertices) {
        scale = std::max(scale, Distance(vertex, centroid));
      }
      mjtNum convex_tolerance = std::max(mjtNum(1e-9), scale * 1e-7);
      wrap.mesh.scale = std::max(scale, mjtNum(1e-9));
      for (int face_index = 0; face_index < face_count; ++face_index) {
        int compiled_face = face_address + face_index;
        MeshFace &face = wrap.mesh.faces[face_index];
        for (int corner = 0; corner < 3; ++corner) {
          face.vertex[corner] = m->mesh_face[3 * compiled_face + corner];
        }
        const Vec3 &a = wrap.mesh.vertices[face.vertex[0]];
        const Vec3 &b = wrap.mesh.vertices[face.vertex[1]];
        const Vec3 &c = wrap.mesh.vertices[face.vertex[2]];
        face.center = Scale(Add(a, Add(b, c)), 1.0 / 3);
        face.normal = Normalize(Cross(Sub(b, a), Sub(c, a)));
        if (Norm(face.normal) < mjMINVAL) {
          *error = "mesh wrap proxy contains a degenerate triangle";
          return std::nullopt;
        }
        if (mesh_route_mode == MeshRouteMode::kConvexSurface &&
            Dot(face.normal, Sub(face.center, centroid)) < 0) {
          face.normal = Scale(face.normal, -1);
        }
        for (int edge = 0; edge < 3; ++edge) {
          int first = face.vertex[edge];
          int second = face.vertex[(edge + 1) % 3];
          if (first > second) {
            std::swap(first, second);
          }
          mesh_edges[{first, second}].push_back(face_index);
        }
      }
      for (const auto &[edge, faces] : mesh_edges) {
        if (faces.size() != 2) {
          *error = "mesh wrap proxy must be closed and manifold";
          return std::nullopt;
        }
        wrap.mesh.neighbors[faces[0]].push_back(
            {faces[1], {edge.first, edge.second}});
        wrap.mesh.neighbors[faces[1]].push_back(
            {faces[0], {edge.first, edge.second}});
      }
      if (mesh_route_mode == MeshRouteMode::kConvexSurface) {
        for (const MeshFace &face : wrap.mesh.faces) {
          const Vec3 &a = wrap.mesh.vertices[face.vertex[0]];
          for (const Vec3 &vertex : wrap.mesh.vertices) {
            if (Dot(face.normal, Sub(vertex, a)) > convex_tolerance) {
              *error = "mesh wrap proxy must be convex; use a nonconvex "
                       "mesh_route_mode for a closed nonconvex mesh";
              return std::nullopt;
            }
          }
        }
      } else {
        mjtNum signed_volume = 0;
        for (const MeshFace &face : wrap.mesh.faces) {
          const Vec3 &a = wrap.mesh.vertices[face.vertex[0]];
          const Vec3 &b = wrap.mesh.vertices[face.vertex[1]];
          const Vec3 &c = wrap.mesh.vertices[face.vertex[2]];
          signed_volume += Dot(a, Cross(b, c)) / 6;
        }
        if (std::abs(signed_volume) <=
            std::max(mjtNum(1e-15), scale * scale * scale * 1e-12)) {
          *error = "nonconvex route mesh must enclose a nonzero volume";
          return std::nullopt;
        }
        if (signed_volume < 0) {
          for (MeshFace &face : wrap.mesh.faces) {
            std::swap(face.vertex[1], face.vertex[2]);
            face.normal = Scale(face.normal, -1);
          }
        }
        for (const auto &[edge, faces] : mesh_edges) {
          const MeshFace &first = wrap.mesh.faces[faces[0]];
          const MeshFace &second = wrap.mesh.faces[faces[1]];
          const std::pair<int, int> edge_copy = edge;
          auto edge_sign = [edge_copy](const MeshFace &face) {
            for (int corner = 0; corner < 3; ++corner) {
              int a = face.vertex[corner];
              int b = face.vertex[(corner + 1) % 3];
              if (a == edge_copy.first && b == edge_copy.second) {
                return 1;
              }
              if (a == edge_copy.second && b == edge_copy.first) {
                return -1;
              }
            }
            return 0;
          };
          if (edge_sign(first) == edge_sign(second)) {
            *error = "nonconvex route mesh faces must have consistent winding";
            return std::nullopt;
          }
        }
      }
      std::vector<std::array<int, 3>> triangles;
      triangles.reserve(wrap.mesh.faces.size());
      for (const MeshFace &face : wrap.mesh.faces) {
        triangles.push_back(face.vertex);
      }
      std::array<int, 2> intersecting_faces = {-1, -1};
      mjtNum intersection_tolerance =
          std::max(mjtNum(1e-11), scale * mjtNum(1e-9));
      if (FindMeshSelfIntersectionCached(wrap.mesh.vertices, triangles,
                                         intersection_tolerance,
                                         &intersecting_faces)) {
        *error = "mesh wrap geometry is self-intersecting (faces " +
                 std::to_string(intersecting_faces[0]) + " and " +
                 std::to_string(intersecting_faces[1]) + ")";
        return std::nullopt;
      }
      mesh_edges.clear();
    }

    int wrap_index = static_cast<int>(route.wraps_.size());
    route.wraps_.push_back(std::move(wrap));
    route.elements_.push_back({Element::Type::kWrap, wrap_index});
  }
  if (route.wraps_.size() > 1) {
    route.composite_pair_active_.assign(route.wraps_.size() - 1, false);
    route.composite_left_segment_.assign(route.wraps_.size() - 1, -1);
    route.composite_right_segment_.assign(route.wraps_.size() - 1, -1);
    route.composite_left_local_.assign(route.wraps_.size() - 1, {0, 0, 0});
    route.composite_right_local_.assign(route.wraps_.size() - 1, {0, 0, 0});
    route.composite_reacquire_cooldown_.assign(route.wraps_.size() - 1, 0);
  }
  return route;
}

void SurfaceEnvelopeRoute::Reset() {
  result_ = Result();
  has_initial_length_ = false;
  initial_length_ = 0;
  initialized_ = false;
  std::fill(composite_pair_active_.begin(), composite_pair_active_.end(),
            false);
  std::fill(composite_left_segment_.begin(), composite_left_segment_.end(), -1);
  std::fill(composite_right_segment_.begin(), composite_right_segment_.end(),
            -1);
  std::fill(composite_reacquire_cooldown_.begin(),
            composite_reacquire_cooldown_.end(), 0);
  for (Wrap &wrap : wraps_) {
    wrap.initialized = false;
    wrap.valid = false;
    wrap.used_fallback = false;
    wrap.contact_points.clear();
    wrap.mesh.strip.clear();
    wrap.mesh.parameters.clear();
    wrap.mesh.active_transitions.clear();
    wrap.mesh.mandatory_transitions.clear();
    wrap.mesh.guided_anchor_valid = false;
    wrap.mesh.guide_plane_valid = false;
  }
}

bool SurfaceEnvelopeRoute::InitializeWrap(
    const mjModel *m, const mjData *d, int wrap_index,
    const std::array<mjtNum, 3> &previous_seed,
    const std::array<mjtNum, 3> &next_seed) {
  return SolveWrap(m, d, wrap_index, previous_seed, next_seed);
}

bool SurfaceEnvelopeRoute::SolveWrap(const mjModel *m, const mjData *d,
                                     int wrap_index,
                                     const std::array<mjtNum, 3> &previous,
                                     const std::array<mjtNum, 3> &next) {
  Wrap *wrap = &wraps_[wrap_index];
  bool can_fallback = wrap->initialized && wrap->valid &&
                      !wrap->contact_points.empty();
  CylinderState saved_cylinder = wrap->cylinder;
  int saved_seed_face = wrap->mesh.seed_face;
  std::vector<int> saved_strip = wrap->mesh.strip;
  std::vector<mjtNum> saved_parameters = wrap->mesh.parameters;
  std::vector<int> saved_active = wrap->mesh.active_transitions;
  std::vector<int> saved_mandatory = wrap->mesh.mandatory_transitions;
  Vec3 saved_guided_anchor = wrap->mesh.guided_anchor;
  bool saved_guided_anchor_valid = wrap->mesh.guided_anchor_valid;
  std::vector<Point> saved_points = wrap->contact_points;
  mjtNum saved_tangent_residual = wrap->tangent_residual;
  mjtNum saved_surface_residual = wrap->surface_residual;
  int saved_iterations = wrap->solver_iterations;
  bool solved = false;
  if (wrap->geom_type == mjGEOM_CYLINDER) {
    solved = SolveCylinder(m, d, wrap, previous, next, !wrap->initialized);
  } else {
    solved = SolveMesh(m, d, wrap, previous, next, !wrap->initialized);
  }
  if (solved) {
    wrap->used_fallback = false;
    return true;
  }
  if (!can_fallback) {
    return false;
  }

  wrap->cylinder = saved_cylinder;
  wrap->mesh.seed_face = saved_seed_face;
  wrap->mesh.strip = std::move(saved_strip);
  wrap->mesh.parameters = std::move(saved_parameters);
  wrap->mesh.active_transitions = std::move(saved_active);
  wrap->mesh.mandatory_transitions = std::move(saved_mandatory);
  wrap->mesh.guided_anchor = saved_guided_anchor;
  wrap->mesh.guided_anchor_valid = saved_guided_anchor_valid;
  wrap->contact_points = std::move(saved_points);
  for (Point &point : wrap->contact_points) {
    point.pos = LocalToWorld(d, wrap->geom_id, point.local_pos);
  }
  wrap->tangent_residual =
      std::max(saved_tangent_residual, mjtNum(1.0001e-5));
  wrap->surface_residual = saved_surface_residual;
  wrap->solver_iterations = saved_iterations;
  wrap->initialized = true;
  wrap->valid = true;
  wrap->used_fallback = true;
  return true;
}

bool SurfaceEnvelopeRoute::SolveCylinder(
    const mjModel *m, const mjData *d, Wrap *wrap,
    const std::array<mjtNum, 3> &previous_world,
    const std::array<mjtNum, 3> &next_world, bool initialize) {
  mjtNum radius = m->geom_size[3 * wrap->geom_id];
  mjtNum half_length = m->geom_size[3 * wrap->geom_id + 1];
  Vec3 previous = WorldToLocal(d, wrap->geom_id, previous_world);
  Vec3 next = WorldToLocal(d, wrap->geom_id, next_world);
  if (std::hypot(previous[0], previous[1]) <= radius + kGeometryTolerance ||
      std::hypot(next[0], next[1]) <= radius + kGeometryTolerance) {
    wrap->valid = false;
    return false;
  }

  if (initialize) {
    Vec3 hint = WorldToLocal(d, wrap->geom_id,
                             PointerVec(d->site_xpos + 3 * wrap->hint_site_id));
    wrap->cylinder.seed_angle = std::atan2(hint[1], hint[0]);
  }

  auto entry_candidates = TangentAngles(previous, radius);
  auto exit_candidates = TangentAngles(next, radius);
  std::array<mjtNum, 4> best_parameters = {0, 0, 0, 0};
  int best_sign = wrap->cylinder.branch_sign;
  int best_iterations = 0;
  mjtNum best_score = std::numeric_limits<mjtNum>::infinity();
  std::vector<int> signs =
      initialize ? std::vector<int>{-1, 1} : std::vector<int>{best_sign};

  for (int sign : signs) {
    for (mjtNum entry_angle : entry_candidates) {
      for (mjtNum exit_angle : exit_candidates) {
        mjtNum delta_angle =
            sign > 0 ? PositiveModulo(exit_angle - entry_angle, 2 * mjPI)
                     : -PositiveModulo(entry_angle - exit_angle, 2 * mjPI);
        if (std::abs(delta_angle) < kMinimumWrapAngle) {
          delta_angle = sign * kMaximumWrapAngle;
        }
        mjtNum unwrapped_exit = entry_angle + delta_angle;
        mjtNum seed_distance = AngularDistanceToArc(
            wrap->cylinder.seed_angle, entry_angle, unwrapped_exit);

        std::vector<mjtNum> axial =
            initialize
                ? std::vector<mjtNum>{Clamp(previous[2], -half_length,
                                            half_length),
                                      Clamp(next[2], -half_length, half_length)}
                : std::vector<mjtNum>{wrap->cylinder.parameters[1],
                                      wrap->cylinder.parameters[3]};
        auto projection = [half_length](std::vector<mjtNum> *value) {
          (*value)[0] = Clamp((*value)[0], -half_length, half_length);
          (*value)[1] = Clamp((*value)[1], -half_length, half_length);
        };
        auto objective = [radius, entry_angle, unwrapped_exit, delta_angle,
                          &previous, &next](const std::vector<mjtNum> &value,
                                            std::vector<mjtNum> *gradient) {
          Vec3 entry = CylinderPoint(radius, radius * entry_angle, value[0]);
          Vec3 exit = CylinderPoint(radius, radius * unwrapped_exit, value[1]);
          mjtNum first_length = std::max(Distance(previous, entry), mjMINVAL);
          mjtNum last_length = std::max(Distance(exit, next), mjMINVAL);
          mjtNum arc_coordinate = radius * delta_angle;
          mjtNum z_delta = value[1] - value[0];
          mjtNum arc_length =
              std::max(std::hypot(arc_coordinate, z_delta), mjMINVAL);
          if (gradient) {
            gradient->assign(2, 0);
            (*gradient)[0] =
                (entry[2] - previous[2]) / first_length - z_delta / arc_length;
            (*gradient)[1] =
                (exit[2] - next[2]) / last_length + z_delta / arc_length;
          }
          return first_length + arc_length + last_length;
        };

        int iterations = 0;
        OptimizeBfgs(&axial, initialize ? 40 : 32, objective, projection,
                     &iterations);
        projection(&axial);
        const int coordinate_sweeps = initialize ? 12 : 8;
        for (int sweep = 0; sweep < coordinate_sweeps; ++sweep) {
          mjtNum maximum_change = 0;
          for (int coordinate = 0; coordinate < 2; ++coordinate) {
            mjtNum lower = -half_length;
            mjtNum upper = half_length;
            constexpr mjtNum kGolden = 0.6180339887498948482;
            mjtNum left = upper - kGolden * (upper - lower);
            mjtNum right = lower + kGolden * (upper - lower);
            std::vector<mjtNum> candidate = axial;
            candidate[coordinate] = left;
            mjtNum left_value = objective(candidate, nullptr);
            candidate[coordinate] = right;
            mjtNum right_value = objective(candidate, nullptr);
            for (int search = 0; search < 32; ++search) {
              if (left_value < right_value) {
                upper = right;
                right = left;
                right_value = left_value;
                left = upper - kGolden * (upper - lower);
                candidate[coordinate] = left;
                left_value = objective(candidate, nullptr);
              } else {
                lower = left;
                left = right;
                left_value = right_value;
                right = lower + kGolden * (upper - lower);
                candidate[coordinate] = right;
                right_value = objective(candidate, nullptr);
              }
            }
            mjtNum updated = 0.5 * (lower + upper);
            maximum_change =
                std::max(maximum_change, std::abs(updated - axial[coordinate]));
            axial[coordinate] = updated;
          }
          ++iterations;
          if (maximum_change < 1e-12) {
            break;
          }
        }
        mjtNum length = objective(axial, nullptr);
        if (!std::isfinite(length)) {
          continue;
        }
        mjtNum score = length + 1000 * radius * seed_distance;
        if (score < best_score) {
          best_score = score;
          best_sign = sign;
          best_iterations = iterations;
          best_parameters = {radius * entry_angle, axial[0],
                             radius * unwrapped_exit, axial[1]};
        }
      }
    }
  }

  if (!std::isfinite(best_score)) {
    wrap->valid = false;
    return false;
  }
  wrap->cylinder.branch_sign = best_sign;
  wrap->cylinder.parameters = best_parameters;

  mjtNum arc_delta = best_parameters[2] - best_parameters[0];
  mjtNum z_delta = best_parameters[3] - best_parameters[1];
  mjtNum arc_length = std::hypot(arc_delta, z_delta);
  int samples = static_cast<int>(
      Clamp(static_cast<mjtNum>(std::ceil(arc_length / 0.0015) + 1), 3, 96));
  wrap->contact_points.clear();
  for (int sample = 0; sample < samples; ++sample) {
    mjtNum fraction = static_cast<mjtNum>(sample) / (samples - 1);
    Vec3 local =
        CylinderPoint(radius, best_parameters[0] + fraction * arc_delta,
                      best_parameters[1] + fraction * z_delta);
    Point point{LocalToWorld(d, wrap->geom_id, local), wrap->body_id, -1,
                wrap->geom_id, -1};
    point.local_pos = local;
    wrap->contact_points.push_back(point);
  }

  Vec3 entry = CylinderPoint(radius, best_parameters[0], best_parameters[1]);
  Vec3 exit = CylinderPoint(radius, best_parameters[2], best_parameters[3]);
  Vec3 arc_tangent_entry = CylinderArcTangent(radius, arc_delta, z_delta,
                                              best_parameters[0] / radius);
  Vec3 arc_tangent_exit = CylinderArcTangent(radius, arc_delta, z_delta,
                                             best_parameters[2] / radius);
  Vec3 straight_entry = Normalize(Sub(entry, previous));
  Vec3 straight_exit = Normalize(Sub(next, exit));
  wrap->tangent_residual =
      std::max(Norm(Sub(straight_entry, arc_tangent_entry)),
               Norm(Sub(straight_exit, arc_tangent_exit)));
  wrap->surface_residual =
      std::max(std::abs(std::hypot(entry[0], entry[1]) - radius),
               std::abs(std::hypot(exit[0], exit[1]) - radius));
  wrap->solver_iterations = best_iterations;
  wrap->initialized = true;
  wrap->valid = true;
  if (std::abs(best_parameters[1]) >= half_length - kCylinderEndTolerance ||
      std::abs(best_parameters[3]) >= half_length - kCylinderEndTolerance) {
    wrap->tangent_residual = std::max(wrap->tangent_residual, mjtNum(1e-4));
  }
  return true;
}

bool SurfaceEnvelopeRoute::SolveMesh(
    const mjModel *m, const mjData *d, Wrap *wrap,
    const std::array<mjtNum, 3> &previous_world,
    const std::array<mjtNum, 3> &next_world, bool initialize) {
  MeshState &mesh = wrap->mesh;
  Vec3 previous = WorldToLocal(d, wrap->geom_id, previous_world);
  Vec3 next = WorldToLocal(d, wrap->geom_id, next_world);
  if (initialize || mesh.seed_face < 0) {
    Vec3 hint = WorldToLocal(d, wrap->geom_id,
                             PointerVec(d->site_xpos + 3 * wrap->hint_site_id));
    mjtNum best_distance = std::numeric_limits<mjtNum>::infinity();
    Vec3 best_closest = {0, 0, 0};
    for (int face_index = 0; face_index < static_cast<int>(mesh.faces.size());
         ++face_index) {
      const MeshFace &face = mesh.faces[face_index];
      std::array<mjtNum, 3> barycentric;
      Vec3 closest = ClosestPointTriangle(
          hint, mesh.vertices[face.vertex[0]], mesh.vertices[face.vertex[1]],
          mesh.vertices[face.vertex[2]], &barycentric);
      mjtNum distance = Distance(hint, closest);
      if (distance < best_distance) {
        best_distance = distance;
        mesh.seed_face = face_index;
        best_closest = closest;
      }
    }
    if (mesh_route_mode_ == MeshRouteMode::kGuidedSurface &&
        mesh.seed_face >= 0) {
      // The hint selects a seed triangle only at initialization.  This point
      // initializes a contact that may slide inside that triangle; later hint
      // motion cannot affect route length or force.
      mesh.guided_anchor = best_closest;
      mesh.guided_anchor_valid = true;
      if (mesh_guide_axis_.has_value() && mesh_guide_weight_ > 0) {
        mesh.guide_plane_origin = best_closest;
        mesh.guide_plane_normal = Normalize(
            WorldVectorToLocal(d, wrap->geom_id, *mesh_guide_axis_));
        mesh.guide_plane_valid = Norm(mesh.guide_plane_normal) > mjMINVAL;
      } else {
        mesh.guide_plane_valid = false;
      }
    }
  }
  if (mesh.seed_face < 0) {
    wrap->valid = false;
    return false;
  }

  const int face_count = static_cast<int>(mesh.faces.size());
  std::vector<int> strip;
  if (!initialize && !mesh.strip.empty()) {
    strip = mesh.strip;
  } else if (mesh_route_mode_ != MeshRouteMode::kConvexSurface) {
    using QueueEntry = std::pair<mjtNum, int>;
    auto run_dijkstra = [&mesh, face_count, this](
                            const std::vector<bool> &forbidden,
                            std::vector<mjtNum> *distance,
                            std::vector<int> *predecessor) {
      distance->assign(face_count, std::numeric_limits<mjtNum>::infinity());
      predecessor->assign(face_count, -1);
      std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                          std::greater<QueueEntry>>
          queue;
      (*distance)[mesh.seed_face] = 0;
      queue.push({0, mesh.seed_face});
      while (!queue.empty()) {
        auto [current_distance, face_index] = queue.top();
        queue.pop();
        if (current_distance != (*distance)[face_index]) {
          continue;
        }
        for (const MeshNeighbor &neighbor : mesh.neighbors[face_index]) {
          if (forbidden[neighbor.face]) {
            continue;
          }
          mjtNum weight = Distance(mesh.faces[face_index].center,
                                   mesh.faces[neighbor.face].center);
          if (mesh.guide_plane_valid && mesh_guide_weight_ > 0) {
            mjtNum first_offset = std::abs(Dot(
                mesh.guide_plane_normal,
                Sub(mesh.faces[face_index].center, mesh.guide_plane_origin)));
            mjtNum second_offset = std::abs(Dot(
                mesh.guide_plane_normal,
                Sub(mesh.faces[neighbor.face].center, mesh.guide_plane_origin)));
            mjtNum normalized_offset =
                0.5 * (first_offset + second_offset) / mesh.scale;
            weight *= 1 + mesh_guide_weight_ * normalized_offset;
          }
          mjtNum candidate = current_distance + weight;
          if (candidate < (*distance)[neighbor.face]) {
            (*distance)[neighbor.face] = candidate;
            (*predecessor)[neighbor.face] = face_index;
            queue.push({candidate, neighbor.face});
          }
        }
      }
    };
    auto closest_on_face = [&mesh](const Vec3 &point, int face_index) {
      const MeshFace &face = mesh.faces[face_index];
      std::array<mjtNum, 3> barycentric;
      return ClosestPointTriangle(point, mesh.vertices[face.vertex[0]],
                                  mesh.vertices[face.vertex[1]],
                                  mesh.vertices[face.vertex[2]], &barycentric);
    };
    // Composite merging identifies adjacent obstacles; it must not pin the
    // departure or arrival contact to the hint's seed triangle.  That pinning
    // turns the inter-surface span into a chord between two arbitrary
    // projections.  Reserve the seed shortcut for a genuinely coincident
    // interface, and otherwise let visibility select the tangent face.
    const mjtNum guided_interface_distance =
        std::max(mjtNum(1e-7), mjtNum(0.05) * route_hysteresis_);
    auto joins_guided_interface = [&](const Vec3 &point) {
      return mesh_route_mode_ == MeshRouteMode::kGuidedSurface &&
             Distance(point, closest_on_face(point, mesh.seed_face)) <=
                 guided_interface_distance;
    };
    auto choose_visible_face = [&](const Vec3 &point_local,
                                   const Vec3 &point_world,
                                   const std::vector<mjtNum> &distance,
                                   const std::vector<int> &predecessor,
                                   const std::vector<bool> &forbidden) {
      std::vector<std::pair<mjtNum, int>> candidates;
      candidates.reserve(face_count);
      for (int face_index = 0; face_index < face_count; ++face_index) {
        if (forbidden[face_index] || !std::isfinite(distance[face_index])) {
          continue;
        }
        const MeshFace &face = mesh.faces[face_index];
        if (Dot(face.normal, Sub(point_local, mesh.vertices[face.vertex[0]])) <
            -kGeometryTolerance) {
          continue;
        }
        Vec3 closest = closest_on_face(point_local, face_index);
        candidates.push_back(
            {distance[face_index] + Distance(point_local, closest),
             face_index});
      }
      std::sort(candidates.begin(), candidates.end());
      for (const auto &[score, face_index] : candidates) {
        static_cast<void>(score);
        int toward_seed = predecessor[face_index];
        if (toward_seed < 0) {
          continue;
        }
        std::array<int, 2> edge = {-1, -1};
        for (const MeshNeighbor &neighbor : mesh.neighbors[face_index]) {
          if (neighbor.face == toward_seed) {
            edge = neighbor.edge;
            break;
          }
        }
        if (edge[0] < 0) {
          continue;
        }
        for (int sample = 0; sample <= 16; ++sample) {
          Vec3 edge_point = EdgePoint(mesh.vertices, edge, sample / 16.0);
          Vec3 edge_world = LocalToWorld(d, wrap->geom_id, edge_point);
          if (SegmentClearOfMesh(m, d, wrap->geom_id, point_world,
                                 edge_world)) {
            return face_index;
          }
        }
      }
      return -1;
    };
    auto path_to_seed = [seed = mesh.seed_face](
                            int face, const std::vector<int> &predecessor) {
      std::vector<int> path;
      for (int current = face; current >= 0; current = predecessor[current]) {
        path.push_back(current);
        if (current == seed) {
          break;
        }
      }
      return path;
    };

    std::vector<bool> forbidden(face_count, false);
    std::vector<mjtNum> first_distance;
    std::vector<int> first_predecessor;
    run_dijkstra(forbidden, &first_distance, &first_predecessor);
    bool entry_at_seed = joins_guided_interface(previous);
    int entry_face = entry_at_seed
                         ? mesh.seed_face
                         : choose_visible_face(previous, previous_world,
                                               first_distance,
                                               first_predecessor, forbidden);
    if (entry_face < 0) {
      mju_warning(
          "nonconvex surface route could not find an entry face on mesh geom %d",
          wrap->geom_id);
      wrap->valid = false;
      return false;
    }
    std::vector<int> first = entry_at_seed
                                 ? std::vector<int>{mesh.seed_face}
                                 : path_to_seed(entry_face, first_predecessor);
    if (first.empty() || first.back() != mesh.seed_face) {
      wrap->valid = false;
      return false;
    }
    for (int face : first) {
      if (face != mesh.seed_face) {
        forbidden[face] = true;
      }
    }

    std::vector<mjtNum> second_distance;
    std::vector<int> second_predecessor;
    run_dijkstra(forbidden, &second_distance, &second_predecessor);
    bool exit_at_seed = joins_guided_interface(next);
    int exit_face = exit_at_seed
                        ? mesh.seed_face
                        : choose_visible_face(next, next_world, second_distance,
                                              second_predecessor, forbidden);
    if (exit_face < 0) {
      mju_warning(
          "nonconvex surface route could not find an exit face on mesh geom %d",
          wrap->geom_id);
      wrap->valid = false;
      return false;
    }
    std::vector<int> second = exit_at_seed
                                  ? std::vector<int>{mesh.seed_face}
                                  : path_to_seed(exit_face, second_predecessor);
    if (second.empty() || second.back() != mesh.seed_face) {
      wrap->valid = false;
      return false;
    }
    std::reverse(second.begin(), second.end());
    strip = std::move(first);
    strip.insert(strip.end(), second.begin() + 1, second.end());
    if (strip.size() < 2) {
      wrap->valid = false;
      return false;
    }
  } else {
    std::vector<mjtNum> distance(face_count,
                                 std::numeric_limits<mjtNum>::infinity());
    std::vector<int> predecessor(face_count, -1);
    using QueueEntry = std::pair<mjtNum, int>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                        std::greater<QueueEntry>>
        queue;
    distance[mesh.seed_face] = 0;
    queue.push({0, mesh.seed_face});
    while (!queue.empty()) {
      auto [current_distance, face_index] = queue.top();
      queue.pop();
      if (current_distance != distance[face_index]) {
        continue;
      }
      for (const MeshNeighbor &neighbor : mesh.neighbors[face_index]) {
        mjtNum weight = Distance(mesh.faces[face_index].center,
                                 mesh.faces[neighbor.face].center);
        mjtNum candidate = current_distance + weight;
        if (candidate < distance[neighbor.face]) {
          distance[neighbor.face] = candidate;
          predecessor[neighbor.face] = face_index;
          queue.push({candidate, neighbor.face});
        }
      }
    }

    auto visible_face = [&mesh](const Vec3 &point, int face_index) {
      const MeshFace &face = mesh.faces[face_index];
      return Dot(face.normal, Sub(point, mesh.vertices[face.vertex[0]])) >=
             -kGeometryTolerance;
    };
    auto to_seed = [&predecessor, seed = mesh.seed_face](int face) {
      std::vector<int> path;
      int current = face;
      while (current >= 0) {
        path.push_back(current);
        if (current == seed) {
          break;
        }
        current = predecessor[current];
      }
      return path;
    };

    auto point_face_distance = [&mesh](const Vec3 &point, int face_index) {
      const MeshFace &face = mesh.faces[face_index];
      std::array<mjtNum, 3> barycentric;
      Vec3 closest = ClosestPointTriangle(
          point, mesh.vertices[face.vertex[0]], mesh.vertices[face.vertex[1]],
          mesh.vertices[face.vertex[2]], &barycentric);
      return Distance(point, closest);
    };

    std::vector<int> best_first;
    std::vector<int> best_second;
    mjtNum best_pair_score = std::numeric_limits<mjtNum>::infinity();
    for (int entry_face = 0; entry_face < face_count; ++entry_face) {
      if (!visible_face(previous, entry_face)) {
        continue;
      }
      std::vector<int> first = to_seed(entry_face);
      if (first.empty() || first.back() != mesh.seed_face) {
        continue;
      }
      std::vector<bool> forbidden(face_count, false);
      for (int face : first) {
        if (face != mesh.seed_face) {
          forbidden[face] = true;
        }
      }

      std::vector<mjtNum> second_distance(
          face_count, std::numeric_limits<mjtNum>::infinity());
      std::vector<int> second_predecessor(face_count, -1);
      std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                          std::greater<QueueEntry>>
          second_queue;
      second_distance[mesh.seed_face] = 0;
      second_queue.push({0, mesh.seed_face});
      while (!second_queue.empty()) {
        auto [current_distance, face_index] = second_queue.top();
        second_queue.pop();
        if (current_distance != second_distance[face_index]) {
          continue;
        }
        for (const MeshNeighbor &neighbor : mesh.neighbors[face_index]) {
          if (forbidden[neighbor.face]) {
            continue;
          }
          mjtNum weight = Distance(mesh.faces[face_index].center,
                                   mesh.faces[neighbor.face].center);
          mjtNum candidate = current_distance + weight;
          if (candidate < second_distance[neighbor.face]) {
            second_distance[neighbor.face] = candidate;
            second_predecessor[neighbor.face] = face_index;
            second_queue.push({candidate, neighbor.face});
          }
        }
      }

      for (int exit_face = 0; exit_face < face_count; ++exit_face) {
        if (!visible_face(next, exit_face) || forbidden[exit_face] ||
            !std::isfinite(second_distance[exit_face])) {
          continue;
        }
        std::vector<int> second;
        int current = exit_face;
        while (current >= 0) {
          second.push_back(current);
          if (current == mesh.seed_face) {
            break;
          }
          current = second_predecessor[current];
        }
        if (second.empty() || second.back() != mesh.seed_face) {
          continue;
        }
        std::reverse(second.begin(), second.end());
        mjtNum score = distance[entry_face] + second_distance[exit_face] +
                       0.25 * point_face_distance(previous, entry_face) +
                       0.25 * point_face_distance(next, exit_face);
        mjtNum corridor_bias = std::max(mjtNum(1e-6), route_hysteresis_);
        if (!mesh.strip.empty() && entry_face == mesh.strip.front()) {
          score -= corridor_bias;
        }
        if (!mesh.strip.empty() && exit_face == mesh.strip.back()) {
          score -= corridor_bias;
        }
        if (score < best_pair_score) {
          best_pair_score = score;
          best_first = first;
          best_second = second;
        }
      }
    }
    if (best_first.empty() || best_second.empty()) {
      wrap->valid = false;
      return false;
    }
    strip = best_first;
    strip.insert(strip.end(), best_second.begin() + 1, best_second.end());
    if (strip.empty()) {
      wrap->valid = false;
      return false;
    }
  }

  std::vector<std::array<int, 2>> shared_edges;
  for (int i = 0; i + 1 < static_cast<int>(strip.size()); ++i) {
    std::array<int, 2> edge = {-1, -1};
    for (const MeshNeighbor &neighbor : mesh.neighbors[strip[i]]) {
      if (neighbor.face == strip[i + 1]) {
        edge = neighbor.edge;
        break;
      }
    }
    if (edge[0] < 0) {
      wrap->valid = false;
      return false;
    }
    shared_edges.push_back(edge);
  }

  const int transition_count = static_cast<int>(shared_edges.size());
  if (transition_count == 0) {
    wrap->valid = false;
    return false;
  }
  std::vector<mjtNum> parameters(transition_count, 0.5);
  bool reuse =
      strip == mesh.strip && mesh.parameters.size() == parameters.size();
  if (reuse) {
    parameters = mesh.parameters;
  }

  if (mesh_route_mode_ != MeshRouteMode::kConvexSurface) {
    auto transition_point = [&mesh, &shared_edges,
                             &parameters](int transition) {
      return EdgePoint(mesh.vertices, shared_edges[transition],
                       parameters[transition]);
    };
    int seed_position = -1;
    for (int face = 0; face < static_cast<int>(strip.size()); ++face) {
      if (strip[face] == mesh.seed_face) {
        seed_position = face;
        break;
      }
    }
    if (mesh_route_mode_ == MeshRouteMode::kGuidedSurface &&
        (seed_position < 0 || !mesh.guided_anchor_valid)) {
      wrap->valid = false;
      return false;
    }
    auto span_clear = [&](int first_transition, const Vec3 &first,
                          int second_transition, const Vec3 &second) {
      if (first_transition >= 0 && second_transition < transition_count &&
          second_transition - first_transition <= 1) {
        return true;
      }
      Vec3 direction = Normalize(Sub(second, first));
      constexpr mjtNum kNormalSideTolerance = 1e-7;
      if (first_transition >= 0 && second_transition < transition_count) {
        if (first_transition < transition_count &&
            Dot(direction, mesh.faces[strip[first_transition + 1]].normal) <
                -kNormalSideTolerance) {
          return false;
        }
        if (second_transition >= 0 &&
            Dot(direction, mesh.faces[strip[second_transition]].normal) >
                kNormalSideTolerance) {
          return false;
        }
      }
      return SegmentClearOfMesh(m, d, wrap->geom_id,
                                LocalToWorld(d, wrap->geom_id, first),
                                LocalToWorld(d, wrap->geom_id, second));
    };
    if (!reuse) {
      auto seed_visible_parameter = [&](bool entry, int transition) {
        mjtNum best_parameter = -1;
        mjtNum best_distance = std::numeric_limits<mjtNum>::infinity();
        for (int sample = 0; sample <= 64; ++sample) {
          mjtNum parameter = sample / 64.0;
          Vec3 point =
              EdgePoint(mesh.vertices, shared_edges[transition], parameter);
          bool visible =
              entry ? span_clear(-1, previous, transition, point)
                    : span_clear(transition, point, transition_count, next);
          if (!visible) {
            continue;
          }
          const Vec3 &endpoint_world = entry ? previous_world : next_world;
          Vec3 point_world = LocalToWorld(d, wrap->geom_id, point);
          mjtNum distance = Distance(endpoint_world, point_world);
          if (distance < best_distance) {
            best_distance = distance;
            best_parameter = parameter;
          }
        }
        return best_parameter;
      };
      mjtNum entry_parameter = seed_visible_parameter(true, 0);
      mjtNum exit_parameter =
          seed_visible_parameter(false, transition_count - 1);
      if (entry_parameter < 0 || exit_parameter < 0) {
        mju_warning(
            "nonconvex surface route could not seed visible endpoint contacts on "
            "mesh geom %d",
            wrap->geom_id);
        wrap->valid = false;
        return false;
      }
      parameters.front() = entry_parameter;
      parameters.back() = exit_parameter;
    }
    auto route_point = [&](int transition) {
      if (transition < 0) {
        return previous;
      }
      if (transition >= transition_count) {
        return next;
      }
      return transition_point(transition);
    };
    auto clear_span = [&](int first, int second) {
      return span_clear(first, route_point(first), second, route_point(second));
    };

    if (!reuse || mesh.active_transitions.empty()) {
      mesh.mandatory_transitions.clear();
      // The seed face fixes the initialized homotopy corridor, but the hint is
      // not a runtime waypoint.  Keep only the two transitions bordering the
      // seed face mandatory; all other contacts may be removed when a shorter
      // collision-free chord satisfies the unilateral contact condition.
      if (seed_position > 0) {
        mesh.mandatory_transitions.push_back(seed_position - 1);
      }
      if (seed_position >= 0 && seed_position < transition_count) {
        mesh.mandatory_transitions.push_back(seed_position);
      }
      std::sort(mesh.mandatory_transitions.begin(),
                mesh.mandatory_transitions.end());
      mesh.mandatory_transitions.erase(
          std::unique(mesh.mandatory_transitions.begin(),
                      mesh.mandatory_transitions.end()),
          mesh.mandatory_transitions.end());

      std::vector<int> boundaries = {-1};
      boundaries.insert(boundaries.end(), mesh.mandatory_transitions.begin(),
                        mesh.mandatory_transitions.end());
      boundaries.push_back(transition_count);
      mesh.active_transitions.clear();
      for (int interval = 0; interval + 1 < static_cast<int>(boundaries.size());
           ++interval) {
        int current = boundaries[interval];
        int target = boundaries[interval + 1];
        while (current < target) {
          if (clear_span(current, target)) {
            break;
          }
          int selected = current + 1;
          for (int candidate = target - 1; candidate > current; --candidate) {
            if (clear_span(current, candidate)) {
              selected = candidate;
              break;
            }
          }
          if (selected < target) {
            mesh.active_transitions.push_back(selected);
          }
          current = selected;
        }
        if (target < transition_count) {
          mesh.active_transitions.push_back(target);
        }
      }
      std::sort(mesh.active_transitions.begin(), mesh.active_transitions.end());
      mesh.active_transitions.erase(std::unique(mesh.active_transitions.begin(),
                                                mesh.active_transitions.end()),
                                    mesh.active_transitions.end());
    } else {
      std::vector<int> repaired;
      int previous_transition = -1;
      std::vector<int> boundaries = mesh.active_transitions;
      boundaries.push_back(transition_count);
      for (int transition : boundaries) {
        if (!clear_span(previous_transition, transition)) {
          for (int skipped = previous_transition + 1; skipped < transition;
               ++skipped) {
            repaired.push_back(skipped);
          }
        }
        if (transition < transition_count) {
          repaired.push_back(transition);
        }
        previous_transition = transition;
      }
      std::sort(repaired.begin(), repaired.end());
      repaired.erase(std::unique(repaired.begin(), repaired.end()),
                     repaired.end());
      mesh.active_transitions = std::move(repaired);
    }

    auto reseed_endpoint_contact = [&](bool entry, int transition) {
      mjtNum best_parameter = -1;
      mjtNum best_distance = std::numeric_limits<mjtNum>::infinity();
      for (int sample = 0; sample <= 96; ++sample) {
        mjtNum parameter = sample / 96.0;
        Vec3 point =
            EdgePoint(mesh.vertices, shared_edges[transition], parameter);
        bool visible =
            entry ? span_clear(-1, previous, transition, point)
                  : span_clear(transition, point, transition_count, next);
        if (!visible) {
          continue;
        }
        const Vec3 &endpoint_world = entry ? previous_world : next_world;
        Vec3 point_world = LocalToWorld(d, wrap->geom_id, point);
        mjtNum candidate_distance = Distance(endpoint_world, point_world);
        if (candidate_distance < best_distance) {
          best_distance = candidate_distance;
          best_parameter = parameter;
        }
      }
      if (best_parameter >= 0) {
        parameters[transition] = best_parameter;
      }
      return best_parameter >= 0;
    };
    bool endpoint_contacts_valid = !mesh.active_transitions.empty();
    if (endpoint_contacts_valid &&
        !clear_span(-1, mesh.active_transitions.front())) {
      endpoint_contacts_valid =
          reseed_endpoint_contact(true, mesh.active_transitions.front());
    }
    if (endpoint_contacts_valid &&
        !clear_span(mesh.active_transitions.back(), transition_count)) {
      endpoint_contacts_valid =
          reseed_endpoint_contact(false, mesh.active_transitions.back());
    }
    if (!endpoint_contacts_valid) {
      if (!initialize) {
        mesh.strip.clear();
        mesh.parameters.clear();
        mesh.active_transitions.clear();
        mesh.mandatory_transitions.clear();
        return SolveMesh(m, d, wrap, previous_world, next_world, true);
      }
      mju_warning(
          "nonconvex surface route endpoint contacts are invalid on mesh geom %d",
          wrap->geom_id);
      wrap->valid = false;
      return false;
    }

    auto optimize_active = [&](int sweeps, int golden_iterations) {
      mjtNum maximum_residual = 0;
      for (int sweep = 0; sweep < sweeps; ++sweep) {
        mjtNum maximum_change = 0;
        for (int active_index = 0;
             active_index < static_cast<int>(mesh.active_transitions.size());
             ++active_index) {
          int transition = mesh.active_transitions[active_index];
          Vec3 before =
              active_index == 0
                  ? previous
                  : transition_point(mesh.active_transitions[active_index - 1]);
          Vec3 after =
              active_index + 1 ==
                      static_cast<int>(mesh.active_transitions.size())
                  ? next
                  : transition_point(mesh.active_transitions[active_index + 1]);
          int before_transition =
              active_index == 0 ? -1
                                : mesh.active_transitions[active_index - 1];
          int after_transition =
              active_index + 1 ==
                      static_cast<int>(mesh.active_transitions.size())
                  ? transition_count
                  : mesh.active_transitions[active_index + 1];
          auto local_objective = [&](mjtNum parameter) {
            Vec3 point =
                EdgePoint(mesh.vertices, shared_edges[transition], parameter);
            if ((before_transition < 0 || transition - before_transition > 1) &&
                !span_clear(before_transition, before, transition, point)) {
              return mjtNum(1e6) + Distance(before, point) +
                     Distance(point, after);
            }
            if ((after_transition >= transition_count ||
                 after_transition - transition > 1) &&
                !span_clear(transition, point, after_transition, after)) {
              return mjtNum(1e6) + Distance(before, point) +
                     Distance(point, after);
            }
            return Distance(before, point) + Distance(point, after);
          };
          mjtNum original_parameter = parameters[transition];
          mjtNum lower = 0;
          mjtNum upper = 1;
          constexpr mjtNum kGolden = 0.6180339887498948482;
          mjtNum left = upper - kGolden * (upper - lower);
          mjtNum right = lower + kGolden * (upper - lower);
          mjtNum left_value = local_objective(left);
          mjtNum right_value = local_objective(right);
          for (int search = 0; search < golden_iterations; ++search) {
            if (left_value < right_value) {
              upper = right;
              right = left;
              right_value = left_value;
              left = upper - kGolden * (upper - lower);
              left_value = local_objective(left);
            } else {
              lower = left;
              left = right;
              left_value = right_value;
              right = lower + kGolden * (upper - lower);
              right_value = local_objective(right);
            }
          }
          mjtNum updated = 0.5 * (lower + upper);
          if (local_objective(updated) >= 1e5) {
            updated = original_parameter;
          }
          maximum_change = std::max(maximum_change,
                                    std::abs(updated - parameters[transition]));
          parameters[transition] = updated;
        }
        maximum_residual = maximum_change;
        if (maximum_change < 1e-10) {
          break;
        }
      }
      return maximum_residual;
    };

    int iterations = 0;
    int optimization_sweeps = initialize ? 12 : 1;
    int golden_iterations = initialize ? 28 : (mesh.guide_plane_valid ? 5 : 8);
    mjtNum coordinate_residual =
        optimize_active(optimization_sweeps, golden_iterations);
    iterations += optimization_sweeps;

    bool changed = true;
    for (int pass = 0; pass < 4 && changed && (initialize || !reuse); ++pass) {
      changed = false;
      for (int active_index = 0;
           active_index < static_cast<int>(mesh.active_transitions.size());
           ++active_index) {
        int transition = mesh.active_transitions[active_index];
        if (std::binary_search(mesh.mandatory_transitions.begin(),
                               mesh.mandatory_transitions.end(), transition)) {
          continue;
        }
        Vec3 point = transition_point(transition);
        Vec3 before =
            active_index == 0
                ? previous
                : transition_point(mesh.active_transitions[active_index - 1]);
        Vec3 after =
            active_index + 1 == static_cast<int>(mesh.active_transitions.size())
                ? next
                : transition_point(mesh.active_transitions[active_index + 1]);
        Vec3 reaction =
            Add(Normalize(Sub(before, point)), Normalize(Sub(after, point)));
        Vec3 normal = Normalize(Add(mesh.faces[strip[transition]].normal,
                                    mesh.faces[strip[transition + 1]].normal));
        int before_transition =
            active_index == 0 ? -1 : mesh.active_transitions[active_index - 1];
        int after_transition =
            active_index + 1 == static_cast<int>(mesh.active_transitions.size())
                ? transition_count
                : mesh.active_transitions[active_index + 1];
        mjtNum removal_gain = Distance(before, point) + Distance(point, after) -
                              Distance(before, after);
        if (Dot(reaction, normal) > 1e-7 &&
            removal_gain > route_hysteresis_ &&
            clear_span(before_transition, after_transition)) {
          mesh.active_transitions.erase(mesh.active_transitions.begin() +
                                        active_index);
          changed = true;
          break;
        }
      }
      if (changed) {
        coordinate_residual =
            std::max(coordinate_residual, optimize_active(6, 22));
        iterations += 6;
      }
    }

    if (mesh.active_transitions.empty()) {
      mju_warning(
          "nonconvex surface route has no active surface contacts on mesh geom %d",
          wrap->geom_id);
      wrap->valid = false;
      return false;
    }
    if (!initialize) {
      int previous_active = -1;
      for (int active : mesh.active_transitions) {
        if (!clear_span(previous_active, active)) {
          mju_warning("nonconvex surface route lost a surface span on mesh geom %d",
                      wrap->geom_id);
          wrap->valid = false;
          return false;
        }
        previous_active = active;
      }
      if (!clear_span(previous_active, transition_count)) {
        mju_warning("nonconvex surface route lost its exit span on mesh geom %d",
                    wrap->geom_id);
        wrap->valid = false;
        return false;
      }
    }
    mjtNum tangent_residual = 0;
    wrap->contact_points.clear();
    for (int active_index = 0;
         active_index < static_cast<int>(mesh.active_transitions.size());
         ++active_index) {
      int transition = mesh.active_transitions[active_index];
      Vec3 point = transition_point(transition);
      Vec3 before =
          active_index == 0
              ? previous
              : transition_point(mesh.active_transitions[active_index - 1]);
      Vec3 after =
          active_index + 1 == static_cast<int>(mesh.active_transitions.size())
              ? next
              : transition_point(mesh.active_transitions[active_index + 1]);
      int before_transition =
          active_index == 0 ? -1 : mesh.active_transitions[active_index - 1];
      int after_transition =
          active_index + 1 == static_cast<int>(mesh.active_transitions.size())
              ? transition_count
              : mesh.active_transitions[active_index + 1];
      auto constrained_length = [&](mjtNum parameter) {
        Vec3 candidate =
            EdgePoint(mesh.vertices, shared_edges[transition], parameter);
        if ((before_transition < 0 || transition - before_transition > 1) &&
            !span_clear(before_transition, before, transition, candidate)) {
          return std::numeric_limits<mjtNum>::infinity();
        }
        if (after_transition >= transition_count) {
          if (!span_clear(transition, candidate, after_transition, after)) {
            return std::numeric_limits<mjtNum>::infinity();
          }
        } else if (after_transition - transition > 1 &&
                   !span_clear(transition, candidate, after_transition,
                               after)) {
          return std::numeric_limits<mjtNum>::infinity();
        }
        return Distance(before, candidate) + Distance(candidate, after);
      };
      mjtNum parameter = parameters[transition];
      mjtNum base_length = constrained_length(parameter);
      constexpr mjtNum kResidualStep = 1e-5;
      for (mjtNum candidate :
           {std::max(mjtNum(0), parameter - kResidualStep),
            std::min(mjtNum(1), parameter + kResidualStep)}) {
        mjtNum step = std::abs(candidate - parameter);
        if (step <= mjMINVAL) {
          continue;
        }
        mjtNum candidate_length = constrained_length(candidate);
        if (std::isfinite(candidate_length)) {
          tangent_residual =
              std::max(tangent_residual,
                       std::max(mjtNum(0), base_length - candidate_length));
        }
      }
      Point contact{LocalToWorld(d, wrap->geom_id, point), wrap->body_id, -1,
                    wrap->geom_id, -1};
      contact.local_pos = point;
      wrap->contact_points.push_back(contact);
    }
    mesh.strip = std::move(strip);
    mesh.parameters = std::move(parameters);
    static_cast<void>(coordinate_residual);
    wrap->tangent_residual = tangent_residual;
    wrap->surface_residual = 0;
    wrap->solver_iterations = iterations;
    wrap->initialized = true;
    wrap->valid = !wrap->contact_points.empty();
    return wrap->valid;
  }

  auto projection = [transition_count](std::vector<mjtNum> *value) {
    for (int transition = 0; transition < transition_count; ++transition) {
      (*value)[transition] = Clamp((*value)[transition], 0, 1);
    }
  };
  auto build_nodes = [&mesh, &shared_edges, transition_count, &previous,
                      &next](const std::vector<mjtNum> &value) {
    std::vector<Vec3> nodes;
    nodes.push_back(previous);
    for (int transition = 0; transition < transition_count; ++transition) {
      nodes.push_back(EdgePoint(mesh.vertices, shared_edges[transition],
                                value[transition]));
    }
    nodes.push_back(next);
    return nodes;
  };
  auto objective = [&mesh, &shared_edges, transition_count,
                    &build_nodes](const std::vector<mjtNum> &value,
                                  std::vector<mjtNum> *gradient) {
    std::vector<Vec3> nodes = build_nodes(value);
    mjtNum length = 0;
    for (int i = 0; i + 1 < static_cast<int>(nodes.size()); ++i) {
      length += Distance(nodes[i], nodes[i + 1]);
    }
    if (!gradient) {
      return length;
    }
    gradient->assign(transition_count, 0);
    std::vector<Vec3> point_gradient(nodes.size(), {0, 0, 0});
    for (int i = 0; i + 1 < static_cast<int>(nodes.size()); ++i) {
      Vec3 direction = Normalize(Sub(nodes[i + 1], nodes[i]));
      point_gradient[i] = Sub(point_gradient[i], direction);
      point_gradient[i + 1] = Add(point_gradient[i + 1], direction);
    }
    for (int transition = 0; transition < transition_count; ++transition) {
      Vec3 edge_direction = Sub(mesh.vertices[shared_edges[transition][1]],
                                mesh.vertices[shared_edges[transition][0]]);
      (*gradient)[transition] =
          Dot(point_gradient[1 + transition], edge_direction);
    }
    return length;
  };

  int iterations = 0;
  OptimizeBfgs(&parameters, initialize ? 100 : 24, objective, projection,
               &iterations);
  projection(&parameters);
  mjtNum coordinate_residual = 0;
  const int coordinate_sweeps = initialize ? 30 : 20;
  const int golden_iterations = initialize ? 32 : 28;
  for (int sweep = 0; sweep < coordinate_sweeps; ++sweep) {
    mjtNum maximum_change = 0;
    for (int coordinate = 0; coordinate < transition_count; ++coordinate) {
      mjtNum lower = 0;
      mjtNum upper = 1;
      constexpr mjtNum kGolden = 0.6180339887498948482;
      mjtNum left = upper - kGolden * (upper - lower);
      mjtNum right = lower + kGolden * (upper - lower);
      std::vector<mjtNum> candidate = parameters;
      candidate[coordinate] = left;
      mjtNum left_value = objective(candidate, nullptr);
      candidate[coordinate] = right;
      mjtNum right_value = objective(candidate, nullptr);
      for (int search = 0; search < golden_iterations; ++search) {
        if (left_value < right_value) {
          upper = right;
          right = left;
          right_value = left_value;
          left = upper - kGolden * (upper - lower);
          candidate[coordinate] = left;
          left_value = objective(candidate, nullptr);
        } else {
          lower = left;
          left = right;
          left_value = right_value;
          right = lower + kGolden * (upper - lower);
          candidate[coordinate] = right;
          right_value = objective(candidate, nullptr);
        }
      }
      mjtNum updated = 0.5 * (lower + upper);
      maximum_change =
          std::max(maximum_change, std::abs(updated - parameters[coordinate]));
      parameters[coordinate] = updated;
    }
    ++iterations;
    coordinate_residual = maximum_change;
    if (maximum_change < 1e-10) {
      break;
    }
  }
  if (!std::isfinite(objective(parameters, nullptr))) {
    wrap->valid = false;
    return false;
  }
  std::vector<Vec3> nodes = build_nodes(parameters);
  mjtNum tangent_residual = coordinate_residual;

  wrap->contact_points.clear();
  for (int node = 1; node + 1 < static_cast<int>(nodes.size()); ++node) {
    Point point{LocalToWorld(d, wrap->geom_id, nodes[node]), wrap->body_id, -1,
                wrap->geom_id, -1};
    point.local_pos = nodes[node];
    wrap->contact_points.push_back(point);
  }
  mesh.strip = std::move(strip);
  mesh.parameters = std::move(parameters);
  wrap->tangent_residual = tangent_residual;
  wrap->surface_residual = 0;
  wrap->solver_iterations = iterations;
  wrap->initialized = true;
  wrap->valid = !wrap->contact_points.empty();
  return wrap->valid;
}

int SurfaceEnvelopeRoute::SolveCompositePairs(const mjModel *m,
                                               const mjData *d) {
  if (composite_merge_distance_ <= 0 || wraps_.size() < 2) {
    return 0;
  }

  auto entry_position = [this, d](const Element &element) {
    if (element.type == Element::Type::kSite) {
      return PointerVec(d->site_xpos + 3 * hard_site_ids_[element.index]);
    }
    return wraps_[element.index].contact_points.front().pos;
  };
  auto exit_position = [this, d](const Element &element) {
    if (element.type == Element::Type::kSite) {
      return PointerVec(d->site_xpos + 3 * hard_site_ids_[element.index]);
    }
    return wraps_[element.index].contact_points.back().pos;
  };

  auto composite_tangent_error = [](const Wrap &left, const Wrap &right) {
    if (left.contact_points.size() < 2 || right.contact_points.size() < 2) {
      return std::numeric_limits<mjtNum>::infinity();
    }
    constexpr mjtNum kSampleLength = 5e-4;
    auto terminal_direction = [kSampleLength](const std::vector<Point> &points,
                                              bool departure) {
      int endpoint_index = departure ? static_cast<int>(points.size()) - 1 : 0;
      Vec3 endpoint = points[endpoint_index].pos;
      Vec3 cursor = endpoint;
      mjtNum remaining = kSampleLength;
      int index = departure ? endpoint_index - 1 : 1;
      while (index >= 0 && index < static_cast<int>(points.size()) &&
             remaining > 0) {
        Vec3 target = points[index].pos;
        mjtNum available = Distance(cursor, target);
        if (available >= remaining && available > mjMINVAL) {
          cursor = Add(cursor,
                       Scale(Sub(target, cursor), remaining / available));
          break;
        }
        remaining -= available;
        cursor = target;
        index += departure ? -1 : 1;
      }
      return departure ? Normalize(Sub(endpoint, cursor))
                       : Normalize(Sub(cursor, endpoint));
    };
    Vec3 bridge = Normalize(Sub(right.contact_points.front().pos,
                                left.contact_points.back().pos));
    return std::max(
        Norm(Sub(terminal_direction(left.contact_points, true), bridge)),
        Norm(Sub(bridge, terminal_direction(right.contact_points, false))));
  };

  std::function<bool(int, Wrap *, Wrap *, const Vec3 &, const Vec3 &)>
      solve_polyline_common_tangent;
  solve_polyline_common_tangent =
      [this, m, d, &solve_polyline_common_tangent](int pair, Wrap *left,
                                                   Wrap *right,
                                                   const Vec3 &previous,
                                                   const Vec3 &next) {
    if (left->geom_type != mjGEOM_MESH || right->geom_type != mjGEOM_MESH ||
        !left->mesh.guided_anchor_valid ||
        !right->mesh.guided_anchor_valid || left->contact_points.size() < 2 ||
        right->contact_points.size() < 2) {
      return true;
    }

    auto closest_anchor = [](const Wrap &wrap) {
      int closest = 0;
      mjtNum best = std::numeric_limits<mjtNum>::infinity();
      for (int index = 0;
           index < static_cast<int>(wrap.contact_points.size()); ++index) {
        mjtNum distance = Distance(wrap.contact_points[index].local_pos,
                                   wrap.mesh.guided_anchor);
        if (distance < best) {
          best = distance;
          closest = index;
        }
      }
      return closest;
    };
    int left_anchor = closest_anchor(*left);
    int right_anchor = closest_anchor(*right);
    if (left_anchor <= 0 ||
        right_anchor + 1 >= static_cast<int>(right->contact_points.size())) {
      return true;
    }

    const int left_count = static_cast<int>(left->contact_points.size());
    const int right_count = static_cast<int>(right->contact_points.size());
    std::vector<mjtNum> left_prefix(left_count, 0);
    for (int index = 1; index < left_count; ++index) {
      left_prefix[index] =
          left_prefix[index - 1] +
          Distance(left->contact_points[index - 1].pos,
                   left->contact_points[index].pos);
    }
    std::vector<mjtNum> right_suffix(right_count, 0);
    for (int index = right_count - 2; index >= 0; --index) {
      right_suffix[index] =
          right_suffix[index + 1] +
          Distance(right->contact_points[index].pos,
                   right->contact_points[index + 1].pos);
    }

    struct Candidate {
      mjtNum objective = std::numeric_limits<mjtNum>::infinity();
      mjtNum tangent_error = std::numeric_limits<mjtNum>::infinity();
      int left_segment = -1;
      int right_segment = -1;
      mjtNum left_parameter = 0;
      mjtNum right_parameter = 0;
      Vec3 left_point = {0, 0, 0};
      Vec3 right_point = {0, 0, 0};
    };
    std::vector<Candidate> candidates;
    bool warm = pair < static_cast<int>(composite_left_segment_.size()) &&
                composite_left_segment_[pair] >= 0 &&
                composite_right_segment_[pair] >= 0;
    int left_begin = 0;
    int left_end = left_count - 2;
    int right_begin = 0;
    int right_end = right_count - 2;
    int warm_left_segment = -1;
    int warm_right_segment = -1;
    if (warm) {
      constexpr int kWarmWindow = 1;
      auto nearest_segment = [](const std::vector<Point> &points,
                                const Vec3 &target, int begin, int end) {
        int best_segment = begin;
        mjtNum best_distance = std::numeric_limits<mjtNum>::infinity();
        for (int segment = begin; segment <= end; ++segment) {
          Vec3 delta = Sub(points[segment + 1].local_pos,
                           points[segment].local_pos);
          mjtNum denominator = Dot(delta, delta);
          mjtNum parameter =
              denominator > mjMINVAL
                  ? Clamp(Dot(Sub(target, points[segment].local_pos), delta) /
                              denominator,
                          0, 1)
                  : 0;
          Vec3 closest = Add(points[segment].local_pos,
                             Scale(delta, parameter));
          mjtNum distance = Distance(target, closest);
          if (distance < best_distance) {
            best_distance = distance;
            best_segment = segment;
          }
        }
        return best_segment;
      };
      warm_left_segment = nearest_segment(
          left->contact_points, composite_left_local_[pair], left_begin,
          left_end);
      warm_right_segment = nearest_segment(
          right->contact_points, composite_right_local_[pair], right_begin,
          right_end);
      left_begin = std::max(left_begin, warm_left_segment - kWarmWindow);
      left_end = std::min(left_end, warm_left_segment + kWarmWindow);
      right_begin = std::max(right_begin, warm_right_segment - kWarmWindow);
      right_end = std::min(right_end, warm_right_segment + kWarmWindow);
      if (left_begin > left_end || right_begin > right_end) {
        warm = false;
        left_begin = 0;
        left_end = left_count - 2;
        right_begin = 0;
        right_end = right_count - 2;
      }
    }
    candidates.reserve((left_end - left_begin + 1) *
                       (right_end - right_begin + 1));
    constexpr mjtNum kGolden = 0.6180339887498948482;
    constexpr mjtNum kTangentSampleLength = 5e-4;
    auto departure_direction = [&](int segment, mjtNum parameter) {
      Vec3 point = Add(
          left->contact_points[segment].pos,
          Scale(Sub(left->contact_points[segment + 1].pos,
                    left->contact_points[segment].pos),
                parameter));
      Vec3 cursor = point;
      mjtNum remaining = kTangentSampleLength;
      int current_segment = segment;
      while (current_segment >= 0 && remaining > 0) {
        Vec3 target = left->contact_points[current_segment].pos;
        mjtNum available = Distance(cursor, target);
        if (available >= remaining && available > mjMINVAL) {
          target = Add(cursor,
                       Scale(Sub(target, cursor), remaining / available));
          return Normalize(Sub(point, target));
        }
        remaining -= available;
        cursor = target;
        --current_segment;
      }
      return Normalize(Sub(point, cursor));
    };
    auto arrival_direction = [&](int segment, mjtNum parameter) {
      Vec3 point = Add(
          right->contact_points[segment].pos,
          Scale(Sub(right->contact_points[segment + 1].pos,
                    right->contact_points[segment].pos),
                parameter));
      Vec3 cursor = point;
      mjtNum remaining = kTangentSampleLength;
      int current_segment = segment;
      while (current_segment + 1 < right_count && remaining > 0) {
        Vec3 target = right->contact_points[current_segment + 1].pos;
        mjtNum available = Distance(cursor, target);
        if (available >= remaining && available > mjMINVAL) {
          target = Add(cursor,
                       Scale(Sub(target, cursor), remaining / available));
          return Normalize(Sub(target, point));
        }
        remaining -= available;
        cursor = target;
        ++current_segment;
      }
      return Normalize(Sub(cursor, point));
    };
    // The hint only chooses the homotopy side.  A taut belt may depart before
    // reaching the first seed face and arrive after leaving the second seed
    // face, thereby replacing the near-contact projections with an internal
    // or external common tangent.
    for (int left_segment = left_begin; left_segment <= left_end;
         ++left_segment) {
      const Vec3 &left_first = left->contact_points[left_segment].pos;
      const Vec3 &left_second = left->contact_points[left_segment + 1].pos;
      mjtNum left_length = Distance(left_first, left_second);
      if (left_length <= mjMINVAL) {
        continue;
      }
      for (int right_segment = right_begin; right_segment <= right_end;
           ++right_segment) {
        const Vec3 &right_first = right->contact_points[right_segment].pos;
        const Vec3 &right_second =
            right->contact_points[right_segment + 1].pos;
        mjtNum right_length = Distance(right_first, right_second);
        if (right_length <= mjMINVAL) {
          continue;
        }
        auto point_on = [](const Vec3 &first, const Vec3 &second,
                           mjtNum parameter) {
          return Add(first, Scale(Sub(second, first), parameter));
        };
        mjtNum tangent_weight =
            10 * std::max(left->mesh.scale, right->mesh.scale);
        auto objective = [&](mjtNum left_parameter,
                             mjtNum right_parameter) {
          Vec3 left_point =
              point_on(left_first, left_second, left_parameter);
          Vec3 right_point =
              point_on(right_first, right_second, right_parameter);
          Vec3 bridge = Normalize(Sub(right_point, left_point));
          Vec3 left_direction =
              departure_direction(left_segment, left_parameter);
          Vec3 right_direction =
              arrival_direction(right_segment, right_parameter);
          Vec3 departure_error = Sub(left_direction, bridge);
          Vec3 arrival_error = Sub(bridge, right_direction);
          mjtNum tangent_penalty = Dot(departure_error, departure_error) +
                                   Dot(arrival_error, arrival_error);
          return left_prefix[left_segment] + left_parameter * left_length +
                 Distance(left_point, right_point) +
                 (1 - right_parameter) * right_length +
                 right_suffix[right_segment + 1] +
                 tangent_weight * tangent_penalty;
        };
        auto minimize_coordinate = [&](bool optimize_left, mjtNum fixed) {
          mjtNum lower = 0;
          mjtNum upper = 1;
          mjtNum first = upper - kGolden * (upper - lower);
          mjtNum second = lower + kGolden * (upper - lower);
          auto value = [&](mjtNum parameter) {
            return optimize_left ? objective(parameter, fixed)
                                 : objective(fixed, parameter);
          };
          mjtNum first_value = value(first);
          mjtNum second_value = value(second);
          int golden_iterations = warm ? 5 : 18;
          for (int iteration = 0; iteration < golden_iterations; ++iteration) {
            if (first_value < second_value) {
              upper = second;
              second = first;
              second_value = first_value;
              first = upper - kGolden * (upper - lower);
              first_value = value(first);
            } else {
              lower = first;
              first = second;
              first_value = second_value;
              second = lower + kGolden * (upper - lower);
              second_value = value(second);
            }
          }
          return mjtNum(0.5) * (lower + upper);
        };
        mjtNum left_parameter = 0.5;
        mjtNum right_parameter = 0.5;
        int coordinate_sweeps = warm ? 1 : 6;
        for (int sweep = 0; sweep < coordinate_sweeps; ++sweep) {
          left_parameter = minimize_coordinate(true, right_parameter);
          right_parameter = minimize_coordinate(false, left_parameter);
        }
        Candidate candidate;
        candidate.objective = objective(left_parameter, right_parameter);
        candidate.left_segment = left_segment;
        candidate.right_segment = right_segment;
        candidate.left_parameter = left_parameter;
        candidate.right_parameter = right_parameter;
        candidate.left_point =
            point_on(left_first, left_second, left_parameter);
        candidate.right_point =
            point_on(right_first, right_second, right_parameter);
        Vec3 candidate_bridge =
            Normalize(Sub(candidate.right_point, candidate.left_point));
        candidate.tangent_error = std::max(
            Norm(Sub(departure_direction(left_segment, left_parameter),
                     candidate_bridge)),
            Norm(Sub(candidate_bridge,
                     arrival_direction(right_segment, right_parameter))));
        candidates.push_back(candidate);
      }
    }
    if (warm && warm_left_segment >= 0 && warm_right_segment >= 0) {
      auto cached_parameter = [](const Point &first, const Point &second,
                                 const Vec3 &target) {
        Vec3 edge = Sub(second.local_pos, first.local_pos);
        mjtNum denominator = Dot(edge, edge);
        return denominator > mjMINVAL
                   ? Clamp(Dot(Sub(target, first.local_pos), edge) /
                               denominator,
                           0, 1)
                   : mjtNum(0);
      };
      Candidate cached;
      cached.left_segment = warm_left_segment;
      cached.right_segment = warm_right_segment;
      cached.left_parameter = cached_parameter(
          left->contact_points[warm_left_segment],
          left->contact_points[warm_left_segment + 1],
          composite_left_local_[pair]);
      cached.right_parameter = cached_parameter(
          right->contact_points[warm_right_segment],
          right->contact_points[warm_right_segment + 1],
          composite_right_local_[pair]);
      auto point_on = [](const Vec3 &first, const Vec3 &second,
                         mjtNum parameter) {
        return Add(first, Scale(Sub(second, first), parameter));
      };
      cached.left_point = point_on(
          left->contact_points[warm_left_segment].pos,
          left->contact_points[warm_left_segment + 1].pos,
          cached.left_parameter);
      cached.right_point = point_on(
          right->contact_points[warm_right_segment].pos,
          right->contact_points[warm_right_segment + 1].pos,
          cached.right_parameter);
      Vec3 bridge = Normalize(Sub(cached.right_point, cached.left_point));
      Vec3 departure =
          departure_direction(warm_left_segment, cached.left_parameter);
      Vec3 arrival =
          arrival_direction(warm_right_segment, cached.right_parameter);
      cached.tangent_error =
          std::max(Norm(Sub(departure, bridge)), Norm(Sub(bridge, arrival)));
      mjtNum left_segment_length = Distance(
          left->contact_points[warm_left_segment].pos,
          left->contact_points[warm_left_segment + 1].pos);
      mjtNum right_segment_length = Distance(
          right->contact_points[warm_right_segment].pos,
          right->contact_points[warm_right_segment + 1].pos);
      mjtNum tangent_weight =
          10 * std::max(left->mesh.scale, right->mesh.scale);
      cached.objective =
          left_prefix[warm_left_segment] +
          cached.left_parameter * left_segment_length +
          Distance(cached.left_point, cached.right_point) +
          (1 - cached.right_parameter) * right_segment_length +
          right_suffix[warm_right_segment + 1] +
          tangent_weight * cached.tangent_error * cached.tangent_error;
      candidates.push_back(cached);
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate &first, const Candidate &second) {
                if (std::abs(first.tangent_error - second.tangent_error) >
                    1e-8) {
                  return first.tangent_error < second.tangent_error;
                }
                return first.objective < second.objective;
              });

    const Candidate *best = nullptr;
    for (const Candidate &candidate : candidates) {
      Vec3 bridge = Normalize(Sub(candidate.right_point, candidate.left_point));
      Vec3 left_direction = departure_direction(
          candidate.left_segment, candidate.left_parameter);
      Vec3 right_direction = arrival_direction(
          candidate.right_segment, candidate.right_parameter);
      if (Dot(left_direction, bridge) <= 0 ||
          Dot(bridge, right_direction) <= 0) {
        continue;
      }
      if (SegmentClearOfMesh(m, d, left->geom_id, candidate.left_point,
                             candidate.right_point) &&
          SegmentClearOfMesh(m, d, right->geom_id, candidate.left_point,
                             candidate.right_point)) {
        best = &candidate;
        break;
      }
    }
    if (best == nullptr) {
      if (warm) {
        composite_left_segment_[pair] = -1;
        composite_right_segment_[pair] = -1;
        return solve_polyline_common_tangent(pair, left, right, previous, next);
      }
      return false;
    }
    composite_left_segment_[pair] = best->left_segment;
    composite_right_segment_[pair] = best->right_segment;
    composite_left_local_[pair] = Add(
        left->contact_points[best->left_segment].local_pos,
        Scale(Sub(left->contact_points[best->left_segment + 1].local_pos,
                  left->contact_points[best->left_segment].local_pos),
              best->left_parameter));
    composite_right_local_[pair] = Add(
        right->contact_points[best->right_segment].local_pos,
        Scale(Sub(right->contact_points[best->right_segment + 1].local_pos,
                  right->contact_points[best->right_segment].local_pos),
              best->right_parameter));

    auto interpolate_point = [](const Point &first, const Point &second,
                                mjtNum parameter) {
      Point point = first;
      point.pos = Add(first.pos, Scale(Sub(second.pos, first.pos), parameter));
      point.local_pos = Add(
          first.local_pos,
          Scale(Sub(second.local_pos, first.local_pos), parameter));
      return point;
    };
    Point left_cut = interpolate_point(
        left->contact_points[best->left_segment],
        left->contact_points[best->left_segment + 1], best->left_parameter);
    Point right_cut = interpolate_point(
        right->contact_points[best->right_segment],
        right->contact_points[best->right_segment + 1], best->right_parameter);

    std::vector<Point> left_points(
        left->contact_points.begin(),
        left->contact_points.begin() + best->left_segment + 1);
    if (Distance(left_points.back().pos, left_cut.pos) > 1e-12) {
      left_points.push_back(left_cut);
    } else {
      left_points.back() = left_cut;
    }
    std::vector<Point> right_points;
    right_points.reserve(right_count - best->right_segment);
    right_points.push_back(right_cut);
    int right_start = best->right_segment + 1;
    if (Distance(right_cut.pos, right->contact_points[right_start].pos) <
        1e-12) {
      ++right_start;
    }
    right_points.insert(right_points.end(),
                        right->contact_points.begin() + right_start,
                        right->contact_points.end());
    left->contact_points = std::move(left_points);
    right->contact_points = std::move(right_points);

    // The candidate above identifies which parts of the two initialized
    // corridors belong to the composite route.  Refine those retained edge
    // variables against the shared bridge instead of accepting a cut through
    // two frozen polylines.  This is a block-coordinate solve of
    //   L = L_left(previous, p) + |q - p| + L_right(q, next),
    // and therefore drives both departure and arrival toward the same common
    // tangent while preserving the selected triangle corridors.
    Wrap fallback_left = *left;
    Wrap fallback_right = *right;
    auto route_tangent_error = [](const Wrap &first, const Wrap &second) {
      if (first.contact_points.size() < 2 ||
          second.contact_points.size() < 2) {
        return std::numeric_limits<mjtNum>::infinity();
      }
      constexpr mjtNum kSampleLength = 5e-4;
      auto departure_direction = [&](const std::vector<Point> &points) {
        Vec3 endpoint = points.back().pos;
        Vec3 cursor = endpoint;
        mjtNum remaining = kSampleLength;
        for (int index = static_cast<int>(points.size()) - 2; index >= 0;
             --index) {
          Vec3 target = points[index].pos;
          mjtNum available = Distance(cursor, target);
          if (available >= remaining && available > mjMINVAL) {
            cursor = Add(cursor,
                         Scale(Sub(target, cursor), remaining / available));
            break;
          }
          remaining -= available;
          cursor = target;
        }
        return Normalize(Sub(endpoint, cursor));
      };
      auto arrival_direction = [&](const std::vector<Point> &points) {
        Vec3 endpoint = points.front().pos;
        Vec3 cursor = endpoint;
        mjtNum remaining = kSampleLength;
        for (int index = 1; index < static_cast<int>(points.size()); ++index) {
          Vec3 target = points[index].pos;
          mjtNum available = Distance(cursor, target);
          if (available >= remaining && available > mjMINVAL) {
            cursor = Add(cursor,
                         Scale(Sub(target, cursor), remaining / available));
            break;
          }
          remaining -= available;
          cursor = target;
        }
        return Normalize(Sub(cursor, endpoint));
      };
      Vec3 bridge = Normalize(Sub(second.contact_points.front().pos,
                                  first.contact_points.back().pos));
      Vec3 departure = departure_direction(first.contact_points);
      Vec3 arrival = arrival_direction(second.contact_points);
      return std::max(Norm(Sub(departure, bridge)),
                      Norm(Sub(bridge, arrival)));
    };
    mjtNum fallback_error = route_tangent_error(*left, *right);
    Wrap refined_left = fallback_left;
    Wrap refined_right = fallback_right;
    mjtNum refined_error = fallback_error;
    const int original_left_active =
        static_cast<int>(fallback_left.mesh.active_transitions.size());
    const int original_right_active =
        static_cast<int>(fallback_right.mesh.active_transitions.size());
    if (warm) {
      Wrap trial_left = fallback_left;
      Wrap trial_right = fallback_right;
      trial_left.mesh.mandatory_transitions.clear();
      trial_right.mesh.mandatory_transitions.clear();
      bool valid =
          SolveMesh(m, d, &trial_left, previous,
                    trial_right.contact_points.front().pos, false) &&
          SolveMesh(m, d, &trial_right, trial_left.contact_points.back().pos,
                    next, false);
      if (valid &&
          SegmentClearOfMesh(m, d, trial_left.geom_id,
                             trial_left.contact_points.back().pos,
                             trial_right.contact_points.front().pos) &&
          SegmentClearOfMesh(m, d, trial_right.geom_id,
                             trial_left.contact_points.back().pos,
                             trial_right.contact_points.front().pos)) {
        mjtNum error = route_tangent_error(trial_left, trial_right);
        if (error + 1e-6 < refined_error) {
          refined_error = error;
          refined_left = std::move(trial_left);
          refined_right = std::move(trial_right);
        }
      }
    } else {
      int left_keep_begin = best->left_segment - 2;
      int left_keep_end = best->left_segment + 4;
      int right_drop_begin = best->right_segment - 3;
      int right_drop_end = best->right_segment + 3;
      for (int left_keep = left_keep_begin; left_keep <= left_keep_end;
           ++left_keep) {
        if (left_keep <= 0 || left_keep > original_left_active) {
          continue;
        }
        for (int right_drop = right_drop_begin; right_drop <= right_drop_end;
             ++right_drop) {
          if (right_drop < 0 || right_drop >= original_right_active) {
            continue;
          }
          Wrap trial_left = fallback_left;
          Wrap trial_right = fallback_right;
          trial_left.mesh.active_transitions.resize(left_keep);
          trial_right.mesh.active_transitions.erase(
              trial_right.mesh.active_transitions.begin(),
              trial_right.mesh.active_transitions.begin() + right_drop);
          trial_left.mesh.mandatory_transitions.clear();
          trial_right.mesh.mandatory_transitions.clear();
          bool valid = true;
          constexpr int kJointSweeps = 10;
          for (int sweep = 0; sweep < kJointSweeps; ++sweep) {
            if (!SolveMesh(m, d, &trial_left, previous,
                           trial_right.contact_points.front().pos, false) ||
                !SolveMesh(m, d, &trial_right,
                           trial_left.contact_points.back().pos, next, false)) {
              valid = false;
              break;
            }
            trial_left.mesh.mandatory_transitions.clear();
            trial_right.mesh.mandatory_transitions.clear();
          }
          if (!valid ||
              !SegmentClearOfMesh(m, d, trial_left.geom_id,
                                  trial_left.contact_points.back().pos,
                                  trial_right.contact_points.front().pos) ||
              !SegmentClearOfMesh(m, d, trial_right.geom_id,
                                  trial_left.contact_points.back().pos,
                                  trial_right.contact_points.front().pos)) {
            continue;
          }
          mjtNum error = route_tangent_error(trial_left, trial_right);
          if (error + 1e-6 < refined_error) {
            refined_error = error;
            refined_left = std::move(trial_left);
            refined_right = std::move(trial_right);
          }
        }
      }
    }
    if (refined_error + 1e-6 < fallback_error) {
      *left = std::move(refined_left);
      *right = std::move(refined_right);
      composite_left_local_[pair] = left->contact_points.back().local_pos;
      composite_right_local_[pair] = right->contact_points.front().local_pos;
    } else {
      *left = std::move(fallback_left);
      *right = std::move(fallback_right);
    }
    return !left->contact_points.empty() && !right->contact_points.empty();
  };

  int total_iterations = 0;
  const mjtNum split_distance =
      composite_merge_distance_ +
      std::max(route_hysteresis_, composite_merge_distance_ * mjtNum(0.25));
  for (int element_index = 1;
       element_index + 2 < static_cast<int>(elements_.size());
       ++element_index) {
    const Element &left_element = elements_[element_index];
    const Element &right_element = elements_[element_index + 1];
    if (left_element.type != Element::Type::kWrap ||
        right_element.type != Element::Type::kWrap ||
        right_element.index != left_element.index + 1) {
      continue;
    }
    int pair = left_element.index;
    Wrap &left = wraps_[pair];
    Wrap &right = wraps_[pair + 1];
    if (!left.valid || !right.valid || left.contact_points.empty() ||
        right.contact_points.empty()) {
      return -1;
    }
    mjtNum gap = Distance(left.contact_points.back().pos,
                          right.contact_points.front().pos);
    bool was_active = composite_pair_active_[pair];
    bool guided_mesh_pair =
        mesh_route_mode_ == MeshRouteMode::kGuidedSurface &&
        left.geom_type == mjGEOM_MESH && right.geom_type == mjGEOM_MESH;
    bool has_common_tangent =
        composite_left_segment_[pair] >= 0 &&
        composite_right_segment_[pair] >= 0;
    if (guided_mesh_pair && was_active && has_common_tangent) {
      for (Point &point : left.contact_points) {
        point.pos = LocalToWorld(d, left.geom_id, point.local_pos);
      }
      for (Point &point : right.contact_points) {
        point.pos = LocalToWorld(d, right.geom_id, point.local_pos);
      }
    }
    bool active = was_active;
    if (guided_mesh_pair && was_active && has_common_tangent) {
      // A finite common-tangent bridge is part of the composite topology and
      // may legitimately exceed the merge distance as the bodies roll.
      active = true;
    } else if (was_active) {
      active = gap <= split_distance;
    } else {
      active = gap <= composite_merge_distance_;
    }
    composite_pair_active_[pair] = active;
    if (!active) {
      continue;
    }

    Vec3 previous = exit_position(elements_[element_index - 1]);
    Vec3 next = entry_position(elements_[element_index + 2]);
    auto clear_mesh_corridor = [](Wrap *wrap) {
      wrap->mesh.strip.clear();
      wrap->mesh.parameters.clear();
      wrap->mesh.active_transitions.clear();
      wrap->mesh.mandatory_transitions.clear();
    };
    auto rebuild_composite_corridors = [&]() {
      // The independent initialization chooses terminal faces from the hint
      // sites. Rebuild against the neighboring surface contact while keeping
      // seed_face and the guide plane, which preserve the initialized side.
      constexpr int kTopologySweeps = 3;
      for (int sweep = 0; sweep < kTopologySweeps; ++sweep) {
        clear_mesh_corridor(&left);
        if (!SolveMesh(m, d, &left, previous,
                       right.contact_points.front().pos, false)) {
          return false;
        }
        left.mesh.mandatory_transitions.clear();
        clear_mesh_corridor(&right);
        if (!SolveMesh(m, d, &right, left.contact_points.back().pos, next,
                       false)) {
          return false;
        }
        right.mesh.mandatory_transitions.clear();
        total_iterations +=
            left.solver_iterations + right.solver_iterations;
      }
      composite_left_segment_[pair] = -1;
      composite_right_segment_[pair] = -1;
      return true;
    };
    if (guided_mesh_pair && !was_active &&
        !rebuild_composite_corridors()) {
      return -1;
    }

    const int composite_sweeps = guided_mesh_pair && was_active ? 0 : 8;
    for (int sweep = 0; sweep < composite_sweeps; ++sweep) {
      if (guided_mesh_pair) {
        // The hints select the initialized homotopy side.  Once two adjacent
        // surfaces are treated as one composite obstacle, their seed-face
        // boundary edges must not remain hard contacts: a true common tangent
        // needs to slide away from those initialization edges as the bodies
        // roll.
        left.mesh.mandatory_transitions.clear();
        right.mesh.mandatory_transitions.clear();
      }
      Vec3 old_left = left.contact_points.back().pos;
      Vec3 old_right = right.contact_points.front().pos;
      if (!SolveWrap(m, d, pair, previous,
                     right.contact_points.front().pos) ||
          !SolveWrap(m, d, pair + 1, left.contact_points.back().pos, next)) {
        return -1;
      }
      total_iterations += left.solver_iterations + right.solver_iterations;
      mjtNum change =
          std::max(Distance(old_left, left.contact_points.back().pos),
                   Distance(old_right, right.contact_points.front().pos));
      if (change < 1e-10) {
        break;
      }
    }
    if (!solve_polyline_common_tangent(pair, &left, &right, previous, next)) {
      if (!guided_mesh_pair || !rebuild_composite_corridors() ||
          !solve_polyline_common_tangent(pair, &left, &right, previous, next)) {
        return -1;
      }
    }
    if (pair < static_cast<int>(composite_reacquire_cooldown_.size()) &&
        composite_reacquire_cooldown_[pair] > 0) {
      --composite_reacquire_cooldown_[pair];
    }
    constexpr mjtNum kReacquireTangentError = 0.34;
    if (guided_mesh_pair && was_active &&
        composite_reacquire_cooldown_[pair] == 0 &&
        composite_tangent_error(left, right) > kReacquireTangentError) {
      // A warm composite route normally searches only neighboring terminal
      // segments.  Rolling can move the true common tangent beyond that local
      // window while the cached local-space corridor still looks valid.  In
      // that case rebuild both terminal corridors from their seed-side guide
      // planes, then keep the globally reacquired bridge only if it improves
      // the KKT/tangent residual.  The seed face continues to select the
      // homotopy side; it is not reintroduced as a physical route point.
      Wrap saved_left = left;
      Wrap saved_right = right;
      int saved_left_segment = composite_left_segment_[pair];
      int saved_right_segment = composite_right_segment_[pair];
      auto saved_left_local = composite_left_local_[pair];
      auto saved_right_local = composite_right_local_[pair];
      mjtNum saved_error = composite_tangent_error(left, right);
      bool reacquired = rebuild_composite_corridors() &&
                        solve_polyline_common_tangent(
                            pair, &left, &right, previous, next);
      mjtNum reacquired_error =
          reacquired ? composite_tangent_error(left, right)
                     : std::numeric_limits<mjtNum>::infinity();
      if (!reacquired || reacquired_error + mjtNum(1e-4) >= saved_error) {
        left = std::move(saved_left);
        right = std::move(saved_right);
        composite_left_segment_[pair] = saved_left_segment;
        composite_right_segment_[pair] = saved_right_segment;
        composite_left_local_[pair] = saved_left_local;
        composite_right_local_[pair] = saved_right_local;
      }
      // A full dual-graph terminal-face search is deliberately infrequent.
      // Warm segment tracking handles ordinary sliding between these global
      // corrections and avoids turning route topology repair into the p95
      // per-step cost.
      composite_reacquire_cooldown_[pair] = 250;
    }
    if (guided_mesh_pair && !was_active) {
      // Feed the first joint solution back into the terminal-face search.  The
      // initial corridors were built against projected hint contacts; this
      // second pass rebuilds them against the actual inter-surface bridge and
      // lets the common tangent move across neighboring triangles.
      constexpr int kTangentCorridorPasses = 2;
      for (int pass = 0; pass < kTangentCorridorPasses; ++pass) {
        if (!rebuild_composite_corridors() ||
            !solve_polyline_common_tangent(pair, &left, &right, previous,
                                            next)) {
          return -1;
        }
      }
    }
    gap = Distance(left.contact_points.back().pos,
                   right.contact_points.front().pos);
  }
  return total_iterations;
}

bool SurfaceEnvelopeRoute::Update(const mjModel *m, const mjData *d) {
  if (!initialized_) {
    int wrap_cursor = 0;
    for (int seed_index = 0;
         seed_index < static_cast<int>(seed_site_ids_.size()); ++seed_index) {
      if (site_roles_[seed_index] != 2) {
        continue;
      }
      Vec3 previous =
          PointerVec(d->site_xpos + 3 * seed_site_ids_[seed_index - 1]);
      Vec3 next = PointerVec(d->site_xpos + 3 * seed_site_ids_[seed_index + 1]);
      if (!InitializeWrap(m, d, wrap_cursor, previous, next)) {
        result_.status = Status::kInvalid;
        return false;
      }
      ++wrap_cursor;
    }
    initialized_ = true;
  }

  auto entry_position = [this, d](const Element &element) {
    if (element.type == Element::Type::kSite) {
      return PointerVec(d->site_xpos + 3 * hard_site_ids_[element.index]);
    }
    return wraps_[element.index].contact_points.front().pos;
  };
  auto exit_position = [this, d](const Element &element) {
    if (element.type == Element::Type::kSite) {
      return PointerVec(d->site_xpos + 3 * hard_site_ids_[element.index]);
    }
    return wraps_[element.index].contact_points.back().pos;
  };

  int total_iterations = 0;
  const int route_sweeps =
      mesh_route_mode_ == MeshRouteMode::kConvexSurface ? kRuntimeSweeps : 2;
  for (int sweep = 0; sweep < route_sweeps; ++sweep) {
    mjtNum maximum_change = 0;
    for (int element_index = 1;
         element_index + 1 < static_cast<int>(elements_.size());
         ++element_index) {
      const Element &element = elements_[element_index];
      if (element.type != Element::Type::kWrap) {
        continue;
      }
      bool composite_active =
          (element.index < static_cast<int>(composite_pair_active_.size()) &&
           composite_pair_active_[element.index]) ||
          (element.index > 0 &&
           element.index - 1 <
               static_cast<int>(composite_pair_active_.size()) &&
           composite_pair_active_[element.index - 1]);
      if (composite_active) {
        // Adjacent active obstacles form one route optimization problem.  An
        // independent update here destroys the terminal-edge active set found
        // by SolveCompositePairs and recreates the visible interface kink.
        continue;
      }
      Wrap &wrap = wraps_[element.index];
      Vec3 old_entry = wrap.contact_points.front().pos;
      Vec3 old_exit = wrap.contact_points.back().pos;
      Vec3 previous = exit_position(elements_[element_index - 1]);
      Vec3 next = entry_position(elements_[element_index + 1]);
      if (!SolveWrap(m, d, element.index, previous, next)) {
        result_.status = Status::kInvalid;
        return false;
      }
      total_iterations += wrap.solver_iterations;
      maximum_change = std::max(
          maximum_change, Distance(old_entry, wrap.contact_points.front().pos));
      maximum_change = std::max(
          maximum_change, Distance(old_exit, wrap.contact_points.back().pos));
    }
    if (maximum_change < 1e-10) {
      break;
    }
  }
  int composite_iterations = SolveCompositePairs(m, d);
  if (composite_iterations < 0) {
    result_.status = Status::kInvalid;
    return false;
  }
  total_iterations += composite_iterations;
  if (!BuildResult(m, d)) {
    bool repaired = false;
    constexpr int kMaximumRepairPasses = 4;
    for (int pass = 0; pass < kMaximumRepairPasses; ++pass) {
      int repaired_wrap_index = repair_wrap_index_;
      bool repair_succeeded = pass == 0 ? RepairFailedSpan(m, d) : false;
      if (!repair_succeeded) {
        repair_succeeded =
            ReinitializeCompositePair(m, d, repaired_wrap_index);
      }
      if (!repair_succeeded) {
        break;
      }
      if (repaired_wrap_index >= 0 &&
          repaired_wrap_index < static_cast<int>(wraps_.size())) {
        total_iterations += wraps_[repaired_wrap_index].solver_iterations;
        int first_pair = std::max(0, repaired_wrap_index - 1);
        int last_pair = std::min(
            repaired_wrap_index,
            static_cast<int>(composite_left_segment_.size()) - 1);
        for (int pair = first_pair; pair <= last_pair; ++pair) {
          composite_left_segment_[pair] = -1;
          composite_right_segment_[pair] = -1;
          composite_reacquire_cooldown_[pair] = 0;
        }
      }
      // A repaired terminal corridor can change a member of an active
      // composite pair.  Re-solve its common tangent before validating all
      // free spans; otherwise the endpoint and inter-surface bridge describe
      // two inconsistent active sets.  BuildResult identifies the next
      // offending span, so a bounded loop can settle coupled repairs.
      int repair_composite_iterations = SolveCompositePairs(m, d);
      if (repair_composite_iterations < 0) {
        break;
      }
      total_iterations += repair_composite_iterations;
      if (BuildResult(m, d)) {
        repaired = true;
        break;
      }
    }
    if (!repaired) {
      result_.status = Status::kInvalid;
      return false;
    }
    result_.status = Status::kDegraded;
    result_.tangent_residual =
        std::max(result_.tangent_residual, mjtNum(1.0001e-5));
  }
  result_.solver_iterations = total_iterations;
  if (!has_initial_length_) {
    initial_length_ = result_.length;
    has_initial_length_ = true;
  }
  return true;
}

bool SurfaceEnvelopeRoute::RepairFailedSpan(const mjModel *m,
                                             const mjData *d) {
  if (repair_wrap_index_ < 0 ||
      repair_wrap_index_ >= static_cast<int>(wraps_.size())) {
    return false;
  }

  int element_index = -1;
  for (int candidate = 0; candidate < static_cast<int>(elements_.size());
       ++candidate) {
    if (elements_[candidate].type == Element::Type::kWrap &&
        elements_[candidate].index == repair_wrap_index_) {
      element_index = candidate;
      break;
    }
  }
  if (element_index <= 0 ||
      element_index + 1 >= static_cast<int>(elements_.size())) {
    return false;
  }

  auto entry_position = [this, d](const Element &element, Vec3 *position) {
    if (element.type == Element::Type::kSite) {
      *position = PointerVec(d->site_xpos + 3 * hard_site_ids_[element.index]);
      return true;
    }
    const Wrap &wrap = wraps_[element.index];
    if (!wrap.valid || wrap.contact_points.empty()) {
      return false;
    }
    *position = wrap.contact_points.front().pos;
    return true;
  };
  auto exit_position = [this, d](const Element &element, Vec3 *position) {
    if (element.type == Element::Type::kSite) {
      *position = PointerVec(d->site_xpos + 3 * hard_site_ids_[element.index]);
      return true;
    }
    const Wrap &wrap = wraps_[element.index];
    if (!wrap.valid || wrap.contact_points.empty()) {
      return false;
    }
    *position = wrap.contact_points.back().pos;
    return true;
  };

  Vec3 previous;
  Vec3 next;
  if (!exit_position(elements_[element_index - 1], &previous) ||
      !entry_position(elements_[element_index + 1], &next)) {
    return false;
  }

  Wrap &wrap = wraps_[repair_wrap_index_];
  Wrap saved = wrap;
  wrap.initialized = false;
  wrap.valid = false;
  wrap.used_fallback = false;
  wrap.contact_points.clear();
  wrap.mesh.seed_face = -1;
  wrap.mesh.strip.clear();
  wrap.mesh.parameters.clear();
  wrap.mesh.active_transitions.clear();
  wrap.mesh.mandatory_transitions.clear();
  wrap.mesh.guided_anchor_valid = false;
  if (!SolveWrap(m, d, repair_wrap_index_, previous, next)) {
    wrap = std::move(saved);
    return false;
  }

  wrap.used_fallback = true;
  wrap.tangent_residual =
      std::max(wrap.tangent_residual, mjtNum(1.0001e-5));
  return true;
}

bool SurfaceEnvelopeRoute::ReinitializeCompositePair(const mjModel *m,
                                                      const mjData *d,
                                                      int wrap_index) {
  int pair = -1;
  if (wrap_index > 0 &&
      wrap_index - 1 < static_cast<int>(composite_pair_active_.size()) &&
      composite_pair_active_[wrap_index - 1]) {
    pair = wrap_index - 1;
  } else if (wrap_index >= 0 &&
             wrap_index < static_cast<int>(composite_pair_active_.size()) &&
             composite_pair_active_[wrap_index]) {
    pair = wrap_index;
  }
  if (pair < 0 || pair + 1 >= static_cast<int>(wraps_.size())) {
    return false;
  }

  int left_element = -1;
  for (int index = 0; index + 1 < static_cast<int>(elements_.size()); ++index) {
    if (elements_[index].type == Element::Type::kWrap &&
        elements_[index].index == pair &&
        elements_[index + 1].type == Element::Type::kWrap &&
        elements_[index + 1].index == pair + 1) {
      left_element = index;
      break;
    }
  }
  if (left_element <= 0 || left_element + 2 >=
                                 static_cast<int>(elements_.size())) {
    return false;
  }

  auto boundary_position = [this, d](const Element &element, bool exit) {
    if (element.type == Element::Type::kSite) {
      return PointerVec(d->site_xpos + 3 * hard_site_ids_[element.index]);
    }
    const Wrap &wrap = wraps_[element.index];
    return exit ? wrap.contact_points.back().pos
                : wrap.contact_points.front().pos;
  };

  Wrap saved_left = wraps_[pair];
  Wrap saved_right = wraps_[pair + 1];
  int saved_left_segment = composite_left_segment_[pair];
  int saved_right_segment = composite_right_segment_[pair];
  auto saved_left_local = composite_left_local_[pair];
  auto saved_right_local = composite_right_local_[pair];

  auto clear_runtime_corridor = [](Wrap *wrap) {
    wrap->initialized = false;
    wrap->valid = false;
    wrap->used_fallback = false;
    wrap->contact_points.clear();
    wrap->mesh.strip.clear();
    wrap->mesh.parameters.clear();
    wrap->mesh.active_transitions.clear();
    wrap->mesh.mandatory_transitions.clear();
  };
  clear_runtime_corridor(&wraps_[pair]);
  clear_runtime_corridor(&wraps_[pair + 1]);

  Vec3 previous = boundary_position(elements_[left_element - 1], true);
  Vec3 next = boundary_position(elements_[left_element + 2], false);
  Vec3 left_anchor = LocalToWorld(d, wraps_[pair].geom_id,
                                  wraps_[pair].mesh.guided_anchor);
  Vec3 right_anchor = LocalToWorld(d, wraps_[pair + 1].geom_id,
                                   wraps_[pair + 1].mesh.guided_anchor);
  bool valid = SolveWrap(m, d, pair, previous, right_anchor) &&
               SolveWrap(m, d, pair + 1, left_anchor, next);
  if (!valid) {
    wraps_[pair] = std::move(saved_left);
    wraps_[pair + 1] = std::move(saved_right);
    composite_left_segment_[pair] = saved_left_segment;
    composite_right_segment_[pair] = saved_right_segment;
    composite_left_local_[pair] = saved_left_local;
    composite_right_local_[pair] = saved_right_local;
    return false;
  }

  composite_left_segment_[pair] = -1;
  composite_right_segment_[pair] = -1;
  composite_reacquire_cooldown_[pair] = 0;
  return true;
}

bool SurfaceEnvelopeRoute::BuildResult(const mjModel *m, const mjData *d) {
  repair_wrap_index_ = -1;
  Result next_result;
  next_result.status = Status::kValid;
  for (const Element &element : elements_) {
    if (element.type == Element::Type::kSite) {
      int site_id = hard_site_ids_[element.index];
      next_result.points.push_back({PointerVec(d->site_xpos + 3 * site_id),
                                    m->site_bodyid[site_id], site_id, -1, -1});
      continue;
    }
    const Wrap &wrap = wraps_[element.index];
    if (!wrap.valid) {
      return false;
    }
    for (Point point : wrap.contact_points) {
      point.wrap_index = element.index;
      if (!next_result.points.empty() &&
          Distance(next_result.points.back().pos, point.pos) < 1e-12 &&
          next_result.points.back().body_id == point.body_id &&
          next_result.points.back().wrap_index == point.wrap_index) {
        continue;
      }
      next_result.points.push_back(point);
    }
    next_result.tangent_residual =
        std::max(next_result.tangent_residual, wrap.tangent_residual);
    next_result.surface_residual =
        std::max(next_result.surface_residual, wrap.surface_residual);
    if (wrap.tangent_residual > 1e-5 || wrap.surface_residual > 1e-7) {
      next_result.status = Status::kDegraded;
    }
  }
  if (next_result.points.size() < 2) {
    return false;
  }

  if (mesh_route_mode_ != MeshRouteMode::kConvexSurface) {
    for (int segment = 0;
         segment + 1 < static_cast<int>(next_result.points.size()); ++segment) {
      const Point &first = next_result.points[segment];
      const Point &second = next_result.points[segment + 1];
      for (const Wrap &wrap : wraps_) {
        if (wrap.geom_type != mjGEOM_MESH) {
          continue;
        }
        // Only adjacent points emitted by the same solved wrap are known to
        // follow its triangle corridor.  Consecutive guided wraps may name the
        // same geom, but their connecting span is still a free chord and must
        // be checked against that mesh.
        bool same_surface_span = first.geom_id == wrap.geom_id &&
                                 second.geom_id == wrap.geom_id &&
                                 first.wrap_index == second.wrap_index;
        if (same_surface_span) {
          continue;
        }
        if (!SegmentClearOfMesh(m, d, wrap.geom_id, first.pos, second.pos)) {
          if (second.wrap_index >= 0 &&
              wraps_[second.wrap_index].geom_id == wrap.geom_id) {
            repair_wrap_index_ = second.wrap_index;
          } else if (first.wrap_index >= 0 &&
                     wraps_[first.wrap_index].geom_id == wrap.geom_id) {
            repair_wrap_index_ = first.wrap_index;
          } else {
            for (int candidate = 0;
                 candidate < static_cast<int>(wraps_.size()); ++candidate) {
              if (wraps_[candidate].geom_id == wrap.geom_id) {
                repair_wrap_index_ = candidate;
                break;
              }
            }
          }
          Vec3 diagnostic_delta = Sub(second.pos, first.pos);
          mjtNum diagnostic_length = Norm(diagnostic_delta);
          mjtNum diagnostic_hit =
              diagnostic_length > mjMINVAL
                  ? RayMesh(m, d, wrap.geom_id, first.pos,
                            Scale(diagnostic_delta, 1 / diagnostic_length))
                  : -1;
          mju_warning(
              "nonconvex route free span intersects mesh geom %d between route "
              "points %d and %d (length %.9g, hit %.9g, endpoint wraps %d/%d, "
              "endpoint geoms %d/%d)",
              wrap.geom_id, segment, segment + 1, diagnostic_length,
              diagnostic_hit, first.wrap_index, second.wrap_index,
              first.geom_id, second.geom_id);
          return false;
        }
      }
    }
  }

  std::vector<Vec3> velocities;
  velocities.reserve(next_result.points.size());
  for (const Point &point : next_result.points) {
    velocities.push_back(PointVelocity(m, d, point));
  }
  const int segment_count = static_cast<int>(next_result.points.size()) - 1;
  std::vector<mjtNum> segment_lengths(segment_count, 0);
  next_result.segment_directions.assign(segment_count, {0, 0, 0});
  next_result.composite_segments.assign(segment_count, false);
  for (int segment = 0; segment < segment_count; ++segment) {
    Vec3 delta = Sub(next_result.points[segment + 1].pos,
                     next_result.points[segment].pos);
    segment_lengths[segment] = Norm(delta);
    if (segment_lengths[segment] > mjMINVAL) {
      next_result.segment_directions[segment] =
          Scale(delta, 1 / segment_lengths[segment]);
    }
    const Point &first = next_result.points[segment];
    const Point &second = next_result.points[segment + 1];
    int pair = first.wrap_index;
    if (pair >= 0 && second.wrap_index == pair + 1 &&
        pair < static_cast<int>(composite_pair_active_.size()) &&
        composite_pair_active_[pair]) {
      next_result.composite_segments[segment] = true;
    }
  }

  for (int segment = 0; segment < segment_count; ++segment) {
    if (!next_result.composite_segments[segment]) {
      continue;
    }
    // A finite bridge is the physical common-tangent free span.  Its force
    // and length-rate direction must remain the geometric bridge direction.
    // Direction averaging is only the limiting treatment for coincident
    // contacts, where the bridge has no measurable direction of its own.
    if (segment_lengths[segment] > 1e-9) {
      continue;
    }
    Vec3 before = {0, 0, 0};
    Vec3 after = {0, 0, 0};
    bool has_before = false;
    bool has_after = false;
    for (int candidate = segment - 1; candidate >= 0; --candidate) {
      if (segment_lengths[candidate] > mjMINVAL &&
          !next_result.composite_segments[candidate]) {
        before = next_result.segment_directions[candidate];
        has_before = true;
        break;
      }
    }
    for (int candidate = segment + 1; candidate < segment_count;
         ++candidate) {
      if (segment_lengths[candidate] > mjMINVAL &&
          !next_result.composite_segments[candidate]) {
        after = next_result.segment_directions[candidate];
        has_after = true;
        break;
      }
    }
    Vec3 common =
        has_before && has_after
            ? Normalize(Add(before, after))
            : (has_before ? before : (has_after ? after : Vec3{0, 0, 0}));
    if (Norm(common) <= mjMINVAL) {
      return false;
    }
    next_result.segment_directions[segment] = common;
    if (has_before && has_after) {
      next_result.tangent_residual =
          std::max(next_result.tangent_residual,
                   segment_lengths[segment] * Norm(Sub(before, after)));
    }
  }

  for (int i = 0; i < segment_count; ++i) {
    mjtNum length = segment_lengths[i];
    const Vec3 &direction = next_result.segment_directions[i];
    if (length < mjMINVAL && !next_result.composite_segments[i]) {
      continue;
    }
    next_result.length += length;
    next_result.velocity +=
        Dot(direction, Sub(velocities[i + 1], velocities[i]));
  }
  result_ = std::move(next_result);
  return std::isfinite(result_.length) && std::isfinite(result_.velocity);
}

} // namespace mujoco::plugin::cable
