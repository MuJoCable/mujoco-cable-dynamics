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

#ifndef MUJOCO_PLUGIN_CABLE_SURFACE_ROUTE_H_
#define MUJOCO_PLUGIN_CABLE_SURFACE_ROUTE_H_

#include <mujoco/mujoco.h>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace mujoco::plugin::cable {

class SurfaceEnvelopeRoute {
 public:
  enum class MeshRouteMode {
    kConvexSurface,
    kTautObstacle,
    kGuidedSurface,
  };

  enum class Status {
    kValid = 0,
    kDegraded = 1,
    kInvalid = 2,
    kUninitialized = 3,
  };

  struct Point {
    std::array<mjtNum, 3> pos = {0, 0, 0};
    int body_id = 0;
    int site_id = -1;
    int geom_id = -1;
    int wrap_index = -1;
    std::array<mjtNum, 3> local_pos = {0, 0, 0};
  };

  struct Result {
    mjtNum length = 0;
    mjtNum velocity = 0;
    mjtNum tangent_residual = 0;
    mjtNum surface_residual = 0;
    int solver_iterations = 0;
    Status status = Status::kUninitialized;
    std::vector<Point> points;
    std::vector<std::array<mjtNum, 3>> segment_directions;
    std::vector<bool> composite_segments;
  };

  static std::optional<SurfaceEnvelopeRoute> Create(
      const mjModel *m, int tendon_id, const std::vector<int> &wrap_geom_ids,
      int site_role_user_index, MeshRouteMode mesh_route_mode,
      mjtNum route_hysteresis, mjtNum composite_merge_distance,
      const std::optional<std::array<mjtNum, 3>> &mesh_guide_axis,
      mjtNum mesh_guide_weight,
      std::string *error);

  SurfaceEnvelopeRoute() = default;

  bool Update(const mjModel *m, const mjData *d);
  void Reset();

  const Result &result() const { return result_; }
  mjtNum initial_length() const { return initial_length_; }
  bool has_initial_length() const { return has_initial_length_; }
  int tendon_id() const { return tendon_id_; }

 private:
  struct MeshFace {
    std::array<int, 3> vertex = {0, 0, 0};
    std::array<mjtNum, 3> normal = {0, 0, 0};
    std::array<mjtNum, 3> center = {0, 0, 0};
  };

  struct MeshNeighbor {
    int face = -1;
    std::array<int, 2> edge = {-1, -1};
  };

  struct MeshState {
    int mesh_id = -1;
    int seed_face = -1;
    std::vector<std::array<mjtNum, 3>> vertices;
    std::vector<MeshFace> faces;
    std::vector<std::vector<MeshNeighbor>> neighbors;
    std::vector<int> strip;
    std::vector<mjtNum> parameters;
    std::vector<int> active_transitions;
    std::vector<int> mandatory_transitions;
    std::array<mjtNum, 3> guided_anchor = {0, 0, 0};
    std::array<mjtNum, 3> guide_plane_origin = {0, 0, 0};
    std::array<mjtNum, 3> guide_plane_normal = {0, 0, 0};
    mjtNum scale = 1;
    bool guided_anchor_valid = false;
    bool guide_plane_valid = false;
    bool convex = true;
  };

  struct CylinderState {
    int branch_sign = 1;
    mjtNum seed_angle = 0;
    std::array<mjtNum, 4> parameters = {0, 0, 0, 0};
  };

  struct Wrap {
    int geom_id = -1;
    int body_id = 0;
    int hint_site_id = -1;
    int geom_type = mjGEOM_NONE;
    bool initialized = false;
    CylinderState cylinder;
    MeshState mesh;
    std::vector<Point> contact_points;
    mjtNum tangent_residual = 0;
    mjtNum surface_residual = 0;
    int solver_iterations = 0;
    bool valid = false;
    bool used_fallback = false;
  };

  struct Element {
    enum class Type { kSite, kWrap };
    Type type = Type::kSite;
    int index = -1;
  };

  bool InitializeWrap(const mjModel *m, const mjData *d, int wrap_index,
                      const std::array<mjtNum, 3> &previous_seed,
                      const std::array<mjtNum, 3> &next_seed);
  bool SolveWrap(const mjModel *m, const mjData *d, int wrap_index,
                 const std::array<mjtNum, 3> &previous,
                 const std::array<mjtNum, 3> &next);
  bool SolveCylinder(const mjModel *m, const mjData *d, Wrap *wrap,
                     const std::array<mjtNum, 3> &previous,
                     const std::array<mjtNum, 3> &next, bool initialize);
  bool SolveMesh(const mjModel *m, const mjData *d, Wrap *wrap,
                 const std::array<mjtNum, 3> &previous,
                 const std::array<mjtNum, 3> &next, bool initialize);
  int SolveCompositePairs(const mjModel *m, const mjData *d);
  bool RepairFailedSpan(const mjModel *m, const mjData *d);
  bool ReinitializeCompositePair(const mjModel *m, const mjData *d,
                                 int wrap_index);
  bool BuildResult(const mjModel *m, const mjData *d);

  int tendon_id_ = -1;
  int site_role_user_index_ = 0;
  MeshRouteMode mesh_route_mode_ = MeshRouteMode::kConvexSurface;
  mjtNum route_hysteresis_ = 0;
  mjtNum composite_merge_distance_ = 0;
  std::optional<std::array<mjtNum, 3>> mesh_guide_axis_;
  mjtNum mesh_guide_weight_ = 0;
  std::vector<int> seed_site_ids_;
  std::vector<int> site_roles_;
  std::vector<int> hard_site_ids_;
  std::vector<Wrap> wraps_;
  std::vector<Element> elements_;
  std::vector<bool> composite_pair_active_;
  std::vector<int> composite_left_segment_;
  std::vector<int> composite_right_segment_;
  std::vector<std::array<mjtNum, 3>> composite_left_local_;
  std::vector<std::array<mjtNum, 3>> composite_right_local_;
  std::vector<int> composite_reacquire_cooldown_;
  Result result_;
  mjtNum initial_length_ = 0;
  bool has_initial_length_ = false;
  bool initialized_ = false;
  int repair_wrap_index_ = -1;
};

}  // namespace mujoco::plugin::cable

#endif  // MUJOCO_PLUGIN_CABLE_SURFACE_ROUTE_H_
