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

#ifndef MUJOCO_PLUGIN_CABLE_UNILATERAL_CABLE_H_
#define MUJOCO_PLUGIN_CABLE_UNILATERAL_CABLE_H_

#include <mujoco/mujoco.h>

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "surface_route.h"

namespace mujoco::plugin::cable {

class UnilateralCable {
public:
  static UnilateralCable *Create(const mjModel *m, int instance);

  explicit UnilateralCable(const mjModel *m, int instance);

  void Reset(mjtNum *plugin_state);
  void Advance(const mjModel *m, mjData *d, int instance) const;
  void Compute(const mjModel *m, mjData *d, int instance, int capability_bit);
  void Visualize(const mjModel *m, mjData *d, mjvScene *scene) const;
  void CopyFrom(const UnilateralCable &source);

  static int StateSize(const mjModel *m, int instance);
  static int SensorDim(const mjModel *m, int instance, int sensor_id);
  static void RegisterPlugin();

private:
  struct EvalResult {
    mjtNum length = 0;
    mjtNum velocity = 0;
    mjtNum free_length = 0;
    mjtNum contraction = 0;
    mjtNum extension = 0;
    mjtNum effective_stiffness = 0;
    mjtNum effective_damping = 0;
    mjtNum tension = 0;
    mjtNum taut = 0;
    mjtNum saturated = 0;
  };

  EvalResult Evaluate(const mjModel *m, mjData *d, int instance,
                      int actuator_id, bool update_hysteresis) const;
  bool BuildExtensionJacobian(const mjModel *m, mjData *d, int actuator_id,
                              std::vector<mjtNum> *jacobian) const;
  mjtNum LocalImplicitTension(const mjModel *m, mjData *d, int instance,
                              int actuator_id,
                              const EvalResult &result) const;
  void ApplyImplicitRhsCorrection(const mjModel *m, mjData *d,
                                  const EvalResult &result,
                                  int actuator_id) const;
  bool ImplicitOperatorActive() const;
  void MultiplyImplicitOperator(const mjModel *m, mjData *d,
                                const mjtNum *vector,
                                mjtNum *result) const;
  void ApplyCapstanFriction(const mjModel *m, mjData *d,
                            const EvalResult &result, int actuator_id) const;
  void ApplySurfaceForces(const mjModel *m, mjData *d,
                          const EvalResult &result) const;
  void ApplySpoolReactionTorque(const mjModel *m, mjData *d,
                                const EvalResult &result) const;
  bool UpdateSurfaceRoute(const mjModel *m, const mjData *d);
  mjtNum RawContraction(const mjModel *m, const mjData *d,
                        int actuator_id) const;
  mjtNum SpoolAngleContraction(mjtNum angle) const;
  mjtNum SpoolAngleContractionDerivative(mjtNum angle) const;

  mjtNum stiffness_ = 0;
  mjtNum damping_ = 0;
  mjtNum slack_ = 0;
  mjtNum control_timeconstant_ = 0;
  mjtNum max_contraction_rate_ = 0;
  mjtNum taut_transition_ = 0;
  mjtNum taut_hysteresis_ = 0;
  mjtNum pretension_offset_ = 0;
  std::optional<mjtNum> max_tension_;
  std::optional<mjtNum> home_length_;
  mjtNum spool_radius_ = 1;
  mjtNum spool_reserve_length_ = 0;
  mjtNum spool_reserve_sign_ = 1;
  bool spool_reaction_torque_ = false;
  int spool_joint_id_ = -1;
  int spool_qposadr_ = -1;
  mjtNum spool_qpos0_ = 0;
  mjtNum capstan_mu_ = 0;
  std::string ctrl_mode_;
  std::string integration_mode_;
  std::string spool_joint_;
  std::string spool_reserve_direction_;
  std::string capstan_direction_;
  std::string route_mode_;
  std::string mesh_route_mode_;
  std::string route_tendon_;
  std::string wrap_geoms_;
  int route_tendon_id_ = -1;
  int site_role_user_index_ = 0;
  mjtNum visual_width_ = 2;
  mjtNum visual_smoothing_timeconstant_ = 0;
  std::optional<SurfaceEnvelopeRoute> surface_route_;
  EvalResult cached_surface_result_;
  bool surface_result_valid_ = false;
  bool warned_invalid_route_ = false;
  std::vector<int> actuators_;
  mutable std::vector<EvalResult> cached_native_results_;
  mutable std::vector<bool> cached_native_valid_;
  mutable std::vector<std::array<mjtNum, 3>> visual_points_;
  mutable mjtNum visual_time_ = -1;
};

} // namespace mujoco::plugin::cable

#endif // MUJOCO_PLUGIN_CABLE_UNILATERAL_CABLE_H_
