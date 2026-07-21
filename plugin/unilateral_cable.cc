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

#include "unilateral_cable.h"

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "native_route.h"

namespace mujoco::plugin::cable {
namespace {

constexpr char kAttrStiffness[] = "stiffness";
constexpr char kAttrDamping[] = "damping";
constexpr char kAttrSlack[] = "slack";
constexpr char kAttrControlTimeconstant[] = "control_timeconstant";
constexpr char kAttrMaxContractionRate[] = "max_contraction_rate";
constexpr char kAttrTautTransition[] = "taut_transition";
constexpr char kAttrTautHysteresis[] = "taut_hysteresis";
constexpr char kAttrIntegrationMode[] = "integration_mode";
constexpr char kAttrHomeLength[] = "home_length";
constexpr char kAttrPretensionOffset[] = "pretension_offset";
constexpr char kAttrMaxTension[] = "max_tension";
constexpr char kAttrCtrlMode[] = "ctrl_mode";
constexpr char kAttrSpoolRadius[] = "spool_radius";
constexpr char kAttrSpoolJoint[] = "spool_joint";
constexpr char kAttrSpoolReserveLength[] = "spool_reserve_length";
constexpr char kAttrSpoolReserveDirection[] = "spool_reserve_direction";
constexpr char kAttrSpoolReactionTorque[] = "spool_reaction_torque";
constexpr char kAttrCapstanMu[] = "capstan_mu";
constexpr char kAttrCapstanDirection[] = "capstan_direction";
constexpr char kAttrRouteMode[] = "route_mode";
constexpr char kAttrMeshRouteMode[] = "mesh_route_mode";
constexpr char kAttrMeshGuideAxis[] = "mesh_guide_axis";
constexpr char kAttrMeshGuideWeight[] = "mesh_guide_weight";
constexpr char kAttrRouteTendon[] = "route_tendon";
constexpr char kAttrWrapGeoms[] = "wrap_geoms";
constexpr char kAttrSiteRoleUserIndex[] = "site_role_user_index";
constexpr char kAttrVisualWidth[] = "visual_width";
constexpr char kAttrRouteHysteresis[] = "route_hysteresis";
constexpr char kAttrCompositeMergeDistance[] = "composite_merge_distance";
constexpr char kAttrVisualSmoothingTimeconstant[] =
    "visual_smoothing_timeconstant";

constexpr int kIntegratedContractionState = 0;
constexpr int kFilteredContractionState = 1;
constexpr int kTautLatchState = 2;
constexpr int kPreviousTensionState = 3;
constexpr int kImplicitInitializedState = 4;
constexpr int kStateSize = 5;

struct WrapPoint {
  std::array<mjtNum, 3> pos = {0, 0, 0};
  int obj = -2;
  int body = -1;
};

std::string ConfigString(const mjModel *m, int instance, const char *attr) {
  const char *value = mj_getPluginConfig(m, instance, attr);
  return value ? std::string(value) : std::string();
}

std::string ConfigToken(const mjModel *m, int instance, const char *attr) {
  std::string value = ConfigString(m, instance, attr);
  value.erase(std::remove_if(value.begin(), value.end(),
                             [](unsigned char c) { return std::isspace(c); }),
              value.end());
  return value;
}

bool IsAutoToken(const std::string &value) {
  return value == "auto" || value == "auto_initial";
}

bool CheckOptionalNumber(const mjModel *m, int instance, const char *attr,
                         bool allow_auto) {
  std::string value = ConfigToken(m, instance, attr);
  if (value.empty() || (allow_auto && IsAutoToken(value))) {
    return true;
  }
  char *end = nullptr;
  std::strtod(value.c_str(), &end);
  return end == value.c_str() + value.size();
}

bool CheckOptionalEnum(const mjModel *m, int instance, const char *attr,
                       const char *const *allowed, int num_allowed) {
  std::string value = ConfigToken(m, instance, attr);
  if (value.empty()) {
    return true;
  }
  for (int i = 0; i < num_allowed; ++i) {
    if (value == allowed[i]) {
      return true;
    }
  }
  return false;
}

std::optional<mjtNum> ReadOptionalNumber(const mjModel *m, int instance,
                                         const char *attr) {
  std::string value = ConfigToken(m, instance, attr);
  if (value.empty() || value == "auto" || value == "auto_initial") {
    return std::nullopt;
  }
  char *end = nullptr;
  mjtNum parsed = std::strtod(value.c_str(), &end);
  if (end != value.c_str() + value.size()) {
    return std::nullopt;
  }
  return parsed;
}

mjtNum ReadNumberOr(const mjModel *m, int instance, const char *attr,
                    mjtNum fallback) {
  return ReadOptionalNumber(m, instance, attr).value_or(fallback);
}

std::optional<std::array<mjtNum, 3>> ReadOptionalVector3(
    const mjModel *m, int instance, const char *attr) {
  std::string value = ConfigString(m, instance, attr);
  std::replace(value.begin(), value.end(), ',', ' ');
  std::istringstream stream(value);
  std::array<mjtNum, 3> result;
  if (!(stream >> result[0] >> result[1] >> result[2])) {
    return std::nullopt;
  }
  std::string trailing;
  if (stream >> trailing || !std::isfinite(result[0]) ||
      !std::isfinite(result[1]) || !std::isfinite(result[2])) {
    return std::nullopt;
  }
  mjtNum norm = std::sqrt(result[0] * result[0] + result[1] * result[1] +
                          result[2] * result[2]);
  if (norm <= mjMINVAL) {
    return std::nullopt;
  }
  for (mjtNum &component : result) {
    component /= norm;
  }
  return result;
}

std::vector<std::string> SplitNames(std::string value) {
  std::replace(value.begin(), value.end(), ',', ' ');
  std::istringstream stream(value);
  std::vector<std::string> names;
  for (std::string name; stream >> name;) {
    names.push_back(name);
  }
  return names;
}

mjtNum ClippedCtrl(const mjModel *m, const mjData *d, int actuator_id) {
  mjtNum ctrl = d->ctrl[actuator_id];
  if (m->actuator_ctrllimited[actuator_id]) {
    const mjtNum *range = m->actuator_ctrlrange + 2 * actuator_id;
    ctrl = mju_clip(ctrl, range[0], range[1]);
  }
  return ctrl;
}

bool IsZeroPoint(const mjtNum *point) {
  return point[0] == 0 && point[1] == 0 && point[2] == 0;
}

int BodyForWrapPoint(const mjModel *m, const mjData *d, int obj,
                     const mjtNum *point) {
  if (obj >= 0) {
    return m->geom_bodyid[obj];
  }
  if (obj != -1 || IsZeroPoint(point)) {
    return -1;
  }

  int best_site = -1;
  mjtNum best_dist = 1e-10;
  for (int site_id = 0; site_id < m->nsite; ++site_id) {
    mjtNum dist = mju_dist3(point, d->site_xpos + 3 * site_id);
    if (dist < best_dist) {
      best_dist = dist;
      best_site = site_id;
    }
  }
  return best_site >= 0 ? m->site_bodyid[best_site] : -1;
}

mjtNum WrapAngle(const mjModel *m, const mjData *d, int geom_id,
                 const WrapPoint &a, const WrapPoint &b) {
  mjtNum ra[3];
  mjtNum rb[3];
  mju_sub3(ra, a.pos.data(), d->geom_xpos + 3 * geom_id);
  mju_sub3(rb, b.pos.data(), d->geom_xpos + 3 * geom_id);
  if (mju_normalize3(ra) < mjMINVAL || mju_normalize3(rb) < mjMINVAL) {
    return 0;
  }
  return std::acos(mju_clip(mju_dot3(ra, rb), -1, 1));
}

void ApplyForceAtPoint(const mjModel *m, mjData *d, int body,
                       const mjtNum *point, const mjtNum *force) {
  if (body <= 0) {
    return;
  }
  mjtNum torque[3] = {0, 0, 0};
  mj_applyFT(m, d, force, torque, point, body, d->qfrc_passive);
}

} // namespace

UnilateralCable *UnilateralCable::Create(const mjModel *m, int instance) {
  static constexpr const char *kNumericAttributes[] = {kAttrStiffness,
                                                       kAttrDamping,
                                                       kAttrSlack,
                                                       kAttrControlTimeconstant,
                                                       kAttrMaxContractionRate,
                                                       kAttrTautTransition,
                                                       kAttrTautHysteresis,
                                                       kAttrHomeLength,
                                                       kAttrPretensionOffset,
                                                       kAttrMaxTension,
                                                       kAttrSpoolRadius,
                                                       kAttrSpoolReserveLength,
                                                       kAttrCapstanMu,
                                                       kAttrSiteRoleUserIndex,
                                                       kAttrVisualWidth,
                                                       kAttrRouteHysteresis,
                                                       kAttrCompositeMergeDistance,
                                                       kAttrVisualSmoothingTimeconstant};
  for (const char *attr : kNumericAttributes) {
    if (!CheckOptionalNumber(m, instance, attr, attr == kAttrHomeLength)) {
      mju_warning("invalid numeric config for mujoco.cable.unilateral: %s",
                  attr);
      return nullptr;
    }
  }
  std::optional<mjtNum> spool_reserve_length =
      ReadOptionalNumber(m, instance, kAttrSpoolReserveLength);
  if (spool_reserve_length.has_value() && *spool_reserve_length < 0) {
    mju_warning("spool_reserve_length must be nonnegative");
    return nullptr;
  }
  static constexpr const char *kNonnegativeAttributes[] = {
      kAttrControlTimeconstant, kAttrMaxContractionRate, kAttrTautTransition,
      kAttrTautHysteresis, kAttrRouteHysteresis,
      kAttrCompositeMergeDistance,
      kAttrVisualSmoothingTimeconstant};
  for (const char *attr : kNonnegativeAttributes) {
    if (ReadNumberOr(m, instance, attr, 0) < 0) {
      mju_warning("%s must be nonnegative", attr);
      return nullptr;
    }
  }
  static constexpr const char *kCtrlModes[] = {
      "target_contraction", "target_spool_angle", "spool_velocity",
      "spool_angular_velocity", "joint_spool_angle"};
  if (!CheckOptionalEnum(m, instance, kAttrCtrlMode, kCtrlModes,
                         sizeof(kCtrlModes) / sizeof(kCtrlModes[0]))) {
    mju_warning("invalid ctrl_mode config for mujoco.cable.unilateral");
    return nullptr;
  }
  static constexpr const char *kIntegrationModes[] = {
      "explicit", "local_implicit", "implicit_compliant"};
  if (!CheckOptionalEnum(m, instance, kAttrIntegrationMode,
                         kIntegrationModes,
                         sizeof(kIntegrationModes) /
                             sizeof(kIntegrationModes[0]))) {
    mju_warning("invalid integration_mode config for "
                "mujoco.cable.unilateral");
    return nullptr;
  }
  std::string integration_mode =
      ConfigToken(m, instance, kAttrIntegrationMode);
  if ((integration_mode == "local_implicit" ||
       integration_mode == "implicit_compliant") &&
      ReadNumberOr(m, instance, kAttrCapstanMu, 0) > 0) {
    mju_warning("implicit cable integration does not yet support Capstan "
                "friction");
    return nullptr;
  }
  if (integration_mode == "implicit_compliant") {
#if mjVERSION_HEADER < 3000000
    mju_warning("implicit_compliant requires a MuJoCo build with passive "
                "plugin derivative support");
    return nullptr;
#else
    if (m->opt.integrator != mjINT_IMPLICIT &&
        m->opt.integrator != mjINT_IMPLICITFAST) {
      mju_warning("implicit_compliant requires implicit or implicitfast");
      return nullptr;
    }
#endif
  }
  if (ConfigToken(m, instance, kAttrCtrlMode) == "joint_spool_angle") {
    std::string joint_name = ConfigToken(m, instance, kAttrSpoolJoint);
    int joint_id = joint_name.empty()
                       ? -1
                       : mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
    if (joint_id < 0) {
      mju_warning("joint_spool_angle requires a valid spool_joint config");
      return nullptr;
    }
    if (m->jnt_type[joint_id] != mjJNT_HINGE) {
      mju_warning("joint_spool_angle spool_joint must name a hinge joint");
      return nullptr;
    }
  }
  static constexpr const char *kSpoolReserveDirections[] = {"positive",
                                                            "negative"};
  if (!CheckOptionalEnum(m, instance, kAttrSpoolReserveDirection,
                         kSpoolReserveDirections,
                         sizeof(kSpoolReserveDirections) /
                             sizeof(kSpoolReserveDirections[0]))) {
    mju_warning(
        "invalid spool_reserve_direction config for mujoco.cable.unilateral");
    return nullptr;
  }
  static constexpr const char *kBooleanValues[] = {"false", "true"};
  if (!CheckOptionalEnum(m, instance, kAttrSpoolReactionTorque, kBooleanValues,
                         sizeof(kBooleanValues) / sizeof(kBooleanValues[0]))) {
    mju_warning("spool_reaction_torque must be true or false");
    return nullptr;
  }
  static constexpr const char *kCapstanDirections[] = {"forward", "reverse"};
  if (!CheckOptionalEnum(m, instance, kAttrCapstanDirection, kCapstanDirections,
                         sizeof(kCapstanDirections) /
                             sizeof(kCapstanDirections[0]))) {
    mju_warning("invalid capstan_direction config for mujoco.cable.unilateral");
    return nullptr;
  }

  static constexpr const char *kRouteModes[] = {"native", "surface"};
  if (!CheckOptionalEnum(m, instance, kAttrRouteMode, kRouteModes,
                         sizeof(kRouteModes) / sizeof(kRouteModes[0]))) {
    mju_warning("invalid route_mode config for mujoco.cable.unilateral");
    return nullptr;
  }

  std::string route_mode = ConfigToken(m, instance, kAttrRouteMode);
  if (route_mode.empty()) {
    route_mode = "native";
  }
  if (route_mode == "surface") {
    static constexpr const char *kMeshRouteModes[] = {
        "convex_surface", "taut_obstacle", "guided_surface"};
    if (!CheckOptionalEnum(m, instance, kAttrMeshRouteMode, kMeshRouteModes,
                           sizeof(kMeshRouteModes) /
                               sizeof(kMeshRouteModes[0]))) {
      mju_warning("invalid mesh_route_mode config for mujoco.cable.unilateral");
      return nullptr;
    }
    std::string route_tendon = ConfigToken(m, instance, kAttrRouteTendon);
    if (route_tendon.empty() ||
        mj_name2id(m, mjOBJ_TENDON, route_tendon.c_str()) < 0) {
      mju_warning("surface route_mode requires a valid route_tendon config");
      return nullptr;
    }
    std::vector<std::string> wrap_names =
        SplitNames(ConfigString(m, instance, kAttrWrapGeoms));
    if (wrap_names.empty()) {
      mju_warning("surface route_mode requires at least one wrap_geom");
      return nullptr;
    }
    for (const std::string &name : wrap_names) {
      if (mj_name2id(m, mjOBJ_GEOM, name.c_str()) < 0) {
        mju_warning("surface route wrap_geoms contains unknown geom: %s",
                    name.c_str());
        return nullptr;
      }
    }
    mjtNum role_index = ReadNumberOr(m, instance, kAttrSiteRoleUserIndex, 0);
    if (role_index < 0 || role_index != std::floor(role_index)) {
      mju_warning("site_role_user_index must be a nonnegative integer");
      return nullptr;
    }
    if (ReadNumberOr(m, instance, kAttrVisualWidth, 2) < 0) {
      mju_warning("visual_width must be nonnegative");
      return nullptr;
    }
    std::string guide_axis = ConfigString(m, instance, kAttrMeshGuideAxis);
    if (!guide_axis.empty() &&
        !ReadOptionalVector3(m, instance, kAttrMeshGuideAxis).has_value()) {
      mju_warning("mesh_guide_axis must contain three finite nonzero numbers");
      return nullptr;
    }
    if (!CheckOptionalNumber(m, instance, kAttrMeshGuideWeight, false) ||
        ReadNumberOr(m, instance, kAttrMeshGuideWeight, 0) < 0) {
      mju_warning("mesh_guide_weight must be nonnegative");
      return nullptr;
    }
  }

  auto *cable = new UnilateralCable(m, instance);
  if (route_mode == "surface" && !cable->surface_route_.has_value()) {
    delete cable;
    return nullptr;
  }
  int associated_actuators = 0;
  for (int actuator_id = 0; actuator_id < m->nu; ++actuator_id) {
    associated_actuators += m->actuator_plugin[actuator_id] == instance;
  }
  if (route_mode == "surface" &&
      associated_actuators != static_cast<int>(cable->actuators_.size())) {
    delete cable;
    return nullptr;
  }
  if (cable->actuators_.empty() && route_mode != "surface") {
    mju_warning("mujoco.cable.unilateral plugin instance has no actuator");
  }
  if (route_mode == "surface" && cable->actuators_.size() > 1) {
    mju_warning(
        "surface route_mode supports at most one actuator per plugin instance");
    delete cable;
    return nullptr;
  }
  return cable;
}

UnilateralCable::UnilateralCable(const mjModel *m, int instance)
    : stiffness_(ReadNumberOr(m, instance, kAttrStiffness, 0)),
      damping_(ReadNumberOr(m, instance, kAttrDamping, 0)),
      slack_(ReadNumberOr(m, instance, kAttrSlack, 0)),
      control_timeconstant_(
          ReadNumberOr(m, instance, kAttrControlTimeconstant, 0)),
      max_contraction_rate_(
          ReadNumberOr(m, instance, kAttrMaxContractionRate, 0)),
      taut_transition_(ReadNumberOr(m, instance, kAttrTautTransition, 0)),
      taut_hysteresis_(ReadNumberOr(m, instance, kAttrTautHysteresis, 0)),
      pretension_offset_(ReadNumberOr(m, instance, kAttrPretensionOffset, 0)),
      max_tension_(ReadOptionalNumber(m, instance, kAttrMaxTension)),
      home_length_(ReadOptionalNumber(m, instance, kAttrHomeLength)),
      spool_radius_(ReadNumberOr(m, instance, kAttrSpoolRadius, 1)),
      spool_reserve_length_(
          ReadNumberOr(m, instance, kAttrSpoolReserveLength, 0)),
      spool_reaction_torque_(
          ConfigToken(m, instance, kAttrSpoolReactionTorque) == "true"),
      capstan_mu_(ReadNumberOr(m, instance, kAttrCapstanMu, 0)),
      ctrl_mode_(ConfigString(m, instance, kAttrCtrlMode)),
      integration_mode_(ConfigToken(m, instance, kAttrIntegrationMode)),
      spool_joint_(ConfigToken(m, instance, kAttrSpoolJoint)),
      spool_reserve_direction_(
          ConfigToken(m, instance, kAttrSpoolReserveDirection)),
      capstan_direction_(ConfigString(m, instance, kAttrCapstanDirection)),
      route_mode_(ConfigToken(m, instance, kAttrRouteMode)),
      mesh_route_mode_(ConfigToken(m, instance, kAttrMeshRouteMode)),
      route_tendon_(ConfigToken(m, instance, kAttrRouteTendon)),
      wrap_geoms_(ConfigString(m, instance, kAttrWrapGeoms)),
      site_role_user_index_(static_cast<int>(
          ReadNumberOr(m, instance, kAttrSiteRoleUserIndex, 0))),
      visual_width_(ReadNumberOr(m, instance, kAttrVisualWidth, 2)),
      visual_smoothing_timeconstant_(ReadNumberOr(
          m, instance, kAttrVisualSmoothingTimeconstant, 0)) {
  if (ctrl_mode_.empty()) {
    ctrl_mode_ = "target_contraction";
  }
  if (integration_mode_.empty()) {
    integration_mode_ = "explicit";
  }
  if (spool_reserve_direction_.empty()) {
    spool_reserve_direction_ = "positive";
  }
  if (spool_reserve_direction_ == "negative") {
    spool_reserve_sign_ = -1;
  }
  if (capstan_direction_.empty()) {
    capstan_direction_ = "forward";
  }
  if (route_mode_.empty()) {
    route_mode_ = "native";
  }
  if (mesh_route_mode_.empty()) {
    mesh_route_mode_ = "convex_surface";
  }
  if (!spool_joint_.empty()) {
    spool_joint_id_ = mj_name2id(m, mjOBJ_JOINT, spool_joint_.c_str());
    if (spool_joint_id_ >= 0) {
      spool_qposadr_ = m->jnt_qposadr[spool_joint_id_];
      spool_qpos0_ = m->qpos0[spool_qposadr_];
    }
  }
  capstan_mu_ = mju_max(0, capstan_mu_);
  for (int i = 0; i < m->nu; ++i) {
    if (m->actuator_plugin[i] == instance) {
      if (m->actuator_trntype[i] != mjTRN_TENDON) {
        mju_warning(
            "mujoco.cable.unilateral actuator must use tendon transmission");
        continue;
      }
      if (route_mode_ == "surface") {
        int tendon_id = m->actuator_trnid[2 * i];
        int configured_tendon =
            mj_name2id(m, mjOBJ_TENDON, route_tendon_.c_str());
        if (tendon_id != configured_tendon) {
          mju_warning("surface route actuator tendon must match route_tendon");
          continue;
        }
        bool zero_gear = true;
        for (int gear_index = 0; gear_index < 6; ++gear_index) {
          zero_gear =
              zero_gear &&
              std::abs(m->actuator_gear[6 * i + gear_index]) <= mjMINVAL;
        }
        if (!zero_gear) {
          mju_warning("surface route actuator requires gear=0 to avoid native "
                      "tendon force");
          continue;
        }
      }
      actuators_.push_back(i);
    }
  }
  if (route_mode_ == "surface") {
    route_tendon_id_ = mj_name2id(m, mjOBJ_TENDON, route_tendon_.c_str());
    std::vector<int> wrap_geom_ids;
    for (const std::string &name : SplitNames(wrap_geoms_)) {
      wrap_geom_ids.push_back(mj_name2id(m, mjOBJ_GEOM, name.c_str()));
    }
    std::string error;
    SurfaceEnvelopeRoute::MeshRouteMode mesh_route_mode =
        SurfaceEnvelopeRoute::MeshRouteMode::kConvexSurface;
    if (mesh_route_mode_ == "taut_obstacle") {
      mesh_route_mode = SurfaceEnvelopeRoute::MeshRouteMode::kTautObstacle;
    } else if (mesh_route_mode_ == "guided_surface") {
      mesh_route_mode = SurfaceEnvelopeRoute::MeshRouteMode::kGuidedSurface;
    }
    surface_route_ = SurfaceEnvelopeRoute::Create(
        m, route_tendon_id_, wrap_geom_ids, site_role_user_index_,
        mesh_route_mode, ReadNumberOr(m, instance, kAttrRouteHysteresis, 0),
        ReadNumberOr(m, instance, kAttrCompositeMergeDistance, 0),
        ReadOptionalVector3(m, instance, kAttrMeshGuideAxis),
        ReadNumberOr(m, instance, kAttrMeshGuideWeight, 20),
        &error);
    if (!surface_route_.has_value()) {
      mju_warning("invalid surface route config: %s", error.c_str());
    }
  }
  cached_native_results_.resize(m->nu);
  cached_native_valid_.assign(m->nu, false);
}

void UnilateralCable::Reset(mjtNum *plugin_state) {
  if (plugin_state) {
    for (int state_index = 0; state_index < kStateSize; ++state_index) {
      plugin_state[state_index] = 0;
    }
  }
  if (surface_route_.has_value()) {
    surface_route_->Reset();
  }
  surface_result_valid_ = false;
  warned_invalid_route_ = false;
  visual_points_.clear();
  visual_time_ = -1;
  std::fill(cached_native_valid_.begin(), cached_native_valid_.end(), false);
}

void UnilateralCable::Advance(const mjModel *m, mjData *d, int instance) const {
  mjtNum *state = d->plugin_state + m->plugin_stateadr[instance];
  int actuator_id = actuators_.empty() ? -1 : actuators_[0];
  if (ctrl_mode_ == "spool_velocity" ||
      ctrl_mode_ == "spool_angular_velocity") {
    if (actuator_id < 0) {
      return;
    }
    mjtNum speed = ClippedCtrl(m, d, actuator_id);
    if (ctrl_mode_ == "spool_angular_velocity") {
      speed *= spool_radius_;
    }
    if (max_contraction_rate_ > 0) {
      speed = mju_clip(speed, -max_contraction_rate_, max_contraction_rate_);
    }
    state[kIntegratedContractionState] += m->opt.timestep * speed;
    state[kIntegratedContractionState] =
        mju_max(0, state[kIntegratedContractionState]);
    return;
  }
  if (control_timeconstant_ <= 0 && max_contraction_rate_ <= 0) {
    return;
  }
  mjtNum target = RawContraction(m, d, actuator_id);
  mjtNum alpha =
      control_timeconstant_ > 0
          ? -std::expm1(-m->opt.timestep / control_timeconstant_)
          : 1;
  mjtNum step = alpha * (target - state[kFilteredContractionState]);
  if (max_contraction_rate_ > 0) {
    mjtNum maximum_step = max_contraction_rate_ * m->opt.timestep;
    step = mju_clip(step, -maximum_step, maximum_step);
  }
  state[kFilteredContractionState] += step;
}

void UnilateralCable::Compute(const mjModel *m, mjData *d, int instance,
                              int capability_bit) {
  if (route_mode_ == "surface") {
    int actuator_id = actuators_.empty() ? -1 : actuators_[0];
    if (capability_bit == mjPLUGIN_PASSIVE) {
      if (!UpdateSurfaceRoute(m, d)) {
        cached_surface_result_ = EvalResult();
        surface_result_valid_ = false;
        if (integration_mode_ == "local_implicit") {
          d->plugin_state[m->plugin_stateadr[instance] +
                          kPreviousTensionState] = 0;
          d->plugin_state[m->plugin_stateadr[instance] +
                          kImplicitInitializedState] = 1;
        }
        if (!warned_invalid_route_) {
          mju_warning(
              "surface cable route is invalid; cable force is disabled");
          warned_invalid_route_ = true;
        }
        return;
      }
      cached_surface_result_ = Evaluate(m, d, instance, actuator_id, true);
      surface_result_valid_ = true;
      warned_invalid_route_ = false;
      ApplyImplicitRhsCorrection(m, d, cached_surface_result_, actuator_id);
      ApplySurfaceForces(m, d, cached_surface_result_);
      ApplySpoolReactionTorque(m, d, cached_surface_result_);
      return;
    }
    if (capability_bit == mjPLUGIN_ACTUATOR) {
      if (actuator_id >= 0) {
        d->actuator_force[actuator_id] =
            surface_result_valid_ ? -cached_surface_result_.tension : 0;
      }
      return;
    }
    if (capability_bit == mjPLUGIN_SENSOR) {
      // Report the route used by the passive-force stage. Retrying here can
      // turn a force-disabled step into a valid sensor sample and hide an
      // intermittent routing failure from diagnostics.
      const SurfaceEnvelopeRoute::Result &route = surface_route_->result();
      for (int sensor_id = 0; sensor_id < m->nsensor; ++sensor_id) {
        if (m->sensor_type[sensor_id] == mjSENS_PLUGIN &&
            m->sensor_plugin[sensor_id] == instance) {
          mjtNum *out = d->sensordata + m->sensor_adr[sensor_id];
          out[0] = cached_surface_result_.length;
          out[1] = cached_surface_result_.velocity;
          out[2] = cached_surface_result_.free_length;
          out[3] = cached_surface_result_.contraction;
          out[4] = cached_surface_result_.extension;
          out[5] = cached_surface_result_.tension;
          out[6] = cached_surface_result_.taut;
          out[7] = cached_surface_result_.saturated;
          out[8] = static_cast<int>(route.status);
          out[9] = route.tangent_residual;
          out[10] = route.surface_residual;
          out[11] = route.solver_iterations;
        }
      }
      return;
    }
  }

  if (capability_bit == mjPLUGIN_ACTUATOR) {
    for (int actuator_id : actuators_) {
      EvalResult result;
      if (integration_mode_ != "explicit" &&
          cached_native_valid_[actuator_id]) {
        result = cached_native_results_[actuator_id];
      } else {
        result = Evaluate(m, d, instance, actuator_id, true);
        if (integration_mode_ != "explicit") {
          cached_native_results_[actuator_id] = result;
          cached_native_valid_[actuator_id] = true;
        }
      }
      d->actuator_force[actuator_id] = -result.tension;
    }
    return;
  }

  if (capability_bit == mjPLUGIN_PASSIVE) {
    for (int actuator_id : actuators_) {
      EvalResult result = Evaluate(m, d, instance, actuator_id, true);
      if (integration_mode_ != "explicit") {
        cached_native_results_[actuator_id] = result;
        cached_native_valid_[actuator_id] = true;
      }
      ApplyImplicitRhsCorrection(m, d, result, actuator_id);
      ApplySpoolReactionTorque(m, d, result);
      if (capstan_mu_ > 0) {
        ApplyCapstanFriction(m, d, result, actuator_id);
      }
    }
    return;
  }

  if (capability_bit == mjPLUGIN_SENSOR) {
    EvalResult result;
    if (!actuators_.empty()) {
      int actuator_id = actuators_[0];
      if (integration_mode_ != "explicit" &&
          cached_native_valid_[actuator_id]) {
        result = cached_native_results_[actuator_id];
      } else {
        result = Evaluate(m, d, instance, actuator_id, false);
      }
    }
    for (int sensor_id = 0; sensor_id < m->nsensor; ++sensor_id) {
      if (m->sensor_type[sensor_id] == mjSENS_PLUGIN &&
          m->sensor_plugin[sensor_id] == instance) {
        mjtNum *out = d->sensordata + m->sensor_adr[sensor_id];
        out[0] = result.length;
        out[1] = result.velocity;
        out[2] = result.free_length;
        out[3] = result.contraction;
        out[4] = result.extension;
        out[5] = result.tension;
        out[6] = result.taut;
        out[7] = result.saturated;
      }
    }
  }
}

int UnilateralCable::StateSize(const mjModel *, int) { return kStateSize; }

int UnilateralCable::SensorDim(const mjModel *m, int instance, int) {
  return ConfigToken(m, instance, kAttrRouteMode) == "surface" ? 12 : 8;
}

mjtNum UnilateralCable::SpoolAngleContraction(mjtNum angle) const {
  mjtNum signed_reserve = spool_reserve_sign_ * spool_reserve_length_;
  mjtNum signed_wound_length = signed_reserve + spool_radius_ * angle;
  return std::abs(signed_wound_length) - spool_reserve_length_;
}

mjtNum UnilateralCable::SpoolAngleContractionDerivative(mjtNum angle) const {
  mjtNum signed_reserve = spool_reserve_sign_ * spool_reserve_length_;
  mjtNum signed_wound_length = signed_reserve + spool_radius_ * angle;
  mjtNum winding_sign =
      signed_wound_length > 0
          ? 1
          : (signed_wound_length < 0 ? -1 : spool_reserve_sign_);
  return winding_sign * spool_radius_;
}

void UnilateralCable::RegisterPlugin() {
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);
  plugin.name = "mujoco.cable.unilateral";
  plugin.capabilityflags |=
      mjPLUGIN_ACTUATOR | mjPLUGIN_SENSOR | mjPLUGIN_PASSIVE;
  plugin.needstage = mjSTAGE_VEL;

  static const char *attributes[] = {kAttrStiffness,
                                     kAttrDamping,
                                     kAttrSlack,
                                     kAttrControlTimeconstant,
                                     kAttrMaxContractionRate,
                                     kAttrTautTransition,
                                     kAttrTautHysteresis,
                                     kAttrIntegrationMode,
                                     kAttrHomeLength,
                                     kAttrPretensionOffset,
                                     kAttrMaxTension,
                                     kAttrCtrlMode,
                                     kAttrSpoolRadius,
                                     kAttrSpoolJoint,
                                     kAttrSpoolReserveLength,
                                     kAttrSpoolReserveDirection,
                                     kAttrSpoolReactionTorque,
                                     kAttrCapstanMu,
                                     kAttrCapstanDirection,
                                     kAttrRouteMode,
                                     kAttrMeshRouteMode,
                                     kAttrMeshGuideAxis,
                                     kAttrMeshGuideWeight,
                                     kAttrRouteTendon,
                                     kAttrWrapGeoms,
                                     kAttrSiteRoleUserIndex,
                                     kAttrVisualWidth,
                                     kAttrRouteHysteresis,
                                     kAttrCompositeMergeDistance,
                                     kAttrVisualSmoothingTimeconstant};
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;
  plugin.nstate = UnilateralCable::StateSize;
  plugin.nsensordata = UnilateralCable::SensorDim;
  plugin.init = +[](const mjModel *m, mjData *d, int instance) {
    auto *cable = UnilateralCable::Create(m, instance);
    if (!cable) {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(cable);
    return 0;
  };
  plugin.destroy = +[](mjData *d, int instance) {
    delete reinterpret_cast<UnilateralCable *>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };
  plugin.copy =
      +[](mjData *dest, const mjModel *, const mjData *source, int instance) {
        auto *destination_cable =
            reinterpret_cast<UnilateralCable *>(dest->plugin_data[instance]);
        const auto *source_cable = reinterpret_cast<const UnilateralCable *>(
            source->plugin_data[instance]);
        destination_cable->CopyFrom(*source_cable);
      };
  plugin.reset =
      +[](const mjModel *, mjtNum *plugin_state, void *plugin_data, int) {
        reinterpret_cast<UnilateralCable *>(plugin_data)->Reset(plugin_state);
      };
  plugin.compute =
      +[](const mjModel *m, mjData *d, int instance, int capability_bit) {
        reinterpret_cast<UnilateralCable *>(d->plugin_data[instance])
            ->Compute(m, d, instance, capability_bit);
      };
  plugin.advance = +[](const mjModel *m, mjData *d, int instance) {
    reinterpret_cast<UnilateralCable *>(d->plugin_data[instance])
        ->Advance(m, d, instance);
  };
  plugin.visualize = +[](const mjModel *m, mjData *d, const mjvOption *,
                         mjvScene *scene, int instance) {
    reinterpret_cast<UnilateralCable *>(d->plugin_data[instance])
        ->Visualize(m, d, scene);
  };
  mjp_registerPlugin(&plugin);
#if mjVERSION_HEADER >= 3000000
  mjpPluginImplicit implicit;
  mjp_defaultPluginImplicit(&implicit);
  implicit.plugin_name = plugin.name;
  implicit.active = +[](const mjModel *, const mjData *d,
                        int instance) -> int {
    return reinterpret_cast<const UnilateralCable *>(
               d->plugin_data[instance])
        ->ImplicitOperatorActive();
  };
  implicit.multiply = +[](const mjModel *m, mjData *d, int instance,
                          const mjtNum *vector, mjtNum *result) {
    reinterpret_cast<const UnilateralCable *>(d->plugin_data[instance])
        ->MultiplyImplicitOperator(m, d, vector, result);
  };
  mjp_registerPluginImplicit(&implicit);
#endif
}

UnilateralCable::EvalResult UnilateralCable::Evaluate(
    const mjModel *m, mjData *d, int instance, int actuator_id,
    bool update_hysteresis) const {
  EvalResult result;
  int tendon_id = route_mode_ == "surface" ? route_tendon_id_
                                           : m->actuator_trnid[2 * actuator_id];
  if (route_mode_ == "surface") {
    result.length = surface_route_->result().length;
    result.velocity = surface_route_->result().velocity;
  } else {
    NativeTendonRoute::Result route =
        NativeTendonRoute::Evaluate(m, d, actuator_id);
    tendon_id = route.tendon_id;
    result.length = route.length;
    result.velocity = route.velocity;
  }
  const mjtNum *state = d->plugin_state + m->plugin_stateadr[instance];
  if (ctrl_mode_ == "spool_velocity" ||
      ctrl_mode_ == "spool_angular_velocity") {
    result.contraction = state[kIntegratedContractionState];
  } else if (control_timeconstant_ > 0 || max_contraction_rate_ > 0) {
    result.contraction = state[kFilteredContractionState];
  } else {
    result.contraction = RawContraction(m, d, actuator_id);
  }
  mjtNum automatic_home =
      route_mode_ == "surface" && surface_route_->has_initial_length()
          ? surface_route_->initial_length()
          : m->tendon_length0[tendon_id];
  mjtNum home = home_length_.value_or(automatic_home);
  result.free_length = home - result.contraction - pretension_offset_;
  result.extension = result.length - result.free_length - slack_;
  bool engaged = result.extension > 0;
  if (taut_hysteresis_ > 0) {
    engaged = state[kTautLatchState] > 0.5;
    if (engaged && result.extension <= 0) {
      engaged = false;
    } else if (!engaged && result.extension >= taut_hysteresis_) {
      engaged = true;
    }
    if (update_hysteresis) {
      d->plugin_state[m->plugin_stateadr[instance] + kTautLatchState] =
          engaged ? 1 : 0;
    }
  }
  if (!engaged || result.extension <= 0) {
    if (update_hysteresis && integration_mode_ == "local_implicit") {
      d->plugin_state[m->plugin_stateadr[instance] +
                      kPreviousTensionState] = 0;
      d->plugin_state[m->plugin_stateadr[instance] +
                      kImplicitInitializedState] = 1;
    }
    return result;
  }
  mjtNum elastic_extension = result.extension;
  mjtNum damping_scale = 1;
  if (taut_transition_ > 0 && result.extension < taut_transition_) {
    elastic_extension =
        result.extension * result.extension / (2 * taut_transition_);
    damping_scale = result.extension / taut_transition_;
    result.effective_stiffness = stiffness_ * damping_scale;
  } else if (taut_transition_ > 0) {
    elastic_extension = result.extension - 0.5 * taut_transition_;
    result.effective_stiffness = stiffness_;
  } else {
    result.effective_stiffness = stiffness_;
  }
  result.effective_damping = damping_ * damping_scale;
  mjtNum raw = stiffness_ * elastic_extension +
               result.effective_damping * result.velocity;
  result.tension = mju_max(0, raw);
  if (integration_mode_ == "local_implicit" && result.tension > 0) {
    result.tension =
        LocalImplicitTension(m, d, instance, actuator_id, result);
  }
  if (max_tension_.has_value() && result.tension > *max_tension_) {
    result.tension = *max_tension_;
    result.saturated = 1;
  }
  result.taut = result.tension > 0;
  if (!result.taut || result.saturated) {
    result.effective_stiffness = 0;
    result.effective_damping = 0;
  }
  if (update_hysteresis && integration_mode_ == "local_implicit") {
    d->plugin_state[m->plugin_stateadr[instance] +
                    kPreviousTensionState] = result.tension;
    d->plugin_state[m->plugin_stateadr[instance] +
                    kImplicitInitializedState] = 1;
  }
  return result;
}

bool UnilateralCable::BuildExtensionJacobian(
    const mjModel *m, mjData *d, int actuator_id,
    std::vector<mjtNum> *jacobian) const {
  jacobian->assign(m->nv, 0);
  if (route_mode_ == "surface") {
    if (!surface_route_.has_value()) {
      return false;
    }
    const std::vector<SurfaceEnvelopeRoute::Point> &points =
        surface_route_->result().points;
    const std::vector<std::array<mjtNum, 3>> &directions =
        surface_route_->result().segment_directions;
    if (points.size() < 2 || directions.size() + 1 != points.size()) {
      return false;
    }

    std::vector<mjtNum> unit_force(m->nv, 0);
    for (int node = 0; node < static_cast<int>(points.size()); ++node) {
      if (points[node].body_id <= 0) {
        continue;
      }
      mjtNum force[3] = {0, 0, 0};
      if (node > 0) {
        mju_addToScl3(force, directions[node - 1].data(), -1);
      }
      if (node + 1 < static_cast<int>(points.size())) {
        mju_addToScl3(force, directions[node].data(), 1);
      }
      mjtNum torque[3] = {0, 0, 0};
      mj_applyFT(m, d, force, torque, points[node].pos.data(),
                 points[node].body_id, unit_force.data());
    }
    for (int dof = 0; dof < m->nv; ++dof) {
      (*jacobian)[dof] = -unit_force[dof];
    }
  } else {
    if (actuator_id < 0) {
      return false;
    }
    int tendon_id = m->actuator_trnid[2 * actuator_id];
#if mjVERSION_HEADER >= 3000000
    int row_address = m->ten_J_rowadr[tendon_id];
    int row_nonzeros = m->ten_J_rownnz[tendon_id];
    for (int entry = 0; entry < row_nonzeros; ++entry) {
      int address = row_address + entry;
      (*jacobian)[m->ten_J_colind[address]] = d->ten_J[address];
    }
#else
    if (mj_isSparse(m)) {
      int row_address = d->ten_J_rowadr[tendon_id];
      int row_nonzeros = d->ten_J_rownnz[tendon_id];
      for (int entry = 0; entry < row_nonzeros; ++entry) {
        int address = row_address + entry;
        (*jacobian)[d->ten_J_colind[address]] = d->ten_J[address];
      }
    } else {
      mju_copy(jacobian->data(), d->ten_J + tendon_id * m->nv, m->nv);
    }
#endif
  }

  if (spool_reaction_torque_ && ctrl_mode_ == "joint_spool_angle" &&
      spool_joint_id_ >= 0 && spool_qposadr_ >= 0) {
    int dof_address = m->jnt_dofadr[spool_joint_id_];
    mjtNum angle = d->qpos[spool_qposadr_] - spool_qpos0_;
    (*jacobian)[dof_address] += SpoolAngleContractionDerivative(angle);
  }
  return mju_norm(jacobian->data(), m->nv) > mjMINVAL;
}

mjtNum UnilateralCable::LocalImplicitTension(
    const mjModel *m, mjData *d, int instance, int actuator_id,
    const EvalResult &result) const {
  std::vector<mjtNum> jacobian;
  if (!BuildExtensionJacobian(m, d, actuator_id, &jacobian)) {
    return result.tension;
  }

  std::vector<mjtNum> inverse_mass_jacobian(m->nv, 0);
  mj_solveM(m, d, inverse_mass_jacobian.data(), jacobian.data(), 1);
  mjtNum inverse_effective_mass =
      mju_dot(jacobian.data(), inverse_mass_jacobian.data(), m->nv);
  if (!(inverse_effective_mass > mjMINVAL) ||
      mju_isBad(inverse_effective_mass)) {
    return result.tension;
  }

  const mjtNum *state =
      d->plugin_state + m->plugin_stateadr[instance];
  if (state[kImplicitInitializedState] < 0.5) {
    return result.tension;
  }
  mjtNum previous_tension = mju_max(0, state[kPreviousTensionState]);

  mjtNum timestep = m->opt.timestep;
  mjtNum beta = timestep * result.effective_damping +
                timestep * timestep * result.effective_stiffness;
  mjtNum numerator = result.tension +
                     beta * inverse_effective_mass * previous_tension;
  mjtNum denominator = 1 + beta * inverse_effective_mass;
  if (!(denominator > mjMINVAL) || mju_isBad(numerator) ||
      mju_isBad(denominator)) {
    return result.tension;
  }
  return mju_max(0, numerator / denominator);
}

void UnilateralCable::ApplyImplicitRhsCorrection(
    const mjModel *m, mjData *d, const EvalResult &result,
    int actuator_id) const {
  if (integration_mode_ != "implicit_compliant" || result.tension <= 0 ||
      result.saturated || result.effective_stiffness <= 0) {
    return;
  }
  std::vector<mjtNum> jacobian;
  if (!BuildExtensionJacobian(m, d, actuator_id, &jacobian)) {
    return;
  }
  mjtNum route_velocity = mju_dot(jacobian.data(), d->qvel, m->nv);
  mjtNum scale =
      -m->opt.timestep * result.effective_stiffness * route_velocity;
  mju_addToScl(d->qfrc_passive, jacobian.data(), scale, m->nv);
}

bool UnilateralCable::ImplicitOperatorActive() const {
#if mjVERSION_HEADER >= 3000000
  if (integration_mode_ != "implicit_compliant") {
    return false;
  }
  auto active_result = [](const EvalResult &result) {
    return result.tension > 0 && !result.saturated &&
           (result.effective_stiffness > 0 ||
            result.effective_damping > 0);
  };
  if (route_mode_ == "surface") {
    return surface_result_valid_ && active_result(cached_surface_result_);
  }
  for (int actuator_id : actuators_) {
    if (cached_native_valid_[actuator_id] &&
        active_result(cached_native_results_[actuator_id])) {
      return true;
    }
  }
#endif
  return false;
}

void UnilateralCable::MultiplyImplicitOperator(
    const mjModel *m, mjData *d, const mjtNum *vector,
    mjtNum *result) const {
#if mjVERSION_HEADER >= 3000000
  if (integration_mode_ != "implicit_compliant") {
    return;
  }
  auto multiply_result = [&](const EvalResult &cable_result,
                             int actuator_id) {
    if (cable_result.tension <= 0 || cable_result.saturated ||
        (cable_result.effective_stiffness <= 0 &&
         cable_result.effective_damping <= 0)) {
      return;
    }
    std::vector<mjtNum> jacobian;
    if (!BuildExtensionJacobian(m, d, actuator_id, &jacobian)) {
      return;
    }
    mjtNum timestep = m->opt.timestep;
    mjtNum coefficient = timestep * cable_result.effective_damping +
                         timestep * timestep *
                             cable_result.effective_stiffness;
    mjtNum projection = mju_dot(jacobian.data(), vector, m->nv);
    mju_addToScl(result, jacobian.data(), coefficient * projection, m->nv);
  };

  if (route_mode_ == "surface") {
    if (surface_result_valid_) {
      int actuator_id = actuators_.empty() ? -1 : actuators_[0];
      multiply_result(cached_surface_result_, actuator_id);
    }
    return;
  }
  for (int actuator_id : actuators_) {
    if (cached_native_valid_[actuator_id]) {
      multiply_result(cached_native_results_[actuator_id], actuator_id);
    }
  }
#else
  (void)m;
  (void)d;
  (void)vector;
  (void)result;
#endif
}

mjtNum UnilateralCable::RawContraction(const mjModel *m, const mjData *d,
                                       int actuator_id) const {
  if (ctrl_mode_ == "target_spool_angle") {
    return actuator_id >= 0
               ? SpoolAngleContraction(ClippedCtrl(m, d, actuator_id))
               : 0;
  }
  if (ctrl_mode_ == "joint_spool_angle") {
    return spool_qposadr_ >= 0
               ? SpoolAngleContraction(d->qpos[spool_qposadr_] - spool_qpos0_)
               : 0;
  }
  return actuator_id >= 0 ? ClippedCtrl(m, d, actuator_id) : 0;
}

bool UnilateralCable::UpdateSurfaceRoute(const mjModel *m, const mjData *d) {
  if (!surface_route_.has_value() || !surface_route_->Update(m, d)) {
    return false;
  }
  return surface_route_->result().status !=
         SurfaceEnvelopeRoute::Status::kInvalid;
}

void UnilateralCable::ApplySurfaceForces(const mjModel *m, mjData *d,
                                         const EvalResult &result) const {
  if (result.tension <= 0 || !surface_route_.has_value()) {
    return;
  }
  const std::vector<SurfaceEnvelopeRoute::Point> &points =
      surface_route_->result().points;
  const std::vector<std::array<mjtNum, 3>> &directions =
      surface_route_->result().segment_directions;
  if (points.size() < 2 || directions.size() + 1 != points.size()) {
    return;
  }

  std::vector<mjtNum> segment_tensions(points.size() - 1, result.tension);
  mjtNum current_tension = result.tension;
  bool reverse = capstan_direction_ == "reverse";
  for (int segment = 0; segment + 1 < static_cast<int>(points.size());
       ++segment) {
    segment_tensions[segment] = current_tension;
    int node = segment + 1;
    if (capstan_mu_ <= 0 || node + 1 >= static_cast<int>(points.size()) ||
        points[node].wrap_index < 0 ||
        points[node - 1].wrap_index != points[node].wrap_index ||
        points[node + 1].wrap_index != points[node].wrap_index) {
      continue;
    }
    mjtNum turn = std::acos(mju_clip(
        mju_dot3(directions[segment].data(), directions[segment + 1].data()),
        -1, 1));
    mjtNum ratio = std::exp(mju_min(capstan_mu_ * turn, mjtNum(20)));
    current_tension =
        reverse ? current_tension * ratio : current_tension / ratio;
  }

  for (int node = 0; node < static_cast<int>(points.size()); ++node) {
    mjtNum force[3] = {0, 0, 0};
    if (node > 0) {
      mju_addToScl3(force, directions[node - 1].data(),
                    -segment_tensions[node - 1]);
    }
    if (node + 1 < static_cast<int>(points.size())) {
      mju_addToScl3(force, directions[node].data(), segment_tensions[node]);
    }
    ApplyForceAtPoint(m, d, points[node].body_id, points[node].pos.data(),
                      force);
  }
}

void UnilateralCable::ApplySpoolReactionTorque(const mjModel *m, mjData *d,
                                               const EvalResult &result) const {
  if (!spool_reaction_torque_ || ctrl_mode_ != "joint_spool_angle" ||
      result.tension <= 0 || spool_joint_id_ < 0 || spool_qposadr_ < 0) {
    return;
  }
  int dof_address = m->jnt_dofadr[spool_joint_id_];
  mjtNum angle = d->qpos[spool_qposadr_] - spool_qpos0_;
  d->qfrc_passive[dof_address] -=
      result.tension * SpoolAngleContractionDerivative(angle);
}

void UnilateralCable::Visualize(const mjModel *m, mjData *d,
                                mjvScene *scene) const {
  if (route_mode_ != "surface" || !surface_route_.has_value() ||
      visual_width_ <= 0) {
    return;
  }
  if (!surface_result_valid_ ||
      surface_route_->result().status ==
          SurfaceEnvelopeRoute::Status::kInvalid) {
    visual_points_.clear();
    visual_time_ = -1;
    return;
  }
  const std::vector<SurfaceEnvelopeRoute::Point> &points =
      surface_route_->result().points;
  if (points.size() < 2) {
    return;
  }
  if (visual_smoothing_timeconstant_ > 0) {
    if (visual_points_.size() != points.size() || visual_time_ < 0 ||
        d->time < visual_time_) {
      visual_points_.resize(points.size());
      for (int point = 0; point < static_cast<int>(points.size()); ++point) {
        visual_points_[point] = points[point].pos;
      }
    } else {
      mjtNum alpha = -std::expm1(
          -(d->time - visual_time_) / visual_smoothing_timeconstant_);
      alpha = mju_clip(alpha, 0, 1);
      for (int point = 1; point + 1 < static_cast<int>(points.size()); ++point) {
        for (int axis = 0; axis < 3; ++axis) {
          visual_points_[point][axis] +=
              alpha * (points[point].pos[axis] - visual_points_[point][axis]);
        }
      }
      visual_points_.front() = points.front().pos;
      visual_points_.back() = points.back().pos;
    }
    visual_time_ = d->time;
  }
  const float *rgba = m->tendon_rgba + 4 * route_tendon_id_;
  for (int segment = 0; segment + 1 < static_cast<int>(points.size());
       ++segment) {
    if (scene->ngeom >= scene->maxgeom) {
      if (!scene->status) {
        mju_warning(
            "surface cable visualization buffer is full; increase maxgeom");
        scene->status = 1;
      }
      return;
    }
    mjvGeom *geom = scene->geoms + scene->ngeom;
    mjv_initGeom(geom, mjGEOM_NONE, nullptr, nullptr, nullptr, nullptr);
    geom->objtype = mjOBJ_UNKNOWN;
    geom->objid = route_tendon_id_;
    geom->category = mjCAT_DECOR;
    geom->segid = scene->ngeom;
    const mjtNum *start = visual_smoothing_timeconstant_ > 0
                              ? visual_points_[segment].data()
                              : points[segment].pos.data();
    const mjtNum *end = visual_smoothing_timeconstant_ > 0
                            ? visual_points_[segment + 1].data()
                            : points[segment + 1].pos.data();
    mjv_connector(geom, mjGEOM_LINE, visual_width_, start, end);
    for (int channel = 0; channel < 4; ++channel) {
      geom->rgba[channel] = rgba[channel];
    }
    ++scene->ngeom;
  }
}

void UnilateralCable::CopyFrom(const UnilateralCable &source) {
  *this = source;
}

void UnilateralCable::ApplyCapstanFriction(const mjModel *m, mjData *d,
                                           const EvalResult &result,
                                           int actuator_id) const {
  if (result.tension <= 0 || capstan_mu_ <= 0) {
    return;
  }

  int tendon_id = m->actuator_trnid[2 * actuator_id];
  int adr = d->ten_wrapadr[tendon_id];
  int num = d->ten_wrapnum[tendon_id];
  if (num < 2) {
    return;
  }

  std::vector<WrapPoint> points;
  points.reserve(num);
  for (int i = 0; i < num; ++i) {
    const mjtNum *xpos = d->wrap_xpos + 3 * (adr + i);
    int obj = d->wrap_obj[adr + i];
    points.push_back(WrapPoint{
        {xpos[0], xpos[1], xpos[2]},
        obj,
        BodyForWrapPoint(m, d, obj, xpos),
    });
  }

  std::vector<mjtNum> segment_tensions(num - 1, result.tension);
  mjtNum current = result.tension;
  bool reverse = capstan_direction_ == "reverse";
  for (int i = 0; i < num - 1; ++i) {
    segment_tensions[i] = current;
    if (points[i].obj >= 0 && points[i].obj == points[i + 1].obj) {
      mjtNum angle = WrapAngle(m, d, points[i].obj, points[i], points[i + 1]);
      mjtNum ratio = std::exp(mju_min(capstan_mu_ * angle, mjtNum(20)));
      if (reverse) {
        current *= ratio;
      } else {
        current /= ratio;
      }
    }
  }

  for (int i = 0; i < num - 1; ++i) {
    if (points[i].body == points[i + 1].body) {
      continue;
    }
    mjtNum delta = segment_tensions[i] - result.tension;
    if (std::abs(delta) <= mjMINVAL) {
      continue;
    }

    mjtNum dir[3];
    mju_sub3(dir, points[i + 1].pos.data(), points[i].pos.data());
    if (mju_normalize3(dir) < mjMINVAL) {
      continue;
    }

    mjtNum force0[3];
    mjtNum force1[3];
    mju_scl3(force0, dir, delta);
    mju_scl3(force1, dir, -delta);
    ApplyForceAtPoint(m, d, points[i].body, points[i].pos.data(), force0);
    ApplyForceAtPoint(m, d, points[i + 1].body, points[i + 1].pos.data(),
                      force1);
  }
}

} // namespace mujoco::plugin::cable
