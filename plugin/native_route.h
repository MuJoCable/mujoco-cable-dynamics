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

#ifndef MUJOCO_PLUGIN_CABLE_NATIVE_ROUTE_H_
#define MUJOCO_PLUGIN_CABLE_NATIVE_ROUTE_H_

#include <mujoco/mujoco.h>

namespace mujoco::plugin::cable {

class NativeTendonRoute {
public:
  struct Result {
    int tendon_id = -1;
    mjtNum length = 0;
    mjtNum velocity = 0;
  };

  static Result Evaluate(const mjModel *m, const mjData *d, int actuator_id) {
    Result result;
    result.tendon_id = m->actuator_trnid[2 * actuator_id];
    result.length = d->ten_length[result.tendon_id];
    result.velocity = d->ten_velocity[result.tendon_id];
    return result;
  }
};

} // namespace mujoco::plugin::cable

#endif // MUJOCO_PLUGIN_CABLE_NATIVE_ROUTE_H_
