// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_PLUGIN_ELASTICITY_PLUGIN_TIMING_H_
#define MUJOCO_PLUGIN_ELASTICITY_PLUGIN_TIMING_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

namespace mujoco::plugin::elasticity {

// Get timing statistics for a plugin instance.
// Returns 0 on success, -1 on error (invalid parameters), -2 if plugin doesn't support timing.
// On success, total_time_ms, applyFT_time_ms, and call_count are filled.
// applyFT_time_ms is the time spent in mj_applyFT calls; rest = total_time_ms - applyFT_time_ms.
int mj_getPluginTiming(const mjModel* m, mjData* d, int instance,
                       double* total_time_ms, double* applyFT_time_ms, int* call_count);

}  // namespace mujoco::plugin::elasticity

#endif  // MUJOCO_PLUGIN_ELASTICITY_PLUGIN_TIMING_H_

