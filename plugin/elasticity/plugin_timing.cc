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

#include "plugin_timing.h"

#include <cstring>
#include <iostream>

#include <mujoco/mjexport.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>

#include "cable.h"
#include "wire.h"
#include "wire_qst.h"

// Forward declaration of core MuJoCo helper to access plugin metadata.
extern "C" MJAPI const mjpPlugin* mjp_getPluginAtSlot(int slot);

namespace mujoco::plugin::elasticity {

// Internal helper: read timing fields from Wire/Cable/WireQST instances.
static int GetPluginTimingInternal(const mjModel* m, mjData* d, int instance,
                                   double* total_time_ms, double* applyFT_time_ms,
                                   int* call_count) {
  if (!m || !d || instance < 0 || instance >= m->nplugin) {
    return -1;  // Invalid instance
  }

  if (!total_time_ms || !applyFT_time_ms || !call_count) {
    return -1;  // Invalid output pointers
  }

  // Get plugin slot and plugin struct
  const int slot = m->plugin[instance];
  const mjpPlugin* plugin = mjp_getPluginAtSlot(slot);
  if (!plugin || !plugin->name) {
    return -1;  // Invalid plugin
  }
  // Get plugin data pointer
  if (!d->plugin_data || !d->plugin_data[instance]) {
    *total_time_ms = 0.0;
    *applyFT_time_ms = 0.0;
    *call_count = 0;
    return 0;  // No plugin data (plugin not initialized)
  }

  // Check plugin name and cast appropriately
  if (std::strcmp(plugin->name, "mujoco.elasticity.wire") == 0) {
    auto* wire = reinterpret_cast<Wire*>(d->plugin_data[instance]);
    *total_time_ms = wire->total_compute_time_ms;
    *applyFT_time_ms = wire->total_applyFT_time_ms;
    *call_count = wire->compute_call_count;
    return 0;
  } else if (std::strcmp(plugin->name, "mujoco.elasticity.cable") == 0) {
    auto* cable = reinterpret_cast<Cable*>(d->plugin_data[instance]);
    *total_time_ms = cable->total_compute_time_ms;
    *applyFT_time_ms = cable->total_applyFT_time_ms;
    *call_count = cable->compute_call_count;
    return 0;
  } else if (std::strcmp(plugin->name, "mujoco.elasticity.wire_qst") == 0) {
    auto* wire_qst = reinterpret_cast<WireQST*>(d->plugin_data[instance]);
    *total_time_ms = wire_qst->total_compute_time_ms;
    *applyFT_time_ms = wire_qst->total_applyFT_time_ms;  // 0 for WireQST
    *call_count = wire_qst->compute_call_count;
    return 0;
  }

  // Plugin type not supported for timing
  *total_time_ms = 0.0;
  *applyFT_time_ms = 0.0;
  *call_count = 0;
  return -2;  // Plugin doesn't support timing
}

}  // namespace mujoco::plugin::elasticity

// C wrapper function - exported from the elasticity plugin library.
// This is intended to be called via ctypes from Python.
extern "C" MJAPI int mj_getPluginTiming(const mjModel* m, mjData* d, int instance,
                                        double* total_time_ms, double* applyFT_time_ms,
                                        int* call_count) {
  return mujoco::plugin::elasticity::GetPluginTimingInternal(
      m, d, instance, total_time_ms, applyFT_time_ms, call_count);
}

