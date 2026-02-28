# Copyright 2022 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Timing utilities for MuJoCo plugin execution."""

import ctypes
import os
from typing import Dict, List, Optional

from mujoco._functions import mj_step as _mj_step
from mujoco._structs import MjData, MjModel


def _load_timing_libraries() -> List[ctypes.CDLL]:
  """Load plugin libraries that export mj_getPluginTiming."""
  libs: List[ctypes.CDLL] = []
  plugin_dir = os.path.join(os.path.dirname(__file__), "plugin")
  if not os.path.isdir(plugin_dir):
    return libs

  for directory, _, filenames in os.walk(plugin_dir):
    for filename in filenames:
      if not filename.endswith((".so", ".dll", ".dylib")):
        continue
      path = os.path.join(directory, filename)
      try:
        handle = ctypes.CDLL(path)
      except OSError:
        continue
      try:
        func = getattr(handle, "mj_getPluginTiming")
      except AttributeError:
        continue
      # Configure prototype once (total_time_ms, applyFT_time_ms, call_count)
      func.argtypes = [
          ctypes.c_void_p,  # const mjModel*
          ctypes.c_void_p,  # mjData*
          ctypes.c_int,     # instance
          ctypes.POINTER(ctypes.c_double),  # total_time_ms
          ctypes.POINTER(ctypes.c_double),  # applyFT_time_ms
          ctypes.POINTER(ctypes.c_int),     # call_count
      ]
      func.restype = ctypes.c_int
      libs.append(handle)
  return libs


_TIMING_LIBS: List[ctypes.CDLL] = _load_timing_libraries()


def _get_plugin_timing_for_instance(
    model: MjModel,
    data: MjData,
    instance: int,
) -> Optional[Dict[str, float]]:
  """Call mj_getPluginTiming for one plugin instance, if available."""
  if not _TIMING_LIBS:
    return None

  m_ptr = ctypes.c_void_p(model._address)  # type: ignore[attr-defined]
  d_ptr = ctypes.c_void_p(data._address)   # type: ignore[attr-defined]

  total_time = ctypes.c_double(0.0)
  applyFT_time = ctypes.c_double(0.0)
  call_count = ctypes.c_int(0)

  for handle in _TIMING_LIBS:
    try:
      func = getattr(handle, "mj_getPluginTiming")
    except AttributeError:
      continue
    result = func(m_ptr, d_ptr, instance,
                  ctypes.byref(total_time), ctypes.byref(applyFT_time),
                  ctypes.byref(call_count))
    if result == 0 and call_count.value > 0:
      avg_time = (
          total_time.value / call_count.value if call_count.value > 0 else 0.0
      )
      rest_time = total_time.value - applyFT_time.value
      return {
          "instance": float(instance),
          "total_time_ms": total_time.value,
          "applyFT_time_ms": applyFT_time.value,
          "rest_time_ms": rest_time,
          "call_count": float(call_count.value),
          "avg_time_ms": avg_time,
      }
  return None


def mj_step_timed(
    model: MjModel,
    data: MjData,
    nstep: int = 1,
    return_timing: bool = False,
) -> Optional[Dict]:
  """Wrapper around mj_step that measures plugin execution time.

  This executes `mj_step` and then queries `mj_getPluginTiming` from any
  loaded plugin libraries (e.g., the elasticity plugins for Wire/Cable).
  """
  _mj_step(model, data, nstep)

  if not return_timing:
    return None

  plugin_timing_list: List[Dict[str, float]] = []
  total_plugin_time_ms = 0.0

  for instance in range(model.nplugin):
    info = _get_plugin_timing_for_instance(model, data, instance)
    if info is None:
      continue
    plugin_timing_list.append(info)
    total_plugin_time_ms += info["total_time_ms"]

  total_applyFT_ms = sum(
      p.get("applyFT_time_ms", 0.0) for p in plugin_timing_list
  )
  total_rest_ms = sum(
      p.get("rest_time_ms", 0.0) for p in plugin_timing_list
  )

  return {
      "plugin_timing": plugin_timing_list,
      "total_plugin_time_ms": total_plugin_time_ms,
      "total_plugin_applyFT_time_ms": total_applyFT_ms,
      "total_plugin_rest_time_ms": total_rest_ms,
      "nsteps": nstep,
  }

