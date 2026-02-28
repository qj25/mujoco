# Summary of Changes for Plugin Timing

This document summarizes the changes made to enable timing of plugin `Compute` calls from the Python API, including a split between **total** compute time, **applyFT** time (calls to `mj_applyFT`), and **rest** (solving algorithm / other).

---

## Files Changed

| File | Change |
|------|--------|
| `plugin/elasticity/wire.h` | Added `total_applyFT_time_ms`. |
| `plugin/elasticity/wire.cc` | Timed the `mj_applyFT` loop; accumulate into `total_applyFT_time_ms` when `timing_enabled`. |
| `plugin/elasticity/cable.h` | Added `total_applyFT_time_ms`. |
| `plugin/elasticity/cable.cc` | Timed each `mj_applyFT` call in the body loop; accumulate into `total_applyFT_time_ms` when `timing_enabled`. |
| `plugin/elasticity/wire_qst.h` | Added `total_applyFT_time_ms` (always 0; WireQST does not call `mj_applyFT`). |
| `plugin/elasticity/plugin_timing.h` | Extended `mj_getPluginTiming` with `applyFT_time_ms` output. |
| `plugin/elasticity/plugin_timing.cc` | Added WireQST support; read and return `total_applyFT_time_ms` for Wire, Cable, WireQST. |
| `python/mujoco/timing.py` | Extended ctypes prototype and return dict with `applyFT_time_ms`, `rest_time_ms`; `mj_step_timed` returns `total_plugin_applyFT_time_ms` and `total_plugin_rest_time_ms`. |

---

## Design Overview

- **No changes to the core MuJoCo library.** Timing is implemented in the elasticity plugin and Python.
- Plugins (Wire, Cable, WireQST) expose **total** compute time and **applyFT** time. **Rest** is computed as `total - applyFT` (solving algorithm and other work).
- The C function `mj_getPluginTiming` and the Python `mj_step_timed` wrapper expose these values.

---

## 1. Plugin-Side: `mj_getPluginTiming`

### Files

| File | Purpose |
|------|---------|
| `plugin/elasticity/plugin_timing.h` | Declares `mj_getPluginTiming` (with `applyFT_time_ms`). |
| `plugin/elasticity/plugin_timing.cc` | Implements `mj_getPluginTiming`; supports Wire, Cable, WireQST. |

### API

```c
int mj_getPluginTiming(const mjModel* m, mjData* d, int instance,
                      double* total_time_ms, double* applyFT_time_ms, int* call_count);
```

- **total_time_ms**: Total time spent in this plugin instance’s `Compute` (ms).
- **applyFT_time_ms**: Time spent inside `mj_applyFT` calls (ms). Wire and Cable accumulate this in a timed loop; WireQST reports 0.
- **call_count**: Number of `Compute` calls.
- **rest** (not returned): `total_time_ms - applyFT_time_ms`.

### Behavior

- Reads timing from `d->plugin_data[instance]` for `mujoco.elasticity.wire`, `mujoco.elasticity.cable`, and `mujoco.elasticity.wire_qst`.
- Uses plugin name to cast to `Wire*`, `Cable*`, or `WireQST*` and reads `total_compute_time_ms`, `total_applyFT_time_ms`, `compute_call_count`.
- Returns `0` on success, `-1` on invalid parameters, `-2` if the plugin type does not support timing.
- Exported as `extern "C" MJAPI` for use from Python via ctypes.

### Build

- `plugin/elasticity/CMakeLists.txt`: `plugin_timing.cc` and `plugin_timing.h` are in `MUJOCO_ELASTICITY_SRCS`.

---

## 2. Plugin Instrumentation: applyFT Timing

### Wire (`wire.h`, `wire.cc`)

- **wire.h**: Added member `double total_applyFT_time_ms = 0.0`.
- **wire.cc**: In `Compute()`, when `timing_enabled`, the loop that calls `mj_applyFT` for each body is timed with `high_resolution_clock`; the per-call elapsed time is summed into a local variable, then added to `total_applyFT_time_ms` after the loop.

### Cable (`cable.h`, `cable.cc`)

- **cable.h**: Added member `double total_applyFT_time_ms = 0.0`.
- **cable.cc**: In `Compute()`, when `timing_enabled`, each `mj_applyFT(m, d, ...)` call is wrapped with `high_resolution_clock`; the elapsed time is added to `total_applyFT_time_ms`. Local variables `t0_ft`/`t1_ft` are used so the existing total compute time (start of `Compute` to end) is unchanged.

### WireQST (`wire_qst.h`)

- **wire_qst.h**: Added member `double total_applyFT_time_ms = 0.0` (documented as always 0, since WireQST writes directly to `qfrc_passive` and does not call `mj_applyFT`).

---

## 3. Python-Side: `mj_step_timed`

### File

| File | Change |
|------|--------|
| `python/mujoco/timing.py` | `mj_step_timed` and ctypes access to `mj_getPluginTiming`; extended for `applyFT_time_ms` and `rest_time_ms`. |

### ctypes Prototype

- `mj_getPluginTiming` is called with six arguments: `(model_ptr, data_ptr, instance, total_time_ms, applyFT_time_ms, call_count)`.
- `applyFT_time_ms` is a `ctypes.POINTER(ctypes.c_double)`.

### Return Value of `_get_plugin_timing_for_instance()`

Each plugin instance’s timing dict now includes:

- `total_time_ms`
- `applyFT_time_ms`
- `rest_time_ms` (computed as `total_time_ms - applyFT_time_ms`)
- `call_count`, `avg_time_ms` (unchanged)

### Return Value of `mj_step_timed()`

When `return_timing=True`, the dict includes:

- `plugin_timing`: list of per-instance dicts (with `total_time_ms`, `applyFT_time_ms`, `rest_time_ms`, etc.).
- `total_plugin_time_ms`: sum of total time over all instances.
- `total_plugin_applyFT_time_ms`: sum of applyFT time over all instances.
- `total_plugin_rest_time_ms`: sum of rest time over all instances.
- `nsteps`: number of steps executed.

---

## Usage

```python
import mujoco

model = mujoco.MjModel.from_xml_path("model.xml")
data = mujoco.MjData(model)

# Run steps and get plugin timing (total / applyFT / rest)
timing = mujoco.mj_step_timed(model, data, nstep=100, return_timing=True)

for info in timing["plugin_timing"]:
    print(f"Instance {info['instance']}: total={info['total_time_ms']:.3f} ms, "
          f"applyFT={info['applyFT_time_ms']:.3f} ms, rest={info['rest_time_ms']:.3f} ms, "
          f"calls={info['call_count']}")
print(f"Total plugin time: {timing['total_plugin_time_ms']:.3f} ms")
print(f"Total applyFT time: {timing['total_plugin_applyFT_time_ms']:.3f} ms")
print(f"Total rest time: {timing['total_plugin_rest_time_ms']:.3f} ms")
```

---

## Limitations

- Only Wire, Cable, and WireQST are supported. Other plugins must add the same timing fields and be handled in `GetPluginTimingInternal` to be included.
- Timing is only recorded when `timingEnabled` (or equivalent) is true in the plugin config.
- Python discovers `mj_getPluginTiming` by loading plugin libraries under `mujoco/plugin/`. If the elasticity plugin is not built or not on that path, `mj_step_timed` will return no timing.
