# Adding Per-Object Timing (Profiling) to a MuJoCo Plugin

This guide explains how to add **per-object timing** (profiling) to a MuJoCo plugin so you can measure the time spent in each plugin instance's `Compute` function. Each plugin object tracks its own timing, but timing can be enabled/disabled globally for all objects of a class.

## 1. Add Timing Variables to the Class

In your plugin's header file (e.g., `wire.h`):

```cpp
class Wire {
public:
    // ...
    double total_compute_time_ms = 0.0;  // Per-object: accumulates time for this instance
    int compute_call_count = 0;          // Per-object: call count for this instance
    bool timing_enabled;          // Static: enables/disables timing for all objects
    void PrintComputeTiming();           // Prints timing info for this object
    // ...
};
```

## 2. Define the Static Member in the Source File

In your plugin's `.cc` file (e.g., `wire.cc`), **outside any function**, add:

```cpp
namespace mujoco::plugin::elasticity {
bool Wire::timing_enabled = true;
}
```

This is required for every `static` member variable you declare in the class.

## 3. Time the Compute Function (Per-Object)

In your `Compute` function, use `std::chrono` to measure elapsed time and accumulate it in the object's members:

```cpp
void Wire::Compute(const mjModel* m, mjData* d, int instance) {
    using namespace std::chrono;
    high_resolution_clock::time_point start, end;
    if (timing_enabled) start = high_resolution_clock::now();
    // ... your Compute logic ...
    if (timing_enabled) {
        end = high_resolution_clock::now();
        double elapsed = duration<double, std::milli>(end - start).count();
        total_compute_time_ms += elapsed;
        compute_call_count++;
    }
}
```

## 4. Print Timing at the End of Simulation (Per-Object)

In your plugin's `destroy` lambda, call the timing print function on the object before deleting it:

```cpp
plugin.destroy = +[](mjData* d, int instance) {
    auto* elasticity = reinterpret_cast<Wire*>(d->plugin_data[instance]);
    elasticity->PrintComputeTiming();
    delete elasticity;
    d->plugin_data[instance] = 0;
};
```

## 5. Implement the Print Function

In your `.cc` file:

```cpp
void Wire::PrintComputeTiming() {
    std::cout << "[Wire] Compute called " << compute_call_count << " times. "
              << "Total time: " << total_compute_time_ms << " ms. "
              << "Average time: " << (compute_call_count ? (total_compute_time_ms / compute_call_count) : 0.0) << " ms." << std::endl;
}
```

## 6. Repeat for Other Plugins

Repeat the above steps for each plugin (e.g., `WireQST`, `Cable`), using their respective class names and member variables.

## 7. Best Practices
- Use **non-static** timing variables for per-object timing.
- Always call the print function on the object (e.g., `obj->PrintComputeTiming();`).
- **Define** every static member variable in a `.cc` file, or you will get linker errors.
- You can call the timing print function at any point (not just destruction) for intermediate profiling.

## Example Output
```
[Wire] Compute called 1000 times. Total time: 123.45 ms. Average time: 0.123 ms.
```

---

**This approach is robust, avoids linker errors, and provides clear per-object timing for MuJoCo plugins.** 