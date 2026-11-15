#pragma once
// One simulated drive: traffic, both sensors and one tracker setup.
#include <cstdint>
#include <cstdio>

#include "fusion/metrics.hpp"
#include "fusion/tracker.hpp"

namespace fusion {

enum class Mode { Radar, Camera, Fused };
const char* mode_name(Mode m);

struct RunOptions {
    Mode mode = Mode::Fused;
    std::uint64_t seed = 1;
    double duration = 60.0;  // s
    double warmup = 5.0;     // s before scoring starts
    bool occlusion = true;   // nearer cars hide farther ones from both sensors
    TrackerConfig tracker;
    std::FILE* trace = nullptr;  // optional CSV of truth and tracks at every scoring step
};

// Both sensors always generate their detections from their own random streams,
// whatever the mode, so every mode sees exactly the same measurements.
Totals run_once(const RunOptions& opt);

}  // namespace fusion
