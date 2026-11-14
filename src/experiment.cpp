#include "fusion/experiment.hpp"

#include <random>

#include "fusion/traffic.hpp"

namespace fusion {

namespace {

constexpr int kTicksPerSecond = 600;  // common multiple of the 20 Hz and 15 Hz sensor rates
constexpr int kRadarEvery = kTicksPerSecond / 20;
constexpr int kCameraEvery = kTicksPerSecond / 15;
constexpr int kScoreEvery = kTicksPerSecond / 10;

std::uint64_t mix(std::uint64_t seed, std::uint64_t stream) {
    std::seed_seq seq{static_cast<std::uint32_t>(seed), static_cast<std::uint32_t>(seed >> 32),
                      static_cast<std::uint32_t>(stream)};
    std::uint32_t out[2];
    seq.generate(out, out + 2);
    return (static_cast<std::uint64_t>(out[0]) << 32) | out[1];
}

}  // namespace

const char* mode_name(Mode m) {
    switch (m) {
        case Mode::Radar: return "radar";
        case Mode::Camera: return "camera";
        case Mode::Fused: return "fused";
    }
    return "?";
}

Totals run_once(const RunOptions& opt) {
    Traffic traffic(mix(opt.seed, 0));
    std::mt19937_64 radar_rng(mix(opt.seed, 1)), camera_rng(mix(opt.seed, 2));
    const SensorSpec radar = radar_spec(), camera = camera_spec();
    Tracker tracker(opt.tracker);
    Evaluator eval;

    const long ticks = static_cast<long>(opt.duration * kTicksPerSecond);
    for (long tick = 1; tick <= ticks; ++tick) {
        traffic.step(1.0 / kTicksPerSecond);
        const double t = static_cast<double>(tick) / kTicksPerSecond;
        const bool radar_now = tick % kRadarEvery == 0, camera_now = tick % kCameraEvery == 0;
        const bool score_now = tick % kScoreEvery == 0 && t >= opt.warmup;
        if (!radar_now && !camera_now && !score_now) continue;
        const std::vector<Truth> truths = traffic.truths();

        if (radar_now) {
            const auto scan = sense(radar, truths, t, radar_rng);
            if (opt.mode != Mode::Camera) tracker.process(scan, radar, t);
        }
        if (camera_now) {
            const auto scan = sense(camera, truths, t, camera_rng);
            if (opt.mode != Mode::Radar) tracker.process(scan, camera, t);
        }
        if (score_now) {
            const auto tracks = tracker.confirmed_at(t);
            eval.add(truths, tracks, 1.0 / 10);
            if (opt.trace) {
                for (const Truth& tr : truths)
                    std::fprintf(opt.trace, "%.2f,truth,%d,%.3f,%.3f\n", t, tr.id, tr.x, tr.y);
                for (const Track& tk : tracks)
                    std::fprintf(opt.trace, "%.2f,track,%d,%.3f,%.3f\n", t, tk.id, tk.x(0, 0), tk.x(1, 0));
            }
        }
    }
    return eval.totals();
}

}  // namespace fusion
