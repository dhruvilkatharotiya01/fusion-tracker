#include "fusion/sensors.hpp"

#include <algorithm>
#include <cmath>

namespace fusion {

namespace {
constexpr double kDeg = kPi / 180.0;
}

SensorSpec radar_spec() {
    // Long-range radar: 20 Hz, about 1 degree bearing noise, some clutter.
    return {SensorKind::Radar, 20.0, 40 * kDeg, 1.0, 160.0, 0.90, 4.0,
            0.25, 0.0, 0.0, 1.0 * kDeg, 0.10, 0.3};
}

SensorSpec camera_spec() {
    // Mono camera: 15 Hz, sharp bearing, range from object size, so the range
    // error grows roughly with the square of distance (about 6.5 m at 80 m).
    return {SensorKind::Camera, 15.0, 30 * kDeg, 2.0, 90.0, 0.95, 0.5,
            0.3, 0.03, 0.0006, 0.15 * kDeg, 0.0, 0.0};
}

double range_sigma(const SensorSpec& s, double range) {
    return s.sigma_range_const + s.sigma_range_lin * range + s.sigma_range_quad * range * range;
}

bool in_view(const SensorSpec& s, double x, double y) {
    const double r = std::hypot(x, y);
    return r >= s.min_range && r <= s.max_range && std::abs(std::atan2(y, x)) <= s.half_fov;
}

namespace {

struct Span {
    double lo, hi;
};

Span angular_span(double x, double y) {
    const double r = std::max(std::hypot(x, y), 0.1);
    const double th = std::atan2(y, x);
    // Half the box's width as seen along the line of sight.
    const double half = 0.5 * (kCarWidth * std::abs(std::cos(th)) + kCarLength * std::abs(std::sin(th))) / r;
    return {th - half, th + half};
}

}  // namespace

double visible_fraction(double x, double y, const std::vector<Truth>& others) {
    const Span target = angular_span(x, y);
    const double r = std::hypot(x, y);
    std::vector<Span> cover;
    for (const Truth& o : others) {
        // Only clearly nearer cars hide the target; this also skips the target itself.
        if (std::hypot(o.x, o.y) > r - kCarLength * 0.5) continue;
        const Span sp = angular_span(o.x, o.y);
        const double lo = std::max(sp.lo, target.lo), hi = std::min(sp.hi, target.hi);
        if (hi > lo) cover.push_back({lo, hi});
    }
    std::sort(cover.begin(), cover.end(), [](const Span& a, const Span& b) { return a.lo < b.lo; });
    double hidden = 0, reach = target.lo;
    for (const Span& c : cover) {
        if (c.hi <= reach) continue;
        hidden += c.hi - std::max(c.lo, reach);
        reach = c.hi;
    }
    return 1.0 - hidden / (target.hi - target.lo);
}

std::vector<Detection> sense(const SensorSpec& s, const std::vector<Truth>& truths, double t,
                             std::mt19937_64& rng, bool occlusion) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::vector<Detection> out;
    const bool radar = s.kind == SensorKind::Radar;

    for (const Truth& tr : truths) {
        if (!in_view(s, tr.x, tr.y)) continue;
        const bool hidden = occlusion && visible_fraction(tr.x, tr.y, truths) < kOccludedBelow;
        if (unit(rng) > (hidden ? s.p_detect_occluded : s.p_detect)) continue;
        const double r = std::hypot(tr.x, tr.y);
        const double rate = (tr.x * tr.vx + tr.y * tr.vy) / r;
        Detection d{s.kind, t, 0, 0, 0, tr.id};
        d.range = std::max(0.1, r + range_sigma(s, r) * gauss(rng));
        d.bearing = std::atan2(tr.y, tr.x) + s.sigma_bearing * gauss(rng);
        d.range_rate = radar ? rate + s.sigma_range_rate * gauss(rng) : 0.0;
        out.push_back(d);
    }

    std::poisson_distribution<int> clutter(s.clutter_mean);
    const int n = clutter(rng);
    for (int i = 0; i < n; ++i) {
        Detection d{s.kind, t, 0, 0, 0, -1};
        d.range = s.min_range + unit(rng) * (s.max_range - s.min_range);
        d.bearing = (2 * unit(rng) - 1) * s.half_fov;
        d.range_rate = radar ? -35.0 + 50.0 * unit(rng) : 0.0;
        out.push_back(d);
    }
    // The tracker must not learn anything from the order of detections.
    std::shuffle(out.begin(), out.end(), rng);
    return out;
}

}  // namespace fusion
